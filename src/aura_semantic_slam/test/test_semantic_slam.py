"""Offline behavior checks. No ROS, Gazebo, Torch, OpenCV or model required."""
from pathlib import Path
from types import SimpleNamespace
import math
import tempfile
import unittest
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from semantic_core import (AnchoredMap, Detection, Frame, OdometryBuffer,
    colour, depth_array, downsample, foreground, inverse, matrix, project, transform)
from semantic_slam_mapper import archive_arrays, pack_cloud


def pose(x=0., y=0., yaw=0.):
    return matrix((x, y, 0), (0, 0, math.sin(yaw/2), math.cos(yaw/2)))


def surface(x=2., y=0., z=.2):
    return np.array([[x+a, y+b, z+c] for a in (-.08, 0, .08)
                     for b in (-.08, 0, .08) for c in (-.08, 0, .08)], np.float32)


def frame(stamp, anchor, points, name="sports ball", confidence=.9):
    return Frame(stamp, anchor, [Detection(name, confidence, points)], np.empty((0, 3), np.float32))


class GeometryTests(unittest.TestCase):
    def test_optical_projection(self):
        xyz = project(np.full((3, 3), 2., np.float32), [2, 0, 1, 0, 2, 1, 0, 0, 1])
        np.testing.assert_allclose(xyz[1, 1], [0, 0, 2])
        np.testing.assert_allclose(xyz[1, 2], [1, 0, 2])
        np.testing.assert_allclose(xyz[2, 1], [0, 1, 2])

    def test_padded_big_endian_depth(self):
        msg = SimpleNamespace(encoding="16UC1", is_bigendian=True, width=2, height=2,
                              step=6, data=b"\x03\xe8\x07\xd0xx\x0b\xb8\x0f\xa0xx")
        np.testing.assert_allclose(depth_array(msg), [[1, 2], [3, 4]])

    def test_depth_rejects_truncated_data(self):
        msg = SimpleNamespace(encoding="32FC1", is_bigendian=False, width=2, height=2, step=8, data=b"bad")
        with self.assertRaises(ValueError):
            depth_array(msg)

    def test_foreground_excludes_far_background_and_invalid_depth(self):
        xyz = np.zeros((4, 4, 3), np.float32)
        xyz[:, :, 2] = 2.
        xyz[0, 0, 2] = 5.
        xyz[0, 1, 2] = np.nan
        p = foreground(xyz, np.ones((4, 4), bool), .2, 6., 1, .8)
        self.assertEqual(len(p), 14)
        np.testing.assert_allclose(p[:, 2], 2.)

    def test_timestamp_interpolation_through_pi(self):
        b = OdometryBuffer()
        b.add(0, "odom", [0, 0, 0], [0, 0, math.sin(math.radians(179)/2), math.cos(math.radians(179)/2)])
        b.add(100, "odom", [2, 0, 0], [0, 0, math.sin(math.radians(-179)/2), math.cos(math.radians(-179)/2)])
        t = b.at(50, 100)
        np.testing.assert_allclose(t[:3, 3], [1, 0, 0])
        np.testing.assert_allclose(t[:3, 0], [-1, 0, 0], atol=1e-6)

    def test_no_pose_extrapolation_or_large_gap(self):
        b = OdometryBuffer()
        b.add(10, "odom", [0, 0, 0], [0, 0, 0, 1])
        b.add(200, "odom", [1, 0, 0], [0, 0, 0, 1])
        self.assertIsNone(b.at(5, 300))
        self.assertIsNone(b.at(250, 300))
        self.assertIsNone(b.at(100, 100))

    def test_reset_is_detected(self):
        b = OdometryBuffer()
        b.add(100, "odom", [0, 0, 0], [0, 0, 0, 1])
        with self.assertRaises(ValueError):
            b.add(90, "odom", [0, 0, 0], [0, 0, 0, 1])

    def test_camera_motion_keeps_object_fixed(self):
        world = surface()
        optical_to_base = matrix([.12, 0, .45], [-.5, .5, -.5, .5])
        robot = pose(1, -1, 1.2)
        optical = transform(world, inverse(robot @ optical_to_base))
        np.testing.assert_allclose(transform(optical, robot @ optical_to_base), world, atol=1e-6)

    def test_voxel_and_point_caps(self):
        points = np.arange(300, dtype=float).reshape(100, 3)
        self.assertEqual(len(downsample(points, .01, 12)), 12)
        self.assertEqual(len(downsample(np.vstack([points, points]), .01, 200)), 100)


class SemanticMapTests(unittest.TestCase):
    def make_map(self, **kwargs):
        m = AnchoredMap(min_observations=3, **kwargs)
        m.update_graph({1: pose()}, [(1, 0, pose(), 0)])
        return m

    def test_confirmation_needs_three_distinct_frames(self):
        m = self.make_map()
        for i in (1, 2):
            m.ingest(frame(i, 1, surface()))
        self.assertEqual(len(m.confirmed()), 0)
        m.ingest(frame(2, 1, surface()))
        self.assertEqual(len(m.objects[1].stamps), 2)
        m.ingest(frame(3, 1, surface()))
        self.assertEqual(len(m.confirmed()), 1)

    def test_revisit_keeps_id_and_colour(self):
        m = self.make_map()
        for i in range(1, 4):
            m.ingest(frame(i, 1, surface()))
        old = m.graph("map")["objects"][0]
        m.ingest(frame(4, 1, surface(2.05)))
        new = m.graph("map")["objects"][0]
        self.assertEqual(old["id"], new["id"])
        self.assertEqual(old["colour_rgb"], new["colour_rgb"])

    def test_two_balls_same_frame_are_distinct(self):
        m = self.make_map()
        for i in range(1, 4):
            m.ingest(Frame(i, 1, [Detection("sports ball", .9, surface(2)),
                                  Detection("sports ball", .9, surface(2.6))], np.empty((0, 3))))
        self.assertEqual(len(m.confirmed()), 2)

    def test_different_classes_do_not_merge(self):
        m = self.make_map()
        m.ingest(Frame(1, 1, [Detection("chair", .9, surface()),
                              Detection("potted plant", .9, surface())], np.empty((0, 3))))
        self.assertEqual(len(m.objects), 2)

    def test_pose_correction_moves_old_surfaces(self):
        m = self.make_map()
        for i in range(1, 4):
            m.ingest(frame(i, 1, surface()))
        before = m.graph("map")["objects"][0]
        m.update_graph({1: pose(.8, -.4)}, [])
        self.assertTrue(m.dirty)
        m.rebuild()
        after = m.graph("map")["objects"][0]
        np.testing.assert_allclose(np.array(after["position_m"])-before["position_m"], [.8, -.4, 0], atol=1e-6)
        self.assertEqual(before["id"], after["id"])

    def test_loop_correction_merges_duplicate_tracks(self):
        m = self.make_map()
        m.update_graph({2: pose(1.3)}, [(2, 4, pose(1.3), 0)])
        for i in range(1, 4):
            m.ingest(frame(i, 1, surface()))
        for i in range(4, 7):
            # A drifted second keyframe creates a duplicate at x=3.3.
            m.ingest(frame(i, 2, surface()))
        self.assertEqual(len(m.confirmed()), 2)
        m.update_graph({2: pose()}, [])
        m.rebuild()
        self.assertEqual(len(m.confirmed()), 1)
        self.assertEqual(m.confirmed()[0].number, 1)
        self.assertEqual(len(m.confirmed()[0].stamps), 6)

    def test_rehearsal_rebases_existing_observations(self):
        m = self.make_map()
        m.ingest(frame(1, 1, surface()))
        before = m.objects[1].centre.copy()
        m.update_graph({1: pose(.4)}, [(1, 2, pose(.4), 0)])
        m.rebuild()
        np.testing.assert_allclose(m.objects[1].centre, before, atol=1e-6)

    def test_relations_update_after_correction(self):
        m = self.make_map()
        m.update_graph({2: pose()}, [(2, 4, pose(), 0)])
        for i in range(1, 4):
            m.ingest(frame(i, 1, surface(), "chair"))
        for i in range(4, 7):
            m.ingest(frame(i, 2, surface(3), "potted plant"))
        self.assertTrue(any(e["relation"] == "near" for e in m.graph("map")["edges"]))
        m.update_graph({2: pose(3)}, [])
        m.rebuild()
        self.assertFalse(any(e["relation"] == "near" for e in m.graph("map")["edges"]))

    def test_unknown_anchor_is_rejected(self):
        m = self.make_map()
        with self.assertRaises(ValueError):
            m.ingest(frame(1, 999, surface()))

    def test_distant_anchor_is_not_reused(self):
        m = self.make_map()
        self.assertIsNone(m.anchor_for(1, pose(10)))
        self.assertEqual(m.anchor_for(1, pose(.2)), 1)

    def test_six_object_scene_retains_six_tracks_after_revisit(self):
        m = self.make_map()
        locations = [("sports ball", -2, -2), ("sports ball", 2, -2), ("sports ball", 0, 2),
                     ("potted plant", -3, 1), ("potted plant", 3, 1), ("chair", 0, -3)]
        for stamp in range(1, 7):
            objects = [Detection(name, .9, surface(x, y)) for name, x, y in locations]
            m.ingest(Frame(stamp, 1, objects, np.empty((0, 3))))
        self.assertEqual(len(m.confirmed()), 6)
        ids = {o.number for o in m.confirmed()}
        m.update_graph({1: pose(.3, .2, .05)}, [])
        m.rebuild()
        self.assertEqual({o.number for o in m.confirmed()}, ids)

    def test_retention_limit_rebuilds_consistently(self):
        m = self.make_map(max_frames=3)
        for i in range(1, 7):
            m.ingest(frame(i, 1, surface()))
        self.assertEqual(list(m.frames), [4, 5, 6])
        self.assertEqual(len(m.confirmed()), 1)
        self.assertEqual(m.confirmed()[0].number, 1)
        self.assertEqual(len(m.confirmed()[0].stamps), 3)

    def test_packed_cloud_has_correct_rgb_and_ids(self):
        b = pack_cloud(np.array([[1, 2, 3]], np.float32), np.array([[12, 34, 56]], np.uint8), np.array([7]))
        row = np.frombuffer(b, dtype=[("xyz", "<f4", (3,)), ("rgb", "<u4"), ("id", "<u4")])[0]
        self.assertEqual(row["rgb"], (12 << 16) | (34 << 8) | 56)
        self.assertEqual(row["id"], 7)

    def test_saved_observations_are_safe_numeric_arrays(self):
        m = self.make_map()
        m.ingest(frame(1, 1, surface()))
        archive = archive_arrays(m)
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/"a.npz"
            np.savez_compressed(p, **archive)
            with np.load(p, allow_pickle=False) as a:
                np.testing.assert_allclose(a["points_in_anchor"], surface())
                self.assertEqual(a["classes"][0], "sports ball")
                self.assertEqual(a["anchor_ids"][0], 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
