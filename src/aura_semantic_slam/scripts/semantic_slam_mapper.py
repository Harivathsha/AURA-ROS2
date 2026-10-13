#!/usr/bin/env python3
"""AURA semantic RGB-D SLAM: inference off-thread, geometry anchored to RTAB-Map."""
from collections import deque
from datetime import datetime
from pathlib import Path
import itertools
import json
import math
import os
import threading
import time
import numpy as np
from semantic_core import (AnchoredMap, Detection, Frame, OdometryBuffer, colour,
                           depth_array, downsample, foreground, image_bgr,
                           inverse, matrix, project, transform)


def ns(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    return matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def transform_matrix(t):
    p, q = t.translation, t.rotation
    return matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def pack_cloud(points, colours, ids):
    data = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                      ("rgb", "<u4"), ("object_id", "<u4")])
    for axis, name in enumerate(("x", "y", "z")):
        data[name] = points[:, axis]
    rgb = colours.astype(np.uint32)
    data["rgb"] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    data["object_id"] = ids
    return data.tobytes()


def save_files(folder, points, colours, ids, graph, archive):
    folder.mkdir(parents=True, exist_ok=True)
    vertices = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                        ("red", "u1"), ("green", "u1"), ("blue", "u1"), ("object_id", "<u4")])
    for axis, name in enumerate(("x", "y", "z")):
        vertices[name] = points[:, axis]
    for axis, name in enumerate(("red", "green", "blue")):
        vertices[name] = colours[:, axis]
    vertices["object_id"] = ids
    header = ("ply\nformat binary_little_endian 1.0\n"+f"element vertex {len(points)}\n"+
              "property float x\nproperty float y\nproperty float z\nproperty uchar red\n"+
              "property uchar green\nproperty uchar blue\nproperty uint object_id\nend_header\n")
    with (folder/"semantic_map.ply.tmp").open("wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(vertices.tobytes())
    os.replace(folder/"semantic_map.ply.tmp", folder/"semantic_map.ply")
    (folder/"scene_graph.json.tmp").write_text(json.dumps(graph, indent=2))
    os.replace(folder/"scene_graph.json.tmp", folder/"scene_graph.json")
    with (folder/"anchor_observations.npz.tmp").open("wb") as handle:
        np.savez_compressed(handle, **archive)
    os.replace(folder/"anchor_observations.npz.tmp", folder/"anchor_observations.npz")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 7), constrained_layout=True)
    nodes = {n["id"]: n for n in graph["objects"]}
    for edge in graph["edges"]:
        if edge["relation"] == "near":
            a, b = nodes[edge["source"]], nodes[edge["target"]]
            ax.plot([a["position_m"][0], b["position_m"][0]],
                    [a["position_m"][1], b["position_m"][1]], color=".6", zorder=1)
    for node in nodes.values():
        x, y = node["position_m"][:2]
        ax.scatter(x, y, s=130, color=np.array(node["colour_rgb"])/255,
                   edgecolors="black", zorder=2)
        ax.annotate(node["label"], (x, y), xytext=(7, 8), textcoords="offset points")
    if not nodes:
        ax.text(.5, .5, "No confirmed objects yet. Inspect the camera overlay.",
                transform=ax.transAxes, ha="center")
    ax.set(xlabel="RTAB-Map X (m)", ylabel="RTAB-Map Y (m)",
           title="AURA semantic SLAM: lines show near relationships")
    ax.set_aspect("equal", adjustable="datalim")
    ax.margins(.2)
    ax.grid(alpha=.25)
    fig.savefig(folder/"scene_graph.tmp.png", dpi=150)
    plt.close(fig)
    os.replace(folder/"scene_graph.tmp.png", folder/"scene_graph.png")


def archive_arrays(core):
    """Safe NPZ arrays (no pickled objects): retain keyframe-local observations."""
    points, offsets, anchors, stamps, tracks, confidence, classes = [], [0], [], [], [], [], []
    for frame in core.frames.values():
        for d in frame.detections:
            points.append(d.points.copy())
            offsets.append(offsets[-1]+len(d.points))
            anchors.append(frame.anchor)
            stamps.append(frame.stamp)
            tracks.append(d.track_id)
            confidence.append(d.confidence)
            classes.append(d.class_name)
    node_ids = sorted(core.optimized_poses)
    return {"points_in_anchor": np.concatenate(points) if points else np.empty((0, 3), np.float32),
            "offsets": np.array(offsets, np.int64), "anchor_ids": np.array(anchors, np.int64),
            "stamps_ns": np.array(stamps, np.int64), "track_ids": np.array(tracks, np.int64),
            "confidence": np.array(confidence, np.float32), "classes": np.array(classes, dtype="U32"),
            "node_ids": np.array(node_ids, np.int64),
            "optimized_poses": np.array([core.optimized_poses[i] for i in node_ids]).reshape((-1, 4, 4))}


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from rtabmap_msgs.msg import MapData, MapGraph, RGBDImage, Info
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, PointField, Image
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import Header, String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener, TransformException

    class SemanticSLAM(Node):
        def __init__(self):
            super().__init__("aura_semantic_slam")
            defaults = {
                "rgbd_topic": "/aura_slam/rgbd_image", "odom_topic": "/aura_slam/odom",
                "map_data_topic": "/aura_slam/mapData", "map_graph_topic": "/aura_slam/mapGraph",
                "info_topic": "/aura_slam/info", "map_frame": "aura_slam_map",
                "base_frame": "aura_slam_base", "optical_frame": "aura_slam_optical",
                "model_path": "~/ai_envs/project/c++_project/models/yolo11s-seg.pt",
                "output_dir": "~/ai_envs/project/c++_project/semantic_slam_results",
                "device": "cpu", "image_size": 640, "torch_threads": 3,
                "confidence": .35, "process_hz": .75, "sync_tolerance": .025,
                "pose_max_gap": .15, "min_range": .20, "max_range": 6.,
                "point_stride": 1, "background_stride": 6, "mask_erode_pixels": 1,
                "depth_band": .8, "min_object_points": 25, "voxel_size": .04,
                "observation_voxel_size": .025, "association_distance": .75,
                "surface_support_radius": .18, "min_observations": 3, "max_objects": 60,
                "max_stored_frames": 600, "max_points_per_object": 5000,
                "max_background_points": 60000, "max_object_points_per_observation": 1500,
                "max_background_points_per_frame": 1800,
                "max_anchor_distance": 1.5, "max_anchor_angle": 1.25,
                "min_map_height": -.10, "max_map_height": 3.,
                "near_distance": 2., "relation_distance": 3., "axis_margin": .3,
                "autosave_seconds": 45., "allowed_classes": ["sports ball", "potted plant", "chair"]}
            self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            numeric = [k for k, v in defaults.items() if isinstance(v, (int, float)) and
                       k not in ("min_map_height", "mask_erode_pixels")]
            if any(not math.isfinite(self.p[k]) or self.p[k] <= 0 for k in numeric):
                raise ValueError("Rates, distances, counts, and time limits must be positive")
            if not 0 < self.p["confidence"] <= 1 or self.p["min_range"] >= self.p["max_range"]:
                raise ValueError("Invalid confidence or depth range")
            if self.p["min_map_height"] >= self.p["max_map_height"] or self.p["mask_erode_pixels"] < 0:
                raise ValueError("Invalid height range or mask erosion")
            self.model_path = Path(self.p["model_path"]).expanduser()
            if not self.model_path.is_file():
                raise FileNotFoundError(f"Segmentation weights not found: {self.model_path}")
            self.folder = Path(self.p["output_dir"]).expanduser()/datetime.now().strftime("aura_%Y%m%d_%H%M%S_%f")
            self.lock, self.save_lock = threading.RLock(), threading.Lock()
            self.stop = threading.Event()
            self.fatal, self.pending, self.extrinsics = None, deque(maxlen=8), None
            self.warning_times, self.counts = {}, {"rgbd": 0, "odom": 0, "mapData": 0, "mapGraph": 0}
            self.last_processed, self.processed, self.loop_closures = -1, 0, 0
            self.odom = OdometryBuffer()
            self.core = AnchoredMap(self.p["voxel_size"], self.p["association_distance"],
                                   self.p["surface_support_radius"], self.p["min_observations"],
                                   self.p["max_objects"], self.p["max_stored_frames"],
                                   self.p["max_points_per_object"], self.p["max_background_points"])
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.cloud_pub = self.create_publisher(PointCloud2, "/aura_semantic_slam/cloud", latched)
            self.marker_pub = self.create_publisher(MarkerArray, "/aura_semantic_slam/markers", latched)
            self.graph_pub = self.create_publisher(String, "/aura_semantic_slam/scene_graph", latched)
            self.overlay_pub = self.create_publisher(Image, "/aura_semantic_slam/overlay", 2)
            self.subscriptions_ = [
                self.create_subscription(RGBDImage, self.p["rgbd_topic"], self.rgbd, qos_profile_sensor_data),
                self.create_subscription(Odometry, self.p["odom_topic"], self.odometry,
                                         QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)),
                self.create_subscription(MapData, self.p["map_data_topic"], self.map_data, 10),
                self.create_subscription(MapGraph, self.p["map_graph_topic"], self.map_graph, 10),
                self.create_subscription(Info, self.p["info_topic"], self.info, 10)]
            self.services_ = [self.create_service(Trigger, "/aura_semantic_slam/save", self.save_service)]
            self.health_timer = self.create_timer(5., self.health,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.worker = threading.Thread(target=self.work, daemon=True)
            self.worker.start()
            self.get_logger().info(f"Semantic SLAM starting; results: {self.folder}")

        def warn(self, key, message):
            now = time.monotonic()
            if now-self.warning_times.get(key, -math.inf) >= 5:
                self.warning_times[key] = now
                self.get_logger().warning(message)

        def fail(self, message):
            self.fatal = message
            self.get_logger().error(message)
            self.stop.set()

        def rgbd(self, msg):
            with self.lock:
                self.pending.append(msg)  # Short bounded buffer; choose newest time-aligned pair.
                self.counts["rgbd"] += 1

        def odometry(self, msg):
            try:
                if msg.child_frame_id != self.p["base_frame"]:
                    raise ValueError(f"Expected odometry child {self.p['base_frame']}, got {msg.child_frame_id}")
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                with self.lock:
                    self.odom.add(ns(msg.header.stamp), msg.header.frame_id,
                                  (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
                    self.counts["odom"] += 1
            except ValueError as error:
                self.fail(str(error))

        def read_graph(self, graph, fallback_frame, nodes=()):
            frame = graph.header.frame_id or fallback_frame
            if frame and frame != self.p["map_frame"]:
                raise ValueError(f"Expected optimized graph in {self.p['map_frame']}, got {frame}")
            if len(graph.poses_id) != len(graph.poses):
                raise ValueError("RTAB-Map pose ids and poses have different lengths")
            optimized = {int(i): pose_matrix(p) for i, p in zip(graph.poses_id, graph.poses) if i > 0}
            metadata = [(int(n.id), int(round(n.stamp*1e9)), pose_matrix(n.pose), int(n.map_id))
                        for n in nodes if n.id > 0]
            with self.lock:
                self.core.update_graph(optimized, metadata)

        def map_data(self, msg):
            try:
                self.read_graph(msg.graph, msg.header.frame_id, msg.nodes)
                self.counts["mapData"] += 1
            except ValueError as error:
                self.fail(str(error))

        def map_graph(self, msg):
            try:
                self.read_graph(msg, msg.header.frame_id)
                self.counts["mapGraph"] += 1
            except ValueError as error:
                self.fail(str(error))

        def info(self, msg):
            if msg.loop_closure_id > 0:
                self.loop_closures += 1
                self.get_logger().info(f"RTAB-Map reports loop closure to keyframe {msg.loop_closure_id}")

        def health(self):
            with self.lock:
                self.get_logger().info(
                    f"Inputs={self.counts} processed={self.processed} "
                    f"anchors={len(self.core.raw_poses)} optimized={len(self.core.optimized_poses)} "
                    f"confirmed={len(self.core.confirmed())} rebuilds={self.core.rebuild_count} "
                    f"loop_reports={self.loop_closures}")

        def next_job(self):
            with self.lock:
                if not self.pending:
                    return None
                if self.extrinsics is None:
                    try:
                        t = self.buffer.lookup_transform(self.p["base_frame"], self.p["optical_frame"], Time())
                        self.extrinsics = transform_matrix(t.transform)
                        self.get_logger().info("Semantic optical mounting transform ready")
                    except TransformException:
                        self.warn("tf", "Waiting for the adapter's optical mounting transform")
                        return None
                for msg in reversed(self.pending):
                    stamp = ns(msg.rgb.header.stamp)
                    if stamp <= self.last_processed:
                        break
                    pose = self.odom.at(stamp, int(self.p["pose_max_gap"]*1e9))
                    if pose is None:
                        continue
                    anchor = self.core.anchor_for(stamp, pose, self.p["max_anchor_distance"], self.p["max_anchor_angle"])
                    if anchor is not None:
                        return msg, stamp, pose, self.extrinsics.copy()
                self.warn("anchor", "Waiting for timestamp-aligned odometry and a nearby RTAB-Map keyframe")
                return None

        def graph_snapshot(self):
            graph = self.core.graph(self.p["map_frame"], self.p["near_distance"],
                                    self.p["relation_distance"], self.p["axis_margin"])
            graph["configuration"] = self.p
            graph["processed_frames"] = self.processed
            graph["reported_loop_closures"] = self.loop_closures
            graph["odometry_parent_frame"] = self.odom.frame
            return graph

        def save(self):
            with self.save_lock:
                with self.lock:
                    if self.core.dirty:
                        self.core.rebuild()
                    cloud = self.core.cloud()
                    graph, archive = self.graph_snapshot(), archive_arrays(self.core)
                save_files(self.folder, *cloud, graph, archive)
                self.get_logger().info(f"Saved {len(graph['objects'])} confirmed objects to {self.folder}")

        def save_service(self, request, response):
            try:
                self.save()
                response.success, response.message = True, str(self.folder)
            except Exception as error:
                response.success, response.message = False, str(error)
            return response

        def publish(self):
            with self.lock:
                cloud, graph = self.core.cloud(), self.graph_snapshot()
            points, colours, ids = cloud
            header = Header(frame_id=self.p["map_frame"], stamp=self.get_clock().now().to_msg())
            msg = PointCloud2(header=header, height=1, width=len(points), is_bigendian=False,
                             is_dense=True, point_step=20, row_step=20*len(points))
            msg.fields = [PointField(name=name, offset=i*4, count=1,
                          datatype=PointField.FLOAT32 if i < 3 else PointField.UINT32)
                          for i, name in enumerate(("x", "y", "z", "rgb", "object_id"))]
            msg.data = pack_cloud(points, colours, ids)
            self.cloud_pub.publish(msg)
            self.graph_pub.publish(String(data=json.dumps(graph)))
            markers = MarkerArray()
            markers.markers.append(Marker(action=Marker.DELETEALL))
            def point(p):
                return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            def marker(namespace, number, kind, rgba):
                m = Marker(header=header, ns=namespace, id=number, type=kind, action=Marker.ADD)
                m.pose.orientation.w = 1.
                m.color.r, m.color.g, m.color.b, m.color.a = rgba
                return m
            by_id = {n["id"]: n for n in graph["objects"]}
            for node in by_id.values():
                rgba = tuple(c/255 for c in node["colour_rgb"])+(1.,)
                low, high = (np.array(node["visible_bounds_m"][k]) for k in ("min", "max"))
                corners = np.array(list(itertools.product(*zip(low, high))))
                box = marker("bounds", node["id"], Marker.LINE_LIST, rgba)
                box.scale.x = .015
                for i, j in itertools.combinations(range(8), 2):
                    if np.count_nonzero(corners[i] != corners[j]) == 1:
                        box.points.extend((point(corners[i]), point(corners[j])))
                markers.markers.append(box)
                text = marker("labels", node["id"], Marker.TEXT_VIEW_FACING, rgba)
                text.scale.z = .16
                text.pose.position = point([*node["position_m"][:2], high[2]+.20])
                text.text = node["label"]
                markers.markers.append(text)
            for i, edge in enumerate(graph["edges"]):
                if edge["relation"] == "near":
                    line = marker("near", i, Marker.LINE_LIST, (.1, .9, 1., .8))
                    line.scale.x = .015
                    line.points = [point(by_id[edge[k]]["position_m"]) for k in ("source", "target")]
                    markers.markers.append(line)
            self.marker_pub.publish(markers)

        def work(self):
            try:
                import cv2
                import torch
                from ultralytics import YOLO
                torch.set_num_threads(self.p["torch_threads"])
                model = YOLO(str(self.model_path), task="segment")
                if model.task != "segment":
                    raise ValueError("A segmentation model ending in -seg.pt is required")
                allowed = set(self.p["allowed_classes"])
                if not allowed.intersection(model.names.values()):
                    raise ValueError("Configured classes are absent from this YOLO model")
                self.get_logger().info("Segmentation loaded. Semantic SLAM ready; drive slowly.")
                last_inference, last_save = -math.inf, time.monotonic()
                while not self.stop.wait(.05):
                    rebuilt = False
                    with self.lock:
                        if self.core.dirty:
                            self.core.rebuild()
                            rebuilt = True
                    if rebuilt:
                        self.get_logger().info("Optimized poses changed: rebuilt semantic surfaces, IDs and graph")
                        self.publish()
                    if time.monotonic()-last_save >= self.p["autosave_seconds"]:
                        self.save()
                        last_save = time.monotonic()
                    if time.monotonic()-last_inference < 1/self.p["process_hz"]:
                        continue
                    job = self.next_job()
                    if job is None:
                        continue
                    msg, stamp, odom_pose, extrinsics = job
                    last_inference = time.monotonic()
                    try:
                        if abs(stamp-ns(msg.depth.header.stamp)) > self.p["sync_tolerance"]*1e9:
                            raise ValueError("RGB and depth acquisition stamps are too far apart")
                        if any(m.header.frame_id != self.p["optical_frame"]
                               for m in (msg.rgb, msg.depth, msg.rgb_camera_info)):
                            raise ValueError("RGB-D inputs must use the adapter's optical frame")
                        image, depth = image_bgr(msg.rgb), depth_array(msg.depth)
                        info = msg.rgb_camera_info
                        if depth.shape != image.shape[:2] or (info.height, info.width) != depth.shape:
                            raise ValueError("RGB, depth and calibration dimensions differ")
                        xyz = project(depth, info.k)
                        result = model.predict(image, device=self.p["device"], imgsz=self.p["image_size"],
                                               conf=self.p["confidence"], retina_masks=True,
                                               verbose=False, max_det=40)[0]
                        detections, mask_union, accepted_masks = [], np.zeros(depth.shape, bool), []
                        raw_labels, geometry_rejected = [], 0
                        if result.masks is not None and result.boxes is not None:
                            masks = result.masks.data.cpu().numpy()
                            if masks.shape[1:] != depth.shape:
                                raise ValueError("YOLO masks do not match the original RGB dimensions")
                            for mask, label, conf in zip(masks, result.boxes.cls.cpu().numpy(), result.boxes.conf.cpu().numpy()):
                                name = model.names[int(label)]
                                raw_labels.append(name)
                                if name not in allowed:
                                    continue
                                mask = (mask > .5).astype(np.uint8)
                                radius = self.p["mask_erode_pixels"]
                                if radius:
                                    mask = cv2.erode(mask, np.ones((2*radius+1, 2*radius+1), np.uint8))
                                mask = mask.astype(bool)
                                points = foreground(xyz, mask, self.p["min_range"], self.p["max_range"],
                                                    self.p["point_stride"], self.p["depth_band"])
                                base_points = transform(points, extrinsics)
                                valid = ((base_points[:, 2] >= self.p["min_map_height"]) &
                                         (base_points[:, 2] <= self.p["max_map_height"]))
                                points = points[valid]
                                if len(points) < self.p["min_object_points"]:
                                    geometry_rejected += 1
                                    continue
                                points = downsample(points, self.p["observation_voxel_size"],
                                                    self.p["max_object_points_per_observation"])
                                detections.append((name, float(conf), points))
                                accepted_masks.append(mask)
                                mask_union |= mask
                        stride = self.p["background_stride"]
                        bg = xyz[::stride, ::stride][~mask_union[::stride, ::stride]]
                        bg = bg[np.all(np.isfinite(bg), axis=1)]
                        bg = bg[(bg[:, 2] >= self.p["min_range"]) & (bg[:, 2] <= self.p["max_range"])]
                        base_bg = transform(bg, extrinsics)
                        bg = bg[(base_bg[:, 2] >= self.p["min_map_height"]) &
                                (base_bg[:, 2] <= self.p["max_map_height"])]
                        bg = downsample(bg, self.p["voxel_size"], self.p["max_background_points_per_frame"])
                        with self.lock:
                            # Choose against the latest graph, not the graph at inference start.
                            anchor = self.core.anchor_for(stamp, odom_pose, self.p["max_anchor_distance"], self.p["max_anchor_angle"])
                            if anchor is None:
                                raise ValueError("No optimized anchor for this observation after inference")
                            local = inverse(self.core.raw_poses[anchor]) @ odom_pose @ extrinsics
                            obs = [Detection(name, conf, transform(points, local).astype(np.float32))
                                   for name, conf, points in detections]
                            ids = self.core.ingest(Frame(stamp, anchor, obs, transform(bg, local).astype(np.float32)))
                            self.last_processed = stamp
                            self.processed += 1
                            counts = {i: len(o.stamps) for i, o in self.core.objects.items()}
                            confirmed = len(self.core.confirmed())
                        overlay = image.copy()
                        if result.boxes is not None:
                            for box in result.boxes:
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                                text = f"YOLO {model.names[int(box.cls[0])]} {float(box.conf[0]):.2f}"
                                cv2.rectangle(overlay, (x1, y1), (x2, y2), (170, 170, 170), 1)
                                cv2.putText(overlay, text, (max(0, x1), max(15, y1-5)),
                                            cv2.FONT_HERSHEY_SIMPLEX, .45, (200, 200, 200), 1)
                        for mask, detection, number in zip(accepted_masks, detections, ids):
                            if number <= 0:
                                continue
                            c = np.array(colour(number)[::-1], np.uint8)
                            overlay[mask] = (.6*overlay[mask]+.4*c).astype(np.uint8)
                            v, u = np.where(mask)
                            text = detection[0].replace(" ", "_")+f"_{number}"
                            if counts.get(number, 0) < self.p["min_observations"]:
                                text += " pending"
                            if len(u):
                                cv2.putText(overlay, text, (int(np.min(u)), max(20, int(np.min(v))+18)),
                                            cv2.FONT_HERSHEY_SIMPLEX, .5, tuple(int(x) for x in c), 2)
                        out = Image(header=msg.rgb.header, height=overlay.shape[0], width=overlay.shape[1],
                                    encoding="bgr8", is_bigendian=False, step=overlay.shape[1]*3,
                                    data=overlay.tobytes())
                        self.overlay_pub.publish(out)
                        self.publish()
                        self.get_logger().info(f"Frames={self.processed} keyframe={anchor} YOLO={sorted(set(raw_labels))} "
                                               f"usable={[d[0] for d in detections]} rejected_geometry={geometry_rejected} "
                                               f"confirmed={confirmed} inference+map={time.monotonic()-last_inference:.2f}s")
                    except (ValueError, TypeError) as error:
                        self.last_processed = stamp
                        self.warn("frame", f"Observation skipped: {error}")
            except Exception as error:
                if not self.stop.is_set():
                    self.fail(f"Semantic worker stopped: {error}")

    rclpy.init()
    node = None
    try:
        node = SemanticSLAM()
        while rclpy.ok() and not node.stop.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop.set()
            node.worker.join()
            try:
                node.save()
            except Exception as error:
                print(f"Final semantic save failed: {error}")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node is not None and node.fatal:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
