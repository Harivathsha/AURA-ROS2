"""ROS-independent, keyframe-anchored static object mapping (NumPy only)."""
from collections import OrderedDict, deque
from dataclasses import dataclass, field
import colorsys
import itertools
import math
import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-10:
        raise ValueError("Invalid quaternion")
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix(position, quaternion):
    t = np.eye(4)
    t[:3, :3] = rotation(quaternion)
    t[:3, 3] = position
    if not np.all(np.isfinite(t)):
        raise ValueError("Non-finite pose")
    return t


def inverse(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ t[:3, 3]
    return result


def transform(points, t):
    return np.asarray(points) @ t[:3, :3].T + t[:3, 3]


def slerp(a, b, fraction):
    a, b = np.asarray(a, float), np.asarray(b, float)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    dot = float(np.dot(a, b))
    if dot < 0:
        b, dot = -b, -dot
    dot = min(1., dot)
    if dot > .9995:
        q = a + fraction * (b-a)
        return q / np.linalg.norm(q)
    angle = math.acos(dot)
    return (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b) / math.sin(angle)


class OdometryBuffer:
    def __init__(self, size=2500):
        self.rows = deque(maxlen=size)
        self.frame = None

    def add(self, stamp, frame, position, quaternion):
        matrix(position, quaternion)
        if not frame:
            raise ValueError("Odometry has no parent frame")
        if self.frame is not None and (frame != self.frame or
                (self.rows and stamp < self.rows[-1][0])):
            raise ValueError("Odometry reset: restart SLAM and the semantic mapper together")
        if self.rows and stamp == self.rows[-1][0]:
            self.rows.pop()
        self.frame = frame
        self.rows.append((stamp, np.asarray(position, float), np.asarray(quaternion, float)))

    def at(self, stamp, max_gap_ns):
        if not self.rows or stamp < self.rows[0][0] or stamp > self.rows[-1][0]:
            return None  # Never extrapolate an old pose to a new image.
        times = [row[0] for row in self.rows]
        i = int(np.searchsorted(times, stamp))
        if times[i] == stamp:
            return matrix(*self.rows[i][1:])
        a, b = self.rows[i-1], self.rows[i]
        if b[0]-a[0] > max_gap_ns:
            return None
        f = (stamp-a[0])/(b[0]-a[0])
        return matrix(a[1]+f*(b[1]-a[1]), slerp(a[2], b[2], f))


def depth_array(msg):
    dtype = {"32FC1": "f4", "16UC1": "u2"}.get(msg.encoding)
    if dtype is None:
        raise ValueError("Depth must be raw 32FC1 metres or 16UC1 millimetres")
    dtype = np.dtype((">" if msg.is_bigendian else "<")+dtype)
    if msg.step < msg.width*dtype.itemsize or len(msg.data) < msg.height*msg.step:
        raise ValueError("Truncated depth image")
    a = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                   strides=(msg.step, dtype.itemsize)).astype(np.float32)
    return a*.001 if msg.encoding == "16UC1" else a


def image_bgr(msg):
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    n = channels.get(msg.encoding)
    if n is None or msg.step < msg.width*n or len(msg.data) < msg.height*msg.step:
        raise ValueError("Unsupported or truncated RGB image")
    image = np.ndarray((msg.height, msg.width, n), np.uint8, buffer=msg.data,
                       strides=(msg.step, n, 1))
    if n == 1:
        image = np.repeat(image, 3, axis=2)
    else:
        image = image[:, :, :3]
    if msg.encoding in ("rgb8", "rgba8"):
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


def project(depth, k):
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
        raise ValueError("Invalid camera intrinsics")
    v, u = np.indices(depth.shape, dtype=np.float32)
    return np.stack(((u-cx)*depth/fx, (v-cy)*depth/fy, depth), axis=-1)


def foreground(xyz, mask, minimum, maximum, stride, band):
    points = xyz[::stride, ::stride][mask[::stride, ::stride]]
    points = points[np.all(np.isfinite(points), axis=1)]
    points = points[(points[:, 2] >= minimum) & (points[:, 2] <= maximum)]
    if not len(points):
        return points
    median = np.median(points[:, 2])
    mad = np.median(np.abs(points[:, 2]-median))
    width = min(band, max(.12, 3*1.4826*mad))
    return points[np.abs(points[:, 2]-median) <= width]


def downsample(points, resolution, limit):
    points = np.asarray(points, np.float32).reshape((-1, 3))
    points = points[np.all(np.isfinite(points), axis=1)]
    if not len(points):
        return points
    _, indexes = np.unique(np.floor(points/resolution).astype(np.int64), axis=0, return_index=True)
    points = points[indexes]
    if len(points) > limit:
        points = points[np.linspace(0, len(points)-1, limit).astype(int)]
    return points


def colour(number):
    return tuple(round(v*255) for v in colorsys.hsv_to_rgb(
        (.07+(number-1)*.61803398875) % 1, .78, .95))


def bounds(points):
    return np.quantile(points, .02, axis=0), np.quantile(points, .98, axis=0)


def match_score(points, obj, gate, support_radius):
    centre = np.median(points, axis=0)
    distance = float(np.linalg.norm(centre-obj.centre))
    if distance > gate:
        return None
    low, high = bounds(points)
    other_low, other_high = obj.bounds()
    gap = np.maximum(0., np.maximum(low-other_high, other_low-high))
    gap_distance = float(np.linalg.norm(gap))
    if gap_distance > min(.35, gate*.55):
        return None
    other = obj.points()
    a = points[np.linspace(0, len(points)-1, min(90, len(points))).astype(int)]
    b = other[np.linspace(0, len(other)-1, min(140, len(other))).astype(int)]
    nearest = np.sqrt(np.min(np.sum((a[:, None]-b[None, :])**2, axis=2), axis=1))
    support = float(np.mean(nearest < support_radius))
    intersection = np.maximum(0., np.minimum(high, other_high)-np.maximum(low, other_low))
    v1 = np.prod(np.maximum(high-low, .03))
    v2 = np.prod(np.maximum(other_high-other_low, .03))
    overlap = float(np.prod(intersection)/(min(v1, v2)+1e-9))
    # Position, visible bounds, and surface support all contribute.
    return .35*distance + .35*float(np.median(nearest)) + .3*gap_distance - .08*support - .05*overlap


@dataclass
class Detection:
    class_name: str
    confidence: float
    points: np.ndarray  # Anchor-base coordinates, not immutable map points.
    track_id: int = 0


@dataclass
class Frame:
    stamp: int
    anchor: int
    detections: list
    background: np.ndarray


@dataclass
class Object:
    number: int
    class_name: str
    voxels: dict = field(default_factory=dict)
    stamps: set = field(default_factory=set)
    confidence_sum: float = 0.
    centre: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def points(self):
        return np.array(list(self.voxels.values()), np.float32).reshape((-1, 3))

    def bounds(self):
        return bounds(self.points())

    @property
    def label(self):
        return self.class_name.replace(" ", "_")+"_"+str(self.number)


class AnchoredMap:
    def __init__(self, resolution=.04, gate=.75, support_radius=.18,
                 min_observations=3, max_objects=60, max_frames=600,
                 object_point_limit=5000, background_point_limit=60000):
        self.resolution, self.gate, self.support_radius = resolution, gate, support_radius
        self.min_observations, self.max_objects, self.max_frames = min_observations, max_objects, max_frames
        self.object_limit, self.background_limit = object_point_limit, background_point_limit
        self.raw_poses, self.optimized_poses, self.node_stamps, self.node_map_ids = {}, {}, {}, {}
        self.frames, self.objects, self.background = OrderedDict(), {}, {}
        self.next_id, self.rebuild_count, self.graph_updates = 1, 0, 0
        self.dirty = False
        self.last_stamp = 0

    def update_graph(self, optimized, nodes):
        """nodes: (id, acquisition_ns, uncorrected_pose, RTAB map_id)."""
        self.graph_updates += 1
        for node_id, stamp, raw, map_id in nodes:
            if node_id <= 0:
                continue
            if node_id in self.node_map_ids and self.node_map_ids[node_id] != map_id:
                raise ValueError("RTAB-Map node ids were reused: restart the integrated launch")
            old = self.raw_poses.get(node_id)
            if old is not None and not np.allclose(old, raw, atol=1e-8, rtol=0):
                # Rehearsal can move a retained signature's odometry pose.
                # Re-express old observations in its new local coordinates.
                delta = inverse(raw) @ old
                for frame in self.frames.values():
                    if frame.anchor == node_id:
                        for detection in frame.detections:
                            detection.points = transform(detection.points, delta).astype(np.float32)
                        frame.background = transform(frame.background, delta).astype(np.float32)
                self.dirty = True
            self.raw_poses[node_id] = raw
            self.node_stamps[node_id] = stamp
            self.node_map_ids[node_id] = map_id
        for node_id, pose in optimized.items():
            if node_id <= 0:
                continue
            old = self.optimized_poses.get(node_id)
            if old is not None and not np.allclose(old, pose, atol=1e-8, rtol=0):
                self.dirty = True
            self.optimized_poses[node_id] = pose

    def anchor_for(self, stamp, odom_pose, max_distance=1.5, max_angle=1.25):
        best = None
        for node_id, raw in self.raw_poses.items():
            if node_id not in self.optimized_poses:
                continue
            relative = inverse(raw) @ odom_pose
            distance = float(np.linalg.norm(relative[:3, 3]))
            angle = math.acos(float(np.clip((np.trace(relative[:3, :3])-1)*.5, -1, 1)))
            if distance <= max_distance and angle <= max_angle:
                score = abs(stamp-self.node_stamps[node_id])/1e9 + distance*.1
                if best is None or score < best[0]:
                    best = score, node_id
        return None if best is None else best[1]

    def insert_voxels(self, store, points, limit):
        for p in downsample(points, self.resolution, limit):
            key = tuple(np.floor(p/self.resolution).astype(int))
            previous = store.pop(key, None)
            store[key] = p if previous is None else (previous+p)*.5
        while len(store) > limit:
            store.pop(next(iter(store)))

    def apply_frame(self, frame):
        pose = self.optimized_poses.get(frame.anchor)
        if pose is None:
            return []
        world = [transform(d.points, pose).astype(np.float32) for d in frame.detections]
        candidates = []
        for index, (detection, points) in enumerate(zip(frame.detections, world)):
            for number, obj in self.objects.items():
                if obj.class_name == detection.class_name:
                    score = match_score(points, obj, self.gate, self.support_radius)
                    if score is not None:
                        candidates.append((score-(.015 if number == detection.track_id else 0), index, number))
        assignment, used = {}, set()
        for _, index, number in sorted(candidates):
            if index not in assignment and number not in used:
                assignment[index] = number
                used.add(number)
        for index, (detection, points) in enumerate(zip(frame.detections, world)):
            if index not in assignment:
                if len(self.objects) >= self.max_objects:
                    continue
                number = detection.track_id
                if number <= 0 or number in self.objects:
                    number = self.next_id
                    self.next_id += 1
                self.next_id = max(self.next_id, number+1)
                self.objects[number] = Object(number, detection.class_name)
                assignment[index] = number
            number = assignment[index]
            detection.track_id = number
            obj = self.objects[number]
            self.insert_voxels(obj.voxels, points, self.object_limit)
            if frame.stamp not in obj.stamps:
                obj.stamps.add(frame.stamp)
                obj.confidence_sum += detection.confidence
            obj.centre = np.median(obj.points(), axis=0)
        self.insert_voxels(self.background, transform(frame.background, pose), self.background_limit)
        return [assignment.get(i, 0) for i in range(len(frame.detections))]

    def rebuild(self):
        self.objects, self.background = {}, {}
        for frame in self.frames.values():
            self.apply_frame(frame)
        self.dirty = False
        self.rebuild_count += 1

    def ingest(self, frame):
        if frame.stamp in self.frames:
            return [d.track_id for d in self.frames[frame.stamp].detections]
        if frame.anchor not in self.optimized_poses:
            raise ValueError("Observation has no optimized anchor pose")
        if self.dirty:
            self.rebuild()
        self.frames[frame.stamp] = frame
        self.last_stamp = max(self.last_stamp, frame.stamp)
        assignment = self.apply_frame(frame)
        if len(self.frames) > self.max_frames:
            self.frames.popitem(last=False)
            self.rebuild()
        return assignment

    def confirmed(self):
        return [obj for obj in self.objects.values() if len(obj.stamps) >= self.min_observations]

    def cloud(self):
        arrays, colours, ids = [], [], []
        semantic_keys = set().union(*(set(obj.voxels) for obj in self.confirmed()))
        background = [p for key, p in self.background.items() if key not in semantic_keys]
        if background:
            arrays.append(np.array(background, np.float32))
            colours.append(np.full((len(background), 3), 100, np.uint8))
            ids.append(np.zeros(len(background), np.uint32))
        for obj in self.confirmed():
            points = obj.points()
            arrays.append(points)
            colours.append(np.tile(colour(obj.number), (len(points), 1)).astype(np.uint8))
            ids.append(np.full(len(points), obj.number, np.uint32))
        if not arrays:
            return np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8), np.empty(0, np.uint32)
        return np.concatenate(arrays), np.concatenate(colours), np.concatenate(ids)

    def graph(self, frame_id, near=2., relation_distance=3., margin=.3):
        nodes = []
        for obj in self.confirmed():
            low, high = obj.bounds()
            nodes.append({"id": obj.number, "label": obj.label, "class": obj.class_name,
                          "colour_rgb": colour(obj.number), "position_m": obj.centre.tolist(),
                          "confidence": obj.confidence_sum/len(obj.stamps),
                          "observations": len(obj.stamps), "first_stamp_ns": min(obj.stamps),
                          "last_stamp_ns": max(obj.stamps),
                          "visible_bounds_m": {"min": low.tolist(), "max": high.tolist()}})
        edges = []
        for a, b in itertools.combinations(nodes, 2):
            delta = np.array(a["position_m"])-b["position_m"]
            distance = float(np.linalg.norm(delta[:2]))
            if distance <= near:
                edges.append({"source": a["id"], "target": b["id"], "relation": "near",
                              "directed": False, "distance_xy_m": distance})
            if distance <= relation_distance:
                for axis, label in ((0, "in_front_of"), (1, "left_of")):
                    if abs(delta[axis]) >= margin:
                        source, target = (a, b) if delta[axis] > 0 else (b, a)
                        edges.append({"source": source["id"], "target": target["id"],
                                      "relation": label, "directed": True, "distance_xy_m": distance})
        return {"frame_id": frame_id, "stamp_ns": self.last_stamp,
                "pose_source": "RTAB-Map optimized keyframe poses",
                "objects": nodes, "edges": edges,
                "statistics": {"retained_frames": len(self.frames), "candidate_objects": len(self.objects),
                               "optimized_keyframes": len(self.optimized_poses),
                               "graph_updates": self.graph_updates, "rebuilds": self.rebuild_count},
                "definitions": {"position": "median of observed surfaces; not a CAD centre",
                                "bounds": "2nd/98th percentiles of observed surfaces",
                                "near": f"XY centre distance <= {near} m",
                                "left_of": f"greater fixed-map Y by >= {margin} m",
                                "in_front_of": f"greater fixed-map X by >= {margin} m",
                                "directional_range": relation_distance,
                                "axes": "fixed RTAB-Map frame, not the current camera viewpoint"}}
