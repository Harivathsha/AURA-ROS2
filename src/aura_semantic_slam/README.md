# AURA semantic RGB-D SLAM

Complete static-object integration for the existing AURA six-object room:
YOLO11 instance segmentation, registered depth-image projection, external
odometry, RTAB-Map optimized keyframe anchoring, object colours and IDs, visible
surface boxes, and a geometric scene graph. A separate package is added; the
existing world, fusion node, mapping package, and test launches are preserved.

The model and room are the ones already working on the laptop. No training or
new Gazebo assets are required. Recognition remains dependent on camera views.

## Install

Stop the robot. Stop the previous RTAB-Map launch and old semantic mapper with
Ctrl+C, allowing their databases/results to save. Close their old RViz windows.
Gazebo, one EKF instance, and teleoperation can remain running.

Download AURA_semantic_slam_complete.zip into ~/Downloads, then:

```bash
cd ~/ai_envs/project/c++_project
unzip ~/Downloads/AURA_semantic_slam_complete.zip \
  -d ~/Downloads/AURA_semantic_slam_complete
cp -a ~/Downloads/AURA_semantic_slam_complete/src/aura_semantic_slam src/
chmod +x src/aura_semantic_slam/scripts/semantic_slam_mapper.py \
         src/aura_semantic_slam/scripts/rgbd_adapter.py

source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-ament-cmake-test python3-numpy python3-matplotlib
colcon build --symlink-install --packages-select aura_semantic_slam
source install/setup.bash

colcon test --packages-select aura_semantic_slam \
  --event-handlers console_direct+
colcon test-result --verbose
```

Use a normal system ROS terminal for the build. If an AI virtual environment is
active, run `deactivate` first so CMake chooses the system Python interpreter.

The existing ROS Jazzy RTAB-Map installation is required; it was installed for
the successful earlier test. The core tests need NumPy only, and use unittest.

## Check the existing model environment

The integrated launch starts the mapper explicitly with
~/ai_envs/robotics_ai/bin/python; the adapter uses /usr/bin/python3. You do not
need to activate the virtual environment in the launch terminal. Both processes
inherit the sourced ROS environment.

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
~/ai_envs/robotics_ai/bin/python - <<'PY'
import sys
import rclpy, rtabmap_msgs.msg, numpy, cv2, torch, matplotlib
from ultralytics import YOLO
from pathlib import Path
p = Path.home()/"ai_envs/project/c++_project/models/yolo11s-seg.pt"
assert p.is_file(), f"Missing model: {p}"
assert YOLO(str(p)).task == "segment"
print("Interpreter:", sys.executable)
print("ROS, RGB-D messages and segmentation environment ready.")
PY
```

These are the dependencies already used by the previous semantic mapper.
The package does not install or upgrade your AI environment. If an import fails,
retain its exact error for diagnosis rather than upgrading unrelated packages.

## Launch with the existing simulation

Keep the six-object Gazebo room running. Keep one EKF instance running if using
the default /aura/odom_fused input. Launch everything else in one terminal:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p semantic_slam_results

ros2 launch aura_semantic_slam semantic_slam.launch.py \
  odom_topic:=/aura/odom_fused \
  2>&1 | tee "semantic_slam_results/console_$(date +%Y%m%d_%H%M%S).log"
```

This launches the adapter, RTAB-Map, RGB-D synchronizer, semantic mapper and
prepared RViz. The extra RTAB-Map GUI is disabled by default to reduce CPU load.
Enable it with `rtabmap_gui:=true` if you want both visualizations.

The normal RTAB-Map map retains RGB surface colours; the prepared RViz shows
object identity colours with grey background. It uses fixed frame aura_slam_map.

To use the simulation reference instead of the EKF, stop this integrated launch
and start a new run with:

```bash
ros2 launch aura_semantic_slam semantic_slam.launch.py odom_topic:=/odom
```

Do not run an old RTAB-Map test launch concurrently: both use /aura_slam topics
and the same alias frames. Gazebo's original camera topics and physical TF stay
available. The old aura_mapping mapper must also stay stopped to avoid excess
CPU load and confusion between its independent map and this corrected map.

## If starting from a stopped simulation

Terminal 1:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch description_new launch_mapping_demo.launch.py
```

Terminal 2:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run aura_fusion fusion_node --ros-args \
  --params-file src/aura_fusion/config/fusion.yaml
```

Terminal 3: run the integrated semantic SLAM launch above.
Use your already-working teleoperation controls in their usual terminal.

## What to expect

Wait for these messages:

```text
Odometry input active: /aura/odom_fused; ...
Optical TF ready: ...
Segmentation loaded. Semantic SLAM ready; drive slowly.
Semantic optical mounting transform ready
Frames=... keyframe=... YOLO=[...] usable=[...] ... confirmed=...
```

Initial messages about missing optical TF or keyframes are expected until
the adapter and RTAB-Map begin processing. They should stop once inputs flow.
Every five wall seconds the mapper reports counts for rgbd, odom, mapData,
mapGraph, anchors, confirmed objects, and rebuilds.

In RViz the image panel shows grey raw YOLO boxes/labels and accepted masks
coloured by object ID. A pending label means fewer than three distinct usable
image observations. After confirmation, coloured surfaces, an ID label and
visible-surface box appear in the 3D map. Cyan lines show near relationships.

Drive as slowly as in the successful RTAB-Map test: approximately 0.05-0.10 m/s
and gentle turns. Pause for 5-8 seconds with each object fully visible. Try balls
at 1-2 m and plants/chair at 1.5-2.5 m. The semantic inference cap is 0.75 frames
per wall second; actual throughput depends on the CPU. RTAB-Map processes at
a configured 1 Hz cap in simulation time. Latest usable pairs are selected from
a short buffer, so YOLO does not build a long processing backlog.

Test the three balls, two plants and one chair, then revisit them. Check that
IDs and colours remain stable. The code does not use Gazebo object names or
force the count to six. Recognition errors and nearby similar objects can
still cause splits or incorrect matches; the matching parameters are starting
values for this room, not a universal tracking solution.

When an existing optimized pose changes, the mapper prints:

```text
Optimized poses changed: rebuilt semantic surfaces, IDs and graph
```

This may occur after loop closure or another graph optimization; it is not
itself proof that a loop closure was accepted. Positive /aura_slam/info
loop_closure_id updates are reported separately.

## Save results

From another sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ai_envs/project/c++_project/install/setup.bash
ros2 service call /aura_semantic_slam/save std_srvs/srv/Trigger '{}'
```

The returned folder is under semantic_slam_results/aura_<timestamp>/:

| File | Contents |
|---|---|
| semantic_map.ply | Current corrected object/background surface points and object IDs |
| scene_graph.json | Confirmed objects, measured bounds, relationships, settings and statistics |
| scene_graph.png | Top-down object graph; lines indicate near |
| anchor_observations.npz | Retained object-local observations, IDs, stamps and optimized anchor poses; no pickled objects |

The mapper autosaves every 45 wall seconds and on Ctrl+C. RTAB-Map stores a
separate fresh database in rtabmap_results/semantic_<timestamp>.db. New launches
start new maps; loading and continuing a prior semantic run is not implemented.

The NPZ records object observations for auditing/reprojection, not the complete
background or a restart checkpoint. During the run both object and background
observations retain their anchors and update after corrections.

## How correction works

For each synchronized RGB-D image:

1. YOLO gives classes and instance masks.
2. Depth is projected with CameraInfo using optical X-right, Y-down, Z-forward.
3. The optical mounting transform and odometry interpolated at acquisition
   time place points relative to a nearby RTAB-Map keyframe's uncorrected pose.
4. That keyframe's optimized pose places observations into aura_slam_map.
5. Same-class observations are matched using distance, surface support and
   bounds; one frame cannot assign two detections to the same object.

The stored transform is:

```text
points_anchor = inverse(T_odom_anchor) * T_odom_robot_at_capture * T_robot_optical * points_optical
points_map    = T_map_anchor_optimized * points_anchor
```

If optimized anchors move, points are reprojected and observations are
reassociated in capture order. Existing IDs are preferred when consistent;
duplicates can merge into the earliest surviving ID. Related graph edges and
labels are regenerated. RTAB-Map signature rehearsal is handled by rebasing
existing observations if a retained node's uncorrected pose changes.

The fixed-map left_of and in_front_of rules follow +Y and +X respectively;
they are not camera-relative semantic judgments. Object bounds cover observed
surfaces, not invisible backs or exact CAD dimensions.

## Memory and operating scope

The mapper is designed for static objects. Moving-object tracking is outside
this package. It retains up to 600 processed frames (about 13 minutes at the
maximum 0.75 Hz), with per-observation and output point caps. When the history
limit is reached, the oldest frame is removed and the remaining map rebuilt.
An object whose every observation has expired can disappear. Increase
max_stored_frames for a longer run if laptop memory permits.

Images must be registered RGB-D from the current single Gazebo camera.
The adapter assumes the unchanged physical camera frame is camera and fixes
its optical interpretation. Do not reset Gazebo, restart the EKF, or change
its odometry source during a run; start a fresh integrated launch afterward.

## Troubleshooting

| Symptom | Check |
|---|---|
| Python environment or weights error, integrated launch exits | Run the explicit environment check; inspect the first semantic-worker error |
| rgbd=0 | ros2 topic hz /aura_slam/rgbd_image; adapter RGB/depth/info counts |
| mapData=0 or anchors=0 | RTAB-Map console and ros2 topic info /aura_slam/mapData -v |
| optimized poses exist but no raw anchors | Inspect /aura_slam/mapData; nodes must include id, stamp and raw pose |
| Waiting for timestamp-aligned odometry | Odometry frequency, stamps, /clock, and pause/reset state |
| Everything grey | Read raw YOLO labels in the overlay and usable/rejected_geometry in the log |
| Correct label but rejected geometry | Distance, visible depth pixels, depth/RGB alignment and erosion |
| Duplicate IDs | Save graph and console; inspect corrected bounds and association rather than forcing an object count |
| Slow processing | Keep RTAB-Map GUI disabled; close old mapper/RViz, lower image_size or process_hz |

To see a topic/service summary:

```bash
ros2 topic list | grep -E 'aura_slam|aura_semantic_slam'
ros2 topic echo /aura_semantic_slam/scene_graph --once
ros2 topic echo /aura_slam/info --field loop_closure_id
```

## Validation and source references

23 offline tests cover timestamp interpolation, optical projection, padded
depth data, pose reset/gap handling, confirmation, identity, co-visible balls,
corrected historical surfaces, duplicate consolidation, signature rebasing,
scene relationships, six-object bookkeeping, point/frame caps and safe exports.
Snapshot generation and Python syntax were also checked. A 63-frame synthetic
six-object correction replay was exercised. These are geometry/bookkeeping
checks, not claims of Gazebo recognition or live ROS integration.

ROS/Gazebo/RTAB-Map live integration must be verified on the laptop.
The mapper reuses the existing pretrained yolo11s-seg.pt without fine-tuning.

Primary message definitions used:
- https://docs.ros.org/en/jazzy/p/rtabmap_msgs/msg/MapGraph.html
- https://docs.ros.org/en/ros2_packages/jazzy/api/rtabmap_msgs/msg/MapData.html
- https://docs.ros.org/en/ros2_packages/jazzy/api/rtabmap_msgs/msg/Node.html
- https://docs.ros.org/en/ros2_packages/jazzy/api/rtabmap_msgs/msg/RGBDImage.html

Package code is MIT licensed. The downloaded Ultralytics model and software
retain their own licenses; they are not distributed in this archive.
