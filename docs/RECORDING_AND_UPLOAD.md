# Three demonstrations, four pictures and three videos

Use one simulation at a time. The planning demonstration uses the maze;
the fusion and semantic mapping demonstrations both use the six-object room.
All commands below refer to ~/ai_envs/project/c++_project.

## 0. Apply this preparation bundle once

Stop the previous demo cleanly first. Save the semantic map if desired, then
stop the old mapper/RTAB-Map, fusion node, teleoperation and Gazebo with Ctrl+C.

Download AURA_recording_github_prep.zip into ~/Downloads:

```bash
cd ~/ai_envs/project/c++_project
unzip ~/Downloads/AURA_recording_github_prep.zip \
  -d ~/Downloads/AURA_recording_github_prep

python3 - <<'PY'
from pathlib import Path
from datetime import datetime
import shutil
root = Path.cwd()
backup = root/"prep_backups"/datetime.now().strftime("%Y%m%d_%H%M%S")
for name in (
    "src/description_new/CMakeLists.txt",
    "src/navigation/CMakeLists.txt",
    "src/description_new/launch/launch_sim.launch.py",
    "src/aura_semantic_slam/launch/semantic_slam.launch.py",
    "README.md", ".gitignore", "THIRD_PARTY_NOTICES.md"):
    source = root/name
    if source.is_file():
        target = backup/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
print("Preserved replaced files in", backup)
PY

cp -a ~/Downloads/AURA_recording_github_prep/. .
mkdir -p media/images media/videos demo_results/ekf demo_results/semantic

source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-ament-cmake-test python3-tk python3-numpy python3-matplotlib

colcon build --symlink-install --packages-select \
  description_new aura_navigation navigation aura_fusion aura_semantic_slam
source install/setup.bash

colcon test --packages-select aura_fusion aura_semantic_slam \
  --event-handlers console_direct+
colcon test-result --verbose
```

Run the build in a normal ROS terminal outside the AI virtual environment.
The semantic launch invokes ~/ai_envs/robotics_ai/bin/python automatically.

## Media names

Save/export your media to exactly these paths:

| Item | Workspace-relative filename |
|---|---|
| Maze overview PNG | media/images/maze_environment.png |
| Six-object room overview PNG | media/images/six_object_environment.png |
| Saved semantic graph PNG | media/images/scene_graph.png |
| EKF four-panel plot PNG | media/images/ekf_comparison.png |
| Planning video | media/videos/maze_planning.mp4 |
| Fusion video | media/videos/ekf_fusion.mp4 |
| Semantic mapping video | media/videos/semantic_slam.mp4 |

Use your normal screen recorder. Export MP4 with H.264; 1080p/30 fps is a
reasonable starting point. Each MP4 must be below 2 GiB for release upload.
The helper collects the graph and plot PNGs; manually save the two Gazebo
environment overview screenshots.

## 1. Maze: custom planning video

Terminal 1 -- Gazebo:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch description_new launch_sim.launch.py \
  world:="$PWD/src/description_new/worlds/aura_maze.sdf" rviz:=false
```

Terminal 2 -- localisation, custom planner, controller and maze RViz:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch navigation aura_maze_navigation.launch.py
```

Terminal 3 -- readiness check:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 lifecycle get /planner_server
ros2 param get /planner_server GridBased.plugin
```

Expected planner state: active. Expected plugin: aura_navigation::AStarPlanner.
AMCL is configured with the matching initial pose (0, 0, yaw 3.14). Wait until
the map, robot and laser scan are visible and aligned. If the initial pose has
been changed, use RViz's 2D Pose Estimate to restore the actual spawn pose.

Capture a clear Gazebo overview as media/images/maze_environment.png. For the
video, show Gazebo and RViz together so the green A* path and robot motion are
both visible. Start screen recording BEFORE sending a goal.

You can use the Nav2 Goal tool, or send a goal to marker A from Terminal 3:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 9.0, y: 6.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}' \
  --feedback
```

The marker centres A=(9,6), B=(-1.5,6), C=(9,-1.5) are free and connected to
the start in the supplied occupancy map. This offline check is not a guarantee
of live navigation success. A single goal is sufficient for the first recording.

Export the video as media/videos/maze_planning.mp4. Finish the navigation goal,
then stop the navigation launch and Gazebo with Ctrl+C. Do not leave Nav2
commanding /cmd_vel while starting a manual-driving demo.

## 2. Six-object room: EKF comparison video

Start a fresh simulation. No fusion node should be running yet.

Terminal 1 -- room:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch description_new launch_mapping_demo.launch.py
```

Terminal 2 -- comparison recorder FIRST:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 src/aura_fusion/scripts/record_comparison.py \
  --live --refresh-hz 2 --output "$PWD/demo_results/ekf" \
  --ros-args -p use_sim_time:=true
```

Wait for: Recording. Start fusion_node now.

Terminal 3 -- then the EKF:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run aura_fusion fusion_node --ros-args \
  --params-file src/aura_fusion/config/fusion.yaml
```

Wait a few seconds stationary so the recorder sees the initial origin.

Terminal 4 -- manual driving:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.10 -p turn:=0.25 -p stamped:=false
```

Focus this terminal to drive: i=forward, j=left turn, l=right turn, k=stop.
Show the Gazebo robot and live four-panel plot together. Record a slow route
with straight motion and several turns, approximately 1-3 minutes, avoiding
collisions. Return near the start if convenient; a closed loop is not required
for the recorder's error calculations.

Take the room overview screenshot as media/images/six_object_environment.png.
Export the video as media/videos/ekf_fusion.mp4.

At the end: stop the robot with k, then Ctrl+C the RECORDER FIRST, while the EKF
and Gazebo are still running. It saves comparison.png, metrics.json and two CSVs.

Collect the plot image:

```bash
cd ~/ai_envs/project/c++_project
python3 tools/collect_demo_results.py ekf
```

This copies the completed four-panel plot to media/images/ekf_comparison.png.
Then stop teleoperation, the EKF and Gazebo. Starting the recorder after an
already-running EKF can miss its initial origin and invalidate the comparison.

## 3. Six-object room: semantic RGB-D SLAM video

Terminal 1 -- fresh room:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch description_new launch_mapping_demo.launch.py
```

Terminal 2 -- fresh EKF:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run aura_fusion fusion_node --ros-args \
  --params-file src/aura_fusion/config/fusion.yaml
```

Terminal 3 -- RTAB-Map, object mapper and prepared RViz:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p semantic_slam_results
ros2 launch aura_semantic_slam semantic_slam.launch.py \
  odom_topic:=/aura/odom_fused show_rviz:=true rtabmap_gui:=false \
  2>&1 | tee "semantic_slam_results/demo_$(date +%Y%m%d_%H%M%S).log"
```

Terminal 4 -- manual driving:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.08 -p turn:=0.20 -p stamped:=false
```

Wait for Segmentation loaded and increasing processed-frame counts. Record
Gazebo and semantic RViz, including the mask overlay, coloured surfaces and ID
labels. Face each object clearly, pause 5-8 seconds, then revisit a few objects.
Keep the slow driving speed that worked in the previous test. Do not start
the old aura_mapping node or another RTAB-Map launch concurrently.

Export the video as media/videos/semantic_slam.mp4. The expected scene contains
three balls, two plants and one chair, but preserve the actual detected result
even if a ball is missed or an extra chair is predicted. Do not force the count.

Stop the robot with k. From another sourced terminal, save and collect:

```bash
cd ~/ai_envs/project/c++_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /aura_semantic_slam/save std_srvs/srv/Trigger '{}'
python3 tools/collect_demo_results.py semantic
```

The helper copies the most recently saved graph/map/object archive into
demo_results/semantic/ and copies its PNG to media/images/scene_graph.png.
Read the printed class counts; they are the actual results used in the README.
Then stop the integrated SLAM launch cleanly, followed by teleop, EKF and Gazebo.

If RViz is needed manually during an active run:

```bash
ros2 run rviz2 rviz2 -d src/aura_semantic_slam/rviz/semantic_slam.rviz \
  --ros-args -p use_sim_time:=true
```

## 4. Prepare the separate GitHub repository after recording

The commands below create a NEW public repository named AURA-ROS2. Choose a
different name in both --repo and gh repo create if desired. They do not replace
an existing GitHub repository. If AURA-ROS2 already exists, use another new name
or integrate the prepared contents into it deliberately rather than force-pushing.

The original workspace remains usable. The helper copies the cleaned project,
four pictures and selected results into ~/ai_envs/project/AURA-ROS2. It also
copies the three videos and yolo11s-seg.pt there for release upload, with those
files excluded from the Git commit.

```bash
sudo apt install git gh
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git

cd ~/ai_envs/project/c++_project
python3 tools/prepare_github.py \
  --destination ~/ai_envs/project/AURA-ROS2 \
  --owner "$(gh api user --jq .login)" --repo AURA-ROS2
```

The helper stops if media/results are missing or the destination is not empty.
The generated README inserts the new EKF metrics and semantic class counts.

## 5. Commit the source, assets, pictures and selected results

```bash
cd ~/ai_envs/project/AURA-ROS2
git init -b main
git config user.name "Harivathsha"
aura_github_user="$(gh api user --jq .login)"
aura_github_id="$(gh api user --jq .id)"
git config user.email "${aura_github_id}+${aura_github_user}@users.noreply.github.com"

git add README.md .gitignore THIRD_PARTY_NOTICES.md \
  src docs tools media/images results
git status --short
git commit -m "Add AURA planning, EKF fusion and semantic SLAM demos"

gh repo create AURA-ROS2 --public --source=. --remote=origin --push \
  --description "ROS 2 Jazzy custom A* planning, wheel-gyro EKF and RGB-D semantic SLAM"
```

The staged list should contain the four PNGs and selected result files. MP4s,
weights, build/install/log directories and old worlds should not be committed.
If Git is already configured with your preferred name/email, keep those settings
instead of changing them.

## 6. Upload all three videos and the active model weights to a release

```bash
cd ~/ai_envs/project/AURA-ROS2
gh release create demo-v1 \
  media/videos/maze_planning.mp4 \
  media/videos/ekf_fusion.mp4 \
  media/videos/semantic_slam.mp4 \
  models/yolo11s-seg.pt \
  --title "AURA: three simulation demonstrations" \
  --notes-file docs/RELEASE_NOTES.md --target main

gh repo view --web
gh release view demo-v1
```

The README's video links resolve to these release assets. The pictures render
directly in the README. Release MP4 links may download/open the file rather than
provide an embedded player in the README.

GitHub blocks normal Git files above 100 MiB. Release assets must be below
2 GiB each, so this upload method supports recordings larger than normal Git's
file limit without adding video files to repository history.

Official references:
- https://cli.github.com/manual/gh_repo_create
- https://cli.github.com/manual/gh_release_create
- https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github
- https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases
