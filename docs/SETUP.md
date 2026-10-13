# Setup on another Ubuntu 24.04 machine

The recordings were made with ROS 2 Jazzy and Gazebo Harmonic. ROS/Gazebo must
already be installed. Clone this repository at the default workspace location
used by the existing mapper configuration:

```bash
mkdir -p ~/ai_envs/project
git clone https://github.com/Harivathsha/AURA-ROS2.git \
  ~/ai_envs/project/c++_project
cd ~/ai_envs/project/c++_project
```

Install the ROS/runtime dependencies:

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rtabmap-ros ros-jazzy-rviz2 ros-jazzy-tf2-ros \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-ament-cmake-test \
  libeigen3-dev python3-numpy python3-matplotlib python3-tk python3-venv

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  description_new aura_navigation navigation aura_fusion aura_semantic_slam
source install/setup.bash
```

For a new machine, create the model environment. On the original laptop,
reuse the existing robotics_ai environment rather than reinstalling it:

```bash
/usr/bin/python3 -m venv --system-site-packages ~/ai_envs/robotics_ai
source ~/ai_envs/robotics_ai/bin/activate
python -m pip install --upgrade pip
python -m pip install "numpy==1.26.4" "opencv-python==4.11.0.86" \
  "matplotlib>=3.6,<3.10"
python -m pip install "torch==2.8.0" "torchvision==0.23.0" \
  --index-url https://download.pytorch.org/whl/cpu
python -m pip install "ultralytics==8.3.228"

mkdir -p ~/ai_envs/project/c++_project/models
cd ~/ai_envs/project/c++_project/models
python - <<'PY'
from ultralytics import YOLO
model = YOLO("yolo11s-seg.pt")
assert model.task == "segment"
print("Segmentation weights ready.")
PY
deactivate
```

Alternatively download yolo11s-seg.pt from the demo-v1 release and place it in
models/. It is omitted from Git commits to keep source clones small.

Run the demonstrations using [the complete guide](RECORDING_AND_UPLOAD.md).
