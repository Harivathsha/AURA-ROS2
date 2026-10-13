# Software and asset notices

The repository integrates ROS 2, Gazebo, Nav2, Eigen, RTAB-Map and Ultralytics
YOLO. These components retain their own licenses.

The downloaded segmentation weights are pretrained Ultralytics weights and
are not trained by this project. The `.pt` file is published as a demo release
asset and is excluded from normal Git commits.

## Chair asset

Chair by Cole Biesemeyer, OpenRobotics.

Source: https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair (version 3).

License: Creative Commons Attribution 4.0 International:
https://creativecommons.org/licenses/by/4.0/

Changes: local resource URI and static placement in the AURA test world.

The asset's `ATTRIBUTION.txt` is retained at
`src/description_new/worlds/aura_mapping_demo_assets/aura_demo_chair/ATTRIBUTION.txt`.

## Package declarations

The supplied package metadata declares Apache-2.0 for `aura_navigation` and
`aura_fusion`, and MIT for `aura_semantic_slam`. `description_new` and `navigation`
still have placeholder license fields in the supplied archive. No blanket
relicensing of inherited files is made by this preparation bundle.
