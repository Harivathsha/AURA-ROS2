# Preparation changes and upload selection

The bundle applies four small build/launch changes:

1. `description_new/CMakeLists.txt`: do not install an empty `src` directory.
2. `navigation/CMakeLists.txt`: the same correction. Git does not retain empty
   directories, so requiring them would break installation after a clone.
3. `launch_sim.launch.py`: default to `aura_maze.sdf`, matching the final planning demo.
4. `semantic_slam.launch.py`: disable the stock RTAB-Map RViz and use
   `show_rviz` for the prepared semantic viewer, so one intended viewer opens.

The robot's mechanics, sensors, maze planner, EKF mathematics and semantic
geometry are unchanged. Before applying the bundle, its instructions preserve
copies of the files being replaced in a local backup directory.

The upload helper copies into a separate destination. It omits:

- Build/install/log outputs, Python caches and editor backups.
- The compiled `aura_wheel_math_todo` executable and unused exercise source files.
- Old library, hospital, bookstore, empty, factory, RL and other unused worlds.
- Old navigation launches, maps and configs not used by the maze demonstration.
- The robot-description CSV export and unused Python package scaffold.

It keeps the five active packages, both selected worlds, the robot meshes and
URDF/Xacro files, registered camera setup, maze map, and the complete six-object
world asset directory including chair textures and attribution.

Legacy description scripts are retained because the current CMake file installs
them. They are not launched by the three recording workflows.

MP4 videos and `.pt` weights are copied into the staging folder but ignored by
Git. They are uploaded as GitHub Release assets. Selected demo result files and
all four PNGs are committed normally. Large RTAB-Map databases and accumulated
working-run directories remain local; they are not required to run the demos.

Do not delete the robot meshes or the six-object asset directory. The helper's
`docs/STAGING_REPORT.md` lists exactly which working source files were omitted.

Two supplied package license fields still say `TODO: License declaration`.
Their metadata has been preserved rather than assigning a new license to
inherited code without its source provenance.
