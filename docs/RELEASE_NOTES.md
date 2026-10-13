Three AURA simulation demonstrations:

1. Custom C++ A* planning through the maze with Nav2 path following.
2. Wheel-only and wheel/gyro EKF odometry compared with simulator reference.
3. RGB-D semantic mapping with YOLO masks, RTAB-Map keyframe anchoring and a scene graph.

The maze is used for planning; the same six-object room is used for fusion and
semantic mapping. The source repository contains four demonstration pictures
and selected numerical/geometry results. This release contains the three MP4
recordings and the pretrained yolo11s-seg.pt weights used by the mapper.

Object recognition and association remain imperfect; see the recorded class
counts and known limitations in README.md.
