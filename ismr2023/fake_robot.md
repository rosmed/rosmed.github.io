---
layout: page
title: Bring Up a Fake UR10
---

Bringup a Fake UR10
===================
To bring up a fake UR10 try the following commands

~~~
export COLCON_WS=~/workspace/ros_ur_driver  # If COLCON_WS hasn't been set.
cd $COLCON_WS
source install/setup.bash
ros2 launch ros2_igtl_bridge spine.launch.py
~~~
