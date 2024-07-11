---
layout: page
title: Controlling the robot from the terminal
---
Back to [Tutorial Home](index)


Setting up the teleop_twist_keyboard
------------------------------------

You can move the robot arm from a ternimal using `teleop_twist_keyboard`. To install it,

~~~~
$ sudo apt-get update
$ sudo apt-get install ros-humble-teleop-twist-keyboard
~~~~


Controlling the robot
---------------------

Open a new terminal and source:

~~~~
$ cd /home/<user name>/ismr24_ws
$ source /opt/ros/humble/setup.bash
$ source install/setup.bash
~~~~

Then run`teleop_twist_keyboard`: 

~~~~
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/spacenav/twist
~~~~

Now you can move the end-effector by pressing the keys (see the screen for instruction).



