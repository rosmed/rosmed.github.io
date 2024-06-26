---
layout: page
title: Exporting URDF from CAD and Visualizing on RViz and 3D Slicer
permalink: /tutorials/export_urdf_from_cad/
---

Overview
========

Unified Robotics Description Format (URDF) is an XML-based file format to describe kinematic and visual configurations of a robot widely used in Robot Operating System (ROS). URDF can be shared among ROS nodes through the ROS node. Some CAD software can export the CAD model into a URDF file using plug-ins. 

The [SlicerROS2](https://slicer-ros2.readthedocs.io/en/latest/index.html) module can read URDF data received through the ROS network and reconstruct the visual robot model in the MRML scene for visualization. This tutorial demonstrate how to export a CAD model from [Autodesk Fusion](https://www.autodesk.com/campaigns/education/fusion-360-education) to URDF, publish it in the ROS environment, and visualize it on RViz and 3D Slicer.

Prerequisite
============
- Exporting CAD (Optional: You can skip this step and download exported URDF)
  - Windows PC or Mac
  - [Autodesk Fusion](https://www.autodesk.com/campaigns/education/fusion-360-education) (Educational license is avaiable for free)
- Loading URDF 
  - Linux PC with 16GB memory
  - ROS2 Humble
  - RViz
  - 3D Slicer built with the SlicerROS2 module ([build instruction](https://slicer-ros2.readthedocs.io/en/latest/pages/getting-started.html#compilation))

Additionally, several ROS tools must be installed using the `apt-get` command.

~~~~
$ sudo apt-get install ros-humble-robot-state-publisher
$ sudo apt-get install ros-humble-joint-state-publisher-gui
~~~~

If you don't have access to a computer with Autodesk Fusion, you can skip the first part of the tutorial, and pull the URDF data via Git instead:

~~~~
$ cd <working directory>
$ git clone https://github.com/tokjun/smart_template_description 
~~~~


Exporting a CAD model from Autodesk Fusion (Optional) 
=====================================================


(Under Construction)


Visualizing the URDF on RViz and 3D SlicerROS2
==============================================

We assume that the ROS workspace is under `~/ros2_ws`, and the URDF is placed under `<working directory>/smart_template_description`.

First, open a terminal and publish the URDF on the ROS using the following command:

~~~~
$ cd ~/ros2_ws
$ source /opt/ros/humble/setup.bash
$ source install/setup.bash
$ ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro <working directory>/smart_template_description/urdf/smart_template.xacro )" 
~~~~

Note that the last line calls `xacro` command within the `robot_description` parameter, as `smart_template.xacro` needs to be converted to the URDF format.
























