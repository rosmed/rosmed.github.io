---
layout: page
title: SlicerROS2 Setup for ISMR2025
permalink: /ismr2025/setup.html
---

# SlicerROS2 Setup - Ubuntu 24.04 (ROS2 Jazzy)

This page provides detailed manual installation instructions for all software components needed for this tutorial. **Note:** If you're using the Docker image as mentioned in the [Prerequisites](prerequisites.html), you can skip these installation steps.

Follow this guide if you prefer to set up the environment manually on your Ubuntu 24.04 system:

## ROS2 Jazzy

- Download and install ROS2 Jazzy - [Instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Other requirements:

```bash
sudo apt-get install ros-dev-tools
sudo apt-get install ros-jazzy-xacro
sudo apt-get install ros-jazzy-tf2-geometry-msgs
sudo apt-get install ros-jazzy-object-recognition-msgs
sudo apt-get install ros-jazzy-moveit
sudo apt-get install python3-colcon-common-extensions

```

## 3D Slicer with SlicerROS2

To use SlicerROS2, we must use a compiled version of 3D Slicer.

### 3D Slicer

To build 3D Slicer, folloing tools and libraries must be installed in the system:

```bash
sudo apt update && sudo apt install git git-lfs build-essential \
libqt5x11extras5-dev qtmultimedia5-dev libqt5svg5-dev qtwebengine5-dev \
libqt5xmlpatterns5-dev qttools5-dev qtbase5-private-dev libxt-dev
```

To build 3D Slicer for SlicerROS2, follow the instruction in [SlicerROS2's Read the Doc page](https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html#build-slicer)
    
Attention: Before you start compiling 3D Slicer, make sure to use the system/native OpenSSL libraries; otherwise, you'll get some errors when compiling the SlicerROS2 module. 


### SlicerROS2

SlicerROS is available at [https://github.com/rosmed/slicer_ros2_module](https://github.com/rosmed/slicer_ros2_module).
To build SlicerROS2, follow the instruction in [SlicerROS2's Read the Doc page](https://slicer-ros2.readthedocs.io/en/latest/pages/gettingstarted.html#compilation)


### Files for Smart Template Demo

The following 3D Slicer extensions are required for the Smart Template Demo

- [SlicerDevelopmentToolbox](https://github.com/QIICR/SlicerDevelopmentToolbox.git)
- [ZFrameRegistration](https://github.com/maribernardes/ZFrameRegistration-3DSlicer.git) (modified to include different ZFrame model selection)

Additionally, [the SmartTemplate demo package](https://github.com/maribernardes/ros2_smart_template_demo) is required.

```bash
mkdir -p ~/ros_ws
cd ros_ws
mkdir src
cd src
git clone https://github.com/maribernardes/ros2_smart_template_demo
cd ~/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build
```


### Files for AMBF Demo

Before installing additional extensions, install the following libraries in the system:

```bash
sudo apt-get install libxcursor-dev
sudo apt-get install libxinerama-dev
sudo apt-get install libasound2-dev libgl1-mesa-dev xorg-dev
sudo apt-get install libusb-1.0.0-dev
```

Setup AMBF in the ROS workspace:

```bash
mkdir ambf_ws; cd ambf_ws; mkdir src; cd src;
git clone https://github.com/LauraConnolly/ambf
cd ambf
git checkout ros2-support
cd ~/ambf_ws
source /opt/ros/jazzy/setup.bash
colcon build
```
You might have to run this build a few times so the packages build in order!

The following 3D Slicer extensions are also required.
- [AMBF Util Slicer Plugin](https://github.com/LCSR-CIIS/ambf_util_slicer_plugin)
- [TimeSeriesAnnotation](https://github.com/SlicerUltrasound/SlicerUltrasound/tree/main/TimeSeriesAnnotation)


[Back to Prerequisites](prerequisites.html) | [Back to Workshop page](index.html)

