---
layout: page
title: Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy
permalink: /tutorials/smarttemplate/setup.html
---

# SlicerROS2 Setup - Ubuntu 24.04 (ROS2 Jazzy)

This page provides detailed manual installation instructions for all software components needed for this tutorial. **Note:** If you're using the Docker image as mentioned in the [Prerequisites](prerequisites.html), you can skip these installation steps.

Follow this guide if you prefer to set up the environment manually on your Ubuntu 24.04 system:

## ROS2 Jazzy

- Download and install ROS2 Jazzy - [Instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Other requirements:

```bash
sudo apt install ros-dev-tools
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-tf2-geometry-msgs
```

- Download [SmartTemplate_demo](https://github.com/maribernardes/ros2_smart_template_demo) in your ROS2 workspace src folder

## 3D Slicer with SlicerROS2

To use SlicerROS2, we must use a compiled version of 3D Slicer.

_3D Slicer compilation:_

- Download [3D Slicer source code](https://github.com/Slicer/Slicer.git) (v. 5.9.0)
- Other requirements:

```bash
sudo apt update && sudo apt install git git-lfs build-essential \
libqt5x11extras5-dev qtmultimedia5-dev libqt5svg5-dev qtwebengine5-dev \
libqt5xmlpatterns5-dev qttools5-dev qtbase5-private-dev libxt-dev
```

- Compile 3D Slicer - [Instructions](https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html#build-slicer)
    
Attention: Before you start compiling 3D Slicer, make sure to use the system/native OpenSSL libraries; otherwise, you'll get some errors when compiling the SlicerROS2 module. You will need to do the following after you ran CMake for the first time:

- In the Slicer-build directory, set Slicer_USE_SYSTEM_OpenSLL to ON using:

```bash
cmake ../Slicer -DSlicer_USE_SYSTEM_OpenSSL=ON -DCMAKE_BUILD_TYPE=Release
```

_SlicerROS2 compilation:_

- Download SlicerROS2 source in your ROS2 worspace src folder: <https://github.com/rosmed/slicer_ros2_module>
- Other requirements:

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install ros-jazzy-object-recognition-msgs
sudo apt install ros-jazzy-moveit
```

- Compile SlicerROS2 module - [Instructions](https://slicer-ros2.readthedocs.io/en/latest/pages/gettingstarted.html#compilation)

_Other required Slicer Modules for this tutorial:_

- SlicerDevelopmentToolbox: [https://github.com/QIICR/SlicerDevelopmentToolbox.git](https://github.com/QIICR/SlicerDevelopmentToolbox.git)
- ZFrameRegistration (modified to include different ZFrame model selection): [https://github.com/maribernardes/ZFrameRegistration-3DSlicer.git](https://github.com/maribernardes/ZFrameRegistration-3DSlicer.git)

[⬅️ Back to Prerequisites](prerequisites) | [Jump to Tutorial Steps ➡️](0-preparation) | [Back to Table of Contents ↩️](index)
