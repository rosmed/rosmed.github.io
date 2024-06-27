---
layout: page
title: Setting Up Additional Software/Files for the Tutorial 
---
Back to [Tutorial Home](index)

> :warning: If you are using the Docker image provided for this tutorial, you can skip this page. 

Setting up additional ROS2 packages (dVRK)
------------------------------------------

Run the following commands to make sure the all packages are installed.

~~~~
$ sudo apt update
$ sudo apt install python3-vcstool python3-colcon-common-extensions python3-pykdl
$ sudo apt install -y libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui  git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev
$ sudo apt install -y ros-humble-joint-state-publisher* ros-humble-xacro ros-humble-ament-* ros-humble-ign-ros2-control ros-humble-ros2-control* ros-humble-joint-state-broadcaster
~~~~


Plus Toolkit
------------

We will use [the Plus Toolkit library](https://plustoolkit.github.io) to generate synthetic ultrasound image. The library will be called from the ultrasound simulation plug-in for Gazebo.

First, install dependencies:
~~~~
$ sudo apt-get install -y libglvnd-dev libqt5x11extras5-dev qtdeclarative5-dev qml-module-qtquick*
~~~~

Then run the following commands:

~~~~
$ cd <working directory>/plus
$ cd <working directory>/plus
$ git clone https://github.com/PlusToolkit/PlusBuild.git
$ mkdir PlusBuild-bin 
$ cd PlusBuild-bin
$ cmake ../PlusBuild -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<working directory>/plus/install -DPLUSBUILD_INSTALL_VTK=ON -DPLUSBUILD_INSTALL_ITK=ON 
$ make -j4
~~~~


Install 2024 ROS2 files
---------------------

The tutorial-specific source codes are available in the GitHub repository. 

First, create a workspace: 

~~~~
$ mkdir -p /root/ismr24_ws/src
$ cd /root/ismr24_ws/src
$ git clone https://github.com/rosmed/ismr24
~~~~

Then run `colcon build` in the workspace

~~~~
$ source /opt/ros/humble/setup.bash
$ rosdep update
$ rosdep install --from-paths src --ignore-src
$ colcon build --cmake-args -DPlusLib_DIR:PATH=<working directory>/plus/PlusBuild-bin/PlusLib-bin -DVTK_DIR=<working directory>/plus/PlusBuild-bin/vtk-bin'
~~~~


