Setting Up ROS2 and 3D Slicer Environment on Ubuntu 22.04
=========================================================

The following instruction outlines the steps to install necessary components for the tutorial in your local computer. We recommend to use [our Docker image], if your goal is to simply evaluate the setup and decide whether SlicerROS2 suites your needs. The tutorial has been tested on Ubuntu 22.04 with ROS2 Humble.  

ROS2 Humble
-----------

To install the base ROS2 package, please follow [the ROS2 Humble Installation page](https://docs.ros.org/en/humble/Installation.html).

Gazebo (Ignition)
-----------------

Gazebo is a dynamic simulation environment that can be integrated with ROS. Gazebo is currently in transition from the old architecture the new one, and there are two variations exist, including "Gazebo Classic" and "Gazebo". Gazebo Classic is the old one phasing out by 2025 (according to the Gazebo website). The current Gazebo was introduced in 2019 and is formerly known as "Ignition". Following a trademark dispute, it has been re-branded as "Gazebo" since 2022. 

To install Gazebo Ignition in the ROS environment is available on [the Gazebo Website](https://gazebosim.org/docs/garden/ros_installation). If you are using ROS2 Humble on Ubuntu 22.04, following command can be used to install Gazebo Ignition:

~~~~
$ sudo apt-get install ros-humble-ros-gz
~~~~



SlicerROS2
----------
While 3D Slicer is distributed in both source and binary packages, we need to use the source package and build on the ROS2 system to use the SlicerROS2 extension. The detailed instruction to build 3D Slicer and SlicerROS2 can be found on [the SlicerROS2 instruction page](https://slicer-ros2.readthedocs.io/en/latest/pages/getting-started.html). 

First, we install the prerequisite packages required for building 3D Slicer:
~~~~
$ sudo apt-get update
$ sudo apt-get install -y git subversion build-essential cmake-curses-gui cmake-qt-gui
$ sudo apt-get install -y qtmultimedia5-dev qttools5-dev libqt5xmlpatterns5-dev libqt5svg5-dev qtwebengine5-dev qtscript5-dev qtbase5-private-dev libqt5x11extras5-dev
$ sudo apt-get install -y libxt-dev libssl-dev
~~~~

In this tutorial we will build the latest release (ver. 5.6.2):
~~~~
$ cd mkdir <home directory>/slicer
$ cd <home directory/slicer
$ clone --branch v5.6.2 https://github.com/slicer/Slicer
$ mkdir Slicer-SuperBuild-Release
$ cd Slicer-SuperBuild-Release
$ cmake -DCMAKE_BUILD_TYPE:STRING=Release -DSlicer_USE_SYSTEM_OpenSSL=ON ../Slicer
$ make -j4
~~~~
This process will take 30 minutes to several hours.


Other files for Tutorial
------------------------
