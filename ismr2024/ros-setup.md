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

3D Slicer
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


Building and Installing Extensions (plug-ins) for 3D Slicer
-----------------------------------------------------------

We will add several Extensions (plug-ins) for 3D Slicer. While 3D Slicer has a AppStore-like GUI called [Extensions Manager](https://slicer.readthedocs.io/en/latest/user_guide/extensions_manager.html) to browse, download, and install those Extensions, we need to build and install them from source code because the 3D Slicer built for SlicerROS2 is currently not compatible with the binary-distributed Extensions. The following Extensions are required:

- SlicerIGSIO
- SlicerIGT
- SlicerOpenIGTLink (Optional)

In the following sections, we build those Extensions and make install packages so that they can be installed from Slicer Extensions. Alternatively, Extensions [can be installed in 3D Slicer from the application setting without creating the packages](https://slicer.readthedocs.io/en/latest/developer_guide/extensions.html). 


### Building SlicerIGSIO

IGISO is a library to provide high-level communication layer for image-guided therapy (IGT) applications, and required for [the SlicerIGT Extension](https://www.slicerigt.org).
Run the following commands to build  

~~~~
$ mkdir -p <working directory>/slicer/modules
$ cd <working directory>/slicer/modules
$ git clone https://github.com/IGSIO/SlicerIGSIO
$ mkdir SlicerIGSIO-build
$ cd SlicerIGSIO-build
$ cmake -DCMAKE_BUILD_TYPE:STRING=Release -DSlicer_DIR:PATH=<working directory>/slicer/Slicer-SuperBuild-Release/Slicer-build ../SlicerIGSIO
$ make -j4
$ cd inner-build
$ make package
$ mkdir -p <working directory>/slicer/packages
$ mv *.tar.gz <working directory>
~~~~

### Building SlicerIGT

SlicerIGT provides a set of tools to build IGT applications. 

~~~~
$ cd <working directory>/slicer/modules
$ git clone https://github.com/SlicerIGT/SlicerIGT
$ mkdir SlicerIGT-build 
$ cd SlicerIGT-build
$ cmake -DCMAKE_BUILD_TYPE:STRING=Release -DSlicer_DIR:PATH=<working directory>/slicer/Slicer-SuperBuild-Release/Slicer-build -DSlicerIGSIO_DIR:PATH=<working directory>/slicer/modules/SlicerIGSIO-build/inner-build ../SlicerIGT 
$ make -j4 
$ make package
$ mkdir -p <working directory>/slicer/packages
$ mv *.tar.gz <working directory>/slicer/packages
~~~~


### Building SlicerOpenIGTLink

OpenIGTLink is not required for this tutorial, but might be useful later, if you need to use Slicer with external software/hardware for IGT applications. 

~~~~
$ cd <working directory>/slicer/modules
$ git clone https://github.com/openigtlink/SlicerOpenIGTLink
$ mkdir SlicerOpenIGTLink-build
$ cd SlicerOpenIGTLink-build
$ cmake -DCMAKE_BUILD_TYPE:STRING=Release -DSlicer_DIR:PATH=/root/slicer/Slicer-SuperBuild-Release/Slicer-build ../SlicerOpenIGTLink
$ make -j4
$ cd inner-build
$ make package
$ mv *.tar.gz <working directory>/slicer/packages
~~~~

### Installing the Extensions

To install the extensions from the packages created above, open the Extensions Manager ("View" -> "Extensions Manager")







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


Other files for Tutorial
------------------------

