Back to [Tutorial Home](https://rosmed.github.io/)

Setting Up ROS Environment for Tutorial
========================================

Please follow [Installation page](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Installing ROS Kinetic
----------------------

Then we install additional packages, including ROS-Industrial, MoveIt!, Universal Robot, and libvtk6-dev. From the terminal,
~~~~
sudo rosdep update -y
sudo apt-get update -y
sudo apt-get dist-upgrade -y
sudo apt-get install -y ros-kinetic-industrial-core
sudo apt-get install -y ros-kinetic-universal-robot
sudo apt-get install -y ros-kinetic-moveit
sudo apt-get install -y ros-kinetic-moveit-visual-tools
sudo apt-get install libvtk6-dev
~~~~

Building OpenIGTLink
--------------------

ROS-IGTL-Bridge depends on the OpenIGTLink library. We install the source files of the libary under ~/igtl/OpenIGTLink and build files under ~/igtl/OpenIGTLink-build
~~~~
cd ~
mkdir igtl
cd igtl
git clone https://github.com/openigtlink/OpenIGTLink.git
mkdir OpenIGTLink-build
cd OpenIGTLink-build
cmake -DBUILD_SHARED_LIBS:BOOL=ON ../OpenIGTLink
make
~~~~

Installing ROS-IGTL-Bridge
--------------------------

First, set up catkin working directory (if it has not been set up). We will use the standard ROS workspace "catkin_ws". If you already have an existing one, we recommend you to move it elsewhere for the duration of this tutorial with
~~~~
mv ~/catkin_ws ~/catkin_ws_old
~~~~
Then, go ahead and create a clean workspace with
~~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
~~~~

Then obtain the code from the github repository:
~~~~
cd ~/catkin_ws/src
git clone https://github.com/openigtlink/ROS-IGTL-Bridge
~~~~

To build the ROS-IGTL-Bridge, run
~~~~
cd ~/catkin_ws/
catkin_make --cmake-args -DOpenIGTLink_DIR:PATH=~/igtl/OpenIGTLink-build
~~~~

Installing ROS packages
-----------------------

(You may skip this step, if you set up the ROS environment using the Docker image provided in this tutorial.)

On the ROS computer, open a terminal and copy two ROS packages into the katkin workspace:
~~~~
cd ~/catkin_ws/src
git clone https://github.com/rosmed/ismr19_description
git clone https://github.com/rosmed/ismr19_moveit
git clone https://github.com/rosmed/ismr19_control
~~~~
Additionally, copy IGTL Exporter, which streams the transformations of the robot link to 3D Slicer via ROS-IGTL-Bridge. 
~~~~
cd ~/catkin_ws/src
git clone https://github.com/tokjun/ros_bx_robot_bridge
~~~~
Then, run catkin_make
~~~~
cd ~/catkin_ws
catkin_make
source devel/setup.bash
~~~~

We will also use a scene file for rviz. You can download it with the following commands:
~~~~
cd ~/
mkdir models
cd models
wget -O ismr19.scene https://bit.ly/2OpSdKM
~~~~


