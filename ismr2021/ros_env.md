Back to [Tutorial Home](https://rosmed.github.io/)

Introduction
============
In this tutorial, we will use ROS Foxy, which supports Ubuntu Linux (Focal Fossa -20.04) 64-bit and Debian Linux (Buster - 10) 64-bit. It also supports other operating systems, but the installation on those operating systems is not as straightforward as on the Linux environment. See [Installation page](https://docs.ros.org/en/foxy/Installation.html) for detail.

If you are new to ROS2, we encourage to use our Docker image. If you want to set up your own ROS environment for the tutorial, please refer to section [Installing ROS on Dedicated ROS Computer](#native_ros)



[Option 1] Using ROS on Docker
===================================================

[Docker](https://www.docker.com/) must be installed prior to the installation. To install the Docker image for the tutorial, open the terminal and run the following command:

~~~~
sudo docker pull rosmed/docker-ubuntu-vnc-desktop-ros2:ismr2021
sudo docker run -it --rm -p 6080:80 -p 28944:18944 rosmed/docker-ubuntu-vnc-desktop-ros2:ismr2021
~~~~

In this example, the HTTP port (port 80) and the OpenIGTLink port (port 18944) on the docker container are mapped to ports 6080 and 28944 on the host computer respectively. The '--rm' option will remove the container upon termination.

To access the desktop, open a web browser (compatible with HTML5) on the same computer, and type the following address in the address bar:
~~~~
http://localhost:6080
~~~~
If the Docker image container is successfully running, the browser should show the desktop screen.

This docker image includes all the software required for the tutorial and does not require installing other packages (i.e. OpenIGTLink and ros2_igtl_bridge)


The detail of this Docker image is outlined in the following page:
- [Docker ROS 2 for ISMR 2021 Tutorial](https://github.com/rosmed/rosmed.github.io/wiki/DockerROS2)


[Option 2] Installing ROS on Dedicated ROS Computer <a name="native_ros"></a>
===================================================

### Installing ROS Foxy
Please follow [Installation page](https://docs.ros.org/en/foxy/Installation.html).


### Building a Custom Robot Driver

The robot driver for this tutorial is available at:
- [Universal Robots ROS2 Driver for ISMR 2021 Tutorial](https://github.com/simonleonard/Universal_Robots_ROS2_Driver)

Please note that the following build process may require a large memory space. If it fails, consider limiting the number of processes using the '--parallel-workers' option.


1. [Install ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html) or [Install ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). This branch will support both distributions until API breaking changes are made, at which point a `galactic` branch will be forked. For using this driver with ROS2 `foxy` checkout [foxy branch](https://github.com/simonleonard/Universal_Robots_ROS2_Driver/tree/foxy).

2. Make sure that `colcon`, its extensions and `vcs` are installed:
~~~~
sudo apt install python3-colcon-common-extensions python3-vcstool
~~~~

3. Create a new ROS2 workspace:
~~~~
export COLCON_WS=~/workspace/ros_ur_driver
mkdir -p $COLCON_WS/src
~~~~

4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
~~~~
cd $COLCON_WS
git clone https://github.com/simonleonard/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.repos
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
~~~~


## Using MoveIt

To use MoveIt some additional packages should be added into workspace:
~~~~
cd $COLCON_WS
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/MoveIt_Support.repos
vcs import src --skip-existing --input src/moveit2/moveit2.repos
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
~~~~

### Building OpenIGTLink

ros2_igtl_bridge depends on the OpenIGTLink library. We install the source files of the libary under ~/igtl/OpenIGTLink and build files under ~/igtl/OpenIGTLink-build
~~~~
cd ~
mkdir igtl
cd igtl
git clone https://github.com/openigtlink/OpenIGTLink.git
mkdir OpenIGTLink-build
cd OpenIGTLink-build
cmake -DBUILD_SHARED_LIBS:BOOL=OFF ../OpenIGTLink
make
~~~~

### Installing ros2_igtl_bridge

First, set up catkin working directory (if it has not been set up).
~~~~
cd $COLCON_WS
rosdep update
git clone -b ismr21 https://github.com/openigtlink/ros2_igtl_bridge  src/ros2_igtl_bridge
rosdep install --ignore-src --from-paths src -y -r
colcon build --parallel-workers 2 --cmake-args -DOpenIGTLink_DIR:PATH=/root/igtl/OpenIGTLink-build -DCMAKE_BUILD_TYPE=Release'
~~~~









