---
layout: page
title: ROS2 Environment
---

Back to [Tutorial Home](/ismr2023/)

Setting Up ROS 2 Environment for Tutorial
=========================================
In this tutorial, we will use ROS Galactic, which supports Ubuntu Linux (Focal Fossa -20.04) 64-bit. It also supports other operating systems, but the installation on those operating systems is not as straightforward as on the Linux environment. See [Installation page](https://docs.ros.org/en/galactic/Installation.html) for detail.

If you are new to ROS2, we encourage to use our Docker image. If you want to set up your own ROS environment for the tutorial, please refer to section [Installing ROS on Dedicated ROS Computer](#native_ros)



[Option 1] Using ROS on Docker
===================================================

[Docker](https://www.docker.com/) must be installed prior to the installation. To install the Docker image for the tutorial, open the terminal and run the following command:

~~~~
docker pull rosmed/docker-ubuntu-vnc-desktop-ros2:ismr2021
docker run -it --rm -p 6080:80 -p 28944:18944 rosmed/docker-ubuntu-vnc-desktop-ros2:ismr2021
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

### Installing ROS Galactic
Please follow [Installation page](https://docs.ros.org/en/galactic/Installation.html).


### Building dVRK packages

Specific instruction to build the dVRK packages for ROS2 Galactic are available here:
- [dVRK ROS 2](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2)

Please note that the following build process may require a large memory space. If it fails, consider limiting the number of processes using the '--parallel-workers' option.

### Install Moveit! Packages

The tutorial uses Moveit! to compute trajectories of the Patient Side Manipulator (PSM). In particular, the [move_group](https://moveit.picknik.ai/humble/doc/concepts/move_group.html) is used to compute Cartesian paths. To install MoveIt! packages use the command

~~~~
sudo apt-get install ros-galactic-moveit*
~~~~

#### Install ISMR 2023 specific packages

This package will serve as in intermediate for converting a pose array sent from Slicer to a service request to the move_group node. Followig this, it sends the resulting trajectory to the dVRK actionlib server to execute the trajectory. 

~~~~
cd ~/ros2_ws
git clone https://github.com/rosmed/ismr23_planner.git src/ismr23
git clone https://github.com/Ketan13294/dvrk_psm_moveit2_config.git src
colcon build
source install/setup.bash
~~~~

## Starting move_group with the PSM

To start the move_group node that is configured for the PSM try the following commands

~~~~
cd ~/ros2_ws
source install/setup.bash
ros2 launch ismr23 demo.launch.py
~~~~

This should bring up the PSM with the MoveIt! Rviz plugin. You can use CTRL-C in your terminal.

## Starting the ros2_control for the PSM

The execution of the trajectory is mananaged by [ROS2 control](https://control.ros.org/master/index.html). To start ROS control try the following commands

~~~~

~~~~




