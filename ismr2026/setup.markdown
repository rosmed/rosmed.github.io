---
layout: page
title: SlicerROS2 Setup for ISMR2026
permalink: /ismr2026/setup.html
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


# vast.ai template and Docker image

For the ISMR2026 workshop, we will use a Vast.ai to run the Linux desktop environment hosted on a cloud computing service and use it through a web browser. Vast.ai is a low-cost, public cloud-based marketplace for renting GPUs, often described as an "Airbnb for GPUs." It allows users to rent underutilized GPUs from data centers or private individuals for AI and other computing that require GPUs. 

Vast.ai has a mechanism to share a Docker container among its users and deploy on a selected GPU server. The ISMR2026 uses this mechanism to distribute a pre-built Linux environment with 3D Slicer and ROS packages required for the tutorial. The following documentation outlines how to build and setup the Docker image for those who are interested in setting up a similar environment. 


## Building the ISMR2026 Docker image

The source code for the Docker image is available at [https://github.com/rosmed/vastai-docker-image](https://github.com/rosmed/vastai-docker-image). The repository is an extention of the official Linux desktop environment provided by vast.ai, and is compatible with GitHub Action allowing users to build Docker images on GitHub.

### The base image

The base image contains a Ubuntu 24.04 desktop environment. Several packages that are not relevant to the workshp are removed to reduce the image size, such as Libre Office (office producitivity software) and Blender (3D rendering). To build the image, open [the Actions page](https://github.com/rosmed/vastai-docker-image/actions), and click the `Build Linux Desktop Slim Image` workflow. 


### The ROSMED image

The ROSMED image is the image used for the ISMRM2026 workshop. It contains ROS2 Jazzy, 3D Slicer 5.10, and other packages required for the tutorial. The Dockerfile can be find under the `derivatives/linux-desktop-rosmed` folder. Although an GitHub Action workflow is available for this Docker image, we recommend to build on a local machine, as the build process takes too long for GitHub Action, and is likely to timeout.

To biuld the ROSMED image, run the following command (assuming that the base image is already available on Docker Hub):

~~~~
$ git clone https://github.com/rosmed/vastai-docker-image
$ cd vastai-docker-image
$ docker build --build-arg VAST_BASE=rosmed/linux-desktop-slim:cuda-12.9-ubuntu24.04-2026-03-24-slim -t rosmed/linux-desktop-rosmed:cuda-12.9-ubuntu24.04-2026-03-25 -f derivatives/linux-desktop-rosmed/Dockerfile derivatives/linux-desktop-rosmed
~~~~


[Back to Prerequisites](prerequisites.html) | [Back to Workshop page](index.html)

