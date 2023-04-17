---
layout: page
title: Docker Image Instruction
---

Back to [Tutorial Home](/ismr2023/)

Multiple Docker image "snapshots" are built for the ISMR Workshop tutorial. Those snapshots are taken at each major step to build the SlicerROS2. Those steps include:
- Base (Ubuntu 20.04, VNC, ROS Galactic, and prerequisite libraries for 3D Slicer and ROS) ~3.26GB
- Slicer (Base + 3D Slicer build, and Extensions) ~17.1GB
- SlicerROS2 (Slicer + SlicerROS2 modules) ~18.1GB
- SlicerROS2 Lightweight (Base + 3D Slicer binary package + Extension binary package + ROS2 workspace folder) 5.03GB

Each image uses the previous image as a base (e.g., SlicerROS2 uses Slicer) except for SlicerROS2 Lightweight, which uses the Base image as a base and incorporates the Slicer and Extension binary packages extracted from SlicerROS2 image. 


Building Docker Image (Not needed for end-users)
------------------------------------------------
The Dockerfile repository can be obtained by cloning the repository at GitHub. Please note that '--recursive' option is required to build the image correctly. `ismr2023` branch is used for the ISMR 2023 workshop:

~~~~
$ git clone --recursive https://github.com/rosmed/docker-ubuntu-vnc-desktop-ros2
$ cd docker-ubuntu-vnc-desktop-ros2
$ git checkout ismr2023
~~~~

To build an image, run the following command:
~~~~
# Base image
$ docker build -f Dockerfile.base.amd64 . -t rosmed/docker-ubuntu-vnc-desktop-base:ismr2023
# Slicer image
$ docker build -f Dockerfile.slicer . -t rosmed/docker-ubuntu-vnc-desktop-slicer:ismr2023
# SlicerROS2 image
$ docker build -f Dockerfile.slicerros2 . -t rosmed/docker-ubuntu-vnc-desktop-slicerros2:ismr2023
# SlicerROS2 Lightweight image
$ docker build -f Dockerfile.slicerros2.lw . -t rosmed/docker-ubuntu-vnc-desktop-slicerros2-lw:ismr2023
~~~~

Please note that the colcon build process in this Docker build may fail due to memory shortage. If it fails, consider increasing the memory size assigned to the Docker image. The current Docker file was tested on macOS Big Sur + Docker Desktop 4.01. Six CPUs, 16GB memory, 1GB swap, and 69.6GB disk image were assigned.

To push the image to Docker Hub:
~~~~
$ docker login -u <<myusername>> 
$ docker push rosmed/docker-ubuntu-vnc-desktop-base:ismr2023
$ docker push rosmed/docker-ubuntu-vnc-desktop-slicer:ismr2023
$ docker push rosmed/docker-ubuntu-vnc-desktop-slicerros2:ismr2023
$ docker push rosmed/docker-ubuntu-vnc-desktop-slicerros2-lw:ismr2023
~~~~
NOTE: User `myusername` has to have write access for the repository.

Running Docker image
---------------------

If you run the image available in the Docker Hub, pull the image using the following command:
~~~~
$ docker pull rosmed/docker-ubuntu-vnc-desktop-slicerros2:ismr2023
~~~~

To execute the docker image, call the following command:
~~~~
$ docker run -it --rm -p 6080:80 rosmed/docker-ubuntu-vnc-desktop-slicerros2:ismr2023
~~~~

In this example, the web port on the docker container will be mapped to port 6080 on the host computer. The '--rm' option will remove the container upon termination. 

Optionally, if you want to connect 3D Slicer with a process running on the host using OpenIGTLink by mapping the local (host) port 28944 to the guest OS's port 18944, use the following command to run the Docker image instead:
~~~~
$ docker run -it --rm -p 6080:80 -p 28944:18944 rosmed/docker-ubuntu-vnc-desktop-slicerros2:ismr2023
~~~~




