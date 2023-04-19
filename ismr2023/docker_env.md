---
layout: page
title: Virtual Environment Using Docker
---

Back to [Tutorial Home](/ismr2023/)

Overview
--------

[Docker](https://www.docker.com/) is an OS-level virtualization platform that allows running a guest operating system isolated from the host operating system. While our ROS application has been only tested on a Linux operating system, you can still run it by running a Linux OS on a Windows or Mac computer using Docker. This page outlines the steps to set up the Docker environment for the tutorial.

Installing Docker
-----------------

Please follow [Get Docker page](https://docs.docker.com/get-docker/).

Obtaininig Docker Image for the Tutorial
----------------------------------------

### (Option 1) Pulling the Docker image via network

Once the Docker is setup on your computer, pull the image using the following command:
~~~~
$ docker pull rosmed/docker-ubuntu-vnc-desktop-slicerros2-lw:ismr2023
~~~~
Please note that this Docker image is a lightweight version and only contains a binary package of 3D Slicer. If you plan to use the Docker image for  3D Slicer module development, you will need a full Docker image with 3D Slicer build files, which can be obtained by:
~~~~
$ docker pull rosmed/docker-ubuntu-vnc-desktop-slicerros2:ismr2023
~~~~

### (Option 2) Loading the Docker image from a file

If you have a Docker image as a compressed file, you can load it by using the following command:
~~~~
$ docker load -i docker-ubuntu-vnc-desktop-slicerros2-lw-ismr2023.tar.gz
~~~~


Starting Docker 
---------------

To execute the docker image, run the following command:
~~~~
$ docker run -it --rm -p 6080:80 rosmed/docker-ubuntu-vnc-desktop-slicerros2-lw:ismr2023
~~~~
(in case of using the full Docker image, specify `rosmed/docker-ubuntu-vnc-desktop-slicerros2:ismr2023` instead).

In this example, the HTTP port (port 80) on the docker container will be mapped to port 6080 on the host computer. The '--rm' option will remove the container upon termination. If the Docker container is successfully started, its desktop environment can be accessed using a web browser by accessing `http://localhost:6080`.


If you are interested in how those Docker images were built, the details can be found in [this page](ISMR2023-Docker-Image-Instruction)).





