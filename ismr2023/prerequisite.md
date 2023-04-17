---
layout: page
title: Prerequisite
---


Back to [Tutorial Home](/ismr2023/)

Prerequisite
============

We encourage all participants to bring their own laptops to follow the tutorial. This page outlines hardware/software requirements. If you are planning to participate in our workshop, we recommend to download/install the following package before the workshop, as it may take some time to download them.

We provide a Docker image of Linux/ROS environment for the tutorial; however, if you prefer to build your own ROS environment on a native Linux system, please refer to [Setting Up ROS Environment for Tutorial](ros_env).


Using Docker
------------

### Hardware

An Intel-based computer that can run either Windows, macOS, or Linux with
- At least 8GB RAM (16+GB recommended)
- Hardware that supports virtualization (For PC, virtualization must be turned on from BIOS. Please refer to [Intel's support page](https://www.intel.com/content/www/us/en/support/articles/000007139/server-products.html) for detail).

or, a Mac with an Apple Silicon CPU (M1 or M2). Please note that part of the tutorial (AI-based segmentation) does not work on the Apple Silicon CPU because of the version of the TensorFlow library included in the Docker image. 

We also recommend bringing a mouse, as it makes it easier to maneuver 3D graphics on the GUI.

### Operating System 

One of the followings:
- Windows 10 or 11
- macOS High Sierra (10.13) or higher (not tested on Apple silicon models)
- Linux (Ubuntu 20.04 or higher equivalent is recommended to run ROS2 natively)


### Docker

We will use [Robot Operating System](http://www.ros.org/), which requires a Linux operating system (Ubuntu 15.10 or 16.04, or Debian 8.11 Jessie). If you are using a Windows or Mac computer, or a computer with a Linux distribution other than those listed here, you will need to run a Linux operating system using virtualization software, [Docker](https://www.docker.com/). Please go to the Docker's page, and install either:
- [Docker Desktop](https://www.docker.com/products/docker-desktop) (Windows 10, macOS Sierra 10.12 or above)
- [Docker CE](https://docs.docker.com/install/) (Linux)


Using Native ROS
----------------
An Intel-based computer with [Linux that is compatible with ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Alternatives/Ubuntu-Development-Setup.html). 





