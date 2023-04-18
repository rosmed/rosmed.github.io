---
layout: page
title: Prerequisite
---

Back to [Tutorial Home](/ismr2023/)

We encourage all participants to bring their own laptops to follow the tutorial. This page outlines hardware/software requirements. If you are planning to participate in our workshop, we recommend to download/install the following package before the workshop, as it may take some time to download them.

We provide a Docker image of Linux/ROS environment for the tutorial. If you are new to ROS2, we encourage to use our Docker image. However, if you prefer to build your own ROS environment on a native Linux system, please refer to [Setting Up ROS Environment for Tutorial - Installing ROS on Dedicated ROS Computer](ros_env#native_ros).


Requirements for Docker Environment
-----------------------------------

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


Requirments for Native Linux Environment
----------------------------------------

An Intel-based computer with [Linux that is compatible with ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Alternatives/Ubuntu-Development-Setup.html). 





