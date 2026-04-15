---
layout: home
title:  ""
image: images/slicer-ros-architecture.png
---


The ROS for Medical Robotics (ROS-MED) Project is an open-source software project to provide an open-source software platform for medical robotics research. Specifically, the project focuses on architectures to seamlessly integrate a robot system with medical image computing software using two popular open-source software packages: Robot Operating System (ROS) and 3D Slicer.


# Why is ROS-MED Project important?

The majority of medical robotics systems involve some form of medical imaging in order to guide and monitor the procedure. Examples include image-guided robotic needle-guide systems and surgical CAD/CAM systems. Those systems often require a wide range of image computing capabilities, such as segmentation of anatomical structures, registration of multiple images, 2D/3D image visualization, image-based planning, and data sharing with the robot controller and the hospital’s picture archiving and communication systems (PACS).

However, the engineering effort to implement those features is often underestimated in medical robotics research due to limited engineering resources or the scope of the project. Fortunately, many of those features have already been implemented and validated in the medical image computing community and are often distributed as 3D Slicer. Therefore it has become essential for academic researchers to take advantage of those existing tools and incorporate them into their own research instead of reinventing the wheel.


# What does ROS-MED Project offer?

The ROS-MED Project offers the following resources to promote open-source solutions for ROS-based medical robotics projects:

## Software Tools for ROS+Slicer Integration

We develop open-source software tools to integrate ROS and 3D Slicer. There are currently two approaches to integrating ROS and 3D Slicer into a single medical robotics system:

- __Network-based integration__([ROS-IGTL-Bridge](https://github.com/openigtlink/ROS-IGTL-Bridge) and [ROS2-IGTL-Bridge](https://github.com/openigtlink/ros2_igtl_bridge)): ROS and 3D Slicer run as independent processes and are "loosely" connected via a local area network to share various types of data, including transforms, text strings, images, etc. ROS and Slicer can run the same or separate hosts. [The OpenIGTLink Protocol](https://openigtlink.org/) is used for network communication. Both ROS and ROS2 are supported.

- __Plug-in-based integration__([SlicerROS2](https://github.com/rosmed/slicer_ros2_module)): ROS resides in a plug-in module for 3D Slicer named "SlicerROS2." In this approach, ROS and 3D Slicer run as a single process allowing "tight" integration of the two. The Slicer can directly access ROS features such as ros parameters and tf2. Currently, only ROS2 is supported for this approach.

Please refer to [the Software page](/software/) for details.

## Online Documentation

We provide online documentation on this website as well as on the GitHub repositories for each software package. 


## Tutorial Events

We organize hands-on tutorial sessions at scientific meetings. The goal of those sessions is for the participants to learn how to set up ROS/3D Slicer for their own medical robotics projects. The materials used in those tutorials, including software, slides, and data, are shared on this website. Please refer to [the Tutorial page](/tutorials/) and [the Events page](/events/).


# Sponsor


![NIBIB Logo](https://www.nibib.nih.gov/sites/default/files/nibib_logo.png){:class="img-responsive"}

The project is supported by [the National Institute of Biomedical Imaging and Bioengineering](https://www.nibib.nih.gov) of [the U.S. National Institutes of Health (NIH)](https://www.nih.gov) under award number R01EB020667, R01EB020610, and P41EB028741. The content is solely the responsibility of the authors and does not necessarily represent the official views of the NIH.




