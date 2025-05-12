---
layout: page
title: Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy 
permalink: /tutorials/smarttemplate/description.html
---

# ROS Robot Description Package for SmartTemplate

Before proceeding with the tutorial steps, it is essential to understand the SmartTemplate robot description provided in the `smart_template_description` package. This package contains the Unified Robot Description Format (URDF) files that define the robot's structure, kinematics, joint limits, and sensor configurations.

## What is URDF?

URDF is an XML format used in ROS to describe a robot model's physical configuration and kinematics. It includes information about each link, joint, and sensor, allowing ROS to simulate the robot's behavior accurately. URDF files are also utilized to define the transformation hierarchy between various robot components.

__Examples of URDF tags:__

- `<link>`: Defines a single rigid body with its visual and collision properties.
- `<joint>`: Specifies how two links are connected. The movement axis is defined using `<axis xyz="..."/>`.
- `<gazebo>`: Contains simulation parameters, such as joint friction, damping, and control gains.
- `<custom_parameters>`: Allows custom parameters, to be specified for later retrieval by ROS nodes.

## Key URDF Files in smart_template_description

- `smart_template.urdf.xacro`: The main entry point for generating the URDF. It uses Xacro (XML Macros) to define the world frame, call all other Xacro files, and fix the robot `base_link` to the world.
- `smart_template.xacro`: This file defines the main robot structure. It includes:
  - Robot structure and link definitions (`base_link`, `vertical_link`, `horizontal_link`, `needle_link`).
  - Joint definitions (`vertical_joint`, `horizontal_joint`, `insertion_joint`), its limits and other custom tags for specific control parameters such as driver channels and encoder count ratios.
  - Mesh file paths for visual representation.
- `materials.xacro`: Defines colors and visual properties for robot components. We kept this one simple for our intended use.
- `zframe.xacro`: Specifies the ZFrame position and orientation, defined within `<custom_parameters>` tags.

## Robot Registration

The **Z-Frame** is a special marker that is visible on MRI scans. It helps us register the robot's position in the scanner coordinate system.

- It is rigidly attached to the robot during setup at a fixed and known position.
- It shows up clearly in the MR image and has a known geometry.
- By detecting it in the image, we can figure out where the robot is relative to the scanner.

To register the robot with the MRI image, we need to know two things:

1. **ZFrame to Scanner transform:** This transform is calculated from the MRI image after detecting the ZFrame (using ZFrameRegistration module).
2. **ZFrame to Robot transform:** This is a fixed transform based on how the Z-Frame is mounted on the robot. It is defined inside the robot's URDF under <custom_parameters>:

~~~~
<custom_parameters>
  <zframe_position value="..."/>
  <zframe_orientation value="..."/>
</custom_parameters>
~~~~

![Z-Frame](images/image1.gif)


[⬅️ Back to SmartTemplate Robot](robot) | [Next: Virtual SmartTemplate ROS2 Node ➡️](ros2_node) | [Back to Table of Contents ↩️](index)
