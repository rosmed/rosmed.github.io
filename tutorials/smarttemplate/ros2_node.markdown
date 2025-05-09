---
layout: page
title: Virtual SmartTemplate ROS2 Node - MRI-Guided Robot-Assisted Prostate Biopsy
permalink: /tutorials/smarttemplate/ros2_node.html
---

# Virtual SmartTemplate ROS2 Node

To support this tutorial, we use a **ROS2 node** called virtual_template. It simulates the behavior of the SmartTemplate robot. It does **not control real hardware**, but instead emulates the robot's motion logic based on simplified kinematics.

- **Robot model parsing**: At startup, reads the robot's URDF (robot_description) to load joint names, joint limits, and encoder-to-joint conversion ratios.
- **Kinematic model**: defines simple **forward kinematics (FK)** and **inverse kinematics (IK)** for 3 prismatic joints (horizontal, vertical, and insertion).
- **Motion emulation**: holds internal state for current and desired joint values. Moves the joints gradually toward the target position in small steps.
- **State publishers:** Periodically publishes the simulated joint states and end-effector (needle tip) pose

## Interfaces

The node is implemented in Python using rclpy and provides the following interfaces:

**Published Topics:**

| **Topic** | **Type** | **Description** |
|:-----------|:---------|:----------------|
| /end_effector_pose | geometry_msgs/PoseStamped | Current needle tip pose in the **world frame**, computed via TF transform from needle_link |
| /joint_states | sensor_msgs/JointState | Current joint values (3 prismatic joints) in meters, as expected by standard ROS tools |

**Subscribed Topics:**

| **Topic** | **Type** | **Description** |
|:-----------|:---------|:----------------|
| /desired_position | geometry_msgs/PoseStamped | Target pose to align the robot tip to, typically in the **world** or **scanner** frame |
| /desired_command | std_msgs/String | Predefined motion commands: HOME, RETRACT, or ABORT |

## Robot Registration

The **Z-Frame** is a special marker that is visible on MRI scans. It helps us register the robot's position in the scanner coordinate system.

- It is rigidly attached to the robot during setup at a fixed and known position.
- It shows up clearly in the MR image and has a known geometry.
- By detecting it in the image, we can figure out where the robot is relative to the scanner.
- To register the robot with the MRI image, we need to know two things:

1. **ZFrame to Scanner transform:** This transform is calculated from the MRI image after detecting the ZFrame (using ZFrameRegistration module).
2. **ZFrame to Robot transform:** This is a fixed transform based on how the Z-Frame is mounted on the robot.

It is defined inside the robot's URDF under <custom_parameters>:

~~~~
<zframe_position value="..."/>
<zframe_orientation value="..."/>
~~~~

![Z-Frame](images/image1.gif)

[⬅️ Back to SmartTemplate Description Package](description.html) | [Next: Tutorial Steps ➡️](tutorial_steps.html)