---
layout: page
title: Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy
permalink: /tutorials/smarttemplate/ros2_node.html
---

# SmartTemplate ROS2 Nodes

## Virtual Template

To support this tutorial, we use a **ROS2 node** called virtual_template. It simulates the behavior of the SmartTemplate robot. It does **not control real hardware**, but instead emulates the robot's motion logic based on simplified kinematics.

- **Robot model parsing**: At startup, reads the robot's URDF (robot_description) to load joint names, joint limits, and encoder-to-joint conversion ratios.
- **Kinematic model**: defines simple **forward kinematics (FK)** and **inverse kinematics (IK)** for 3 prismatic joints (horizontal, vertical, and insertion).
- **Motion emulation**: holds internal state for current and desired joint values. Moves the joints gradually toward the target position in small steps.
- **State publishers:** Periodically publishes the simulated joint states and end-effector (needle tip) pose

### Interfaces

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

## World Pose Listener

In addition, the smart_template_demo package includes the world_pose_listener node, which listens to the /world_pose topic. When a new pose is received, it triggers a TF static broadcast to update the base_link frame with respect to the world frame. This is crucial for maintaining the correct TF tree structure during simulated motion.

[⬅️ Back to SmartTemplate Description Package](description) | [Next: Tutorial Steps ➡️](0-preparation) | [Back to Table of Contents ↩️](index)
