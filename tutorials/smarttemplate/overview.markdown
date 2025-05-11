---
layout: page
title: Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy
permalink: /tutorials/smarttemplate/overview.html
---


# Overview

This tutorial demonstrates how to integrate a virtual version of the **SmartTemplate** robotic system with **SlicerROS2** for planning and monitoring MRI-guided robot-assisted prostate biopsies. This tutorial provides a simplified but functional framework to understand the real-world integration of medical imaging (3D Slicer) and robotics (ROS 2) for image-guided interventions.

The tutorial is focused on:

- Connecting robot kinematics to MRI-guided workflow
- Simulating robotic needle alignment and insertion using a virtual robot
- Performing coordinate transformations between image, robot, and world frames using tf2

## Clinical Context

In **MRI-guided transperineal prostate biopsy**, a needle is inserted through the patient's perineum to reach a lesion within the prostate, identified on MRI.

Traditionally, this procedure uses a **needle guide template** placed perpendicular to the perineum. The physician selects a pre-defined hole in the template that best aligns the needle along a straight-line trajectory to the target.

However, the fixed hole grid imposes **discretization limitations**, affecting targeting precision.

## Robot-Assisted Solution: SmartTemplate

The **SmartTemplate** is a robotic needle guide that replaces the fixed template grid. It allows **continuous positioning** of the needle guide across the template plane, improving targeting accuracy by eliminating hole discretization.

The robot-assisted biopsy workflow consists of:

- **Registration** of the robot to the MRI coordinate system
- **Target selection** within the prostate on MR images
- **Needle alignment** outside the patient, along a straight-line trajectory to the target
- **Needle insertion**, passing through the perineum and reaching the target

## What You'll Do in This Tutorial

In this tutorial, you will:

- Use a **virtual SmartTemplate robot** in a simulated ROS 2 + 3D Slicer environment
- **Register** the robot to the MRI scanner coordinates using ZFrame fiducials
- **Select a target** in the prostate using the 3D Slicer medical imaging software
- From within 3D Slicer, **command the robot's motion** to align and insert the needle along a straight-line trajectory to the target

[⬅️ Back to Prerequisites](prerequisites.html) | [Next: SmartTemplate Robot ➡️](robot.html)
