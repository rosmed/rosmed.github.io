---
layout: page
title: MRI-Guided Robot-Assisted Prostate Biopsy with SmartTemplate
permalink: /tutorials/smarttemplate/
---

# MRI-Guided Robot-Assisted Prostate Biopsy with SmartTemplate

- Mariana Bernardes - Brigham and Women's Hospital and Harvard Medical School (mcostabernardesmatias@bwh.harvard.edu)
- Junichi Tokuda - Brigham and Women's Hospital and Harvard Medical School (tokuda@bwh.harvard.edu) 

This tutorial demonstrates how to integrate a virtual version of the **SmartTemplate** robotic system with **SlicerROS2** for planning and monitoring MRI-guided robot-assisted prostate biopsies.

## Table of Contents

1. [Prerequisites](prerequisites.html)
   - Required Files
   - Required Software
   - Hardware Requirements
   - Knowledge Prerequisites

2. [Overview](overview.html)
   - Clinical Context
   - Robot-Assisted Solution: SmartTemplate
   - What You'll Do in This Tutorial

3. [SmartTemplate Robot](robot.html)
   - Kinematic Chain
   - Joint Definitions

4. [SmartTemplate Description Package](description.html)
   - What is URDF?
   - Key URDF Files

5. [Virtual SmartTemplate ROS2 Node](ros2_node.html)
   - Interfaces
   - Robot Registration

6. [Tutorial Steps](tutorial_steps.html)
   - Preparation: Launch SmartTemplate and 3D Slicer
   - Step 1: Load MR images and register ZFrame fiducials
   - Step 2: Load SmartTemplate robot in 3DSlicer
   - Step 3: Register SmartTemplate to the scanner
   - Step 4: Make a straight needle insertion using the robot GUI
   - Step 5: Make a targeted insertion using SlicerROS2 publishers
   - Step 6: Read needle position using SlicerROS2 subscriber
   - Step 7: Send a robot command using SlicerROS2 publishers

