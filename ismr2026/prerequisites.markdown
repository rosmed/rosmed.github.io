---
layout: page
title: Prerequisites - ISMR2026 Tutorial
permalink: /ismr2026/prerequisites.html
---

# Prerequisites

Before starting this tutorial, please ensure you have the following prerequisites in place. This will help you successfully complete the SmartTemplate robot-assisted prostate biopsy tutorial.

## Knowledge Prerequisites

This tutorial assumes basic familiarity with:
- ROS2 concepts (nodes, topics, tf2)
- 3D Slicer interface
- Basic understanding of medical image-guided interventions

If you're new to these concepts, we recommend reviewing the following resources before starting:
- [ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [3D Slicer Documentation](https://slicer.readthedocs.io/en/latest/user_guide/getting_started.html)
- [SlicerROS2 Documentation](https://slicer-ros2.readthedocs.io/)

## Computer Environment

You can use either a pre-configured Docker image, or a native Linux machine (Ubuntu 24.04).

### Option 1: Use the Linux desktop environment on the cloud through vast.ai

Vast.ai is a low-cost, public cloud-based marketplace for renting GPUs, often described as an "Airbnb for GPUs." It allows users to rent underutilized GPUs from data centers or private individuals for AI and other computing that require GPUs. Users can launch a new Linux instance with preloaded software packages from a "template." The template for this tutorial can be found at [ISMR2026 Tutorial Template](https://cloud.vast.ai?ref_id=424992&template_id=6072a53b0c32f0ac80aebee5462852ad).

To create a new instance using this template:

1. Open the above link.
2. Select a machine from the list by clicking the `RENT` button. Besides the price, choose a machine with a decent spec in nearby area. Performance, especially the loading time depends on the machine.
3. The instance starts as soon as the machine is selected. Open the `Instances` page (by clicking "instances" on the left menu) and wait until the blue button becomes `Open`.
4. Click the `Open` button. The browser opens a new Window (Make sure that your browser allows a new pop-up window before this) with a list of applications.
5. Click the `Launch Application` button in either `Selkies Low Latency Desktop` or `Apache Guacamole Desktop (VNC)`. Either button opens a new window with the desktop GUI, though Selkies may offer better performance because it uses a newer mechanism (WebRTC). If Selkies does not work, try VNC. It may take for a while before the desktop environment becomes ready.

### Option 2: Native Linux Machine

 Hardware Requirements

- A computer with at least 16GB RAM and 20GB free disk space
- When using Docker, ensure your system meets Docker requirements and has adequate resources allocated to Docker

#### Software
If you prefer to install the components manually on Ubuntu 24.04, you'll need:

1. **ROS2 Jazzy** - The Robot Operating System (ROS)
2. **3D Slicer (v5.10.0)** - Medical image visualization and processing platform
3. **SlicerROS2 Module** - Integration between 3D Slicer and ROS2
4. **Additional modules** - For specialized functionality like Z-frame registration
5. **SmartTemplate Demo Repository** - The robot description and control code

Detailed installation instructions for all required software can be found in the [Setup](setup.html) page.


[⬅️ Back to ISMR 2025 Workshop Page](index.html)
