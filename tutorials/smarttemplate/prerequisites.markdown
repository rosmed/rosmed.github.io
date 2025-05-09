---
layout: page
title: Prerequisites - MRI-Guided Robot-Assisted Prostate Biopsy
permalink: /tutorials/smarttemplate/prerequisites.html
---

# Prerequisites

Before starting this tutorial, please ensure you have the following prerequisites in place. This will help you successfully complete the SmartTemplate robot-assisted prostate biopsy tutorial.

## Required Files

Download the following files which are required for the tutorial:

- **MR image files:**
  - AX_T1_VIBE_fs_tra_320.nrrd
  - COR_TSE_T2_COVER_ZFRAME.nrrd
- **ROI file:**
  - ReachableVolume.mrk.json
- **Python commands:**
  - python_console_commands.txt

## Software Options

### Docker (Recommended)

For the easiest setup, we recommend using our pre-configured Docker image which includes all the necessary software:

```bash
docker pull ghcr.io/maribernardes/smart_template_demo:ros2
```

### Manual Installation (Alternative)

If you prefer to install the components manually on Ubuntu 24.04, you'll need:

1. **ROS2 Jazzy** - The Robot Operating System framework
2. **3D Slicer (v5.9.0)** - Medical image visualization and processing platform
3. **SlicerROS2 Module** - Integration between 3D Slicer and ROS2
4. **Additional modules** - For specialized functionality like Z-frame registration
5. **SmartTemplate Demo Repository** - The robot description and control code

Detailed installation instructions for all required software can be found in the [Setup](setup.html) page.

## Hardware Requirements

- A computer with at least 8GB RAM and 20GB free disk space
- When using Docker, ensure your system meets Docker requirements and has adequate resources allocated to Docker

## Knowledge Prerequisites

This tutorial assumes basic familiarity with:
- ROS2 concepts (nodes, topics, tf2)
- 3D Slicer interface
- Basic understanding of medical image-guided interventions

If you're new to these concepts, we recommend reviewing the following resources before starting:
- [ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [3D Slicer Documentation](https://slicer.readthedocs.io/en/latest/user_guide/getting_started.html)
- [SlicerROS2 Documentation](https://slicer-ros2.readthedocs.io/)

[⬅️ Back to Table of Contents](index.html) | [Next: Overview ➡️](overview.html) | [Jump to Setup ↗️](setup.html)
