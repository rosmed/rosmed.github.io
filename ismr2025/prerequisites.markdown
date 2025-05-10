---
layout: page
title: Prerequisites - ISMR2025 Tutorial
permalink: /ismr2025/prerequisites.html
---

# Prerequisites

Before starting this tutorial, please ensure you have the following prerequisites in place. This will help you successfully complete the SmartTemplate robot-assisted prostate biopsy tutorial.

## Required Files

All required files are included in the Docker image (see the next section). They can also be downloaded from the following GitHub repositories. 

Example image data are available in [this GitHub repository](https://github.com/rosmed/ismr2025). The repository includes:

- **MR image files:**
  - AX_T1_VIBE_fs_tra_320.nrrd
  - COR_TSE_T2_COVER_ZFRAME.nrrd
- **ROI file:**
  - ReachableVolume.mrk.json
- **Python commands:**
  - python_console_commands.txt

The demo ROS package for the SmartTemplate demo is available from [the SmartTemplate Demo repository](ttps://github.com/maribernardes/ros2_smart_template_demo].


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

### Option 1: Download and Run the Docker image (Recommended) 


Pull the image using the following command:
~~~~
$ docker pull rosmed/docker-ubuntu-vnc-desktop-slicerros2-lw:ismr2025
~~~~

Please note that this Docker image is a lightweight version and only contains a binary package of 3D Slicer. If you plan to use the Docker image for  3D Slicer module development, you will need a full Docker image with 3D Slicer build files, which can be obtained by:

~~~~
$ docker pull rosmed/docker-ubuntu-vnc-desktop-slicerros2:ismr2025
~~~~

To execute the docker image, run the following command:
~~~~
$ docker run -it --rm -p 6080:80 rosmed/docker-ubuntu-vnc-desktop-slicerros2-lw:ismr2025
~~~~
(in case of using the full Docker image, specify `rosmed/docker-ubuntu-vnc-desktop-slicerros2:ismr2025` instead).

In this example, the HTTP port (port 80) on the docker container will be mapped to port 6080 on the host computer. The '--rm' option will remove the container upon termination. If the Docker container is successfully started, its desktop environment can be accessed using a web browser by accessing `https://localhost:6080`.


### Option 2: Native Linux Machine

 Hardware Requirements

- A computer with at least 16GB RAM and 20GB free disk space
- When using Docker, ensure your system meets Docker requirements and has adequate resources allocated to Docker

#### Software
If you prefer to install the components manually on Ubuntu 24.04, you'll need:

1. **ROS2 Jazzy** - The Robot Operating System (ROS)
2. **3D Slicer (v5.8.2)** - Medical image visualization and processing platform
3. **SlicerROS2 Module** - Integration between 3D Slicer and ROS2
4. **Additional modules** - For specialized functionality like Z-frame registration
5. **SmartTemplate Demo Repository** - The robot description and control code

Detailed installation instructions for all required software can be found in the [Setup](setup.html) page.


[⬅️ Back to ISMR 2025 Workshop Page](index.html)
