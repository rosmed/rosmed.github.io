---
layout: page
title: "Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy"
permalink: /tutorials/smarttemplate/0-preparation.html
---

# Preparation: Launch SmartTemplate and 3D Slicer

Files are available on [the tutorial GitHub repository](https://github.com/rosmed/ismr2025). 
The following files for this tutorial are under the `smarttemlplate_example` directory.  
- AX_T1_VIBE_fs_tra_320.nrrd
- COR_TSE_T2_COVER_ZFRAME.nrrd
- ReachableVolume.mrk.json
- python_console_commands.txt

On the terminal, source the ROS2 workspace contains your SmartTemplate_demo build:

```bash
source path_to_ros2_ws/install/setup.bash
```
Note that, if you are running [the tutorial Docker image](prerequisites), `path_to_ros2_ws` is `/root/ros2_ws`.

Now, run the Slicer application from the command line. In another terminal window, source your ROS2 workspace again and launch the robot:

```bash
source path_to_ros2_ws/install/setup.bash
ros2 launch smart_template_demo robot.launch.py
```

[⬅️ Previous: Virtual SmartTemplate ROS2 Node](ros2_node) | [Next Step: Load MR images and register ZFrame fiducials ➡️](1-load-images) | [Back to Table of Contents ↩️](index)
