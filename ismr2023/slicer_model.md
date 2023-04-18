---
layout: page
title: Slicer Model 
---

**Part 1: AI segmentation from tracked ultrasound images.**

Back to [Tutorial Home](/ismr2023/)

For this part of the workshop we'll demonstrate how to segment a tumor volume from tracked ultrasound images. 

The data for this part of the tutorial can be found [here](https://github.com/rosmed/ismr2023_files/tree/master/data). 

Detailed instructions are available [here](https://github.com/rosmed/ismr2023_files/tree/master/presentations). 

We've pretrained a network for this part of the workshop on ultrasound images with tumors in them. 

1. Load the scene "ISMRWorkshop-tongueSeg.mrb" - found here: https://github.com/rosmed/ismr2023_files/blob/master/data/ISMRWorkshop-tongueSeg.mrb

2. Update your Red Slice view by selecting the push pin icon in the top left. The top selector in the Dropdown should be set to None, the middle should be set to Predictino Volume and the third should be set to Image_Reference. The value of the second selector (beside 'PredictedVolume' should also be 0.5).

3. Switch to the "Sequences" module and play "AIseg-Tongue" to watch the live prediction.

4. Switch to the "Volume Reconstruction" module. Select 'Live volume reconstruction', Create a new node called 'PredictedTumor' for Output volume node, 'ROI' for ROI Node, Linear for interpolation node and press "Apply".

5. Switch to the "Volume rendering" module and Render "PredictedTumor" using the "MR-Angio" display preset.

You will now see a white volue in the scene that's reconstructed as you place the sequence.


References: 
* [https://www.youtube.com/watch?v=WyscpAee3vw](https://www.youtube.com/watch?v=WyscpAee3vw)
* [https://www.slicerigt.org/wp/user-tutorial/](https://www.slicerigt.org/wp/user-tutorial/)


**Part 2: Intro to SlicerROS2.**

For this portion of the workshop we walk through examples from the SlicerROS2 read the docs. Available here: https://slicer-ros2.readthedocs.io/en/latest/index.html

We package standard ROS2 communication mechanisms as MRML nodes in 3D Slicer (https://slicer.readthedocs.io/en/latest/developer_guide/mrml_overview.html). 

<img width="612" alt="image" src="https://user-images.githubusercontent.com/36430552/232659172-0c4257b4-300d-470f-8aa9-56ff8e3a58f1.png">

To get started with SlicerROS2, we'll be using a simulated patient side manipulator (PSM) for the dVRK. 

1. Start by building the dVRK robot package in ROS2: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2


2. Once that's built, launch the virtual PSM: 
3


