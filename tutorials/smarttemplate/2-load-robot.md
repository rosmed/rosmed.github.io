---
layout: page
title: "Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy"
permalink: /tutorials/smarttemplate/2-load-robot.html
---

# Step 2: Load SmartTemplate robot in 3DSlicer

Open the "ROS2" module in Slicer:

![ROS2 module](images/image6.jpg)

Click the "+ Add new robot" button:

![Add new robot](images/image7.jpg)

Configure parameters and click button "Load robot"
 - Robot name: smart_template
 - Parameter node name: /robot_state_publisher
 - Parameter name: robot_description
 - Fixed frame: world

![Robot parameters](images/image8.jpg)

Observe the SmartTemplate robot loaded in the 3D view:

![SmartTemplate loaded](images/image9.jpg)

Using the SmartTemplate GUI, move the robot using the arrow buttons and observe the respective motion in Slicer

![SmartTemplate GUI](images/image10.png)

[⬅️ Previous: Load MR images](1-load-images) | [Next: Register SmartTemplate to the scanner ➡️](3-register-smarttemplate) | [Back to Table of Contents ↩️](index)