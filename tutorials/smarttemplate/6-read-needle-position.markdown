---
layout: page
title: "Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy"
permalink: /tutorials/smarttemplate/6-read-needle-position.html
---

# Step 6: Read needle position using SlicerROS2 subscriber

In the python console, create a subscriber for `/end_effector_pose` topic and print the last message:

~~~~
subEEPose = rosNode.CreateAndAddSubscriberNode('PoseStamped', '/end_effector_pose') 
print(subEEPose.GetLastMessage())
~~~~

You can also visualize the last messages on the SlicerROS2 module. Click the subscribed topic to see last message and compare with desired target. Then observe that PoseStamped messages are displayed by the dialog with position in meters and orientation in quaternion, while the vtkPoseMessage printed in the python console displays the homogeneous transform with translation in mm.


![Subscriber message](images/image23.jpg)

[⬅️ Previous: Targeted insertion](5-targeted-insertion) | [Next: Send a robot command ➡️](7-send-robot-command) | [Back to Table of Contents ↩️](index)
