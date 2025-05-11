---
layout: page
title: "Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy"
permalink: /tutorials/smarttemplate/3-register-smarttemplate.html
---

# Step 3: Register SmartTemplate to the scanner (world)

Open the python console (click the button in the menu bar)

![Python console](images/image11.jpg)

Use the provided python commands in the console. They will:
- Get the robot node from the scene
- Get the `robot_description` from the ROS2 topic publisher by SmartTemplate
- Recover ZframeToRobot information from the URDF custom parameters
- Use the ZframeToScanner registration to calculate the final RobotToScanner transform (robot world pose)
- Create a ROS2 publisher and publish to the `\world_listener` topic (which is subscribed by SmartTemplate to send a `tf_static_broadcast` to update the tf tree)

In the ROS2 terminal where the SmartTemplate was launched, you can read an indication that the `world_pose_listener` node was triggered by the `\world_pose` topic published by SlicerROS2, and this caused the `tf_static_broadcaster` to update the world transform:

```
[INFO] [<timestamp_value>] [world_pose_listener]: Updated static transform world -> base_link published.
```

The SmartTemplate pose will automatically update to the registered pose with respect to the world (scanner). Also, observe the publisher we created in the SlicerROS2 GUI and the topic message just sent:

![Robot registration](images/image12.jpg) ![Topic message](images/image13.jpg)

[⬅️ Previous: Load SmartTemplate robot](2-load-robot) | [Next: Make a straight needle insertion ➡️](4-straight-needle-insertion) | [Back to Table of Contents ↩️](index)
