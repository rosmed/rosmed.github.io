---
layout: page
title: Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy
permalink: /tutorials/smarttemplate/tutorial_steps.html
---

# Tutorial Steps

## Preparation: Launch SmartTemplate and 3D Slicer

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

## Step 1: Load MR images and register ZFrame fiducials

First, load MR images in 3DSlicer (drag the provided files and drop at the 3D Slicer interface). Then open "ZFrameRegistrationWithROI" module

![ZFrameRegistrationWithROI module](images/image2.jpg)

To register the fiducial marker model to the MR image:
- Select the horizontal ZFrame model
- Select the "COR TSE T2 COVER ZFrame" volume
- Select index of slices 9 to 12
- Define the ROI in the coronal view (green viewer) that appropriately covers the fiducials
- Click the ✓ button

![ZFrame registration](images/image3.jpeg)

Observe the resultant Linear Transform node that was created to represent the transform from ZFrame to Scanner coordinates:

![Transform node](images/image4.jpg) ![Transform coordinates](images/image5.jpg)

## Step 2: Load SmartTemplate robot in 3DSlicer

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

## Step 3: Register SmartTemplate to the scanner (world)

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

## Step 4: Make a straight needle insertion using the robot GUI

Open the "Models" module

![Models module](images/image14.jpg)

Select the `needle_link` model and edit its color and Slice Visibility:

![Needle link properties](images/image15.jpg)

Now, turn on the visualization of the provided "AX T1 VIBE" image so that you can see it in the axial, coronal, sagittal and 3D viewers:

![Image visualization](images/image16.jpeg)

Using the SmartTemplate GUI, change the insertion value from 5.0 to 100.0 mm and click the + button to insert the needle. Observe the insertion as it progresses by scrolling through the MRI volume slices and rotating the 3D view:

![Needle insertion](images/image17.png)

Now use the SmartTemplate GUI to fully retract the needle by clicking the "RETRACT" button and observe the updates in the 3D Slicer viewers

## Step 5: Make a targeted insertion using SlicerROS2 publishers

- Load the ReachableVolume ROI in 3D Slicer (drag the provided file and drop it at 3D Slicer interface)
- Open the "Markups" Module and click on the button to add a Point List:

![Markups module](images/image18.jpg)

- It will generate a Point List automatically named "F"
- Now, let's include one point to the list by clicking in the MR image in the axial viewer (red). Select a target within the reachable volume to the needle insertion.
- In our example, we selected (7.6, 27.0, -140.0) in RAS coordinates:

![Target selection](images/image19.jpg)

- After the target point "F-1" is defined, use the provided python commands in the console. They will:
  - Create a `/desired_position` publisher
  - Calculate the target in robot coordinates
  - OBS: We could publish the desired target in world coordinates and let the robot node deal with the conversion to the robot's base coordinate frame (`base_link`) using tf. However, due to the nature of our needle insertion application, we aim to decouple the horizontal and vertical alignment of the needle (Phase 1) from the needle insertion motion (Phase 2). For this reason, we chose to express the target in `base_link` coordinates, allowing us to compute the alignment position more directly and independently.

| **Phase 1: Needle alignment** | **Phase 2: Needle insertion** |
|:--------------------------------|:--------------------------------|
| Align the SmartTemplate with the target | Insert the needle to the target depth |
| ![Needle alignment](images/image20.jpg) | ![Needle insertion](images/image21.jpg) |

- Observe the final needle placement in all planes to confirm the correct needle placement:

![Final needle placement](images/image22.png)

## Step 6: Read needle position using SlicerROS2 subscriber

- In the python console, create a subscriber for `/end_effector_pose` topic and print the last message:

```python
subEEPose = rosNode.CreateAndAddSubscriberNode('PoseStamped', '/end_effector_pose') 
print(subEEPose.GetLastMessage())
```

- You can also visualize the last messages on the SlicerROS2 module.
- Click the subscribed topic to see last message and compare with desired target.
- Observe that PoseStamped messages are displayed by the dialog with position in meters and orientation in quaternion, while the vtkPoseMessage printed in the python console displays the homogeneous transform with translation in mm

![Subscriber message](images/image23.jpg)

## Step 7: Send a robot command using SlicerROS2 publishers

- In the python console, create a publisher for `/desired_command` topic:

```python
pubCommand = rosNode.CreateAndAddPublisherNode('String', '/desired_command')
```

- Send "RETRACT" message and observe the robot fully retract the needle:

```python
pubCommand.Publish('RETRACT')
```

![Retract command](images/image24.jpg)

- Send "HOME" message and observe the robot go to its initial position:

```python
pubCommand.Publish('HOME')
```

![Home command](images/image25.jpg)

[⬅️ Back to SmartTemplate ROS2 Nodes](ros2_node.html) | [Back to Table of Contents ↩️](index.html)
