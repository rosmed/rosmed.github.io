---
layout: page
title: "Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy"
permalink: /tutorials/smarttemplate/5-targeted-insertion.html
---

# Step 5: Make a targeted insertion using SlicerROS2 publishers

Load the ReachableVolume ROI in 3D Slicer (drag the provided file and drop it at 3D Slicer interface) and open the "Markups" Module and click on the button to add a Point List.

![Markups module](images/image18.jpg)

It will generate a Point List automatically named "F". Now, let's include one point to the list by clicking in the MR image in the axial viewer (red). Select a target within the reachable volume to the needle insertion.  In our example, we selected (7.6, 27.0, -140.0) in RAS coordinates:

![Target selection](images/image19.jpg)

After the target point "F-1" is defined, use the following python commands in the console (also provided in the command file):

~~~~
# Create publisher for /desired_position
pubGoal = rosNode.CreateAndAddPublisherNode('PoseStamped', '/desired_position')

# Get target in scanner coordinates
targetMarkupsNode = slicer.util.getFirstNodeByName('F', className='vtkMRMLMarkupsFiducialNode')
target_scanner = targetMarkupsNode.GetNthControlPointPosition(0)

# Calculate initial alignment (insertion depth [Y coordinate] set to 0.0)
# OBS: Another approach would be to use a tf_lookup from 'world' to 'base_link' to get mat_RobotToScanner 
t_ScannerToRobot = vtk.vtkTransform()
t_ScannerToRobot.SetMatrix(mat_RobotToScanner)
t_ScannerToRobot.Inverse()
target = t_ScannerToRobot.TransformPoint(target_scanner)

mat_desired_position = vtk.vtkMatrix4x4()
mat_desired_position.Identity()
mat_desired_position.SetElement(0,3,target[0])
mat_desired_position.SetElement(2,3,target[2])
~~~~

~~~~
#### PHASE 1: ALIGNMENT ####
# Send alignment position message
goal_msg = pubGoal.GetBlankMessage()
h = goal_msg.GetHeader()
h.SetFrameId('base_link')
goal_msg.SetPose(mat_desired_position)
pubGoal.Publish(goal_msg)
print('Desired position = [%.4f, %.4f, %.4f]mm - base_link frame' % (target[0], 0.0, target[2]))
~~~~

After those commands, wait for the robot to finish alignment. Then,

~~~~
#### PHASE 2: INSERTION ####
# Send target position message
mat_desired_position.SetElement(1,3,target[1])
goal_msg.SetPose(mat_desired_position)
pubGoal.Publish(goal_msg)
print('Desired position = [%.4f, %.4f, %.4f]mm - base_link frame' % (target[0], target[1], target[2]))
~~~~


They will:
  - Create a `/desired_position` publisher
  - Calculate the target in robot coordinates
  - OBS: We could publish the desired target in world coordinates and let the robot node deal with the conversion to the robot's base coordinate frame (`base_link`) using tf. However, due to the nature of our needle insertion application, we aim to decouple the horizontal and vertical alignment of the needle (Phase 1) from the needle insertion motion (Phase 2). For this reason, we chose to express the target in `base_link` coordinates, allowing us to compute the alignment position more directly and independently.

| **Phase 1: Needle alignment** | **Phase 2: Needle insertion** |
|:--------------------------------|:--------------------------------|
| Align the SmartTemplate with the target | Insert the needle to the target depth |
| ![Needle alignment](images/image20.jpg) | ![Needle insertion](images/image21.jpg) |


Observe the final needle placement in all planes to confirm the correct needle placement:

![Final needle placement](images/image22.png)

[⬅️ Previous: Straight needle insertion](4-straight-needle-insertion) | [Next: Read needle position ➡️](6-read-needle-position) | [Back to Table of Contents ↩️](index)
