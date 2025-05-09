---
layout: page
title: MRI-Guided Robot-Assisted Prostate Biopsy with SmartTemplate
permalink: /tutorials/smarttemplate/
---

<font color=red>!!This page is under construction. !!</font>

# MRI-Guided Robot-Assisted Prostate Biopsy with SmartTemplate

Mariana Bernardes and Junichi Tokuda -- Brigham and Women's Hospital and
Harvard Medical School (mcostabernardesmatias@bwh.harvard.edu;
tokuda@bwh.harvard.edu)

## Objectives

This tutorial demonstrates how to integrate a virtual version of the
**SmartTemplate** robotic system with **SlicerROS2** for planning and
monitoring MRI-guided robot-assisted prostate biopsies. This tutorial
provides a simplified but functional framework to understand the
real-world integration of medical imaging (3D Slicer) and robotics (ROS
2) for image-guided interventions.

The tutorial is focused on:

-   Connecting robot kinematics to MRI-guided workflow

-   Simulating robotic needle alignment and insertion using a virtual
    robot

-   Performing coordinate transformations between image, robot, and
    world frames using tf2

## Clinical Context

In **MRI-guided transperineal prostate biopsy**, a needle is inserted
through the patient\'s perineum to reach a lesion within the prostate,
identified on MRI.

Traditionally, this procedure uses a **needle guide template** placed
perpendicular to the perineum. The physician selects a pre-defined hole
in the template that best aligns the needle along a straight-line
trajectory to the target.

However, the fixed hole grid imposes **discretization limitations**,
affecting targeting precision.

## Robot-Assisted Solution: SmartTemplate

The **SmartTemplate** is a robotic needle guide that replaces the fixed
template grid. It allows **continuous positioning** of the needle guide
across the template plane, improving targeting accuracy by eliminating
hole discretization.

The robot-assisted biopsy workflow consists of:

-   **Registration** of the robot to the MRI coordinate system

-   **Target selection** within the prostate on MR images

-   **Needle alignment** outside the patient, along a straight-line
    trajectory to the target

-   **Needle insertion**, passing through the perineum and reaching the
    target

## What You'll Do in This Tutorial

In this tutorial, you will:

-   Use a **virtual SmartTemplate robot** in a simulated ROS 2 + 3D
    Slicer environment

-   **Register** the robot to the MRI scanner coordinates using ZFrame
    fiducials

-   **Select a target** in the prostate using the 3D Slicer medical
    imaging software

-   From within 3D Slicer, **command the robot's motion** to align and
    insert the needle along a straight-line trajectory to the target

# SmartTemplate Robot

The SmartTemplate robot is a **3 degrees-of-freedom (3-DOF) prismatic
robot** designed to align a needle guide in 3D space for MRI-guided
interventions. It consists of three translational joints that move along
orthogonal axes.

## Kinematic Chain

~~~~
world
└── base_joint (fixed)
     └── base_link
          └── vertical_joint (Z-axis)
               └── vertical_link
                    └── horizontal_joint (X-axis)
                         └── horizontal_link
                              └── insertion_joint (Y-axis)
                                   └── needle_link (end-effector)
~~~~

## Joint Definitions

| **Joint Name** | **Type** | **Axis** | **Parent Link** | **Child Link** | **Range (m)** |
|:--------------|:---------|:---------|:---------------|:--------------|:-------------|
| vertical_joint | prismatic | Z (0,0,1) | base_link | vertical_link | ±0.025 |
| horizontal_joint | prismatic | X (1,0,0) | vertical_link | horizontal_link | ±0.03 |
| insertion_joint | prismatic | Y (0,1,0) | horizontal_link | needle_link | 0.000 → 0.115 |

**vertical_joint**: Adjusts height of the needle guide above the
perineum (Z-axis)

**horizontal_joint**: Aligns the guide laterally (X-axis)

**insertion_joint**: Controls needle insertion depth (Y-axis), i.e.,
into the patient

### SmartTemplate Description Package

Before proceeding with the tutorial steps, it is essential to understand the SmartTemplate robot description provided in the smart_template_description package. This package contains the Unified Robot Description Format (URDF) files that define the robot\'s structure, kinematics, joint limits, and sensor configurations.

### What is URDF?

URDF is an XML format used in ROS to describe a robot model\'s physical configuration and kinematics. It includes information about each link, joint, and sensor, allowing ROS to simulate the robot's behavior accurately. URDF files are also utilized to define the transformation hierarchy between various robot components.

__Examples of URDF tags:__

- \<link\>: Defines a single rigid body with its visual and collision properties.
- \<joint\>: Specifies how two links are connected. The movement axis is defined using \<axis xyz=\"\...\"/\>.
- \<gazebo\>: Contains simulation parameters, such as joint friction, damping, and control gains.
- \<custom_parameters\>: Allows custom parameters, to be specified for later retrieval by ROS nodes.


### Key URDF Files in smart_template_description

smart_template.urdf.xacro: The main entry point for generating the URDF. It uses Xacro (XML Macros) to define the world frame, call all other Xacro files, and fix the robot base_link to the world.

smart_template.xacro: This file defines the main robot structure. It includes:
- Robot structure and link definitions (base_link, vertical_link, horizontal_link, needle_link).
- Joint definitions (vertical_joint, horizontal_joint, insertion_joint), its limits and other custom tags for specific control parameters such as driver channels and encoder count ratios.
- Mesh file paths for visual representation.

materials.xacro: Defines colors and visual properties for robot
components. We kept this one simple for our intended use.

zframe.xacro: Specifies the ZFrame position and orientation, defined
within \<custom_parameters\> tags.

### Virtual SmartTemplate ROS2 Node

To support this tutorial, we use a **ROS2 node** called
virtual_template. It simulates the behavior of the SmartTemplate robot.
It does **not control real hardware**, but instead emulates the robot's
motion logic based on simplified kinematics.

**Robot model parsing**: At startup, reads the robot's URDF
(robot_description) to load joint names, joint limits, and
encoder-to-joint conversion ratios.

**Kinematic model**: defines simple **forward kinematics (FK)** and
**inverse kinematics (IK)** for 3 prismatic joints (horizontal,
vertical, and insertion).

**Motion emulation**: holds internal state for current and desired joint
values. Moves the joints gradually toward the target position in small
steps.

**State publishers:** Periodically publishes the simulated joint states
and end-effector (needle tip) pose

## Interfaces

The node is implemented in Python using rclpy and provides the following
interfaces:

**Published Topics:**

| **Topic** | **Type** | **Description** |
|:-----------|:---------|:----------------|
| /end_effector_pose | geometry_msgs/PoseStamped | Current needle tip pose in the **world frame**, computed via TF transform from needle_link |
| /joint_states | sensor_msgs/JointState | Current joint values (3 prismatic joints) in meters, as expected by standard ROS tools |

**Subscribed Topics:**

| **Topic** | **Type** | **Description** |
|:-----------|:---------|:----------------|
| /desired_position | geometry_msgs/PoseStamped | Target pose to align the robot tip to, typically in the **world** or **scanner** frame |
| /desired_command | std_msgs/String | Predefined motion commands: HOME, RETRACT, or ABORT |

## Robot Registration

The **Z-Frame** is a special marker that is visible on MRI scans. It
helps us register the robot\'s position in the scanner coordinate
system.

-   It is rigidly attached to the robot during setup at a fixed and known position.
-   It shows up clearly in the MR image and has a known geometry.
-   By detecting it in the image, we can figure out where the robot is relative to the scanner.
-   To register the robot with the MRI image, we need to know two things:

1.  **ZFrame to Scanner transform:** This transform is calculated from the MRI image after detecting the ZFrame (using ZFrameRegistration module).
2.  **ZFrame to Robot transform:** This is a fixed transform based on how the Z-Frame is mounted on the robot.

It is defined inside the robot's URDF under \<custom_parameters\>:
>
> \<zframe_position value=\"\...\"/\>
> \<zframe_orientation value=\"\...\"/\>

![](images/image1.gif)

# SlicerROS2 Setup - Ubuntu 24.04 (ROS2 Jazzy)

These steps are not necessary when the provided Docker image, but if you
try to implement the demo in your our system, you might want to know the
requirements:

## ROS2 Jazzy

-   Download and install ROS2Jazzy - [Instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
-   Other requirements:

> sudo apt install ros-dev-tools sudo apt install ros-jazzy-xacro sudo
> apt install ros-jazzy-tf2-geometry-msgs

-   Download [SmartTemplate_demo](https://github.com/maribernardes/ros2_smart_template_demo) in your ROS2 workspace src folder
    


## 3D Slicer with SlicerROS2

To use SlicerROS2, we must use a compiled version of 3D Slicer.

_3D Slicer compilation:_

-   Download [3D Slicer source code](https://github.com/Slicer/Slicer.git) (v. 5.9.0)
-   Other requirements:

~~~
sudo apt update && sudo apt install git git-lfs build-essential \\
libqt5x11extras5-dev qtmultimedia5-dev libqt5svg5-dev qtwebengine5-dev
libqt5xmlpatterns5-dev qttools5-dev qtbase5-private-dev \\ libxt-dev
~~~

-   Compile 3D Slicer - [Instructions](https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html#build-slicer)
    
Attention: Before you start compiling 3D Slicer, make sure to use the
system/native OpenSSL libraries; otherwise, you'll get some errors when
compiling the SlicerROS2 module. You will need to do the following after
you ran CMake for the first time:

-   In the Slicer-build directory, set Slicer_USE_SYSTEM_OpenSLL to ON
    using:
~~~~
cmake ../Slicer -DSlicer_USE_SYSTEM_OpenSSL=ON -DCMAKE_BUILD_TYPE=Release
~~~~

_SlicerROS2 compilation:_

-   Download SlicerROS2 source in your ROS2 worspace src folder: <https://github.com/rosmed/slicer_ros2_module>
-   Other requirements:

~~~~
> sudo apt install python3-colcon-common-extensions sudo apt install
> ros-jazzy-object-recognition-msgs sudo apt install ros-jazzy-moveit
~~~~
-  Compile SlicerROS2 module - [Instructions](https://slicer-ros2.readthedocs.io/en/latest/pages/gettingstarted.html#compilation)


_Other required Slicer Modules for this tutorial:_

> SlicerDevelopmentToolbox:
> [https://github.com/QIICR/SlicerDevelopmentToolbox.git](https://github.com/QIICR/SlicerDevelopmentToolbox.git)
>
> ZFrameRegistration (modified to include different ZFrame model
> selection):
> [https://github.com/maribernardes/ZFrameRegistration-3DSlicer.git](https://github.com/maribernardes/ZFrameRegistration-3DSlicer.git)

# Instructions

## Preparation: Launch SmartTemplate and 3D Slicer

-   Download the tutorial provided files:
    -   Two MR image files:
        -   AX_T1_VIBE_fs_tra_320.nrrd
        -   COR_TSE_T2_COVER_ZFRAME.nrrd
    -   ReachableVolume.mrk.json
    -   python_console_commands.txt
-   On the terminal, source the ROS2 workspace contains your SmartTemplate_demo build: source path_to_ros2_ws/install/setup.bash
-   Now, run the Slicer application from the command line.
-   In another terminal window, source your ROS2 workspace again and
    launch the robot:

~~~~
source path_to_ros2_ws/install/setup.bash
~~~~

> ros2 launch smart_template_demo robot.launch.py

## Step 1: Load MR images and register ZFrame fiducials

-   Load MR images in 3DSlicer (drag the provided files and drop at the 3D Slicer interface)
-   Open \"ZFrameRegistrationWithROI\" module

> ![](images/image2.jpg)

-   Select the horizontal ZFrame model
-   Select the \"COR TSE T2 COVER ZFrame\" volume
-   Select index of slices 9 to 12
-   Define the ROI in the coronal view (green viewer) that appropriately covers the fiducials
-   Click the ✓ button

![](images/image3.jpeg)

-   Observe the resultant Linear Transform node that was created to
    represent the transform from ZFrame to Scanner coordinates:

![](images/image4.jpg) ![](images/image5.jpg)

## Step 2: Load SmartTemplate robot in 3DSlicer

-   Open the \"ROS2\" module in Slicer:

![](images/image6.jpg)

-   Click the \"+ Add new robot\" button:

> ![](images/image7.jpg)

-   Configure parameters and click button \"Load robot\"
    -   Robot name: smart_template
    -   Parameter node name: /robot_state_publisher
    -   Parameter name: robot_description
    -   Fixed frame: world

![](images/image8.jpg)

-   Observe the SmartTemplate robot loaded in the 3D view:

> ![](images/image9.jpg)

-   Using the SmartTemplate GUI, move the robot using the arrow buttons
    and observe the respective motion in Slicer

> ![](images/image10.png)

horizontal

vertical

insertion

incremental values

absolute values

## Step 3: Register SmartTemplate to the scanner (world)

-   Open the python console (click the button in the menu bar)

> ![](images/image11.jpg)

-   Use the provided python commands in the console. They will:
    -   Get the robot node from the scene
    -   Get the robot_description from the ROS2 topic publisher by SmartTemplate
    -   Recover ZframeToRobot information from the URDF custom parameters
    -   Use the ZframeToScanner registration to calculate the final RobotToScanner transform (robot world pose)
    -   Create a ROS2 publisher and publish to the \\world_listener topic (which is subscribed by SmartTemplate to send a tf_static_broadcast to update the tf tree)
-   In the ROS2 terminal where the SmartTemplate was launched, you can read an indication that the world_pose_listener node was triggered by the \\world_pose topic published by SlicerROS2, and this caused the tf_static_broadcaster to update the world transform:

\[INFO\]\[\<timestamp_value\>\]\[world_pose_listener\]: Updated static
transform world -\> base_link published.

-   The SmartTemplate pose will automatically update to the registered pose with respect to the world (scanner). Also, observe the publisher we created in the SlicerROS2 GUI and the topic message just sent:

![](images/image12.jpg) ![](images/image13.jpg)

**Step 4: Make a straight needle insertion using the robot GUI**

-   Open the \"Models\" module

![](images/image14.jpg)

-   Select the needle_link model and edit its color and Slice Visibility:

![](images/image15.jpg)

-   Now, turn on the visualization of the provided \"AX T1 VIBE\" image so that you can see it in the axial, coronal, sagittal and 3D viewers:

![](images/image16.jpeg)

-   Using the SmartTemplate GUI, change the insertion value from 5.0 to 100.0 mm and click the + button to insert the needle. Observe the insertion as it progresses by scrolling through the MRI volume slices and rotating the 3D view:

![](images/image17.png)

-   Now use the SmartTemplate GUI to fully retract the needle by clicking the \"RETRACT\" button and observe the updates in the 3D Slicer viewers

## Step 5: Make a targeted insertion using SlicerROS2 publishers

-   Load the ReachableVolume ROI in 3D Slicer (drag the provided file and drop it at 3D Slicer interface)
-   Open the \"Markups\" Module and click on the button to add a Point List:

![](images/image18.jpg)

-   It will generate a Point List automatically named \"F\"
-   Now, let's include one point to the list by clicking in the MR image in the axial viewer (red). Select a target within the reachable volume to the needle insertion.
-   In our example, we selected (7.6, 27.0, -140.0) in RAS coordinates:

![](images/image19.jpg)

-   After the target point \"F-1\" is defined, use the provided python
    commands in the console. They will:
    -   Create a /desired_position publisher
    -   Calculate the target in robot coordinates
    -   OBS: We could publish the desired target in world coordinates and let the robot node deal with the conversion to the robot\'s base coordinate frame (base_link) using tf. However, due to the nature of our needle insertion application, we aim to decouple the horizontal and vertical alignment of the needle (Phase 1) from the needle insertion motion (Phase 2). For this reason, we chose to express the target in base_link coordinates, allowing us to compute the alignment position more directly and independently.

| **Phase 1: Needle alignment** | **Phase 2: Needle insertion** |
|:--------------------------------|:--------------------------------|
| Align the SmartTemplate with the target | Insert the needle to the target depth |
| ![](images/image20.jpg) | ![](images/image21.jpg) |

-   Observe the final needle placement in all planes to confirm the
    correct needle placement:

![](images/image22.png)

## Step 6: Read needle position using SlicerROS2 subscriber

-   In the python console, create a subscriber for /end_effector_pose
    topic and print the last message:

> subEEPose = rosNode.CreateAndAddSubscriberNode(\'PoseStamped\',
> \'/end_effector_pose\') print(subEEPose.GetLastMessage())

-   You can also visualize the last messages on the SlicerROS2 module.

-   Click the subscribed topic to see last message and compare with
    desired target.

-   Observe that PoseStamped messages are displayed by the dialog with
    position in meters and orientation in quaternion, while the
    vtkPoseMessage printed in the python console displays the
    homogeneous transform with translation in mm

> ![](images/image23.jpg)

**Step 7: Send a robot command using SlicerROS2 publishers**

-   In the python console, create a publisher for /desired_command
    topic:

pubCommand = rosNode.CreateAndAddPublisherNode(\'String\',
\'/desired_command\')

-   Send \"RETRACT\" message and observe the robot fully retract the
    needle:

> pubCommand.Publish(\'RETRACT\')
>
> ![](images/image24.jpg)

-   Send \"HOME\" message and observe the robot go to its initial
    position:

pubCommand.Publish(\'HOME\')

> ![](images/image25.jpg)
