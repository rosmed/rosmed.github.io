---
layout: page
title: "Follow-along: Motion planning with SlicerROS2 and MoveIt"
permalink: /embc2026/tutorial_part3.html
---

# Tutorial Part 3: Motion planning with SlicerROS2 and MoveIt

## Overview

In this portion of the workshop, users will get to try the new "Motion Control" module in SlicerROS2 to define trajectories relative to patient images. You will launch a simulated Universal Robot, load it into 3D Slicer, add a patient spine model and a custom end-effector, and use the Motion Control module to plan a trajectory around the patient anatomy.

**Prerequisites:** Complete the environment setup in the [Prerequisites](https://rosmed.github.io/ismr2026/prerequisites.html) page and launch a vast.ai container as described in [Launching a Container on Vast.ai](https://rosmed.github.io/ismr2026/container.html). All commands below are run inside Konsole terminals in the vast.ai desktop environment.

---

## Part 6: Launch the Universal Robot Simulation

### Step 56: Open a Konsole Terminal

From the vast.ai desktop, open the applications menu and launch **Konsole**.

![Applications menu open on the vast.ai desktop with Konsole highlighted](images/embc_56_open_konsole_terminal.png)

### Step 57: Launch the Universal Robot Example

In the terminal, navigate to the ROS 2 workspace, source the workspace overlay, and launch the Universal Robot simulation:

```
cd ../home/user/ros2_ws/
source install/setup.bash
ros2 launch slicer_ros2_module ur_sim_control.launch.py
```

![Konsole terminal showing the commands used to launch the Universal Robot simulation](images/embc_57_launch_ur_example.png)

### Step 58: Confirm the Simulation Starts

Wait for the launch output to settle. You should see the controller manager load the joint trajectory, freedrive mode, and tool contact controllers, finishing with a message that the spawner process completed cleanly.

![Terminal output showing the Universal Robot controllers loading successfully](images/embc_58_confirm_ur_launches.png)

---

## Part 7: Launch SlicerROS2 and Load the Robot

### Step 59: Launch SlicerROS2

Open a new Konsole tab, source the workspace again, and launch 3D Slicer with the SlicerROS2 module:

```
cd ../home/user/ros2_ws/
source install/setup.bash
ros2 launch slicer_ros2_module slicer.launch.py
```

![Konsole terminal showing the commands used to launch SlicerROS2](images/embc_59_launch_slicerros2.png)

### Step 60: Confirm Slicer Opens

3D Slicer should open to the default Welcome module with an empty Four-Up layout.

![3D Slicer running with the default Welcome module and an empty scene](images/embc_60_confirm_slicer_opens.png)

### Step 61: Open the Module Finder

Press **Ctrl+F** to open the Module Finder and search for **ros2**.

![Slicer Module Finder with "ros2" entered in the search field, listing Motion Control, Tests, and Slicer ROS](images/embc_61_module_finder_ros2.png)

### Step 62: Open the Slicer ROS 2 Panel

Select **Motion Control** to load the module, then open the **Slicer ROS 2** collapsible section near the bottom of the panel.

![Motion Control module panel with the collapsed Slicer ROS 2 section at the bottom](images/embc_62_slicer_ros2_panel.png)

### Step 63: Add a New Robot

With the **Robots** tab selected, press **+ Add New Robot**.

![Slicer ROS 2 panel with the Robots tab open and the + Add New Robot button visible](images/embc_63_add_new_robot.png)

### Step 64: Load the Robot

Leave the default **Robot Name** (`robot1`), **Parameter Node Name**, and **Parameter Name** fields as they are, and press **Load Robot**.

![Slicer ROS 2 panel with the new robot's fields filled in and the Load Robot button ready](images/embc_64_load_robot.png)

### Step 65: Confirm the Robot Loads

The **Robot** dropdown at the top of the module now shows **`ros2:robot:robot1`**, and the robot model appears in the 3D view.

![Motion Control module with `ros2:robot:robot1` selected and the robot visible in the 3D view](images/embc_65_confirm_robot_loads.png)

### Step 66: Use the Selected Robot

Make sure **`ros2:robot:robot1`** is selected in the **Robot** dropdown, then press **Use Selected Robot**.

![Robot tab with `ros2:robot:robot1` selected and the Use Selected Robot button ready to press](images/embc_66_use_selected_robot.png)

### Step 67: Confirm the Robot Controls Appear

The **Robot Appearance**, **Joint State**, and **MoveIt** sections expand, and the robot is now fully visible in the 3D view.

![Motion Control module with the Robot Appearance, Joint State, and MoveIt sections expanded](images/embc_67_confirm_robot_controls.png)

---

## Part 8: Explore the Motion Control Module

### Step 68: Explore Each of the Tabs

Take a moment to click through the **Robot**, **Joint Control**, **3D Control**, **Trajectory**, and **Obstacles** tabs to see what each one offers.

<video controls muted playsinline width="100%" poster="images/embc_68_explore_tabs_poster.png">
  <source src="images/embc_68_explore_tabs.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

---

## Part 9: Add Patient Models and a Custom End-Effector

### Step 69: Clone the Tutorial Data

Open a new Konsole terminal and clone the repository containing the patient model and end-effector data:

```
cd ../home/user
git clone https://github.com/LauraConnolly/slicerros2_tutorial.git
```

![Konsole terminal showing the git clone command and its output](images/embc_69_clone_tutorial_data.png)

### Step 70: Load the Data

Back in Slicer, click the **Add Data** button, then select **Choose File(s) to Add** in the dialog that appears.

![Add data into the scene dialog with Choose Directory to Add and Choose File(s) to Add buttons](images/embc_70_load_the_data.png)

### Step 71: Select the Data Files

Navigate to the `data` folder inside the repository you just cloned (`/home/user/slicerros2_tutorial/data`) and select **Spine_and_EE.mrb**.

![File browser navigated to the slicerros2_tutorial data folder with Spine_and_EE.mrb visible](images/embc_71_select_data_files.png)

### Step 72: Confirm the Load

The MRB bundle is listed in the Add Data dialog. Press **OK** to load it into the scene.

![Add data dialog listing Spine_and_EE.mrb, ready for the OK button to be pressed](images/embc_72_confirm_data_load.png)

### Step 73: Confirm the Data Appears

The spine model and the custom end-effector model are now loaded into the scene, though not yet attached to the robot.

![3D view showing the robot with the spine and end-effector models loaded nearby](images/embc_73_confirm_data_appears.png)

### Step 74: Open the Data Module

Press **Ctrl+F** again and search for the **Data** module.

![Module Finder with "data" entered, listing the Data, Data Probe, and Sample Data modules](images/embc_74_open_data_module.png)

### Step 75: Open the Transform Hierarchy Tab

In the Data module, press the **Transform hierarchy** tab.

![Data module with the Subject hierarchy tab open, showing the Transform hierarchy tab to select](images/embc_75_transform_hierarchy_tab.png)

### Step 76: Expand the Transform Trees

Expand the robot's transform tree so you can see the individual link transforms, along with the loose **spine** and end-effector (**Transform**) nodes at the bottom of the list.

![Transform hierarchy tab with the robot's transform tree expanded and the spine and Transform nodes listed below it](images/embc_76_expand_transform_trees.png)

### Step 77: Drag Each Model to Its Location

Drag the **spine** node onto the appropriate frame in the transform tree, and drag the end-effector transform onto the robot's tool flange. Each model now moves and tracks with its assigned link.

<video controls muted playsinline width="100%" poster="images/embc_77_drag_models_poster.png">
  <source src="images/embc_77_drag_models.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

### Step 78: Return to the Motion Control Module

Press the recent modules button in the toolbar and select **Motion Control** to switch back.

![Recent modules dropdown showing Data, Motion Control, and Welcome to Slicer](images/embc_78_return_motion_control.png)

---

## Part 10: Motion Planning Around Patient Models

### Step 79: Add the Spine as an Obstacle

Select the **Obstacles** tab, then choose **spine** from the **Model Node** dropdown.

![Obstacles tab with the Model Node dropdown open, showing the spine model in the list](images/embc_79_add_spine_obstacle.png)

### Step 80: Confirm the Obstacle Is Added

Press **Add to MoveIt**. The spine model now appears in the obstacle table, and MoveIt begins planning around it.

![Obstacles tab showing the spine model added to the obstacle table, with planning handles visible in the 3D view](images/embc_80_confirm_obstacle_added.png)

### Step 81: Plan a Trajectory

With the spine registered as an obstacle, use the interactive handles to plan a trajectory that touches key points on the spine model while avoiding collisions.

<video controls muted playsinline width="100%" poster="images/embc_81_plan_trajectory_poster.png">
  <source src="images/embc_81_plan_trajectory.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

---

# Summary

In this session, you:

- Launched a simulated Universal Robot and connected it to 3D Slicer through SlicerROS2
- Loaded the robot into the Motion Control module and confirmed its controls
- Explored the Robot, Joint Control, 3D Control, Trajectory, and Obstacles tabs
- Loaded a patient spine model and a custom end-effector, and attached them to the robot's transform tree
- Registered the spine model as an obstacle in MoveIt
- Planned a robot trajectory that touches key points on the patient anatomy while avoiding collisions

<div class="tutorial-nav"><a href="tutorial_part2.html" class="btn-secondary">Back to Part 2</a><a href="tutorial_part4.html" class="btn-primary">Go to Next Step →</a></div>
