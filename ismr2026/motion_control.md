---
layout: page
title: Motion Control with SlicerROS2
permalink: /ismr2026/motion_control.html
---

# Overview

In this interactive session, you will launch a simulated Universal Robots UR5 arm with MoveIt motion planning, connect it to 3D Slicer through the SlicerROS2 module, load patient anatomy and a custom end-effector into the scene, and plan trajectories that interact with the anatomical models using the new ROS2 Motion Control module. 
**Prerequisites:** Complete the environment setup in the [Prerequisites](prerequisites.html) page and launch a vast.ai container as described in [Launching a Container on Vast.ai](container.html). All commands below are run inside Konsole terminals in the vast.ai desktop environment.

---

## Part 1: Launch the Universal Robot Example

### Step 1: Open a Konsole Terminal

From the vast.ai desktop, open the applications menu and launch **Konsole**.

![KDE applications menu with Konsole highlighted on the vast.ai desktop](images/interactive_session_1_open_konsole.png)

### Step 2: Launch the UR Robot Driver

In the Konsole terminal, run the UR robot driver with mock hardware enabled. This starts the controller stack for a simulated UR5 without requiring a physical robot:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5 \
    robot_ip:=192.168.56.101 \
    use_mock_hardware:=true \
    launch_rviz:=false
```

![Konsole terminal showing the ur_control.launch.py command being entered](images/interactive_session_2_ur_driver_command.png)

Wait for the controller manager to finish loading. You should see log output indicating that the `passthrough_trajectory_controller`, `freedrive_mode_controller`, and `tool_contact_controller` have been loaded and configured.

![Terminal output showing the UR controllers loading and configuring successfully](images/interactive_session_3_ur_driver_output.png)

### Step 3: Launch MoveIt in a Second Terminal

Open a second Konsole terminal and launch the MoveIt configuration for the UR5:

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5 \
    use_mock_hardware:=true
```

![Second Konsole terminal running the ur_moveit.launch.py command](images/interactive_session_4_moveit_command.png)

### Step 4: Confirm the RViz MoveIt Viewer Opens

The MoveIt launch brings up an RViz window with the MoveIt MotionPlanning panel. You should see the UR5 robot model rendered in the 3D view with the MotionPlanning controls on the left.

![RViz window showing the UR5 robot with the MoveIt MotionPlanning panel](images/interactive_session_5_rviz_moveit.png)

### Step 5: Locate the MotionPlanning Panel

The lower-left portion of the RViz window contains the MotionPlanning panel, with tabs for Context, Planning, Joints, Scene Objects, and others. This is where you will configure the planner in the next step.

![RViz window with the MotionPlanning panel highlighted in the lower-left](images/interactive_session_6_motion_planning_panel.png)

### Step 6: Select the CHOMP Planning Library

Switch to the **Context** tab of the MotionPlanning panel. In the **Planning Library** dropdown, change the selection from `ompl` to `chomp`. The label at the top of the panel will update from OMPL to CHOMP once the change is applied.

![Three-panel view of the MotionPlanning Context tab showing the change from ompl to chomp](images/interactive_session_7_chomp_selection.png)

---

## Part 2: Launch SlicerROS2

### Step 7: Start 3D Slicer with SlicerROS2

Open a third Konsole terminal and launch Slicer with the SlicerROS2 module. The commands below deactivate the default conda environment, switch to the ROS 2 workspace, source the overlays, and start Slicer:

```bash
conda deactivate
cd ../home/user/ros2_ws/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch slicer_ros2_module slicer.launch.py
```

![Terminal running the slicer_ros2_module launch sequence](images/interactive_session_8_launch_slicer_command.png)

### Step 8: Confirm Slicer Launches

After a short load time, the 3D Slicer welcome screen appears with an empty 3D view.

![3D Slicer welcome screen with an empty scene](images/interactive_session_9_slicer_welcome.png)

### Step 9: Open the SlicerROS2 Module

Press **Ctrl+F** to open the Module Finder, then type `ROS2` into the search box. Select **ROS2** from the results and click **OK**.

![Slicer Module Finder dialog with ROS2 entered in the search field](images/interactive_session_10_find_ros2_module.png)

---

## Part 3: Load the Robot into Slicer

### Step 10: Add a New Robot

In the ROS2 module panel, click **+ Add new robot**. A configuration form appears with fields for the robot name, parameter node name, parameter name, fixed frame, and Tf2 prefix.

![ROS2 module panel with the +Add new robot button highlighted](images/interactive_session_11_add_new_robot.png)

### Step 11: Load the Robot

The default values (`robot1`, `/robot_state_publisher`, `robot_description`) match the running UR5 driver from Part 1. Click **Load robot** to connect Slicer to the robot description being published by ROS 2.

![ROS2 module with the Load robot button highlighted](images/interactive_session_12_load_robot.png)

### Step 12: Confirm the Robot Appears in the 3D View

The UR5 model is rendered in the Slicer 3D view. You should see the robot's base and first links positioned at the world origin.

![Slicer 3D view showing the UR5 base and shoulder loaded from the robot description](images/interactive_session_13_robot_loaded.png)

---

## Part 4: Switch to the ROS2 Motion Control Module

### Step 13: Open the ROS2 Motion Control Module

Press **Ctrl+F** again and search for `ROS2 Motion`. Select **ROS2 Motion Control** from the results and click **OK**.

![Module Finder dialog with ROS2 Motion Control selected](images/interactive_session_14_motion_control_module.png)

### Step 14: Select the Robot and Press Use

In the Motion Control module, make sure `ros2:robot:robot1` is selected in the **Robot Name** dropdown, then click **Use** to bind the module to the loaded robot.

![Motion Control panel with ros2:robot:robot1 selected and the Use button highlighted](images/interactive_session_15_select_robot.png)

### Step 15: Confirm the Full Robot Renders

Once bound, the full UR5 arm is rendered in Slicer with all joints and links visible.

![Slicer 3D view showing the complete UR5 robot after pressing Use](images/interactive_session_16_robot_in_motion_control.png)

### Step 16: Explore the Motion Control Tabs

Explore each of the tabs in the Motion Control panel: **Load Robot**, **Joint Control**, **3D Control**, and **Trajectory**. Each tab exposes a different interaction mode with the robot.

![Motion Control panel showing the tab row with the UR5 rendered in the 3D view](images/interactive_session_17_explore_tabs.png)

---

## Part 5: Add Patient Models and a Custom End-Effector

### Step 17: Clone the Tutorial Repository

Open a new Konsole terminal and clone the tutorial repository, which contains the spine model, custom end-effector geometry, and supporting data files:

```bash
cd ../home/user
git clone https://github.com/LauraConnolly/slicerros2_tutorial.git
```

![Terminal showing the cd and git clone commands](images/interactive_session_18_clone_tutorial_repo.png)

### Step 18: Open the Add Data Dialog

Back in Slicer, click the **Add Data** button in the toolbar (the icon at the top-left of the main toolbar).

![Slicer main window with the Add Data button highlighted in the toolbar](images/interactive_session_19_add_data_button.png)

### Step 19: Choose Files to Add

In the Add Data dialog, click **Choose File(s) to Add**.

![Add Data dialog with the Choose File(s) to Add button highlighted](images/interactive_session_20_choose_files.png)

### Step 20: Select the Spine and End-Effector Bundle

Navigate to `/home/user/slicerros2_tutorial/data/` and select **Spine_and_EE.mrb**. Confirm the selection and click **OK**.

![File browser showing Spine_and_EE.mrb selected, with the Add Data dialog confirming the file](images/interactive_session_21_navigate_data_folder.png)

### Step 21: Confirm the Data Loads

After the bundle loads, the 3D view shows the UR5 robot together with the spine model and custom end-effector geometry near its base.

![Slicer 3D view with the UR5, spine, and custom end-effector all rendered](images/interactive_session_22_data_loaded.png)

---

## Part 6: Attach the End-Effector to the Robot Transform Tree

### Step 22: Open the Data Module

Press **Ctrl+F** again and search for the **Data** module. Select **Data** from the results and click **OK**.

![Module Finder dialog with the Data module selected](images/interactive_session_23_open_data_module.png)

### Step 23: Switch to the Transform Hierarchy Tab

In the Data module, click the **Transform hierarchy** tab to expose the scene's transform tree. This view lets you reparent models under specific robot links so they move with the robot.

![Data module with the Transform hierarchy tab highlighted](images/interactive_session_24_transform_hierarchy_tab.png)

### Step 24: Drag Each Model onto Its Target Transform

Expand the robot transform tree and drag each loaded end-effector model onto the corresponding goal transform:
- ur5_stylus → `ros2:tf2_lookup:wrist_3_linkToFlange`
- ur5_stylus_goal → `tool0_goal_transform`

Once the models are reparented, they follow the robot as its pose updates. Note that in practice these should be added through the urdf file, this is a quick workaround for the purpose of this tutorial.

![Transform hierarchy showing the expanded goal-transform tree with models attached](images/interactive_session_25_drag_models_to_transforms.png)

### Step 25: Return to the ROS2 Motion Control Module

Click the **recent modules** button (the small dropdown arrow next to the module navigation buttons in the toolbar) and select **ROS2 Motion Control** to switch back without re-searching.

![Recent modules dropdown showing ROS2 Motion Control in the list](images/interactive_session_26_recent_modules.png)

---

## Part 7: Plan Motion Around the Patient Models

### Step 26: Plan a Trajectory to the Spine

With the end-effector attached and the spine in the scene, you can now plan a trajectory that touches key points on the spine model. Use the **3D Control** tab of the Motion Control panel and the interactive goal marker in the 3D view to position the target, then trigger planning.

![Slicer 3D view with the UR5, interactive goal marker, and spine model positioned for trajectory planning](images/interactive_session_27_motion_planning_spine.png)

---

# Summary

In this session, you:
- Launched a simulated UR5 with the UR robot driver and MoveIt using mock hardware
- Switched the MoveIt planner to CHOMP from the Context tab
- Connected 3D Slicer to ROS 2 through the SlicerROS2 module and loaded the robot description
- Bound the robot to the ROS2 Motion Control module for interactive control
- Added a spine anatomical model and custom end-effector to the Slicer scene
- Used the module to plan robot motions relative to the anatomical model

<div class="tutorial-nav"><a href="index.html" class="btn-secondary">Back to Workshop Page</a></div>
