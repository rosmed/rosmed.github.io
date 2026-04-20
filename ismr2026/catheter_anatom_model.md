---
layout: page
title: Simulating Catheter Navigation through Anatomy
permalink: /ismr2026/catheter_anatom_model.html
---

# Overview

In this tutorial, we combine the anatomical models created in [Creating an Anatomical Model](anatomical_model.html) with the catheter simulation from [Creating and Testing a Catheter Model](catheter_model.html). The catheter generator script accepts a `--slicer-scene` argument that reads the saved anatomy scene and embeds the STL mesh files as static bodies in the Gazebo world. This allows the catheter to be navigated through a patient-specific anatomical environment and visualized simultaneously in Gazebo, RViz, and 3D Slicer.

**Prerequisites:** Complete both [Creating an Anatomical Model](anatomical_model.html) and [Creating and Testing a Catheter Model](catheter_model.html) before starting this tutorial. You will need the anatomy scene file saved at `~/Documents/anatomy_scene/`.

---

## Part 1: Regenerate the Catheter Package with Anatomy

### Step 1: Generate the Catheter Package with the Anatomy Scene

Navigate to the ROS2 workspace source directory and run the catheter generator with the `--slicer-scene` flag pointing to the anatomy scene saved in the previous tutorial:

```bash
ls ~/Documents/anatomy_scene # Check the MRML file name
cd ~/ros2_ws/src
python3 ~/flexible_catheter_simulation/catheter_generator.py \
    --controller \
    --N 50 --D 0.003 --L1 0.04 --L2 0.5 --L3 0.01 \
    --K 10.0 --Kd 0.1 --Kf 0.01 --M 0.5 \
    --output control_catheter_test \
    --slicer-scene ~/Documents/anatomy_scene/2026-04-16-Scene.mrml # Replace the MRML file name with the one found above
```

The `--slicer-scene` argument reads the anatomy scene file and includes the STL meshes (heart, aorta, vessels, etc.) as static entities in the Gazebo world. The script prints the list of anatomy files it found and includes them as `anatomy_link_*` bodies in the generated package.

![Terminal showing catheter_generator.py command with --slicer-scene flag and anatomy file list](images/catheter_anatom_1_generate_catheter.png)

The generated package now contains both the catheter URDF and anatomy mesh files, with each anatomical structure added as a static link.

![Terminal showing the full list of generated anatomy and catheter files, followed by colcon build starting](images/catheter_anatom_2_colcon.png)

### Step 2: Build the Workspace

```bash
cd ..
colcon build
```

Wait for the build to complete, then launch the simulation (see below).

---

## Part 2: Launch the Simulation with Anatomy

### Step 3: Launch the Catheter Simulation

Source the workspace and launch:

```bash
source install/setup.bash
ros2 launch control_catheter_test control_catheter_test_launch.py
```

![Terminal showing the ros2 launch command](images/catheter_anatom_3_launch_catheter.png)

Gazebo Sim and RViz will open. The Gazebo Entity Tree now lists all the anatomical structures alongside the catheter (`heart`, `aorta`, `pulmonary_vein`, `brachocephalic_trunk`, `right_subclavian_artery`, etc.). RViz shows the full anatomy rendered in 3D.

![Gazebo entity tree showing anatomy bodies and RViz showing 3D anatomy with catheter](images/catheter_anatom_4_rviz_gazebo.png)

### Step 4: Explore the Views

Zoom into the Gazebo viewport to see the catheter (thin line) positioned near the anatomical models.

![Gazebo zoomed in showing catheter line alongside the anatomy](images/catheter_anatom_5_gazebo_zoom.png)

Continue zooming to see the catheter and anatomy detail. The catheter starts above the anatomy at its default spawn position.

![Further zoomed Gazebo and RViz views showing catheter and anatomy alignment](images/catheter_anatom_6_gazebo_zoom.png)

In RViz, the anatomy is rendered as colored surface models with the catheter visible as a thin line entering from below. In Gazebo, the anatomy may be hidden under the ground (z=0) and invisible until bringing the view point below the ground. 

![RViz zoomed view showing heart and vessels with catheter entering from below](images/catheter_anatom_7_rviz_zoom.png)

---

## Part 3: Position the Catheter Below the Anatomy

The catheter spawns at the world origin by default, which may place it inside or above the anatomy. Use the `initial_z` launch argument to shift the catheter's starting position downward so it begins below the anatomy for a realistic insertion simulation.

### Step 5: Relaunch with an Initial Z Offset

Stop the current simulation (Ctrl+C in the launch terminal), then relaunch with the offset:

```bash
ros2 launch control_catheter_test control_catheter_test_launch.py initial_z:=-0.9
```

![Terminal showing the relaunch command with initial_z:=-0.5 argument](images/catheter_anatom_8_launch_catheter_w_offset.png)

### Step 6: Launch the Teleop Controller

In the second terminal, launch the keyboard teleop:

```bash
ros2 run control_catheter_test catheter_keyboard_teleop.py
```

![RViz showing anatomy from above with catheter starting below, second terminal launching teleop](images/catheter_anatom_9_launch_teleop.png)

### Step 7: Navigate the Catheter into the Anatomy

Use the keyboard controls to insert and steer the catheter:

```
W / S   → Insert / Retract  (Z trans)
A / D   → CCW    / CW       (Z rot)
↑ / ↓   → Y+     / Y-       (Y trans)
← / →   → X+     / X-       (X trans)
Q       → Quit
```

Use `↑`, `↓`, `←`, `→` keys to align the catheter to the aorta entry (the red large vessel). Once it is aligned, press **W** repeatedly to advance the catheter upward into the anatomy. The RViz view will show the catheter bending and advancing through the anatomical structures in real time.

![RViz showing catheter inserted into the heart anatomy from below, bending inside the vessel](images/catheter_anatom_10_align_catheter.png)

---

## Part 4: Visualize in 3D Slicer

### Step 8: Source the Workspace and Start Slicer

Open a new terminal, source `~/ros2_ws/install/setup.bash`, and start 3D Slicer:

```bash
cd # Move to the home directory
source ros2_ws/install/setup.bash
./start-slicer-ros2.bash
```

![Terminal showing source and start-slicer-ros2.bash commands with simulation running in background](images/catheter_anatom_11_source.png)

Wait for Slicer to fully launch.

![Terminal showing Slicer startup output with simulation continuing in background](images/catheter_anatom_12_launch_slicer.png)

### Step 9: Open the ROS2 Module

In 3D Slicer, go to **Modules → IGT → ROS2**.

![Slicer Modules menu with ROS2 highlighted under IGT](images/catheter_anatom_13_open_ros2.png)

### Step 10: Add and Load the Robot

Click **+ Add new robot** and fill in the connection fields:
- **Robot name**: `robot1`
- **Parameter node name**: `/robot_state_publisher`
- **Parameter name**: `robot_description`
- **Fixed frame**: `world`
- **Tf2 prefix**: (leave empty)

Then click **Load robot**.

![ROS2 module with robot configuration filled in and Load robot button visible](images/catheter_anatom_14_add_new_robot.png)

After loading, the form fields become grayed out and the Slicer 3D view displays both the anatomical models and the catheter robot model together.

![ROS2 module after load with 3D view showing anatomy and catheter combined](images/catheter_anatom_15_load.png)

### Step 11: Adjust the View Angle

Rotate the 3D view to get a good perspective on the catheter inside the anatomy. You can see the heart, aorta, and other vessels rendered as solid surface models, with the catheter model running through the aorta.

![Slicer 3D view showing heart and vessels with catheter entering from below at a good viewing angle](images/catheter_anatom_16_view_angle.png)

---

## Part 5: Manage Model Visibility and Opacity

### Step 12: Open the Models Module

Go to **Modules → Models** to manage the visibility and opacity of individual anatomy and catheter model nodes.

![Slicer Modules menu with Models highlighted, 3D view showing anatomy and catheter](images/catheter_anatom_17_open_models.png)

The Models module lists all nodes in the scene, including:
- `anatomy_link_heart_model`, `anatomy_link_aorta_model`, `anatomy_link_pulmonary_vein_model`, and other vessel models
- `base_origin_model`, `bending_link_*_model` nodes for each catheter segment


### Step 13: Adjust Anatomy Opacity

Select an anatomy model node (i.e., `anatomy_link_*_model`) and adjust the **Opacity** slider in the Display panel to `0.5`. The Display panel is below the node list, and may need to be scrolled down to be visible. Reducing the opacity of the anatomy models makes the catheter path visible through the vessel walls.

![Models module showing the full list of anatomy and catheter model nodes](images/catheter_anatom_18_select_models.png)


![Models module with a model selected and the Opacity slider visible in the Display panel](images/catheter_anatom_19_change_opacity.png)

---

## Part 6: Teleoperate Catheter Insertion in Slicer

### Step 14: Insert the Catheter

With Slicer open and the robot loaded, return to the teleop terminal and use the keyboard to advance the catheter. The 3D Slicer view updates in real time as the catheter moves, showing the insertion path through the anatomy.

![Slicer 3D view showing catheter inserted into the anatomy with teleop terminal visible](images/catheter_anatom_20_insert.png)

Continue inserting to navigate deeper into the anatomy. The catheter segments bend dynamically as it collide with the vessel wall.

![Slicer 3D view showing catheter advanced further into the anatomy from a different angle](images/catheter_anatom_21_insert.png)

---

# Summary

In this tutorial, you:
- Used `--slicer-scene` to embed patient anatomy into the Gazebo simulation
- Launched a combined catheter + anatomy simulation in Gazebo and RViz
- Used `initial_z` to position the catheter entry point below the anatomy
- Connected 3D Slicer to the simulation via SlicerROS2
- Navigated the catheter through the anatomical models using keyboard teleop, visualized simultaneously across Gazebo, RViz, and Slicer

[Back to Workshop Page](index.html) | [Go to Next Step](motion_control.html)
