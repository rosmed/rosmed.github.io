---
layout: page
title: Creating a Catheter Model for SlicerROS2
permalink: /ismr2026/catheter_model.html
---

# Overview

In this tutorial, we will simulate a flexible catheter in the Gazebo dynamic simulator and visualize it in both RViz and 3D Slicer via the SlicerROS2 module. Because Gazebo is primarily designed for rigid body simulation and does not natively support soft-body objects, we model a flexible catheter as a serial robot connected via universal joints and rotary springs.

A Python script is used to generate a URDF file that defines this serial robot from a set of physical parameters (length, stiffness, damping, etc.). The generated ROS2 package can then be built and launched directly.

The workflow has four main stages:
1. Clone the repository and generate a catheter model
2. Build the ROS2 workspace and launch the simulation
3. Control the catheter with keyboard teleop
4. Visualize the catheter in 3D Slicer via SlicerROS2

---

## Part 1: Clone the Repository and Generate a Catheter Model

### Step 1: Clone the Flexible Catheter Simulation Repository

Open a terminal in the Linux desktop and clone the repository:

```bash
cd  # Move to the home directory
git clone https://github.com/rosmed/flexible_catheter_simulation
```

![Terminal showing the git clone command](images/catheter_1_git_clone.png)

Wait for the clone to complete. The repository will be downloaded to `~/flexible_catheter_simulation`.

![Terminal showing the clone completed successfully](images/catheter_2_git_clone.png)

### Step 2: Source the ROS2 Workspace

Navigate to the ROS2 workspace and source the setup file:

```bash
cd ros2_ws
source install/setup.bash
```

![Terminal showing cd ros2_ws and source install/setup.bash](images/catheter_3_ros_source.png)

### Step 3: Install Required Python Packages and Generate the Catheter Model

Install the required Python dependencies: 
```bash
pip install numpy
pip install catkin-pkg
```

![Terminal showing pip install and the catheter_generator.py command with parameters](images/catheter_4_install_numpy.png)


Run the catheter generator script to create the ROS2 package:
```bash
cd src  # Move to the home directory
python3 ~/flexible_catheter_simulation/catheter_generator.py \
    --controller \
    --N 12 --D 0.003 --L1 0.20 --L2 0.5 --L3 0.05 --K 0.2 --M 0.5 \ 
    --output control_catheter_test
```

The key parameters are:
- `--N`:  Number of segments
- `--D`:  Segment diameter (m)
- `--L1`: Base link length (m)
- `--L2`: Bending section total lengths (m)
- `--L3`: Tip link length (m)
- `--K`:  Joint spring stiffness (Nm/rad)
- `--M`:  Total catheter mass (Kg)
- `--Kd`: Joint damping coefficient
- `--Kf`: Joint friction coefficient


![Terminal showing the generated package files and the start of colcon build](images/catheter_5_generate_catheter.png)

The script generates a complete ROS2 package named `control_catheter_test` containing the URDF, launch files, world file, and teleop script.


---

## Part 2: Build the ROS2 Workspace and Launch the Simulation

### Step 4: Build with colcon

From the `ros2_ws` directory, build the workspace:

```bash
cd .. # Move to ~/ros2_ws 
colcon build
```

The build may take several minutes. Some deprecation warnings are expected and can be ignored.

![Terminal showing colcon build in progress](images/catheter_6_colcon.png)

Wait for the build to complete. The summary should show 3 packages finished.

![Terminal showing colcon build complete with summary](images/catheter_7_colcon.png)

### Step 5: Source the Updated Workspace

After building, source the workspace again to pick up the new package:

```bash
source install/setup.bash
```

![Terminal showing source install/setup.bash after build](images/catheter_8_source.png)

### Step 6: Open a Second Terminal and Source It

Open a new terminal tab (use **New Tab** in Konsole), navigate to the workspace, and source it:

```bash
cd ros2_ws # Move to ~/ros2_ws 
source install/setup.bash
```

Keep both terminals open — the first will run the simulation, the second will run the teleop controller.

![Two terminal windows side by side, both sourced](images/catheter_9_open_second_terminal.png)

### Step 7: Launch the Catheter Simulation

In the **first terminal**, launch the simulation:

```bash
ros2 launch control_catheter_test control_catheter_test_launch.py
```

![First terminal showing the ros2 launch command](images/catheter_10_launch_catheter.png)

Gazebo Sim and RViz will open. 

![Gazebo Sim and RViz windows opening side by side](images/catheter_11_rviz_gazebo.png)

Arrange and resize the windows so both are visible. 

![Windows resized to show Gazebo and RViz side by side](images/catheter_12_window_resize.png)

Zoom in on both Gazebo and RViz to see the catheter model.

![Zoomed-in views showing the catheter in Gazebo and RViz](images/catheter_13_zoom.png)

---

## Part 3: Control the Catheter with Keyboard Teleop

### Step 8: Launch the Keyboard Teleop Controller

In the **second terminal**, run the teleop script:

```bash
ros2 run control_catheter_test catheter_keyboard_teleop.py
```

![Second terminal showing the ros2 run teleop command](images/catheter_14_launch_teleop.png)

The Catheter Keyboard Teleop interface will appear in the terminal:

```
Catheter Keyboard Teleop
  W / S   → Insert  / Retract  (Z trans)
  A / D   → CCW     / CW       (Z rot)
  ↑ / ↓   → Y+      / Y-       (Y trans)
  ← / →   → X+      / X-       (X trans)
  Q       → Quit
```

![Catheter Keyboard Teleop control panel in terminal](images/catheter_15_teleop.png)

### Step 9: Control the Catheter

Click on the teleop terminal to give it keyboard focus, then use the keys shown to move the catheter base (at the bottom). Both Gazebo and RViz will update in real time as you press keys. As the catheter base moves quickly, the catheter bends slightly on both Gazebo and RViz.

![Gazebo and RViz showing catheter bending in response to teleop input](images/catheter_16_teleop_press_key.png)

---

## Part 4: Modify and Rebuild with Different Parameters (Optional)

You can regenerate the catheter model with different parameters and rebuild. Stop the running simulation (Ctrl+C in the first terminal), then:

### Step 10: Regenerate the Catheter Package

```bash
cd ~/ros2_ws/src
python3 ~/flexible_catheter_simulation/catheter_generator.py \
    --controller \
    --N 30 --D 0.003 --L1 0.10 --L2 0.5 --L3 0.05 \
    --K 1.0 --Kd 0.0001 --Kf 0.0005 --M 0.5 \
    --output control_catheter_test
```

![Terminal showing the regenerate command with updated parameters](images/catheter_17_tgenerate_catheter.png)

### Step 11: Rebuild

```bash
cd ..
colcon build
```

![Terminal showing colcon build running after regeneration](images/catheter_18_colcon.png)

### Step 12: Relaunch the Simulation

After the build finishes, source the workspace and relaunch:

```bash
source install/setup.bash
ros2 launch control_catheter_test control_catheter_test_launch.py
```

![Terminal showing the relaunch command after rebuild](images/catheter_19_launch_catheter.png)

Relaunch the teleop in the second terminal as before and press keys to move the catheter base. The shaft of the catheter looks more flexible than Step 9. 

![Gazebo, RViz, and teleop terminal all visible with updated catheter model](images/catheter_20_teleop.png)

---

## Part 5: Visualize the Catheter in 3D Slicer via SlicerROS2

### Step 13: Start 3D Slicer

Open a new terminal and start Slicer (keep the Gazebo simulation and teleop running):

```bash
./start-slicer-ros2.bash
```

![Terminal running start-slicer-ros2.bash with Gazebo and teleop visible in background](images/catheter_21_start_slicer.png)

### Step 14: Open the ROS2 Module in Slicer

Once Slicer opens, go to the Modules menu and navigate to **IGT → ROS2**.

![Slicer Modules menu with ROS2 highlighted under IGT](images/catheter_22_open_ros2.png)

### Step 15: Add a New Robot

In the ROS2 module panel, click **+ Add new robot**.

![ROS2 module panel showing the Add new robot button](images/catheter_23_add_new_robot.png)

### Step 16: Configure the Robot Connection

Fill in the robot connection fields:
- **Robot name**: `robot1`
- **Parameter node name**: `/robot_state_publisher`
- **Parameter name**: `robot_description`
- **Fixed frame**: `world`
- **Tf2 prefix**: (leave empty)

![ROS2 module robot configuration form with world entered in Fixed frame](images/catheter_24_enter_world.png)

### Step 17: Load the Robot

Click **Load robot** to connect to the running ROS2 simulation.

![Robot configuration form with Load robot button highlighted](images/catheter_25_load_robot.png)

After loading, the form fields will become grayed out, indicating the robot is connected.

![Robot form with fields grayed out after successful load](images/catheter_26_load_robot.png)

### Step 18: Switch to 3D Only View

To see the catheter model clearly, switch the Slicer layout to **3D only**. Click the layout selector button in the toolbar and choose **3D only**.

![Slicer layout selector dropdown with 3D only highlighted](images/catheter_27_change_view.png)

The 3D view will now show the catheter robot model as a line, matching the pose published by the ROS2 simulation.

![Slicer 3D only view showing the catheter model](images/catheter_28_change_view.png)

### Step 19: Compare the Catheter Shape on All Three Viewers

Click the teleop terminal and use the keyboard controls to move the catheter. Gazebo, RViz, and 3D Slicer will all update simultaneously, showing the catheter pose in real time across all three environments.

![Gazebo, RViz, and Slicer 3D view all showing the catheter with teleop terminal at bottom](images/catheter_29_teleop.png)

---

# Background and References

1. [SlicerROS2 Repository](https://github.com/rosmed/slicer_ros2_module)
2. [SlicerROS2 Documentation](https://slicer-ros2.readthedocs.io/en/v1.0/)
3. [SlicerROS2 Tutorials](https://rosmed.github.io/tutorials/)
4. [Flexible Catheter Simulation Repository](https://github.com/rosmed/flexible_catheter_simulation)

[Back to Tutorial Index](tutorial_index.html) | [Back to Workshop page](index.html)
