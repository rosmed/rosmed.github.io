---
layout: page
title: Intro to SlicerROS2
---

Back to [Tutorial Home](/ismr2023/)

**Part 2: Intro to SlicerROS2**

For this portion of the workshop we walk through examples from the SlicerROS2 read the docs. Available here: https://slicer-ros2.readthedocs.io/en/latest/index.html

We package standard ROS2 communication mechanisms as MRML nodes in 3D Slicer (https://slicer.readthedocs.io/en/latest/developer_guide/mrml_overview.html). 

<img width="612" alt="image" style="text-align:center" src="https://user-images.githubusercontent.com/36430552/232659172-0c4257b4-300d-470f-8aa9-56ff8e3a58f1.png">

To get started with SlicerROS2, we'll be using a simulated patient side manipulator (PSM) for the dVRK. 

1. Start by building the dVRK robot package in ROS2: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2


2. Once that's built, launch the virtual PSM: 

````
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit
ros2 run dvrk_robot dvrk_console_json -j share/console/console-PSM1_KIN_SIMULATED.json
````

3. Select "Power on" and press "Home"
<img width="370" alt="image" style="text-align:center" src="https://user-images.githubusercontent.com/36430552/232660134-27a85349-6c65-41a9-be6c-60007797262e.png">

4. In a seperate terminal, launch the robot_state_pubslisher (this is how we tell Slicer where the links are) 

````
source ~/ros2_ws/install/setup.bash
ros2 launch dvrk_model dvrk_state_publisher.launch.py arm:=PSM1
````

5. Finally, in a third terminal, navigate to your inner build folder (usually in Slicer-SuperBuild-xxx/Slicer-build/)

````
source ~/ros2_ws/install/setup.bash 
source ~/opt/ros/galactic/setup.bash
./Slicer
````

6. Now you can visualize the robot by switching to the ROS2 module and entering "PSM1/robot_state_publisher" in the second selector: 
<img width="469" alt="image" style="text-align:center" src="https://user-images.githubusercontent.com/36430552/232660578-37974c02-7379-4d4d-ac22-41e82ed317d3.png">

Note: you will need to zoom out to see the robot!

6. Get your robot to do a little dance by running in a seperate terminal

````
ros2 run dvrk_python dvrk_arm_test.py -a PSM1
````

<img width="422" alt="image" style="text-align:center" src="https://user-images.githubusercontent.com/36430552/232660691-d3dd4a8d-5799-4cb8-862f-40817897c24a.png">





