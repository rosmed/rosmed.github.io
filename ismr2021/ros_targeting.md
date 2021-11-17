Back to [Tutorial Home](https://rosmed.github.io/)


Launch a fake UR10 on RViz
==========================

To bring up a fake UR10, try the following commands from a terminal
~~~
cd  workspace/ros_ur_driver
source install/setup.bash
ros2 launch ros2_igtl_bridge spine.launch.py
~~~

Load a spine model on Rviz
==========================

On the RViz window:
- Under the "MotionPlanning" section, open "Scene Object"
- Click the "Import" button.
- Select "/root/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/scene/spine.scene" and click "Open".
- Click the "Publish" button.

Run a ros2_igtl_bridge
======================

Open a new terminal and run the following commands:

~~~~
$ cd workspace/ros_ur_driver
$ source install/setup.bash
$ ros2 run ros2_igtl_bridge igtl_node
~~~~

When the node prompt to enter the type, choose 'SERVER' (option 1):

~~~~
Please type <1> or <2> to run node as OpenIGTLink client or server
1 : SERVER
2 : CLIENT
~~~~

Then, the node asks for a socket port. Enter `18944`:

~~~~
Input socket port:
18944
~~~~

Connect 3D Slicer to ROS
========================

Once the ROS nodes become ready, open 3D Slicer (if has not been opened yet), and open the OpenIGTLinkIF module (Modules menu-> "IGT" -> "OpenIGTLinkIF"). Then create a new node by clicking the "+" button under the "Connectors" list, and configure it from the "Properties" section as follows:
- Name: "IGTLConnector" (default)
- Type: "Client" (default)
- Status: Leave unchecked (default)
- MRMLNodeAlgorithm: Leave unchecked
- Hostname: localhost (if Docker is used) or the IP of the ROS machine (for non-Docker environment)
- Port: 28944 (if Docker is used with "-p 28944:18944" option) or 18944 (default for non-Docker environment)

After configuring the connector, click the "Active" checkbox. If the 3D Slicer is successfully connected to `ros2_igtl_bridge`, the status field on the connector list of the OpenIGTLinkIF will show "ON."


Send Entry and Target Points
============================

Define entry and target points on 3D Slicer:
- Open the "Markups" module
- Define an entry point
  - Under "Create Markups", click "Point List"
  - Click an entry point on the image.
  - Under "Node", click the name of the newly created point (e.g., "F") and rename it to "Entry"
  - Under "Control Points", click the name of the point (e.g., "F-1") and rename it to "Entry"
- Define an target point
  - Under "Create Markups", click "Point List"
  - Click a target point on the image.
  - Under "Node", click the name of the newly created point (e.g., "F_2") and rename it to "Target"
  - Under "Control Points", click the name of the point (e.g., "F_2-1") and rename it to "Target"

Then send them to ROS using OpenIGTLink.
- Open the "OpenIGTLinkIF" module ("IGT"-> "OpenIGTLinkIF")
- Under the "I/O COnfiguration" section
  - Click "Scene"->IGTLConnecctor1"->"OUT"
  - Select "Target" from the node selector menu below I/O configuration and click " + ".
  - Click the "Send" button.
  - Select "Entry" from the node selector menu below I/O configuration and click " + ".
  - Click the "Send" button.

At this point, the robot model on RViz should start moving to the entry point and then to the target point (in the spine model).





  





