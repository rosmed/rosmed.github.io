Back to [Tutorial Home](https://rosmed.github.io/)

Testing OpenIGTLink Communication Between Slicer and ROS
========================================================

Starting ros2_igtl_bridge
-------------------------

(See [Setting Up ROS 2 Environment for Tutorial](ros_env.md) if you are using the Docker image provided for this tutorial.)

Open two terminal windows. On the first one, run the igtl_node with the following steps:

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

Please note that, if you have followed [the Docker instruction](ros_env.md), TCP port `18944` is mapped to TCP port `28944` on the host OS. Other programs on the host OS can connect to the igtl_host running on this Docker image through TCP port `28944`.


On the second terminal window, run the igtl_test_publisher with the following steps:
~~~~
$ cd workspace/ros_ur_driver
$ source install/setup.bash
$ ros2 run ros2_igtl_bridge igtl_test_publisher
~~~~

`igtl_test_publisher` will start pushing messages immediately.


Setting up 3D Slicer OpenIGTLink and Receive Messages from ROS
---------------------------------------------------------------

Once the ROS nodes become ready, open 3D Slicer (if has not been opened yet), and open the OpenIGTLinkIF module (Modules menu-> "IGT" -> "OpenIGTLinkIF"). Then create a new node by clicking the "+" button under the "Connectors" list, and configure it from the "Properties" section as follows:
- Name: "IGTLConnector" (default)
- Type: "Client" (default)
- Status: Leave unchecked (default)
- MRMLNodeAlgorithm: Leave unchecked
- Hostname: localhost (if Docker is used) or the IP of the ROS machine (for non-Docker environment)
- Port: 28944 (if Docker is used with "-p 28944:18944" option) or 18944 (default for non-Docker environment)

After configuring the connector, click the "Active" checkbox. If the 3D Slicer is successfully connected to `ros2_igtl_bridge`, the status field on the connector list of the OpenIGTLinkIF will show "ON."

3D Slicer is supposed to receive the following messages:
1. test_point (Markups Fiducial)
2. test_string (String)
3. test_transform (Linear Transform)
4. test_pose_array (IGTLTrackingDataSplitter)

1-3 appears on the `Data` module with the same names. `test_pose_array` is split into individual poses (`POSE_0`, `POSE_1`, ..., `POSE_4`) and appears as four separate linear transform nodes on the `Data` module.














