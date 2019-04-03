Back to [Tutorial Home](https://rosmed.github.io/)

Prerequisite
============

We encourage all participants to bring their own laptops to follow the tutorial. This page outlines hardware/software requirements. If you are planning to participate in our workshop, we recommend to download/install the following package before the workshop, as it may take some time to download them.

We provide a Docker image of Linux/ROS environment for the tutorial; however, if you prefer to build your own ROS environment on a native Linux system, please refer to [Setting Up ROS Environment for Tutorial](ros_environment).

Hardware
--------

An Intel-based computer that can run either Windows, MacOS X, or Linux with
- At least 4GB RAM (8+GB recommended)
- Hardware that supports virtualization (For PC, virtualization must be turned on from BIOS. Please refer to [Intel's support page](https://www.intel.com/content/www/us/en/support/articles/000007139/server-products.html) for detail).

We also recommend bringing a mouse, as it makes it easier to maneuver 3D graphics on the GUI.

Operating System
----------------

One of the followings:
- Windows 7 or higher (Windows 10 recommended)
- macOS Sierra 10.12 or higher
- Linux 16.04 or higher


3D Slicer
---------

In this tutorial, we will use 3D Slicer version 4.8.1 Binaries are available from the following links:
- [Slicer 4.8.1 for Linux (Intel 64-bit)](http://slicer.kitware.com/midas3/download/item/330417/Slicer-4.8.1-linux-amd64.tar.gz)
- [Slicer 4.8.1 for macOS (Intel 64-bit)](http://slicer.kitware.com/midas3/download/item/330418/Slicer-4.8.1-macosx-amd64.dmg)
- [Slicer 4.8.1 for Windows (Intel 64-bit)](http://slicer.kitware.com/midas3/download/item/329467/Slicer-4.8.1-win-amd64.exe)

**Note for Mac users:** If the system pops up a window warning that "Slicer.app can't be opened because it is from an unidentified developer" when 3D Slicer is launched for the first time, please start the Slicer application by clicking the icon with right mouse button (or click with a Ctrl key) and select "Open" from the pull down menu. Then you will be prompted to confirm that you are opening the application. Slicer will be launched once "Open" button is clicked.

We will not use the latest version of 3D Slicer (4.10.1 as of March 2019) in this tutorial, as it still has a few issues with transferring points and polydata to and from ROS-IGTL-Bridge.  

After installing and launching 3D Slicer, open the Extension Manager ("View" -> "Extension Manager"), and install the following extension:

- **SlicerIGT**

After restarting the 3D Slicer, you should see plug-in modules included in the extension under "IGT" section of the modules menu.


3D Slicer Scene
---------------

First, download [the Slicer scene for ISMR19](http://bit.ly/2HTFHTl). This scene contains:
- Scene description file (Scene-ISMR19.mrml)
- MR image of the patient (MRI.nrrd)
- 3D model of the patient (SkullDrilled1.stl)
- 3D model of the links of the robot (*.stl except for SkullDrilled1.stl)
- Transformation matrix for the links (*.h5)

Decompress the zip file using a zip tool. On a terminal on Linux/Mac, this can be done by:
~~~~
unzip SlicerScene-ISMR19.zip
~~~~


Docker
------

We will use [Robot Operating System](http://www.ros.org/), which requires a Linux operating system (Ubuntu 15.10 or 16.04, or Debian 8.11 Jessie). If you are using a Windows or Mac computer, or a computer with a Linux distribution other than those listed here, you will need to run a Linux operating system using virtualization software, [Docker](https://www.docker.com/). Please go to the Docker's page, and install either:
- [Docker Desktop](https://www.docker.com/products/docker-desktop) (Windows 10, macOS Sierra 10.12 or above)
- [Docker Toolbox](https://docs.docker.com/toolbox/) (older Windows and Mac)
- [Docker CE](https://docs.docker.com/install/) (Linux)


Installing Docker Image for the Tutorial
----------------------------------------


The base Docker image (vnc-ros-kinetic-full) developed by Christian Henkel was derived from [a Docker Hub repository](https://hub.docker.com/r/ct2034/vnc-ros-kinetic-full/). This Docker image comes with:
- Ubuntu 16.04
- ROS Kinetic
- HTML5 VNC Server

Our Docker image contains the following additional packages:
- ROS-Industrial
- ROS Universal Robot package
- ROS MoveIt! package
- OpenIGTLink
- ROS-IGTL-Bridge

Our Dockerfile is available at [GitHub](https://github.com/rosmed/docker-ros-igtl) and its image is distributed at [Docker Hub](https://cloud.docker.com/u/rosmed/repository/docker/rosmed/docker-ros-igtl).

~~~~
docker pull rosmed/docker-ros-igtl
~~~~

If you load Docker image from an offline file,

~~~~
docker load -i docker-ros-igtl.docker
~~~~

Then, run the image:

~~~~
docker run -it --rm -p 6080:80 -p 28944:18944 rosmed/docker-ros-igtl
~~~~

In this example, the HTTP port (port 80) and the OpenIGTLink port (port 18944) on the docker container are mapped to ports 6080 and 28944 on the host computer respectively. The '--rm' option will remove the container upon termination.



To access the desktop, open a web browser (compatible with HTML5) on the same computer, and type the following address in the address bar:
~~~~
http://localhost:6080
~~~~
If the Docker image container is successfully running, the browser should show the desktop screen.

This docker image includes all the software required for the tutorial and does not require to install other packages (i.e. OpenIGTLink and ROS-IGTL-Bridge)


