---
layout: page
title: Slicer Environment 
---

Back to [Tutorial Home](/ismr2023/)


Installing 3D Slicer and Extensions (Skip if using Docker) 
---------------------------------------------------------

To complete the tutorial, you need to build 3D Slicer from the source code. The [build instruction](https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html) is available on the Slicer Web Site. To use SlicerROS2 module, make sure to make sure to use the system/native OpenSSL libraries instead of the version that is downloaded as part of the building process. To use system/native OpenSSL, after you ran CMake, in the Slicer build directory, set `Slicer_USE_SYSTEM_OpenSLL` `ON` using `cmake . -DSlicer_USE_SYSTEM_OpenSSL=ON` or ccmake.

After installing and launching 3D Slicer, open the Extension Manager ("View" -> "Extension Manager"), and install the following extension:

- SlicerOpenIGTLink: Communication interface to hardware (see [the GitHub page](https://github.com/openigtlink/SlicerOpenIGTLink) for details.)
- SlicerIGT: Utility modules for IGT (registration and visualization)
- SlicerIGSIO
- ParallelProcesses (see [the GitHub page](https://github.com/pieper/SlicerParallelProcessing) for more information.)


Installing SegmentationUNet (Intel-based PC/Mac only)
-----------------------------------------------------

If your host computer is equipped with x86 CPU(s), you can run deep-learning-based ultrasound segmentation. 


#### For Docker Users:
To install the segmentation module, run the installation script that comes with the Docker image:
~~~~
# ./install-segmentation-unet.bash
~~~~
Do not use this script if your host computer is not an x86 system (e.g., Mac with Apple Silicon CPU). The segmentation module installs the TensorFlow library that uses Intel's AVX instructions, which are unavailable in the emulated environment. Once the module has been installed, the Slicer will not start properly.

When Slicer is launched for the first time after installing the SegmenationUNet module, it will automatically install the TensorFlow library into Slicer's Python envrionment.

#### For Native OS Environment 
The SegmentationUNet module is available as part of SlicerIGT/aigt at [GitHub]. You can either clone the repository using a git command:
~~~~
git clone https://github.com/SlicerIGT/aigt
~~~~
or download [a zip file](https://github.com/SlicerIGT/aigt/archive/refs/heads/master.zip) and extract files. The source code for SegmentationUNet (`SegmentationUNet.py`) can be found under `aigt/SlicerExtension/LiveUltrasoundAi/SegmentationUNet/`.

To install the SegmentationUNet to your 3D Slicer:

- Open Slicer / Edit / Application Settings / Modules
- Drop the SegmentationUNet.py file in the area in Slicer settings called Additional module paths
- Press OK on the Settings window and restart Slicer application

When Slicer is launched for the first time after installing the SegmenationUNet module, it will automatically install the TensorFlow library into Slicer's Python envrionment.


Installing SlicerROS2
---------------------

#### Building SlicerROS2 the Native Environment (Skip if using Docker Environment) 

The build instruction for SlicerROS2 is available at [the SlicerROS2 documentation](https://slicerros2module.readthedocs.io/en/latest/pages/getting-started.html#pre-requisites). 

First, source the ROS setup script:
~~~~
source /opt/ros/galactic/setup.bash
~~~~

Obtian the source code. Assuming that your ROS2 workspace is located at `~/ros2_ws`: 
~~~~
$ cd ~/ros2_ws/src
$ git clone https://github.com/rosmed/slicer_ros2_module/blob/devel/docs/index.rst
~~~~

Then, build the module by running the following command:
~~~~
colcon build --cmake-args -DSlicer_DIR:PATH=/<your home dir>/slicer/Slicer-SuperBuild-Debug/Slicer-build
~~~~



#### Testing SlicerROS2 on Docker (Skip if using Native Environment)

The 3D Slicer binary is installed under /root/slicer/Slicer-5.2 (Lightweight Docker image) or /root/slicer/Slicer-SuperBuild-Release/Slicer-build (full Docker image). Before launching 3D Slicer, make sure to source ROS's setup script:
~~~~
# source /root/ros2_ws/install/setup.bash
# <path to 3D Slicer>/Slicer
~~~~
Alternatively, run the launch script that comes with the Docker image:
~~~~
# ./start-slicer-ros2.bash
~~~~
If successful, 3D Slicer's main window should appear on the desktop. 



Files for Tutorial
------------------


We will use the following files for the tutorial:
- Scene file: [Q006_SagittalSpineScan_Demo.mrb](https://1drv.ms/u/s!AhiABcbe1DByhKVbAdzf_qwwhPdbTw?e=mbFLzt)
- Trained AI model: [SagittalSpine_05.h5 ](https://1drv.ms/u/s!AhiABcbe1DByhKVRv4S0PaaxXTiz8w?e=Rk3csS)








