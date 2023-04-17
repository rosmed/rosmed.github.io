---
layout: page
title: Slicer Environment 
---

Back to [Tutorial Home](/ismr2023/)


Docker Environment
------------------

### Testing 3D Slicer


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


### Installing the SegmentationUNet module (Intel-based machine only)


If your host computer is equipped with x86 CPU(s), you can run deep-learning-based ultrasound segmentation. To install the segmentation module, run the installation script that comes with the Docker image:
~~~~
# ./install-segmentation-unet.bash
~~~~
Do not use this script if your host computer is not an x86 system (e.g., Mac with Apple Silicon CPU). The segmentation module installs the TensorFlow library that uses Intel's AVX instructions, which are unavailable in the emulated environment. Once the module has been installed, the Slicer will not start properly.



Native Environment
------------------

### Installing Slicer and Extension

To complete the tutorial, you need to use Slicer 5.2 or higher. 

- [3D Slicer Download Page](https://download.slicer.org) (Click "Preview Release" for your platform)

After installing and launching 3D Slicer, open the Extension Manager ("View" -> "Extension Manager"), and install the following extension:

- SlicerOpenIGTLink: Communication interface to hardware (see [the GitHub page](https://github.com/openigtlink/SlicerOpenIGTLink) for details.)
- SlicerIGT: Utility modules for IGT (registration and visualization)
- SlicerIGSIO
- ParallelProcesses (see [the GitHub page](https://github.com/pieper/SlicerParallelProcessing) for more information.)

### Installing SegmentationUNet


#### For Windows users, or Mac/Linux users who run 3D Slicer from a terminal
The SegmentationUNet module is available as part of SlicerIGT/aigt at [GitHub]. You can either clone the repository using a git command:
~~~~
git clone https://github.com/SlicerIGT/aigt
~~~~
or download [a zip file](https://github.com/SlicerIGT/aigt/archive/refs/heads/master.zip) and extract files. The source code for SegmentationUNet (`SegmentationUNet.py`) can be found under `aigt/SlicerExtension/LiveUltrasoundAi/SegmentationUNet/`.

To install the SegmentationUNet to your 3D Slicer:

- Open Slicer / Edit / Application Settings / Modules
- Drop the SegmentationUNet.py file in the area in Slicer settings called Additional module paths
- Press OK on the Settings window and restart Slicer application


#### For Mac users who want to use the launcher to start 3D Slicer
The current version of the SegmentationUNet module may not work properly if 3D Slicer is launched from the launcher on macOS, because it tries to output a log file where the Slicer is launched. If you want to avoid the issue, use the code in the `ismr2021-mac` branch in [a forked repository]((https://github.com/rosmed/aigt/), which output a log file in the home directory. 
~~~~
git clone -b ismr2021-mac https://github.com/rosmed/aigt
~~~~
After cloning the code, follow the steps above to install the module. 

When Slicer is launched for the first time after installing the SegmenationUNet module, it will automatically install the TensorFlow library into Slicer's Python envrionment.   


Files for Tutorial
------------------


We will use the following files for the tutorial:
- Scene file: [Q006_SagittalSpineScan_Demo.mrb](https://1drv.ms/u/s!AhiABcbe1DByhKVbAdzf_qwwhPdbTw?e=mbFLzt)
- Trained AI model: [SagittalSpine_05.h5 ](https://1drv.ms/u/s!AhiABcbe1DByhKVRv4S0PaaxXTiz8w?e=Rk3csS)








