Back to [Tutorial Home](https://rosmed.github.io/)

3D Slicer
---------

To complete the tutorial, you need to use Slicer a preview version 2021-09-23 or later, which is available at:

- [3D Slicer Download Page](https://download.slicer.org) (Click "Preview Release" for your platform)

**Note for Mac users:** If the system pops up a window warning that "Slicer.app can't be opened because it is from an unidentified developer" when 3D Slicer is launched for the first time, please start the Slicer application by clicking the icon with right mouse button (or click with a Ctrl key) and select "Open" from the pull down menu. Then you will be prompted to confirm that you are opening the application. Slicer will be launched once "Open" button is clicked.

After installing and launching 3D Slicer, open the Extension Manager ("View" -> "Extension Manager"), and install the following extension:

- SlicerOpenIGTLink: Communication interface to hardware (see [the GitHub page](https://github.com/openigtlink/SlicerOpenIGTLink) for details.)
- SlicerIGT: Utility modules for IGT (registration and visualization)
- SlicerIGSIO
- ParallelProcesses (see [the GitHub page](https://github.com/pieper/SlicerParallelProcessing) for more information.)


Installing SegmentationUNet
---------------------------

### For Windows users, or Mac/Linux users who run 3D Slicer from a terminal
The SegmentationUNet module is available as part of SlicerIGT/aigt at [GitHub]. You can either clone the repository using a git command:
~~~~
git clone https://github.com/SlicerIGT/aigt
~~~~
or download [a zip file](https://github.com/SlicerIGT/aigt/archive/refs/heads/master.zip) and extract files. The source code for SegmentationUNet (`SegmentationUNet.py`) can be found under `aigt/SlicerExtension/LiveUltrasoundAi/SegmentationUNet/`.

To install the SegmentationUNet to your 3D Slicer:

- Open Slicer / Edit / Application Settings / Modules
- Drop the SegmentationUNet.py file in the area in Slicer settings called Additional module paths
- Press OK on the Settings window and restart Slicer application


### For Mac users who want to use the launcher to start 3D Slicer
The current version of the SegmentationUNet module may not work properly if 3D Slicer is launched from the launcher on macOS, because it tries to output a log file where the Slicer is launched. If you want to avoid the issue, use the code in the `ismr2021-mac` branch in [a forked repository]((https://github.com/rosmed/aigt/), which output a log file in the home directory. 
~~~~
git clone -b ismr2021-mac https://github.com/rosmed/aigt
~~~~
After cloning the code, follow the steps above to install the module.


Installing TensorFlow in 3D Slicer
----------------------------------

If you want TensorFlow to use your GPU, install CUDA v. 11.3 and CuDNN v. 8.2. For downloads and further instructions, check out the [NVidia website](https://developer.nvidia.com/cuda-toolkit-archive).

In Slicer / View/ Python Interactor use command:
~~~~
>>> pip_install('tensorflow')
~~~~
Once install process ends, you may test your environment:
~~~~
>>> import tensorflow as tf
>>> tf.config.list_physical_devices()
~~~~





Files for Tutorial
------------------

We will use the following files for the tutorial:
- Scene file: [Q006_SagittalSpineScan_Demo.mrb](https://1drv.ms/u/s!AhiABcbe1DByhKVbAdzf_qwwhPdbTw?e=mbFLzt)
- Trained AI model: [SagittalSpine_05.h5 ](https://1drv.ms/u/s!AhiABcbe1DByhKVRv4S0PaaxXTiz8w?e=Rk3csS)








