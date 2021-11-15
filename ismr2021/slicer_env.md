Back to [Tutorial Home](https://rosmed.github.io/)

3D Slicer
---------

To complete the tutorial, you need to use Slicer a preview version 2021-09-23 or later, which is available at:

- [3D Slicer Download Page](https://download.slicer.org) (Clicke "Preview Release" for your platform)

**Note for Mac users:** If the system pops up a window warning that "Slicer.app can't be opened because it is from an unidentified developer" when 3D Slicer is launched for the first time, please start the Slicer application by clicking the icon with right mouse button (or click with a Ctrl key) and select "Open" from the pull down menu. Then you will be prompted to confirm that you are opening the application. Slicer will be launched once "Open" button is clicked.

After installing and launching 3D Slicer, open the Extension Manager ("View" -> "Extension Manager"), and install the following extension:

- SlicerOpenIGTLink: Communication interface to hardware
- SlicerIGT: Utility modules for IGT (registration and visualization)
- SlicerIGSIO
- ParallelProcesses


Installing SegmentationUNet
---------------------------

SegmentationUNet module is available as part of SlicerIGT/aigt at [GitHub]. You can either clone the repository using a git command:
~~~~
git clone https://github.com/SlicerIGT/aigt
~~~~
or download [a zip file](https://github.com/SlicerIGT/aigt/archive/refs/heads/master.zip) and extract files. The source code for SegmentationUNet (`SegmentationUNet.py`) can be found under `aigt/SlicerExtension/LiveUltrasoundAi/SegmentationUNet/`. To install the SegmentationUNet to your 3D Slicer:

- Open Slicer / Edit / Application Settings / Modules
- Drop the SegmentationUNet.py file in the area in Slicer settings called Additional module paths
- Press OK on the Settings window and restart Slicer application









