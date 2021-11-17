Back to [Tutorial Home](https://rosmed.github.io/)

Introduction
============

In this tutorial, we will create a surgical plan using a 3D model of the spine reconstructed from tracked ultrasound images. Following steps are required to accomplish this task:

- Load Slicer scene with data
- Set up input and output for segmentation module
- Initialize prediction by updating input
- Visualize prediction
- Set up volume reconstruction
- Start ultrasound replay
- Set up volume rendering to visualize spine volume
- Registration


Load Slicer Scene with Data
===========================

Before loading the file, make sure to download the following files:
- [A Slicer scene file including tracked ultrasound images (Q006_SagittalSpineScan_Demo.mrb)](https://1drv.ms/u/s!AhiABcbe1DByhKVbAdzf_qwwhPdbTw?e=mbFLzt)
- [Trained AI model for segmentation (SagittalSpine_05.h5)](https://1drv.ms/u/s!AhiABcbe1DByhKVRv4S0PaaxXTiz8w?e=Rk3csS)


The scene file can be loaded to 3D Slicer as follows:
- Click the "Data" button near the left end of the tool bar, or choose "File"->"Add Data".
- On the "Add data into the scene" dialog box, click "Choose File(s) to Add" to open a file selection window.
- Navigate to the folder where you downloaded "Q006_SagittalSpineScan_Demo.mrb" and select it.
- Click "OK" on the "Add data into the scene" dialog box.

Please also check if the "Sequence brower" tool bar is visible on the window. If not, turn it on by selecting "View"->"Toolbars" -> "Sequence browser".


Set up Input and Output for Segmentation Module
===============================================

Segmentation module can be opened from the "Modules" menu by selecting "Ultrasound" -> "Segmentation UNet". Configure the module as follows:

- Input/output
  - Traind model: Click the "..." button and select the Trained AI model downloaded above ("SagittalSpine_05.h5")
  - Input volume: Select "Image_Image"
  - Prediction volume: Select "Create new Volume as...". In the dialog box, enter "Prediction" as a new name.
- Optional parameters
  - Prediction transform: Select "Create new Volume as...". In the dialog box, enter "PredictionToRas" as a new name.
  - Use separate process: In the current version of the module, this option might cause an error on mac. If you are using Mac, it is safe to turn this off. (This may slow down the reconstruction process.)

Initialize Prediction by Updating Input
=======================================
Once the moduel is configured, click "Apply segmentation" button, and then click the "Next frame" button on the Sequence bar to fully start the AI process. 


Visualize prediction
====================

First, change the window layout to "Four-Up" layout so that the original ultrasound frame, prediction image, and 3D rendering are all visible on the window.

Configure the Green viewers as follows:
- Click the "pin" icon at the top left corner to show the toolbar.
- Uncheck the "Link" button (next to the ">>" button).
- From the volume selector, select "Prediction".

Apply the PredictionToRas transform to the "Predition" image as follows:
- Open the "Data" module.
- Click the "Transform hierarchy" tab.
- Drag and drop the "Prediction" volume under "PredictionToRas"

Finally, configure the viewers to follow the imaging plane.
- Open the "Volume Reslice Driver" module ("IGT" -> "Volume Reslice Driver")
- Under "Advanced option":
  - Turn on "Show advanced option"
- Under the Green slice:
  - Driver: Choose "Prediction".
  - Check the "Flip" check box.
  

Set up volume reconstruction
============================

The volume reconstruction module creates a 3D volumetric image from a tracked image. Configure the module as follows to use the prediction image:

- Open the "Volume Reconstruction" module ("IGT" -> "Volume Reconstruction").
- Input Method: Select "Live volume reconstruction".
- Input volume node: Select "Prediction".
- Output volume node: Select "Create a New Volume" and name it "Reconstructed Volume".
- ROI node: Select "R"

Once the module is configured, click the "Start" button at the bottom of the module panel.


Start ultrasound replay
=======================

From the Sequence tool bar, click "Playback" button to start ultrasound replay. The green viewer should start showing the prediction image. 


Set up volume rendering to visualize spine volume
=================================================

To visualize the prediction in 3D, setup volume rendering as follows:
- Open the "Volume Rendering"
- Volume: Select "ReconstructedVolume".
- Preset: Select "MR-Default".
- Click the eye-shaped icon next to the "Volume" selector to activate volume rendering
- Adjust threshold using the "Shift" slider.

Stop Volume Reconstruction
==========================

When the volume becomes ready, stop playback, and stop Volume Reconstruction and Segmentation UNet. Also,
- Switch to 3D-only view
- Under the "Annotation" module, hide "R" ROI by clicking the eye-shaped icon in the "Vis" column.


Registration
============

First, load the spine models on 3D Slicer:
- Click the "Data" button near the left end of the tool bar, or choose "File"->"Add Data".
- On the "Add data into the scene" dialog box, click "Choose File(s) to Add" to open a file selection window.
- Find and select the following files and click the "Open" button:
  - LumbarSpineModel.stl
  - PlannedNeedleModel.stl
- Then click the "OK" button on the "Add data into the scene" button.


Next, setup fiducial registration:
- Open the "Fiducial Registration Wizard" module ("IGT" -> "Fiducial Registration Wizard").
- Under From fiducials:
  - 


