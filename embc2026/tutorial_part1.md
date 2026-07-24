---
layout: page
title: "Follow-along: Building a navigation-ready surgical scene"
permalink: /embc2026/tutorial_part1.html
---

# Tutorial Part 1: Building a navigation ready surgical scene

In this interactive session, you will build a complete image-guided navigation scene in 3D Slicer: load and segment a CT image of a spine phantom, pivot-calibrate a tracked needle, register the patient anatomy to an optical tracker, and finally load a Micromate robot through SlicerROS2 and register it into the same coordinate frame.

The session is presented in two parts — **Building a navigation-ready surgical scene** (Laura Connolly, PhD) and **Loading and registering the robot to the patient** (Michelle Song, PhD Student).

**Prerequisites:** Complete the environment setup in the [Prerequisites](https://rosmed.github.io/ismr2026/prerequisites.html) page and launch a vast.ai container as described in [Launching a Container on Vast.ai](https://rosmed.github.io/ismr2026/container.html). All commands below are run inside Konsole terminals in the vast.ai desktop environment, and all data files are located in `/home/user/workshop_data/`.

---

## Part 1: Getting Started with 3D Slicer

### Step 1: Open a Konsole Terminal

From the vast.ai desktop, open the applications menu and launch **Konsole**. This is the terminal you will use to launch Slicer.

### Step 2: Navigate to the ROS 2 Workspace

In the terminal, move from the default `/workspace` directory into the ROS 2 workspace:

```
cd ../
cd home/user/ros2_ws
```

![Konsole terminal showing the cd commands into the ros2_ws workspace](images/embc_02_navigate_ros2_ws.png)

### Step 3: Launch 3D Slicer

Source the workspace overlay and start Slicer with the SlicerROS2 module:

```
source install/setup.bash
ros2 run slicer_ros2_module slicer
```

![Konsole terminal showing the source and ros2 run commands used to launch Slicer](images/embc_03_launch_slicer.png)

### Step 4: Make Slicer Full-Screen

Once the Slicer window appears, maximize it so the full interface is visible.

![3D Slicer running full-screen with the default empty scene](images/embc_04_slicer_fullscreen.png)

### Step 5: Switch the Layout

The default layout is **Four-Up**. Use the layout selector in the toolbar to try **3D only**, then switch back to Four-Up.

![Slicer toolbar with the layout selector dropdown open](images/embc_05_switch_layout.png)

### Step 6: Find the Module Selector

The module drop-down in the toolbar is how you switch between the different Slicer modules used throughout this tutorial.

![Slicer module selection dropdown expanded in the toolbar](images/embc_06_module_dropdown.png)

### Step 7: Open the Python Console

Open the Python console. This is how you can run code directly inside 3D Slicer.

![Slicer with the Python interactor console open at the bottom of the window](images/embc_07_python_console.png)

---

## Part 2: Load and Segment the Medical Image

### Step 8: Open the Add Data Dialog

Close the Python console, then click the **Add Data** button in the toolbar.

![Slicer main window with the Add Data button highlighted in the toolbar](images/embc_08_add_data_button.png)

### Step 9: Choose Files to Add

In the Add Data dialog, click **Choose File(s) to Add**.

![Add Data dialog with the Choose Files to Add button](images/embc_09_choose_files.png)

### Step 10: Select the CT Image

Navigate to `/home/user/workshop_data/` and select **CTImage_SpinePhantom.nrrd**.

![File browser showing CTImage_SpinePhantom.nrrd selected in the workshop_data folder](images/embc_10_select_ct_image.png)

### Step 11: Confirm the Load

Click **OK** to load the image into the scene.

![Add Data dialog with the CT image listed and the OK button ready](images/embc_11_confirm_load.png)

### Step 12: Confirm the CT Image Appears

You should now see a CT image of the spine phantom in the slice views.

![Slicer slice views showing the loaded CT image of the spine phantom](images/embc_12_ct_spine_loaded.png)

### Step 13: Open the Segment Editor

Switch to the **Segment Editor** module.

![Slicer module selector with Segment Editor chosen](images/embc_13_segment_editor.png)

### Step 14: Create Two Segments

Press **Add** twice to create two segmentations — one for the background and one for the bone.

![Segment Editor panel showing two newly added segments](images/embc_14_add_two_segments.png)

### Step 15: Select the Paint Tool

Select the paint brush icon from the effects row.

![Segment Editor with the Paint effect selected](images/embc_15_paint_brush_tool.png)

### Step 16: Set the Brush Diameter

Change the diameter of the brush to **1%** so you can place fine seed strokes.

![Segment Editor paint options with the brush diameter set to 1 percent](images/embc_16_brush_diameter.png)

### Step 17: Paint the Seeds

Paint on the slices to seed both segments: yellow on the outside (background), green in the bone regions.

![Slice views with yellow background seeds and green bone seeds painted on the spine](images/embc_17_paint_seeds.png)

### Step 18: Initialize Grow from Seeds

Select the **Grow from seeds** effect and press **Initialize**. This may take a second.

![Segment Editor with the Grow from seeds effect selected and initializing](images/embc_18_grow_from_seeds.png)

### Step 19: Apply the Segmentation

When it finishes, select **Apply** to commit the grown segmentation.

![Segment Editor showing the completed grow-from-seeds result with the Apply button](images/embc_19_apply_segmentation.png)

### Step 20: Show the Segmentation in 3D

Select **Show 3D** to render the segmented spine in the 3D view.

![Slicer 3D view showing the segmented spine surface model](images/embc_20_show_3d_spine.png)

---

## Part 3: Register the Patient to the Optical Tracker

### Step 21: Open the Data Module

Select the **Data** button in the top left to load in the pre-recorded tracking data.

![Slicer toolbar with the Data module button highlighted](images/embc_21_data_module_load.png)

### Step 22: Load the Patient Registration Scene

Select the file called **PatientRegistration.mrb**, click **Open**, then press **OK**.

![File dialog with PatientRegistration.mrb selected](images/embc_22_open_patient_registration.png)

### Step 23: Confirm the Scene Loads

You should see the spine and four points around it. These are the corners of the gel block.

![Slicer 3D view showing the spine model with four fiducial points at the gel block corners](images/embc_23_spine_with_fiducials.png)

### Step 24: Open the Pivot Calibration Module

Switch to the **Pivot Calibration** module.

![Slicer module selector with the Pivot Calibration module chosen](images/embc_24_pivot_calibration_module.png)

### Step 25: Create the Output Transform

Under the **Output** tab, select **Create new transform as...**.

![Pivot Calibration panel with the Output transform selector open](images/embc_25_create_output_transform.png)

### Step 26: Name the Transform

Name the new transform **NeedleTipToNeedle**.

![Transform naming dialog with NeedleTipToNeedle entered](images/embc_26_name_needletiptoneedle.png)

### Step 27: Select the Pivot Calibration Sequence

On the sequence toolbar at the top, change the current sequence to **PivotCalibration**.

![Sequence browser toolbar with the PivotCalibration sequence selected](images/embc_27_select_pivot_sequence.png)

### Step 28: Play the Sequence

Press the **Play** button in the top left and the crosshair button in the 3D viewer. You should see the tracked needle moving through its pivot motion.

![Slicer 3D view showing the tracked needle during playback with the crosshair enabled](images/embc_28_play_and_crosshair.png)

### Step 29: Run the Pivot Calibration

Press **Play** and then **Start Pivot Calibration**. You may need to slow the playback speed to **5.0 fps** so the calibration captures enough of the motion.

<video controls muted playsinline width="100%" poster="images/embc_29_run_pivot_calibration_poster.png">
  <source src="images/video1.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

### Step 30: Return to the Data Module

Switch back to the **Data** module.

![Slicer module selector with the Data module chosen](images/embc_30_switch_to_data_module.png)

### Step 31: Attach the Calibration to the Needle Transform

Drag **NeedleTipToNeedle** onto the **NeedleToTracker** transform in the Data module, so the calibrated tip travels with the tracked needle.

<video controls muted playsinline width="100%" poster="images/embc_31_drag_needletip_transform_poster.png">
  <source src="images/video2.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

### Step 32: Open the Fiducial Registration Wizard

Switch to the **Fiducial Registration Wizard** module.

![Slicer module selector with the Fiducial Registration Wizard chosen](images/embc_32_fiducial_registration_wizard.png)

### Step 33: Create the Target Point List

In the **To Fiducials** section, select **Create new point list as...**.

![Fiducial Registration Wizard panel with the To Fiducials selector open](images/embc_33_create_point_list.png)

### Step 34: Name the Point List

Call the new fiducial list **OpticalTrackerFiducials_CT**.

![Point list naming dialog with OpticalTrackerFiducials_CT entered](images/embc_34_name_optical_fiducials.png)

### Step 35: Set the Placement Transforms

Expand **Place fiducials using transforms** and select **NeedleTipToNeedle** as the second transform. This means points are recorded at the calibrated needle tip rather than at the tracker marker.

![Fiducial Registration Wizard with the placement transforms expanded and NeedleTipToNeedle selected](images/embc_35_place_using_transforms.png)

### Step 36: Select the Registration Sequence

Switch the sequence browser to **FiducialRegistration_CTToOptical** in the top bar.

![Sequence browser toolbar with FiducialRegistration_CTToOptical selected](images/embc_36_select_registration_sequence.png)

### Step 37: Play the Sequence

Press **Play** to see the coordinate model move through the recorded registration motion.

![Slicer 3D view showing the coordinate model moving during sequence playback](images/embc_37_play_coordinate_model.png)

### Step 38: Capture the Four Registration Points

Select the **Place** button as the sequence plays through the four registration points, capturing each corner of the gel block as the needle tip reaches it.

<video controls muted playsinline width="100%" poster="images/embc_38_place_registration_points_poster.png">
  <source src="images/video3.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

### Step 39: Create the Registration Transform

Press **Create new Transform as...** to hold the resulting registration.

![Fiducial Registration Wizard with the output transform selector open](images/embc_39_create_registration_transform.png)

### Step 40: Name the Registration

Call the registration **CTToOpticalTracker**.

![Transform naming dialog with CTToOpticalTracker entered](images/embc_40_name_cttoopticaltracker.png)

### Step 41: Update and Check the Error

Press **Update**. A registration error value is reported at the bottom of the module panel — use this to judge how well the four points aligned.

![Fiducial Registration Wizard showing the computed registration error](images/embc_41_registration_error.png)

### Step 42: Return to the Data Module

Go back to the **Data** module.

![Slicer module selector with the Data module chosen](images/embc_42_return_to_data_module.png)

### Step 43: Apply the Registration to the Anatomy

Drag the CT image and the segmentation onto the **CTToOpticalTracker** transform. The patient anatomy is now positioned in the optical tracker's coordinate frame.

<video controls muted playsinline width="100%" poster="images/embc_43_apply_registration_transform_poster.png">
  <source src="images/video4.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

<div class="tutorial-nav"><a href="container.html" class="btn-secondary">Back to Container Setup</a><a href="tutorial_part2.html" class="btn-primary">Go to Next Step →</a></div>