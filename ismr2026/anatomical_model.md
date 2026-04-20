---
layout: page
title: Creating an Anatomical Model for SlicerROS2
permalink: /ismr2026/anatomical_model.html
---

# Overview

This tutorial walks through creating 3D anatomical surface models from a CT image using 3D Slicer and the TotalSegmentator extension. The resulting STL files can be loaded into SlicerROS2 as anatomical reference models during robot-assisted procedures.

The workflow has five main stages:
1. Load a CT image in 3D Slicer
2. Run TotalSegmentator to automatically segment anatomy
3. Select segments of interest and hollow them
4. Export the segments as 3D surface models
5. Clean up the scene and save as STL files

---

## Part 1: Load a CT Image in 3D Slicer

### Step 1: Start 3D Slicer

Open a terminal in the Linux desktop and run the startup script:

```bash
source ~/ros2_ws/install/setup.bash 
ros2 launch slicer_ros2_module slicer.launch.py 
```

![Terminal showing the start-slicer-ros2.bash command](images/anatom2_1_open_slicer.png)

### Step 2: Open the Sample Data Module

Once 3D Slicer opens, go to **File → Download Sample Data** to open the Sample Data module.

![3D Slicer with File menu open showing Download Sample Data](images/anatom2_2_open_image.png)

### Step 3: Load the CTChest Dataset

In the Sample Data panel, click the **CTChest** thumbnail to load it. Slicer will download and load the CT chest scan.

![Sample Data panel with CTChest highlighted](images/anatom2_3_select_image.png)

### Step 4: Verify the Image is Loaded

The CT image will appear in the three slice views (axial, coronal, and sagittal). Verify that the chest anatomy is visible before continuing.

![CTChest loaded in the three slice views](images/anatom2_4_view_image.png)

---

## Part 2: Run TotalSegmentator

### Step 5: Open the TotalSegmentator Module

From the Modules menu, navigate to **Segmentation → TotalSegmentator**.

![Modules menu with TotalSegmentator highlighted under Segmentation](images/anatom2_5_open_totalsegmentator.png)

### Step 6: Configure and Run Segmentation

In the TotalSegmentator panel:
- **Input volume**: CTChest
- **Segmentation task**: total
- **Speed**: Normal

Click **Apply** to start the segmentation.

![TotalSegmentator panel with settings configured and Apply button](images/anatom2_6_start_segmentation.png)

### Step 7: Confirm Python Package Installation

If this is the first time running TotalSegmentator, a dialog will appear asking to install required Python packages. Click **OK** to proceed.

![Python package installation confirmation dialog](images/anatom2_7_install.png)

### Step 8: Wait for Package Installation

Slicer will display a status message while installing the `nnunetv2` package. This step may take several minutes and only needs to be done once.

![Status bar showing nnunetv2 package installation in progress](images/anatom2_8_install_message.png)

### Step 9: Wait for Segmentation to Complete

Once packages are installed, TotalSegmentator will download model weights and run the segmentation. Progress is shown in the panel. This typically takes 2–5 minutes.

![TotalSegmentator panel showing download progress percentages](images/anatom2_9_segmentation_progress.png)

### Step 10: Review Segmentation Results

When complete, the status log will show "Processing finished." Colored segment overlays will appear on all three slice views.

![Segmentation complete with colored overlays on slice views](images/anatom2_10_segmentation_result.png)

### Step 11: Enable 3D View

Check the **Show 3D** checkbox in the Outputs section to display the segmentation in the 3D view.

![Show 3D checkbox visible in TotalSegmentator panel](images/anatom2_11_segmentation_result_3d.png)

The 3D view will now show the segmented anatomy, including the ribcage, heart, and major vessels.

![3D view showing segmented chest anatomy](images/anatom2_12_segmentation_result_3d.png)

---

## Part 3: Select Segments of Interest

### Step 12: Switch to the Segmentations Module

Open the Modules menu and navigate to **Segmentations**.

![Modules menu with Segmentations highlighted](images/anatom2_13_open_segmentations.png)

### Step 13: Show Only Selected Segments

In the Segmentations panel, the full list of segmented structures is shown. To isolate specific structures, select them in the list, then right-click and choose **Show only selected segments**.

![Segmentations panel with context menu showing Show only selected segments](images/anatom2_14_only_selected_segments.png)

### Step 14: Select the Structures You Need

Scroll through the segment list and select the anatomical structures relevant to your application. For this tutorial, select the heart, aorta, pulmonary vein, and major vessels, though the aorta is the only structure we need for the catheter simulation. The 3D view updates to show only the selected structures.

![Segment list with heart selected and 3D view showing the heart](images/anatom2_15_select_segments.png)

---

## Part 4: Hollow the Segments

Hollowing the segments converts solid volumetric segmentations into thin-shell surface models, which are more suitable for 3D printing or visualization.

### Step 15: Switch to Segment Editor

From the Modules menu, navigate to **Segment Editor** (or click the **Segment Editor** button in the toolbar).

![Modules menu navigating to Segment Editor](images/anatom2_16_open_editor.png)

### Step 16: Open the Hollow Effect

In the Segment Editor, select any segment that are visible (i.e., one selected in the previous step in the Segmentations) from the list. Then click the **Hollow** effect button in the effects toolbar.

![Segment Editor with heart selected and Hollow effect panel open](images/anatom2_18_open_hollow.png)

### Step 17: Configure Hollow Settings

Set the following parameters:
- **Use current segment as**: inside surface
- **Shell thickness**: 3.00 mm
- Check **Apply to visible segments** to hollow all visible segments at once 

![Hollow effect panel showing inside surface and 1mm shell thickness settings](images/anatom2_19_hollow.png)

### Step 18: Set Apply to Visible Segments

Select the **Apply to visible segments** checkbox to process all currently visible segments.

![Apply to visible segments dropdown set to Visible](images/anatom2_20_apply_visible.png)

### Step 19: Apply the Hollow Effect

Click **Apply** to run the hollow operation.

![Apply button in the Hollow effect panel](images/anatom2_21_apply.png)

### Step 20: Review Hollow Result

The segments will now appear as outlines (hollow shells) in the slice views, and the 3D view will show the thin-shell models.

![Slice views and 3D view showing hollowed heart and vessel segments](images/anatom2_22_hollow_result.png)

---

## Part 5: Export Segments as Surface Models

### Step 21: Switch to the Segmentations Module

From the Modules menu, navigate to **Segmentations**.

![Modules menu with Segmentations highlighted](images/anatom2_23_open_segmentations.png)

### Step 22: Configure the Export

Scroll down to the **Export/Import models and labelmaps** section and set:
- **Operation**: Export
- **Output type**: Models
- **Output node**: Export models to new folder

![Segmentations panel showing export settings with Models output type](images/anatom2_24_export_model.png)

### Step 23: Set Exported Segments to Visible

Under **Advanced**, set **Exported segments** to **Visible** so that only the currently visible segments are exported.

![Advanced section with Exported segments set to Visible](images/anatom2_25_export_visible.png)

### Step 24: Run the Export

Click **Export**. Slicer will create a new folder in the scene containing one model node per segment.

![Export button being clicked](images/anatom2_26_export.png)

---

## Part 6: Organize the Scene

### Step 25: Open the Data Module

From the Modules menu, navigate to **Data**.

![Modules menu with Data highlighted](images/anatom2_27_open_data.png)

### Step 26: Inspect the Scene Tree

The Data module shows the scene hierarchy. You will see:
- The **CTChest** volume node
- The **CTChest segmentation** node with all segments
- The **CTChest segmentation-models** folder containing the exported model nodes

![Data module scene tree showing CTChest, segmentation, and model nodes](images/anatom2_28_data.png)

The models folder contains one node per exported segment (heart, aorta, pulmonary vein, etc.).

![Scene tree with CTChest segmentation-models folder expanded showing individual model nodes](images/anatom2_29_fold_segmentation.png)

### Step 27: Delete the CT Volume and Segmentation Nodes

Right-click the **CTChest** volume node and **CTChest segmentation**, and select **Delete**. Do not delete **Chest segmentation-models**.

![Right-click context menu on CTChest node showing Delete option](images/anatom2_30_delete_nodes.png)

When prompted to delete the subject hierarchy branch, click **Yes to All**.

![Delete subject hierarchy branch confirmation dialog](images/anatom2_31_delete_nodes.png)

After deletion, only the model nodes remain in the scene.

![Scene tree showing only the CTChest segmentation-models folder and model nodes](images/anatom2_32_select_models.png)

### Step 28: Move Models to the Top Level

Select all model nodes in the scene tree and drag them up to the **Scene** root level, out of the segmentation-models folder.

![Scene tree with all model nodes selected and highlighted](images/anatom2_33_move_models.png)

After moving, all models are directly under **Scene**.

![Scene tree with all model nodes moved directly under Scene](images/anatom2_34_move_models.png)

### Step 29: Delete the Empty Folder

Right-click the now-empty **CTChest segmentation-models** folder and select **Delete**.

![Right-click context menu on the empty folder showing Delete option](images/anatom2_35_delete_folder.png)

### Step 30: Rename the Model Nodes

To avoid ' ' (space) in the file names, replace the all spaces in the node names with '_' (underscore). To do so, right-click each model node and select **Rename**.

![Right-click context menu on a model node showing Rename option](images/anatom2_36_rename_nodes.png)

Edit the name to replace the space(s) with underscore(s) in the rename dialog and click **OK**.

![Rename dialog with new name left_atrial_appendage entered](images/anatom2_37_rename_nodes.png)

Repeat for each model node with a name that contains any underscore.

![Scene tree with all models renamed and 3D view showing the complete vascular model](images/anatom2_38_open_save.png)

---

## Part 7: Save the Slicer scene

### Step 31: Open the Save Dialog

Go to **File → Save** (or press **Ctrl+S**) to open the Save Scene and Unsaved Data dialog. The dialog lists all nodes and their current file format.

![Save Scene and Unsaved Data dialog showing model files in VTK format](images/anatom2_39_save_dialog.png)

### Step 32: Change File Format to STL

For each model node, click the **File Format** dropdown and select **Stereolithography Mesh (.stl)**.

![File format dropdown showing STL option highlighted](images/anatom2_40_stl_format.png)

### Step 33: Choose a Save Directory

Click **Change directory for selected files** to open the Find Directory dialog. Navigate to `/home/user/Documents`.

![Find Directory dialog showing /home/user/Documents](images/anatom2_41_select_folder.png)

### Step 34: Create a New Folder

Click the **New Folder** button in the toolbar to create a new folder.

![Find Directory dialog with a new folder being created](images/anatom2_42_create_folder.png)

### Step 35: Name the Folder

Type a name for the folder (e.g., `anatomy_scene`) and press Enter.

![New folder renamed to anatomy_scene in the Find Directory dialog](images/anatom2_43_create_folder.png)

Confirm the folder name is set correctly.

![anatomy_scene folder shown in the Find Directory dialog](images/anatom2_44_rename_folder.png)

### Step 36: Select the Folder

Click the `anatomy_scene` folder to select it, then click **Choose**.

![anatomy_scene folder selected and Choose button highlighted](images/anatom2_45_select_folder.png)

### Step 37: Save All Files

The save dialog now shows all model files in STL format with the directory set to `Documents/anatomy_scene`. Click **Save** to write all files.

![Save dialog showing all models in STL format in the anatomy_scene directory, Save button highlighted](images/anatom2_46_save_scene.png)

The STL files are now saved to `~/Documents/anatomy_scene/` and are ready to be loaded into SlicerROS2.

---

<div class="tutorial-nav"><a href="index.html" class="btn-secondary">Back to Workshop Page</a><a href="catheter_model.html" class="btn-primary">Go to Next Step →</a></div>
