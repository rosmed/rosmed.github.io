---
layout: page
title: "Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy"
permalink: /tutorials/smarttemplate/1-load-images.html
---

# Step 1: Load MR images and register ZFrame fiducials

First, load MR images in 3DSlicer (drag the provided files and drop at the 3D Slicer interface). Then open "ZFrameRegistrationWithROI" module

![ZFrameRegistrationWithROI module](images/image2.jpg)

To register the fiducial marker model to the MR image:
- Select the horizontal ZFrame model
- Select the "COR TSE T2 COVER ZFrame" volume
- Select index of slices 9 to 12
- Define the ROI in the coronal view (green viewer) that appropriately covers the fiducials
- Click the ✓ button

![ZFrame registration](images/image3.jpeg)

Observe the resultant Linear Transform node that was created to represent the transform from ZFrame to Scanner coordinates:

![Transform node](images/image4.jpg) ![Transform coordinates](images/image5.jpg)

[⬅️ Previous: Preparation](0-preparation) | [Next: Load SmartTemplate robot in 3DSlicer ➡️](2-load-robot) | [Back to Table of Contents ↩️](index)