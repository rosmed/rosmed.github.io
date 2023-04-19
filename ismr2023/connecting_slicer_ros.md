---
layout: page
title: Image registration and path planning in Slicer
---

Back to [Tutorial Home](/ismr2023/)

**Part 3: Image registration and path planning in Slicer**

1. Load the scene called "RegisteredScene-dVRKToTongue.mrb" found here: https://github.com/rosmed/ismr2023_files/blob/master/data/RegisteredScene-dVRKToTongue.mrb

<img width="596" alt="image" style="text-align:center" src="https://user-images.githubusercontent.com/36430552/232954693-d4582dff-ed22-434d-a3fa-032549557a95.png">


2. Get familiar with the scene - Switch to the "Volume Rendering" module and press the eyeball on the top left to hide and show the tumor in the tongue. 

3. Switch to the "Fiducial Registration Wizard" module. Here you should see a list of points called CtRegistrationPts and dVRKRegistrationPoints - this is how we transform our CT image from the image coordinate system to the robot coordinate system for path planning. We've already populated these lists for you but Slicer 38 & 39 in the instructions help explain how we did this.

4. Switch to the "Markups" module and hide and show the points on the tongue and in the robot coordinate system to see what it looks like. 

5. Switch back to the "Fiducial Registration Wizard" module. Make sure the output transform is called CtTodVRK and the Result transform type is set to Rigid.

<img width="386" alt="image" style="text-align:center" src="https://user-images.githubusercontent.com/36430552/232952919-2878cf2a-0c07-413d-b055-87f66f7ed24b.png">


6. Press update! You'll see an RMS error popup. This corresponds to the registration error (in mm).

7. Now we need to apply this transform to our Ct image and the tongue tumor. Switch to the "Data" module, flip to the "Transform hierarchy" tab and drag the "TongueTumorToTongue" transform and the "4: Head Skull WF_OXR_SFOV_ShortScan ..." under CtTodVRK. 

<img width="452" alt="image" style="text-align:center" src="https://user-images.githubusercontent.com/36430552/232955251-75cd293f-1d30-4c67-accd-83b2ff6b5a6d.png">


You should now have a registered scene! The next few steps we'll demonstrate on our computer because they require a physical connection to the dVRK. 



