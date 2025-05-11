---
layout: page
title: Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy
permalink: /tutorials/smarttemplate/robot.html
---

# SmartTemplate Robot

The SmartTemplate robot is a **3 degrees-of-freedom (3-DOF) prismatic robot** designed to align a needle guide in 3D space for MRI-guided interventions. It consists of three translational joints that move along orthogonal axes. The robot was designed based on the 2DoF Motorized Template [[1,2](#references)] and extended to 3-DoF in a recent study [[3](#reference)].

## Kinematic Chain

~~~~
world
└── base_joint (fixed)
     └── base_link
          └── vertical_joint (Z-axis)
               └── vertical_link
                    └── horizontal_joint (X-axis)
                         └── horizontal_link
                              └── insertion_joint (Y-axis)
                                   └── needle_link (end-effector)
~~~~

## Joint Definitions

| **Joint Name** | **Type** | **Axis** | **Parent Link** | **Child Link** | **Range (m)** |
|:--------------|:---------|:---------|:---------------|:--------------|:-------------|
| vertical_joint | prismatic | Z (0,0,1) | base_link | vertical_link | ±0.025 |
| horizontal_joint | prismatic | X (1,0,0) | vertical_link | horizontal_link | ±0.03 |
| insertion_joint | prismatic | Y (0,1,0) | horizontal_link | needle_link | 0.000 → 0.115 |

- **vertical_joint**: Adjusts height of the needle guide above the perineum (Z-axis)
- **horizontal_joint**: Aligns the guide laterally (X-axis)
- **insertion_joint**: Controls needle insertion depth (Y-axis), i.e., into the patient

![SmartTemplate Robot](images/image9.jpg)


## References {#references}
1. Song SE, Tokuda J, Tuncali K, Tempany CM, Zhang E, Hata N. Development and preliminary evaluation of a motorized needle guide template for MRI-guided targeted prostate biopsy. IEEE Trans Biomed Eng. 2013 Nov;60(11):3019-27. doi: 10.1109/TBME.2013.2240301. Epub 2013 Jan 15. PMID: 23335658; PMCID: PMC3778164.
2. Tilak G, Tuncali K, Song SE, Tokuda J, Olubiyi O, Fennessy F, Fedorov A, Penzkofer T, Tempany C, Hata N. 3T MR-guided in-bore transperineal prostate biopsy: A comparison of robotic and manual needle-guidance templates. J Magn Reson Imaging. 2015 Jul;42(1):63-71. doi: 10.1002/jmri.24770. Epub 2014 Sep 27. PMID: 25263213; PMCID: PMC4376663.
3. Bernardes MC, Moreira P, Lezcano D, Foley L, Tuncali K, Tempany C, Kim JS, Hata N, Iordachita I, Tokuda J. In Vivo Feasibility Study: Evaluating Autonomous Data-Driven Robotic Needle Trajectory Correction in MRI-Guided Transperineal Procedures. IEEE Robot Autom Lett. 2024 Oct;9(10):8975-8982. doi: 10.1109/lra.2024.3455940. Epub 2024 Sep 6. PMID: 39371576; PMCID: PMC11448709.

 
 

[⬅️ Back to Overview](overview.html) | [Next: SmartTemplate Description Package ➡️](description.html)
