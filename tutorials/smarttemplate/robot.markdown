---
layout: page
title: SmartTemplate Robot - MRI-Guided Robot-Assisted Prostate Biopsy
permalink: /tutorials/smarttemplate/robot.html
---

# SmartTemplate Robot

The SmartTemplate robot is a **3 degrees-of-freedom (3-DOF) prismatic robot** designed to align a needle guide in 3D space for MRI-guided interventions. It consists of three translational joints that move along orthogonal axes.

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

[⬅️ Back to Overview](overview.html) | [Next: SmartTemplate Description Package ➡️](description.html)