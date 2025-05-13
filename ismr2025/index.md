---
layout: page
title: ISMR25 Workshop/ Open-Source Software for Intelligent Image-Guided Medical Robots
permalink: /ismr2025/
---

![ISMRPhoto](/images/ismr2024.jpg){:class="img-responsive" width="400px"}


__Event Date: May 14, 2025 (In-Person)__

# Table of Contents

- [Overview](#overview)
- [Intended Audience](#intended-audience)
- [Time Table](#time-table)
- [Preparation](#preparation)
- [Links](#links)
- [References](#references)
- [Past Workshops](#past-workshops)
- [Organizers](#organizers)
- [Acknowledgements](#acknowledgements)
- [Contact](#contact)


# Overview

The development of medical robots and AI is transforming the landscape of healthcare today. These tools offer several clinical advantages, such as dexterous manipulation of surgical instruments, surgical task automation, accurate digital patient models, and intraoperative tissue characterization. However, creating such systems requires the integration of AI, robots, imaging devices, and tracking equipment. This integration is a challenging barrier to entry for researchers to overcome before they can even begin innovating. Furthermore, many medical robotic systems rely on custom APIs or expensive research licenses that make them inaccessible.   

Recently, several open-source platforms have gained momentum to facilitate the integration of various software and hardware components into a single system. Those platforms enable modular and component-based development for rapid prototyping, reducing the initial startup burden. Those tools can incorporate dynamic robot simulation, allowing for a seamless transition between the virtual and physical environments for testing, which is crucial for the development of intelligent and AI-driven medical robotics.    

This workshop will consist of presentation and hands-on sessions. In the presentation session, we will introduce some of the open-source software and hardware platforms for prototyping image-guided robotic surgical systems. We will outline the available mechanisms for connecting complementary software environments in image-guided robotics and outline gaps, needs and opportunities. The workshop participants will be exposed to the latest efforts for the interoperability between 3D Slicer, ROS2, AMBF, Gazebo (Ignition) and the da Vinci Research Kit (dVRK). We will also cover various open-source tools that have been developed by the community for training and deploying AI models for surgical robotic systems.   

In the tutorial session, we will present two demonstrations of systems that combine these tools for clinical applications that will showcase the interoperability between different environments. We will also help the participants to recreate the demonstrations by sharing instructions about the available repositories and docker containers.

**Keywords**: Image-guided interventions, Navigation, Open-source software, Software-hardware integration, Surgical CAD/CAM


# Intended Audience

Researchers, engineers, and students working in the field of medical robotics and image-guided interventions are welcome to join. The tutorial sessions would be particularly useful for those who are already engaged or will be engaged in the design, implementation, and clinical translation of a system for image-guided robot-assisted interventions. For the hands-on session, we strongly recommend the audience bring their own laptop computers to follow the hands-on tutorial during the session. While the tutorial will not involve coding, some experience in running commands on a UNIX-like system and compiling open-source software using Make or CMake would be helpful.


# Time Table
<style>
    table th:first-of-type { width: 20%; }
    table th:nth-of-type(2) { width: 55%; }
    table th:nth-of-type(3) { width: 25%; }
</style>

    
| Time        | Agenda                                                  | Speaker        |
|-------------|---------------------------------------------------------|:--------------:|
| 8:00-9:00   | Registration                                            |                |
| 9:00-10:30  | Session 1: AI for Surgical Robotics                     |                |
|             | Machine Learning with the da Vinci Research Kit (dVRK)  | Peter Kazanzides |
|             | Automating Medical Needle Robots in Living Tissue: Open Source Data Sets and AI Software  | Ron Alterovitz |
|             | Learning from Demonstration for Autonomy in Soft Tissue Surgery        | Alan Kuntz     |
|             | Towards Indirect Force Estimation via Learning-based Visual Proprioception in Minimally Invasive Telesurgery   | Zonghe Chua    |
|             | From Monocular Vision to Autonomous Action: Guiding Tumor Resection via 3D Reconstruction | Jie Ying Wu    |
| 10:30-11:00 | Break and Refreshments                                  |                |
| 11:00-12:30 | Session 2: Open Source Tools                            |                |
|             | Open-source medical imaging and image guided therapy    | Junichi Tokuda |
|             | Open-source medical robotics                            | Anton Deguet   |
|             | Open-source dynamic simulation                          | Adnan Munawar  |
|             | Open-source AI development                              | Axel Krieger   |
|             | Open-source continuum surgical robot                    | Loris Fichera  |
|             | Panel Discussion                                        |                |
| 12:30-13:30 | Lunch                                                   |                |
| 13:30-15:00 | Session 3: Demonstrations of Integrated Systems (Part 1)| Mariana Bernardes |
| 15:00-15:30 | Break and Refreshments                                  |                |
| 15:30-17:00 | Session 3: Demonstrations of Integrated Systems (Part 2)| Laura Connolly |
| 17:00       | Closing                                                 |                |


# Tutorial Materials

## Prerequisites

See [the prerequisites](prerequisites)

*Instructions and materials will be provided here.*

## Session 3 (Part 1): Creating and loading the visual/kinematic of a custom robot in 3D Slicer

In this tutorial, we will explore the basic features of SlicerROS2, including loading and visualizing URDF data in 3D Slicer, and connecting it to ROS via topics (publishers and subscribers).
- [MRI-Guided Robot-Assisted Prostate Biopsy with SmartTemplate](/tutorials/smarttemplate/)

## Session 3 (Part 2): Integration of 3D Slicer with AI-based segmentation, ROS2, and AMBF 

This tutorial demonstrates a more advanced use-case scenario, where an anatomical model is created in 3D Slicer and used in a dynamic simulator (AMBF) to simulate the physical interaction between a manipulator and the anatomy.
- [Building your own ultrasound based virtual fixture with AMBF and SlicerROS2](/tutorials/ultrasound_ai_ambf_integration/)


# Links

- [ISMR Workshop](http://www.ismr.gatech.edu)
- [3D Slicer](https://www.slicer.org/)
- [Robot Operating System](http://www.ros.org/)
- [SlicerROS2](https://github.com/rosmed/slicer_ros2_module)


# References

- Connolly L, Kumar AS, Mehta KK, Al-Zogbi L, Kazanzides P, Mousavi P, Fichtinger G, Krieger A, Tokuda J, Taylor RH, Leonard S, Deguet A. [SlicerROS2: A Research and Development Module for Image-Guided Robotic Interventions](https://doi.org/10.1109/TMRB.2024.3464683). IEEE Transactions on Medical Robotics and Bionics. 2024;6(4):1334-1344. doi: 10.1109/TMRB.2024.3464683.
- Connolly L, Deguet A, Leonard S, Tokuda J, Ungi T, Krieger A, Kazanzides P, Mousavi P, Fichtinger G, Taylor RH. [Bridging 3D Slicer and ROS2 for Image-Guided Robotic Interventions](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9324680/). Sensors (Basel). 2022 Jul 17;22(14):5336. doi: 10.3390/s22145336. PMID: 35891016; PMCID: PMC9324680.
- Fedorov A, Beichel R, Kalpathy-Cramer J, Finet J, Fillion-Robin JC, Pujol S, Bauer C, Jennings D, Fennessy F, Sonka M, Buatti J, Aylward S, Miller JV, Pieper S, Kikinis R. [3D Slicer as an image computing platform for the Quantitative Imaging Network](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3466397/). Magn Reson Imaging. 2012 Nov;30(9):1323-41. doi: 10.1016/j.mri.2012.05.001. Epub 2012 Jul 6. PubMed PMID: 22770690; PubMed Central PMCID: PMC3466397.


# Past Workshops

- [ISMR 2024](/ismr2024/index)
- [ISMR 2023](/ismr2023/index)
- [ISMR 2021](/ismr2021/index)
- [ISMR 2019](/ismr2019/index)


# Organizers

- [**Junichi Tokuda**](https://tokuda-lab.bwh.harvard.edu/team/junichi-tokuda/), Brigham and Women's Hospital and Harvard Medical School, Boston, MA, USA
- [**Peter Kazanzides**](https://engineering.jhu.edu/faculty/peter-kazanzides/), Johns Hopkins University, Baltimore, MD, USA
- [**Laura Connolly**](https://labs.cs.queensu.ca/perklab/members/laura-connolly/), Queen's University, Kingston, ON, Canada
- [**Simon Leonard**](https://www.cs.jhu.edu/~sleonard/), Johns Hopkins University, Baltimore, MD, USA
- [**Lydia Al-Zogbi**](https://www.vanderbilt.edu/vise/visepeople/lidia-al-zogbi/), Vanderbilt University, Nashville, TN
- [**Mariana Bernardes**](https://tokuda-lab.bwh.harvard.edu/team/mariana-bernardes/), Brigham and Women's Hospital and Harvard Medical School, Boston, MA, USA
- [**Anton Deguet**](https://malonecenter.jhu.edu/people/anton-deguet/), Johns Hopkins University, Baltimore, MD, USA
- [**Axel Krieger**](https://me.jhu.edu/faculty/axel-krieger/), Johns Hopkins University, Baltimore, MD, USA
- [**Pedro Moreira**](https://tokuda-lab.bwh.harvard.edu/team/pedro-moreira/), Brigham and Women's Hospital and Harvard Medical School, Boston, MA, USA
- [**Loris Fichera**](https://www.wpi.edu/people/faculty/lfichera), Worcester Polytechnic Institute, Worcester, MA, USA
- [**Adnan Munawar**](https://www.linkedin.com/in/adnan-munawar-56190a200), Johns Hopkins University, Baltimore, MD, USA


# Acknowledgements

This work is supported in part by:
- U.S. National Institutes of Health (R01EB020667, R01EB020667-05S1, R01EB020610, P41EB028741)
- U.S. National Science Foundation (OISE-1927354, OISE-1927275)
- Ontario Consortium for Adaptive Interventions in Radiation Oncology (OCAIRO)
- SparKit project
- CANARIE's Research Software Program

![NIBIB Logo](https://www.nibib.nih.gov/sites/default/files/nibib_logo.png){:class="img-responsive"} 

![NSF Logo](/images/NSF_Logo.png){:class="img-responsive" width="120px"}


![BWH](/images/BWH_Logo.png){:class="img-responsive" width="360px"}
![HMS](/images/HMS_Logo.png){:class="img-responsive" width="200px"}
![JHU](/images/JHU_Logo.png){:class="img-responsive" width="240px"}
![WPI](/images/WPI_Logo.png){:class="img-responsive" width="180px"}
![Queens](/images/Queens_Logo.png){:class="img-responsive" width="200px"}


# Contact

[Junichi Tokuda, Ph.D.](https://tokuda-lab.bwh.harvard.edu/team/junichi-tokuda/)

Associate Professor of Radiology
Brigham and Women's Hospital / Harvard Medical School
