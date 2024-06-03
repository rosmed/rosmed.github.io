---
layout: page
title: ISMR24 Workshop/ Leveraging SlicerROS2 for Simulation of Image-Guided Assisted Interventions in Gazebo (Ignition)  
permalink: /ismr2024/
---

![ISMRPhoto](/images/ismr2019_tutorial.jpg){:class="img-responsive" width="400px"}


## Event Date: June 3, 2024 (In-Person)

# Table of Contents
- [Overview](#overview)
- [Intended Audience](#audience)
- [Time Table](#time)
- [Tutorial](#tutorial)
- [Links](#links)
- [Past Workshops](#past)
- [Organizers](#organizers)
- [Acknowledgements](#acknowledgements)
- [Contact](#contact)


# Overview

Attendees of this tutorial will learn how to use Gazebo (formerly known as Ignition), open-source dynamic simulation software, in combination with 3D Slicer, open-source medical image computing platform, to simulate image-guided robot-assisted interventions (IGRI). 

Dynamic simulations have become an indispensable tool in developing mobile and industrial robotic systems. Gazebo supports a broad range of robots and sensors, such as GPS, IMU, camera and LIDAR, allowing the developers to simulate the interaction of the robot and environment before building a hardware system. However, the benefits to image-guided robot intervention have been limited due to the lack of integration with medical image computing environment, and the lack of support for the simulation of sensors typically used in medical robotics, particularly imaging scanners, such as ultrasound, MRI and CT.

Fortunately, those limitations have been diminished in recent years thanks to efforts to integrate medical imaging and robotics software environments. As demonstrated in our previous workshop at ISMR'23, a new Slicer plug-in, SlicerROS2, facilitates the integration of 3D Slicer with hardware and software supported by ROS2 and enables rapid prototyping of an IGRI system Gazebo, which has been extensively used in the ROS community, can be incorporated into such an IGRI system seamlessly. By facilitating the introduction of reliable dynamic simulation to the IGRI community, we hope to remove the barrier presented by purchasing, integrating and maintaining heterogeneous equipment and to accelerate the development cycle by removing time consuming calibration procedures.

Alongside ROS2, Gazebo (formerly called Ignition) introduces a new environment for dynamic simulations in which robots, sensors and environments can be simulated. Although Gazebo supports a broad range of robots, and sensors, these are mainly used in non-medical applications such as GPS, IMU, camera and LIDAR. Gazebo, however, supports sensor plugins, in which new sensors (and actuators) can be implemented and loaded into a simulated world. In this tutorial we will demonstrate how to extend the capability of Gazebo to simulate an ultrasound probe mounted on a robot. The user will load a patient model into a simulated world along with a robot-held ultrasound probe and another robot equipped with a biopsy needle. The images from the probe will be produced by a Gazebo ultrasound plugin and visualized in 3D Slicer by using SlicerROS2 (introduced at ISMR'23). The user will define a target position in 3D Slicer to control the
second robot mounted with the needle to move to the target position. 

The participants of this tutorial will experience the development of designing and building a Gazebo plugin to fill specific IGRI needs, configure the sensor parameters, load and run a simulation. Additionally, the communication between Gazebo and ROS2 will be presented and, finally, SlicerROS2 will be used for rendering the images in Slicer. Participants will also be exposed to the pitfalls of simulated environments by promoting modularity and preserving compatibility with real hardware.

Finally, the steps for translating from simulation to real hardware will be presented by presenting a small scale IGRI task that was previously debugged and tested in simulation. Although the emphasis of this tutorial will be on the simulation, modularity best-practices will be presented to facilitate the dissemination of packages among the community.

**Keywords**: Image-guided interventions, navigation, open-source software, software-hardware integration, dynamic simulation

# Intended audience

Researchers, engineers, and students working in the field of medical robotics and image-guided interventions are welcomed to join. The tutorial would be particularly useful for those who are already engaged or will be engaged in the design, implementation, and clinical translation of a system for image-guided robot-assisted interventions. We strongly recommend the audience to bring their own laptop computers to follow the hands-on tutorial during the session. While the tutorial will not involve coding, some experience of running commands on a UNIX-like system and compiling open-source software using Make or CMake would be helpful.


# Time Table

| Time  | Agenda                                                                              | Speaker           |
|-------|-------------------------------------------------------------------------------------|-------------------|
| 13:30 | Opening remarks and introduction                                                    | Junichi Tokuda    |
| 13:35 | 3D Slicer as a Platform for Image-Guided Therapy                                    | Mariana Bernardes |
| 13:50 | Tongue Tumor Resection - Clinical Background, Challenges and Current Progress       | Jiawai Ge         |
| 14:05 | Introduction to ROS2 and Gazebo                                                     | Simon Leonard     |
| 14:30 | Tutorial Session 1                                                                  | Simon Leonard     |
| 15:30 | Break                                                                               |                   |
| 15:45 | Tutorial Session 2                                                                  | Simon Leonard     |
| 16:45 | Closing                                                                             |                   |


# Preparation

1. [Prerequisite](prerequisite.md)
2. [Running ROS2 on Docker](docker-ros.md)  

# Tutorial

1. [Visualizing your simulation in 3D Slicer](slicer-instructions.md) 
2. [Gazebo/Ignition introduction](https://www.cs.jhu.edu/~sleonard/ismr24.pdf)
   
# Links

- [SSMR/ISMR Workshop](http://www.ismr.gatech.edu)
- [3D Slicer](https://www.slicer.org/)
- [Robot Operating System](http://www.ros.org/)
- [SlicerROS2](https://github.com/rosmed/slicer_ros2_module)


# References

- Connolly L, Deguet A, Leonard S, Tokuda J, Ungi T, Krieger A, Kazanzides P, Mousavi P, Fichtinger G, Taylor RH. [Bridging 3D Slicer and ROS2 for Image-Guided Robotic Interventions. Sensors (Basel)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9324680/). 2022 Jul 17;22(14):5336. doi: 10.3390/s22145336. PMID: 35891016; PMCID: PMC9324680.
- Fedorov A, Beichel R, Kalpathy-Cramer J, Finet J, Fillion-Robin JC, Pujol S, Bauer C, Jennings D, Fennessy F, Sonka M, Buatti J, Aylward S, Miller JV, Pieper S, Kikinis R. [3D Slicer as an image computing platform for the Quantitative Imaging Network](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3466397/). Magn Reson Imaging. 2012 Nov;30(9):1323-41. doi: 10.1016/j.mri.2012.05.001. Epub 2012 Jul 6. PubMed PMID: 22770690; PubMed Central PMCID: PMC3466397.
- Frank T, Krieger A, Leonard S, Patel NA, Tokuda J. [ROS-IGTL-Bridge: an open network interface for image-guided therapy using the ROS environment](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5543207/.) Int J Comput Assist Radiol Surg. 2017 Aug;12(8):1451-1460. doi: 10.1007/s11548-017-1618-1. Epub 2017 May 31. PubMed PMID: 28567563; PubMed Central PMCID: PMC5543207. 
- Tauscher S, Tokuda J, Schreiber G, Neff T, Hata N, Ortmaier T. [OpenIGTLink interface for state control and visualisation of a robot for image-guided therapy systems](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4265315/). Int J Comput Assist Radiol Surg. 2015 Mar;10(3):285-92. doi: 10.1007/s11548-014-1081-1. Epub 2014 Jun 13. PubMed PMID: 24923473; PubMed Central PMCID: PMC4265315. 
- Tokuda J, Fischer GS, Papademetris X, Yaniv Z, Ibanez L, Cheng P, Liu H, Blevins J, Arata J, Golby AJ, Kapur T, Pieper S, Burdette EC, Fichtinger G, Tempany CM, Hata N. [OpenIGTLink: an open network protocol for image-guided therapy environment](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC2811069/). Int J Med Robot. 2009 Dec;5(4):423-34. doi: 10.1002/rcs.274. PubMed PMID: 19621334; PubMed Central PMCID: PMC2811069. 
- Ungi T, Lasso A, Fichtinger G. [Open-source platforms for navigated image-guided interventions](https://www.ncbi.nlm.nih.gov/pubmed/?term=27344106). Med Image Anal. 2016 Oct;33:181-186. doi: 10.1016/j.media.2016.06.011. Epub 2016 Jun 15. Review. PubMed PMID: 27344106.


# Past Workshops

- [ISMR 2019](/ismr2019/index)
- [ISMR 2021](/ismr2021/index)
- [ISMR 2023](/ismr2023/index)


# Organizers

- [Junichi Tokuda, Ph.D.](https://tokuda-lab.bwh.harvard.edu/), Brigham and Women’s Hospital and Harvard Medical School, Boston, MA
- [Axel Krieger, Ph.D.](https://me.jhu.edu/faculty/axel-krieger/), Johns Hopkins University, Baltimore, MD
- [Simon Leonard, Ph.D.](https://www.cs.jhu.edu/~sleonard/), Johns Hopkins University, Baltimore, MD
- [Mark Fuge, Ph.D.](http://ideal.umd.edu/team/mark-fuge), University of Maryland, College Park, MD
- [Anton Deguet](https://malonecenter.jhu.edu/people/anton-deguet/), Johns Hopkins University, Baltimore, MD
- [Laura Connolly](https://medicreate.cs.queensu.ca/all-students/laura-connolly/), Queen's University, Kingston, ON, Canada
- [Lidia Al-Zogbi](https://www.linkedin.com/in/lidia-alzogbi), Johns Hopkins University, Baltimore, MD
- [Mariana Bernardes, Ph.D.](https://tokuda-lab.bwh.harvard.edu/team/mariana-bernardes/), Brigham and Women's Hospital and Harvard Medical, Boston, MA 
- [Pedro Moreira, Ph.D.](https://tokuda-lab.bwh.harvard.edu/team/pedro-moreira/), Brigham and Women's Hospital and Harvard Medical School, Boston, MA

# Acknowledgements

This work is supported in part by:
- U.S. National Institutes of Health (R01EB020667, R01EB020667-05S1, R01EB020610, P41EB028741)
- Ontario Consortium for Adaptive Interventions in Radiation Oncology (OCAIRO)
- SparKit project
- CANARIE’s Research Software Program

![NIBIB Logo](https://www.nibib.nih.gov/sites/default/files/nibib_logo.png){:class="img-responsive"}

![BWH](/images/BWH_Logo.png){:class="img-responsive" width="360px"}
![HMS](/images/HMS_Logo.png){:class="img-responsive" width="200px"}
![JHU](/images/JHU_Logo.png){:class="img-responsive" width="240px"}
![UMD](/images/UMD_Logo.png){:class="img-responsive" width="240px"}
![Queens](/images/Queens_Logo.png){:class="img-responsive" width="200px"}


# Contact

[Junichi Tokuda, Ph.D.](https://scholar.harvard.edu/tokuda)

Associate Professor of Radiology
Brigham and Women's Hospital / Harvard Medical School

tokuda at bwh.harvard.edu



