---
layout: page
title:  ISMR23 Workshop / Building Software Systems for Image-Guided Robot-Assisted Interventions with SlicerROS2
permalink: /ismr2023/
---

April 19, 2023

# Table of Contents
- [Overview](#overview)
- [Organizers](#organizers)
- [Time Table](#time)
- [Tutorial](#tutorial)
- [Intended Audience](#audience)
- [Links](#links)
- [Contact](#contact)

# Overview

The goal of this half-day hands-on tutorial is to learn how to integrate medical image computing software into a system for image-guided robot-assisted interventions (IGRI), in which 2D/3D medical images are used for planning, navigation, monitoring, and validation of robot-assisted procedures. Those systems often demand the implementation of a wide range of image computing features such as image segmentation, registration, 2D/3D visualization, and informatics, along with robotics features. Today’s growing interest in AI-based treatment planning and guidance is further expanding this demand.

However, the engineering effort to implement common image computing and robotics features is often underestimated in academic research due to limited resources and/or the scope of the project. Fortunately, many of those features have already been implemented, validated, and made available as open-source software in research communities. By combining those features as building blocks, researchers can develop a high-quality IGRI system without reinventing the wheel. Those features are often packaged in a software environment, which provides a unified application program interface (API) and a graphical user interface so that developers and users can combine those blocks with minimal programming effort for their applications. Notable examples for such environemnts are 3D Slicer for medical image computing, and Robot Operating System (ROS) for robotics.

A specific engineering challenge in developing IGRI systems is that no single software environment can support both medical image computing and robotics features. As each software environment has its own internal data representation and data management mechanism, it is not always straightforward to combine two environments and make them work as a single system. This challenge has been partially addressed by ROS-IGTL-Bridge, a software bridge that connects 3D Slicer and ROS via TCP-based socket communication, though this solution merely facilitates low-level data exchange between the two environments. ROS-IGTL-Bridge was demonstrated at our hands-on workshops at ISMR 2019 and 2021.

Following the success of previous workshops and the continued demand for the integration of 3D Slicer and ROS, the team has recently stepped up the effort and developed a brand new 3D Slicer plug-in module called SlicerROS2 . SlicerROS2 directly incorporates ROS version 2’s essential features into 3D Slicer, including messaging middleware (DDS) and a scene graph (tf2), allowing more synchronous and seamless integration between the two platforms than ROS-IGTL-Bridge.

In this last iteration of our tutorial series, the participants will build a simple IGRI system using SlicerROS2 on their own laptop computers. This simple IGRI system will allow a user to plan a procedure on a 3D image, execute the plan using a virtual robot, and visualize the 3D model of the robot along with reformatted images of the patient as feedback. At the end of the tutorial, the participants are expected to have a working demo system on their laptops, which could potentially be used as a template for their own research project. The organizers will also bring a da Vinci Research Kit (dVRK) patient side manipulator (PSM) and have the participants execute their task on a real robot.


**Keywords**: Image-guided interventions, Navigation, Open-source software, Software-hardware integration, Surgical CAD/CAM


[![ROS-IGTL-Bridge Demo](http://img.youtube.com/vi/CA4x5cZQKpk/0.jpg)](https://www.youtube.com/watch?v=CA4x5cZQKpk "ROS-IGTL-Bridge Demo")

# Organizers

- [Junichi Tokuda, Ph.D.](https://scholar.harvard.edu/tokuda), Brigham and Women’s Hospital and Harvard Medical School, Boston, MA
- [Axel Krieger, Ph.D.](https://me.jhu.edu/faculty/axel-krieger/), Johns Hopkins University, Baltimore, MD
- [Simon Leonard, Ph.D.](https://www.cs.jhu.edu/faculty/simon-leonard/), Johns Hopkins University, Baltimore, MD
- [Tamas Ungi, M.D., Ph.D.](http://perk.cs.queensu.ca/users/ungi), Queen’s University, Kingston, ON, Canada
- Anton Deguet, Johns Hopkins University, Baltimore, MD
- Laura Connolly, Queen’s University, Kingston, ON, Canada
- Aravind S Kumar, Johns Hopkins University, Baltimore, MD 
- Lydia Al-Zogbi, Johns Hopkins University, Baltimore, MD
- [Mark Fuge, Ph.D.](http://ideal.umd.edu/team/mark-fuge), University of Maryland, College Park, MD
- Pedro Moreira, Ph.D., Brigham and Women's Hospital and Harvard Medical School, Boston, MA
- Mariana Bernardes, Brigham and Women’s Hospital and Harvard Medical School, Boston, MA 

# Time Table (TBD)

- 08:30am : Opening remarks and introduction (Junichi Tokuda)
- 08:40am : Building navigation software using 3D Slicer, PLUS, and SlicerIGT (Tamas Ungi)
- 08:50am : Robot-Assisted Vertoplasty (Axel Kerieger)
- 09:00am : Tutorial Session 1: Building a patient model from ultrasound image using 3D Slicer
- 10:00am: (Break)
- 10:30am: Tutorial Session 2: Integrating ROS and 3D Slicer

# Tutorial
- Step 0: [Prerequisite](prerequisite)
- Step 1: Setting up environment
  - [3D Slicer](slicer_env)
  - [ROS](ros_env)
- Step 2: [Testing OpenIGTLink Communication Between Slicer and ROS](ros_igtl_test)
- Step 3: [Creating a 3D model for surgical planning](ismr2021/slicer_planning.md)
- Step 4: Setting up [Universal Robot Arm on ROS](ismr2021/fake_robot.md)
- Step 5: [Targeting](ismr2021/ros_targeting.md)

# Intended audience

Researchers, engineers, and students working in the field of medical robotics and image-guided interventions are welcomed to join. The tutorial would be particularly useful for those who are already engaged or will be engaged in the design, implementation, and clinical translation of a system for image-guided robot-assisted interventions. We strongly recommend the audience to bring their own laptop computers to follow the hands-on tutorial during the session. While the tutorial will not involve coding, some experience of running commands on a UNIX-like system and compiling open-source software using Make or CMake would be helpful.


# Links
- [SSMR/ISMR'21 Workshop](http://www.ismr.gatech.edu)
- [Venu: Marcus Nanotechnology Building at Georgia Tech](http://www.ismr.gatech.edu/venue)
- [Prerequisite for this tutorial](prerequisite.md)
- [3D Slicer](https://www.slicer.org/)
- [Robot Operating System](http://www.ros.org/)

# References

- Fedorov A, Beichel R, Kalpathy-Cramer J, Finet J, Fillion-Robin JC, Pujol S, Bauer C, Jennings D, Fennessy F, Sonka M, Buatti J, Aylward S, Miller JV, Pieper S, Kikinis R. [3D Slicer as an image computing platform for the Quantitative Imaging Network](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3466397/). Magn Reson Imaging. 2012 Nov;30(9):1323-41. doi: 10.1016/j.mri.2012.05.001. Epub 2012 Jul 6. PubMed PMID: 22770690; PubMed Central PMCID: PMC3466397.
- Frank T, Krieger A, Leonard S, Patel NA, Tokuda J. [ROS-IGTL-Bridge: an open network interface for image-guided therapy using the ROS environment](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5543207/.) Int J Comput Assist Radiol Surg. 2017 Aug;12(8):1451-1460. doi: 10.1007/s11548-017-1618-1. Epub 2017 May 31. PubMed PMID: 28567563; PubMed Central PMCID: PMC5543207. 
- Tauscher S, Tokuda J, Schreiber G, Neff T, Hata N, Ortmaier T. [OpenIGTLink interface for state control and visualisation of a robot for image-guided therapy systems](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4265315/). Int J Comput Assist Radiol Surg. 2015 Mar;10(3):285-92. doi: 10.1007/s11548-014-1081-1. Epub 2014 Jun 13. PubMed PMID: 24923473; PubMed Central PMCID: PMC4265315. 
- Tokuda J, Fischer GS, Papademetris X, Yaniv Z, Ibanez L, Cheng P, Liu H, Blevins J, Arata J, Golby AJ, Kapur T, Pieper S, Burdette EC, Fichtinger G, Tempany CM, Hata N. [OpenIGTLink: an open network protocol for image-guided therapy environment](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC2811069/). Int J Med Robot. 2009 Dec;5(4):423-34. doi: 10.1002/rcs.274. PubMed PMID: 19621334; PubMed Central PMCID: PMC2811069. 
- Ungi T, Lasso A, Fichtinger G. [Open-source platforms for navigated image-guided interventions](https://www.ncbi.nlm.nih.gov/pubmed/?term=27344106). Med Image Anal. 2016 Oct;33:181-186. doi: 10.1016/j.media.2016.06.011. Epub 2016 Jun 15. Review. PubMed PMID: 27344106.

# Acknowledgements

This work is supported in part by:
- U.S. National Institutes of Health (R01EB020667, R01EB020610, P41EB028741)
- Ontario Consortium for Adaptive Interventions in Radiation Oncology (OCAIRO)
- SparKit project
- CANARIE’s Research Software Program

Contributors:
- Tobias Frank, M.Sc. Leibniz Universität Hannover
- Niravkumar Patel, Ph.D., Johns Hopkins University

# Contact

[Junichi Tokuda, Ph.D.](https://scholar.harvard.edu/tokuda)

Associate Professor of Radiology
Brigham and Women's Hospital / Harvard Medical School

tokuda at bwh.harvard.edu



