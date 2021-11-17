November 19, 2021

<!-- Please see [prerequiste](ismr2019/prerequisite), if you are planning to participate.-->

# Table of Contents
- [Overview](#overview)
- [Organizers](#organizers)
- [Time Table](#time)
- [Tutorial](#tutorial)
- [Intended Audience](#audience)
- [Links](#links)
- [Contact](#contact)

# Overview

The goal of this half-day hands-on tutorial is to learn how to integrate medical image computing software into a system for image-guided robot-assisted interventions, in which 2D/3D medical images are used for planning, navigation, monitoring, and validation. Examples of such robot-assisted systems include image-guided robotic needle-guide systems and surgical CAD/CAM systems. Those systems often require a wide range of image computing capabilities such as segmentation of anatomical structures, registration of multiple images, 2D/3D image visualization, image-based planning, and data sharing with the robot controller and the hospital’s picture archiving and communication systems (PACS). Integration of a solid medical image computing platform into a robotic system is becoming more important than ever with the growing interest in AI-based treatment planning and guidance. 
However, the engineering effort to implement those features is often underestimated in academic research due to limited engineering resources or the scope of the project. Fortunately, many of those features have already been implemented and validated in the research community and often distributed as open-source software. Therefore it has become essential for academic researchers to take advantage of those existing tools and incorporate them into their own research instead of reinventing the wheel. 
Following our successful hands-on tutorial hosted at ISMR 2019 and the continued demand for the training materials which was made available online, the team is planning a hands-on tutorial with new contents featuring up-to-date software platforms and AI-based planning/guidance. Like the previous tutorial, the session will consist of presentations by several experts in the field, followed by a hands-on tutorial. The presentations will review the software systems for image-guided robot-assisted interventions with real-world use-cases in the context of academic research. In the tutorial session, the participants will build a system for image-guided robot-assisted interventions on their own laptop computers using state-of-the-art research platforms for medical imaging and robotics, namely 3D Slicer 4.13 and Robot Operating System 2 (ROS2). The two platforms will be integrated using an open network communication protocol, OpenIGTLink. Then the participant will build a demo system that allows a user to plan a procedure on a 3D image using AI-based segmentation, execute the plan using a virtual robot, and visualize the 3D model of the robot along with reformatted images of the patient as feedback. At the end of the tutorial, the participants are expected to have a working demo system on their laptops, which could potentially be used as a template for their own research project.
The workshop will be organized in conjunction with a separate half-day workshop on “Image-guided percutaneous interventions” which will feature a number of talks from experts in this research field. Participants who attend both workshops can gain knowledge about ongoing research topics and hands-on skills to tackle the existing problems. 

**Keywords**: Image-guided interventions, Navigation, Open-source software, Software-hardware integration, Surgical CAD/CAM


[![ROS-IGTL-Bridge Demo](http://img.youtube.com/vi/CA4x5cZQKpk/0.jpg)](https://www.youtube.com/watch?v=CA4x5cZQKpk "ROS-IGTL-Bridge Demo")

# Organizers

- [Junichi Tokuda, Ph.D.](https://scholar.harvard.edu/tokuda), Brigham and Women’s Hospital and Harvard Medical School
- [Tamas Ungi, M.D., Ph.D.](http://perk.cs.queensu.ca/users/ungi), Queen’s University
- [Axel Krieger, Ph.D.](https://me.jhu.edu/faculty/axel-krieger/), Johns Hopkins University
- [Simon Leonard, Ph.D.](https://www.cs.jhu.edu/faculty/simon-leonard/), Johns Hopkins University
- [Mark Fuge, Ph.D.](http://ideal.umd.edu/team/mark-fuge), University of Maryland
- Lydia Al-Zogbi, Johns Hopkins University
- Pedro Moreira, Ph.D., Brigham and Women's Hospital and Harvard Medical School

# Time Table

- 08:30am : Opening remarks and introduction (Junichi Tokuda)
- 08:40am : Building navigation software using 3D Slicer, PLUS, and SlicerIGT (Tamas Ungi)
- 08:50am : Robot-Assisted Vertoplasty (Axel Kerieger)
- 09:00am : Tutorial Session 1: Building a patient model from ultrasound image using 3D Slicer
- 10:00am: (Break)
- 10:30am: Tutorial Session 2: Integrating ROS and 3D Slicer

# Tutorial
- Step 0: [Prerequisite](ismr2021/prerequisite.md)
- Step 1: Setting up environment
  - [3D Slicer](ismr2021/slicer_env.md)
  - [ROS](ismr2021/ros_env.md)
- Step 2: [Testing OpenIGTLink Communication Between Slicer and ROS](ismr2021/ros_igtl_test.md)
- Step 3: [Creating a 3D model for surgical planning](ismr2021/slicer_planning.md)
- Step 4: Setting up [Universal Robot Arm on ROS](ismr2021/fake_robot.md)
- Step 5: [Targeting](ismr2021/ros_targeting.md)

# Intended audience

Researchers, engineers, and students working in the field of medical robotics and image-guided interventions are welcomed to join. The tutorial would be particularly useful for those who are already engaged or will be engaged in the design, implementation, and clinical translation of a system for image-guided robot-assisted interventions. We strongly recommend the audience to bring their own laptop computers to follow the hands-on tutorial during the session. While the tutorial will not involve coding, some experience of running commands on a UNIX-like system and compiling open-source software using Make or CMake would be helpful.


# Links
- [SSMR/ISMR'21 Workshop](http://www.ismr.gatech.edu)
- [Venu: Marcus Nanotechnology Building at Georgia Tech](http://www.ismr.gatech.edu/venue)
- [Prerequiste for this tutorial](prerequisite)
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



