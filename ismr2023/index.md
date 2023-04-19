---
layout: page
title:  ISMR23 Workshop / Building Software Systems for Image-Guided Robot-Assisted Interventions with SlicerROS2
permalink: /ismr2023/
---

![ISMRPhoto](/images/ismr2019_tutorial.jpg){:class="img-responsive" width="400px"}


## Event Date: April 19, 2023

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

The goal of this half-day hands-on tutorial is to learn how to integrate medical image computing software into a system for image-guided robot-assisted interventions (IGRI), in which 2D/3D medical images are used for planning, navigation, monitoring, and validation of robot-assisted procedures. Those systems often demand the implementation of a wide range of image computing features such as image segmentation, registration, 2D/3D visualization, and informatics, along with robotics features. Today’s growing interest in AI-based treatment planning and guidance is further expanding this demand.

However, the engineering effort to implement common image computing and robotics features is often underestimated in academic research due to limited resources and/or the scope of the project. Fortunately, many of those features have already been implemented, validated, and made available as open-source software in research communities. By combining those features as building blocks, researchers can develop a high-quality IGRI system without reinventing the wheel. Those features are often packaged in a software environment, which provides a unified application program interface (API) and a graphical user interface so that developers and users can combine those blocks with minimal programming effort for their applications. Notable examples for such environemnts are 3D Slicer for medical image computing, and Robot Operating System (ROS) for robotics.

A specific engineering challenge in developing IGRI systems is that no single software environment can support both medical image computing and robotics features. As each software environment has its own internal data representation and data management mechanism, it is not always straightforward to combine two environments and make them work as a single system. This challenge has been partially addressed by ROS-IGTL-Bridge, a software bridge that connects 3D Slicer and ROS via TCP-based socket communication, though this solution merely facilitates low-level data exchange between the two environments. ROS-IGTL-Bridge was demonstrated at our hands-on workshops at ISMR 2019 and 2021.

Following the success of previous workshops and the continued demand for the integration of 3D Slicer and ROS, the team has recently stepped up the effort and developed a brand new 3D Slicer plug-in module called SlicerROS2 . SlicerROS2 directly incorporates ROS version 2’s essential features into 3D Slicer, including messaging middleware (DDS) and a scene graph (tf2), allowing more synchronous and seamless integration between the two platforms than ROS-IGTL-Bridge.

In this last iteration of our tutorial series, the participants will build a simple IGRI system using SlicerROS2 on their own laptop computers. This simple IGRI system will allow a user to plan a procedure on a 3D image, execute the plan using a virtual robot, and visualize the 3D model of the robot along with reformatted images of the patient as feedback. At the end of the tutorial, the participants are expected to have a working demo system on their laptops, which could potentially be used as a template for their own research project. The organizers will also bring a da Vinci Research Kit (dVRK) patient side manipulator (PSM) and have the participants execute their task on a real robot.

**Keywords**: Image-guided interventions, Navigation, Open-source software, Software-hardware integration, Surgical CAD/CAM

# Intended audience

Researchers, engineers, and students working in the field of medical robotics and image-guided interventions are welcomed to join. The tutorial would be particularly useful for those who are already engaged or will be engaged in the design, implementation, and clinical translation of a system for image-guided robot-assisted interventions. We strongly recommend the audience to bring their own laptop computers to follow the hands-on tutorial during the session. While the tutorial will not involve coding, some experience of running commands on a UNIX-like system and compiling open-source software using Make or CMake would be helpful.


# Time Table

| Time  | Aganda                                                                              | Speaker           |
|-------|-------------------------------------------------------------------------------------|-------------------|
| 13:30 | Opening remarks and introduction                                                    | Junichi Tokuda    |
| 13:35 | Tongue Tumor Resection - Clinical Background, Challenges and Current Progress       | Lydia Al-Zogbi    |
| 13:50 | 3D Slicer as a Platform for Image-Guided Therapy                                    | Mariana Bernardes |
| 14:05 | Tutorial Session 1 - Building a patient model from ultrasound image using 3D Slicer | Laura Connolly    |
| 14:35 | Tutorial Session 2 - Intro to SlicerROS2                                            | Laura Connolly    |
| 15:05 | Break                                                                               |                   |
| 15:15 | Tutorial Session 3 - Path planning and registration with SlicerROS2                 | Laura Connolly    |
| 16:30 | Closing                                                                             |                   |


# Tutorial
- Step 0: [Prerequisite](prerequisite)
- Step 1: Setting up the environment
  - [Virtual Environment (Docker)](docker_env) 
  - [3D Slicer](slicer_env)
  - [ROS](ros_env)
- Step 2: [Creating an anatomical model from ultrasound images using AI-based segmentation](slicer_model)
- Step 3: [Surgical Planning](slicer_planning)
- Step 4: [Connecting 3D Slicer and ROS](connecting_slicer_ros)

# Links
- [SSMR/ISMR Workshop](http://www.ismr.gatech.edu)
- [Prerequisite for this tutorial](prerequisite.md)
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


# Organizers

- [Junichi Tokuda, Ph.D.](https://scholar.harvard.edu/tokuda), Brigham and Women’s Hospital and Harvard Medical School, Boston, MA
- [Axel Krieger, Ph.D.](https://me.jhu.edu/faculty/axel-krieger/), Johns Hopkins University, Baltimore, MD
- [Simon Leonard, Ph.D.](https://www.cs.jhu.edu/faculty/simon-leonard/), Johns Hopkins University, Baltimore, MD
- [Tamas Ungi, M.D., Ph.D.](http://perk.cs.queensu.ca/users/ungi), Queen’s University, Kingston, ON, Canada
- [Anton Deguet](https://malonecenter.jhu.edu/people/anton-deguet/), Johns Hopkins University, Baltimore, MD
- [Laura Connolly](http://perk.cs.queensu.ca/users/connolly), Queen’s University, Kingston, ON, Canada
- [Aravind S Kumar](https://in.linkedin.com/in/aravind-s-kumar-091571165?original_referer=https%3A%2F%2Fwww.google.com%2F), Johns Hopkins University, Baltimore, MD 
- [Lydia Al-Zogbi](https://www.linkedin.com/in/lidia-alzogbi), Johns Hopkins University, Baltimore, MD
- [Kapi Ketan Mehta](https://www.linkedin.com/in/kapi-ketan-mehta), Johns Hopkins University, Baltimore, MD
- [Mark Fuge, Ph.D.](http://ideal.umd.edu/team/mark-fuge), University of Maryland, College Park, MD
- [Pedro Moreira, Ph.D.](https://scholar.harvard.edu/pedromoreira/home), Brigham and Women's Hospital and Harvard Medical School, Boston, MA
- [Mariana Bernardes, Ph.D.](https://scholar.harvard.edu/bernardes), Brigham and Women’s Hospital and Harvard Medical School, Boston, MA 


# Acknowledgements

This work is supported in part by:
- U.S. National Institutes of Health (R01EB020667, R01EB020667-05S1(Administrative Supplement), R01EB020610, P41EB028741)
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



