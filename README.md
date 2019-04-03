Wednesday, April 3, 2018, 1:30-5:00pm

Please see [prerequiste](ismr2019/prerequisite), if you are planning to participate.

# Table of Contents
- [Overview](#overview)
- [Organizers](#organizers)
- [Time Table](#time)
- [Tutorial](#tutorial)
- [Intended Audience](#audience)
- [Links](#links)
- [Contact](#contact)


# Overview

The goal of this hands-on tutorial is to learn how to integrate medical image computing software into a system for image-guided robot-assisted interventions, in which 2D/3D medical images are used for planning, navigation, monitoring, and validation. Examples of such robot-assisted systems include image-guided robotic needle-guide systems and surgical CAD/CAM systems. Those systems often require a wide range of image computing features such as segmentation of anatomical structures, registration of multiple images, 2D/3D image visualization, image-based planning, and data sharing with the robot controller and the hospital’s picture archiving and communication systems (PACS). However, the engineering effort to implement those features is often ignored or underestimated in academic research due to limited engineering resources or the scope of the project. Fortunately, many of those features have already been implemented and validated by other researchers in the research community and often provided as open-source software toolkits. Therefore it has become essential for academic researchers to take advantage of those existing tools and incorporate them into their own research instead of reinventing the wheel.

The session will consist of presentations by several experts in the field, followed by a hands-on tutorial. The presentations will overview the software systems for image-guided robot-assisted interventions with real-world use-cases in the context of academic research. In the tutorial session, the participants will build a system for image-guided robot-assisted interventions on their own laptop computers using popular research platforms for medical imaging and robotics, namely 3D Slicer and Robot Operating System (ROS). The two platforms will be seamlessly integrated using an open network communication protocol OpenIGTLink. Then the participant will build a demo system that allows a user to plan a procedure on a 3D image, execute the plan using a virtual robot, and visualize the feedback from the robot with the 3D image. At the end of the tutorial, the participants are expected to have a working demo system on their laptop, which could potentially be used as a template for their own research project.

**Keywords**: Image-guided interventions, Navigation, Open-source software, Software-hardware integration, Surgical CAD/CAM


[![ROS-IGTL-Bridge Demo](http://img.youtube.com/vi/CA4x5cZQKpk/0.jpg)](https://www.youtube.com/watch?v=CA4x5cZQKpk "ROS-IGTL-Bridge Demo")

# Organizers

- [Junichi Tokuda, Ph.D.](https://scholar.harvard.edu/tokuda), Brigham and Women’s Hospital and Harvard Medical School
- [Tamas Ungi, M.D., Ph.D.](http://perk.cs.queensu.ca/users/ungi), Queen’s University
- [Axel Krieger, Ph.D.](https://mrelab.com/professor-axel-krieger/), University of Maryland
- [Simon Leonard, Ph.D.](https://www.cs.jhu.edu/faculty/simon-leonard/), Johns Hopkins University

# Time Table

- 1:30pm-1:40pm : Opening Remarks and Introduction Junichi Tokuda, Brigham and Women's Hospital/Harvard Medical School
- 1:40pm-2:30pm : Invited Talks
  - 1:40pm : James Ferguson, Vanderbilt University
  - 1:50pm : Nobuhiko Hata, Ph.D., Brigham and Women's Hospital/Harvard Medical School
  - 2:00pm : Axel Kriekger, Ph.D., University of Maryland
  - 2:10pm : Gregory Fischer, Ph.D., Worcester Polytechnic Institute
  - 2:20pm : Tamas Ungi, M.D., Ph.D., Queen's University
- 2:30pm - 3:00pm : Hands-on Tutorial Session 1
- 3:00pm - 3:30pm : Coffee Break
- 3:30pm - 5:00pm : Hands-on Tutorial Session 2


# Tutorial

Please download Tutorial Slide from [here](http://bit.ly/2YIBW8l).

In the Hands-on Tutorial sessions, we will build a virtual needle-guide robot as an example of image-guided robot-assisted interventions systems. The tutorial includes the following steps:

- Setting up ROS and 3D Slicer
- Setting up Universal Robot on ROS
- Planning Procedure on 3D Slicer
- Robot-to-image registration
- Execution of the plan

# Intended audience

Researchers, engineers, and students working in the field of medical robotics and image-guided interventions are welcomed to join. The tutorial would be particularly useful for those who are already engaged or will be engaged in the design, implementation, and clinical translation of a system for image-guided robot-assisted interventions. We strongly recommend the audience to bring their own laptop computers to follow the hands-on tutorial during the session. While the tutorial will not involve coding, some experience of running commands on a UNIX-like system and compiling open-source software using Make or CMake would be helpful.


# Links
- [SSMR/ISMR'19 Workshop](http://www.ismr.gatech.edu)
- [Venu: Marcus Nanotechnology Building at Georgia Tech](http://www.ismr.gatech.edu/venue)
- [Prerequiste for this tutorial](ismr2019/prerequisite)
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
- U.S. National Institutes of Health (R01EB020667, R01EB020610, P41EB015898)
- Ontario Consortium for Adaptive Interventions in Radiation Oncology (OCAIRO)
- SparKit project
- CANARIE’s Research Software Program

Contributors:
- Tobias Frank, M.Sc. Leibniz Universität Hannover
- Niravkumar Patel, Ph.D., Johns Hopkins University

# Contact

[Junichi Tokuda, Ph.D.](https://scholar.harvard.edu/tokuda)

Assistant Professor of Radiology
Brigham and Women's Hospital / Harvard Medical School

tokuda at bwh.harvard.edu



