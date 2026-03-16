---
layout: page
title: ISMR26 Workshop/ SlicerROS2 as an In Silico Testing Environment for Medical Robotics Research
permalink: /ismr2026/
---

![ISMRPhoto](/images/ismr2024.jpg){:class="img-responsive" width="400px"}


## Event Date: April 22, 2026 (Half-Day Workshop, 9:00AM - 12:30PM)

## Venue: The University of Tennessee, Knoxville (Building/Room TBD)

# Table of Contents

- [Overview](#overview)
- [Intended Audience](#intended-audience)
- [Time Table](#time-table)
- [Materials](#materials)
- [Links](#links)
- [Past Workshops](#past-workshops)
- [Organizers](#organizers)
- [Contact](#contact)


# Overview

Robot-assisted surgery and intervention have become integral to medicine over the past two decades. Recent rapid advances in machine learning (ML) have shifted the focus of medical robotics research toward automating parts of surgical tasks to enhance surgeons' skills, reduce cognitive burden, and support decision-making, ultimately improving treatment safety and outcomes. This focus shift has also changed how medical robotics research is carried out, because development of autonomous systems generally requires (a) a large amount of training data for ML, which is often not readily available for preclinical research, and (b) a greater emphasis on *system* and *acceptance testing* of the integrated system rather than *unit* and *integration testing* of individual components. Those requirements promoted a growing demand for an *in silico* simulation environment that allows the observation of the dynamic behaviour of the robotic system without physical robotic hardware.

In recent years, several open-source physics simulation tools have become available to support *in silico* simulation across a wide range of medical and non-medical robotics research, including Gazebo, NVIDIA Isaac, AMBF, SOFA, and MuJoCo. While each tool offers different capabilities, their open-source development model enables roboticists to integrate their robots into the model for *in silico* simulation. There have also been efforts to integrate those simulation tools into a robotic platform, such as the Robot Operating System (ROS), enabling low-code integration of a given robotic system into the simulation environment. Meanwhile, rapid advances in AI-based segmentation tools have made it easier, faster, and more robust to construct patient-specific 3D anatomical models from images. 3D Slicer, a medical image computing software that offers a set of AI-based segmentation tools, has already integrated with ROS through SlicerROS2, enabling a streamlined *in silico* simulation workflow, including the generation of a patient-specific organ model, executing dynamic simulation, and visualizing the outcome.

This workshop will focus on *in silico* testing of a custom robot for medical robotics research using open-source tools. The workshop will consist of two parts: A presentation session will offer a series of lectures that provide an overview of open-source software platforms for the development and in silico testing of image-guided robotic surgical systems. The workshop participants will be introduced to the latest efforts to achieve interoperability among 3D Slicer, ROS2, AMBF, and Gazebo. We will also cover various open-source tools developed by the community for training and deploying AI models for surgical robotic systems.

The presentation session is followed by a tutorial in which an *in silico* simulation of the surgical robot system is demonstrated using 3D Slicer and ROS. Using a demo robot implemented in ROS and an example clinical image from a public database, participants will learn how to (a) set up a virtual robot on 3D Slicer and ROS2 to visualize and control it, (b) create a 3D geometric model of the subject on 3D Slicer using AI-based segmentation tools, and (c) run dynamic simulation to test the robot's ability to navigate through the workspace inside the subject. All source code, data, and instructions will be provided so that participants can test the demo on their own computers, either on-site during the event or at home after the event.

**Keywords**: Image-guided interventions, Navigation, Open-source software, Software-hardware integration, In silico simulation


# Intended Audience

Researchers, engineers, and students working in medical robotics and image-guided interventions are welcome to join. The tutorial sessions would be particularly useful for those already engaged in, or who will be engaged in, the design, implementation, and clinical translation of a system for image-guided robot-assisted interventions. For the hands-on session, we strongly recommend that participants bring their own laptops to follow the tutorial. While the tutorial will not involve coding, some experience in running commands on a UNIX-like system and compiling open-source software using Make or CMake would be helpful.


# Time Table

| Time  | Agenda                                                                                          | Speaker        |
|-------|-------------------------------------------------------------------------------------------------|:--------------:|
| 9:00  | **Session 1: Showcasing Medical Robotics Research with Open Source Software and In Silico Simulation Tools** | |
|       | Overview of open-source platforms for image-guided robotic surgical systems                     | TBD            |
|       | Interoperability among 3D Slicer, ROS2, AMBF, NVIDIA Isaac, Gazebo, and SOFA                   | Adnan Munawar  |
|       | TBD                                                                                             | Rafael Palomar |
|       | TBD                                                                                             | Ruofeng Wei    |
| 10:30 | **Session 2: Hands-On Tutorial on In Silico Medical Robot Testing (Part 1)**                    | Junichi Tokuda |
|       | In silico evaluation of an example surgical robot in a virtual patient using 3D Slicer, ROS2, and Gazebo | |
| 11:30 | **Session 3: Hands-On Tutorial on In Silico Medical Robot Testing (Part 2)**                    | Laura Connolly |
| 12:30 | Closing                                                                                         |                |


# Materials

We will post materials here a week before the event.


# Links

- [ISMR Workshop](http://www.ismr.gatech.edu)
- [3D Slicer](https://www.slicer.org/)
- [Robot Operating System](http://www.ros.org/)
- [SlicerROS2](https://github.com/rosmed/slicer_ros2_module)


# Past Workshops

- [ISMR 2025](/ismr2025/)
- [ISMR 2024](/ismr2024/index)
- [ISMR 2023](/ismr2023/index)
- [ISMR 2021](/ismr2021/index)
- [ISMR 2019](/ismr2019/index)


# Organizers

- [**Junichi Tokuda**](https://tokuda-lab.bwh.harvard.edu/team/junichi-tokuda/), Brigham and Women's Hospital and Harvard Medical School, Boston, MA, USA
- [**Laura Connolly**](https://labs.cs.queensu.ca/perklab/members/laura-connolly/), Johns Hopkins University, Baltimore, MD, USA
- [**Simon Leonard**](https://www.cs.jhu.edu/~sleonard/), Johns Hopkins University, Baltimore, MD, USA
- [**Lydia Al-Zogbi**](https://www.vanderbilt.edu/vise/visepeople/lidia-al-zogbi/), Vanderbilt University, Nashville, TN, USA
- [**Mariana Bernardes**](https://tokuda-lab.bwh.harvard.edu/team/mariana-bernardes/), Brigham and Women's Hospital and Harvard Medical School, Boston, MA, USA
- [**Anton Deguet**](https://malonecenter.jhu.edu/people/anton-deguet/), Johns Hopkins University, Baltimore, MD, USA
- [**Axel Krieger**](https://me.jhu.edu/faculty/axel-krieger/), Johns Hopkins University, Baltimore, MD, USA
- [**Pedro Moreira**](https://tokuda-lab.bwh.harvard.edu/team/pedro-moreira/), Brigham and Women's Hospital and Harvard Medical School, Boston, MA, USA


# Contact

[Junichi Tokuda, Ph.D.](https://tokuda-lab.bwh.harvard.edu/team/junichi-tokuda/)

Associate Professor of Radiology
Brigham and Women's Hospital / Harvard Medical School
