---
layout: page
title: "EMBC26 Workshop / Prototyping Image-Guided Medical Robotic Systems: A Hands-On Workshop and Tutorial"
permalink: /embc2026/
---

<div style="position:relative;padding-bottom:56.25%;height:0;max-width:800px;border-radius:4px;overflow:hidden;margin-bottom:1em;">
  <iframe src="https://www.youtube.com/embed/7v0s-cNiC9A" title="YouTube video player" style="position:absolute;top:0;left:0;width:100%;height:100%;border:0;" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>


# Event Details
- Date: July 26, 2026 (Half-Day Workshop, 1:00PM - 5:00PM)
- Venue: Room 703, 255 Front St W, Toronto, ON M5V 2W6, Canada


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

The development of image-guided medical robotics is transforming the landscape of healthcare today. These tools offer several clinical advantages, such as dexterous manipulation of surgical instruments, surgical task automation, accurate digital patient models, and intraoperative tissue characterization. However, creating such systems requires the integration of AI, robots, imaging devices, and tracking equipment. This integration is a challenging barrier to entry for researchers to overcome before they can even begin innovating. Furthermore, many medical robotic systems rely on custom APIs or expensive research licenses that make them inaccessible. 
Recently, several open-source platforms have gained momentum to facilitate the integration of various software and hardware components into a single system. These platforms enable modular and component-based development for rapid prototyping, reducing the initial startup burden. These tools can also incorporate dynamic robot simulation, allowing for a seamless transition between the virtual and physical environments for testing, which is crucial for the development of intelligent and AI-driven medical robotics.  
This workshop will introduce participants to the landscape of image-guided robotics while providing a practical hands-on tutorial and demonstration. Our aim is for participants to leave with a clear understanding of how to begin prototyping medical robotic systems tailored to their own clinical use cases.


**Keywords**: Image-guided interventions, Navigation, Open-source software, Software-hardware integration, Motion planning


# Intended Audience

Researchers, engineers, and students working in medical robotics and image-guided interventions are welcome to join. The tutorial sessions would be particularly useful for those already engaged in, or who will be engaged in, the design, implementation, and clinical translation of a system for image-guided robot-assisted interventions. For the hands-on session, we strongly recommend that participants bring their own laptops to follow the tutorial. While the tutorial will not involve coding, some experience in running commands on a UNIX-like system and compiling open-source software using Make or CMake would be helpful.

# Time Table

| Time            | Agenda                                                              | Speaker         |
|-----------------|---------------------------------------------------------------------|:---------------:|
| 1:00 – 1:05 pm  | Welcome, Introductions, and Workshop Overview                       | Everyone        |
| 1:05 – 1:50 pm  | Foundations of Image-Guided Medical Robotics                        | Pedro Moreira   |
| 1:50 – 2:25 pm  | Follow-along: Building a navigation-ready surgical scene                | Laura Connolly  |
| 2:25 – 2:55 pm  | Follow-along: Robot registration to the patient / phantom               | Michelle Song   |
| 2:55 – 3:10 pm  | Break                                                               | —               |
| 3:10 – 3:45 pm  | Follow-along: Motion planning with SlicerROS2 and MoveIt                | Kaito Hara-Lee  |
| 3:45 – 4:15 pm  | Follow-along: Designing an Exemplary 3D Slicer Task Module for a Cobot  | Lueder A. Kahrs |
| 4:15 – 5:00 pm  | Hands-on rotation with robots (Participants will be split into 3 groups and rotate through stations)   | Everyone        |

# Materials

The tutorial sessions will follow the workflow outlined in the following pages. Participants are expected to perform the follow-along sessions in SlicerROS2 using a cloud-based Linux desktop environment provided by this workshop. The all materials of this hands-on tutorial, including source codes and instructions, are all open-sourced and provided free of charge. 

## Preparation
- [Prerequisite](/embc2026/prerequisites)
- [Launching a container on vast.ai](/embc2026/container)

## Tutorial
- [Tutorial Part 1](/embc2026/tutorial_part1)
- [Tutorial Part 2](/embc2026/tutorial_part2)
- [Tutorial Part 3](/embc2026/tutorial_part3)
- [Tutorial Part 4](/embc2026/tutorial_part4)


# Links

- [3D Slicer](https://www.slicer.org/)
- [Robot Operating System](http://www.ros.org/)
- [SlicerROS2](https://github.com/rosmed/slicer_ros2_module)


# Past Workshops

TBD

# Organizers

- [**Junichi Tokuda**](https://tokuda-lab.bwh.harvard.edu/team/junichi-tokuda/), Brigham and Women's Hospital and Harvard Medical School, Boston, MA, USA ([tokuda@bwh.harvard.edu](mailto:tokuda@bwh.harvard.edu))
- [**Laura Connolly**](https://labs.cs.queensu.ca/perklab/members/laura-connolly/), Johns Hopkins University, Baltimore, MD, USA
- [**Pedro Moreira**](https://tokuda-lab.bwh.harvard.edu/team/pedro-moreira/), Brigham and Women's Hospital and Harvard Medical School, Boston, MA, USA ([plopesdafrotamoreira@bwh.harvard.edu](mailto:plopesdafrotamoreira@bwh.harvard.edu))
- [**Lueder A. Kahrs**](https://bme.utoronto.ca/faculty-research/core-faculty/lueder-kahrs/), University of Toronto, Toronto, ON, Canada ([lueder.kahrs@utoronto.ca](mailto:lueder.kahrs@utoronto.ca))
- [**Kaito Hara-Lee**](https://labs.cs.queensu.ca/perklab/members/kaito-hara-lee/), Queen's University, Kingston, ON, Canada
- **Michelle Song**, Johns Hopkins University, Baltimore, MD, USA ([msong33@jh.edu](mailto:msong33@jh.edu))
- [**Simon Leonard**](https://www.cs.jhu.edu/~sleonard/), Johns Hopkins University, Baltimore, MD, USA ([sleonard@jhu.edu](mailto:sleonard@jhu.edu))
- [**Lidia Al-Zogbi**](https://www.vanderbilt.edu/vise/visepeople/lidia-al-zogbi/), Vanderbilt University, Nashville, TN, USA ([lalzogb1@jhu.edu](mailto:lalzogb1@jhu.edu))
- [**Mariana Bernardes**](https://tokuda-lab.bwh.harvard.edu/team/mariana-bernardes/), Brigham and Women's Hospital and Harvard Medical School, Boston, MA, USA ([mcostabernardesmatias@bwh.harvard.edu](mailto:mcostabernardesmatias@bwh.harvard.edu))
- [**Anton Deguet**](https://malonecenter.jhu.edu/people/anton-deguet/), Johns Hopkins University, Baltimore, MD, USA ([anton.deguet@jhu.edu](mailto:anton.deguet@jhu.edu))
- [**Axel Krieger**](https://me.jhu.edu/faculty/axel-krieger/), Johns Hopkins University, Baltimore, MD, USA ([axel@jh.edu](mailto:axel@jh.edu))


# Acknowledgements

This work is supported in part by NIH grants (R01EB020667, R01EB034359, R01EB036015, R01CA235134) and [NIH-funded National Center for Image Guided Therapy at Brigham and Women's Hospital (P41EB028741)](https://www.ncigt.org).


# Contact

[Junichi Tokuda, Ph.D.](https://tokuda-lab.bwh.harvard.edu/team/junichi-tokuda/)

Associate Professor of Radiology
Brigham and Women's Hospital / Harvard Medical School
