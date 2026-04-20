---
layout: page
title: SlicerROS2 Setup for ISMR2026 -- 3D Slicer Extensions
permalink: /ismr2026/slicer_setup.html
---


# 3D Slicer Extensions for Segmentation -- nnInteractive

In this tutorial, we will use [nnInteractive](https://github.com/MIC-DKFZ/nnInteractive) to segment the vessels. nnInteractive is an interactive segmentation tool that allows users to segment a tissue region on any 3D image by specifying the region with simple annotations (e.g., points, bouding box, scribbles). nnInteractive uses a server-client computing model, where a client (e.g., 3D Slicer) communicates with the segmentation server to run the segmentation model. The client provides a GUI for the user to annotate the image, and sends those annotations and the image to the server over the network. The server runs the segmentation model and returns the resultant label data back to the user iterface so that the user can immeidately browse the result. The details of the model and the architecture are outlined in [the arxiv paper published by the original developers of nnInteractive](https://arxiv.org/abs/2503.08373).


## Installation

The detailed instruction can be found on [the SlicerNNInteractive GitHub repository](https://github.com/coendevente/SlicerNNInteractive). 

### 3D Slicer Extensions

The 3D Slicer Extension is available on the ExtensionsManager and can be installed with the following steps:

1. Open 3D Slicer. On the ISMR2026 vast.ai container, run `source ~/ros2_ws/install/setup.bash; ros2 launch slicer_ros2_module slicer.launch.py` on the terminal.
2. Open the Extensions Manager (`View` -> `Extensions Manager`).
3. Open the `Install Extensions` tab, and go to the `Segmentation` category.
4. Find `nnInteractive`. (Can be found by typing "nnInteractive" in the search box at the top right corner of the Extensions Manager window.)
5. Click the `Install` button.
6. Restart 3D Slicer.
7. If successful, the extension can be found from the `Modules` pull-down menu (`Segmentation` -> `nnInteractive`).

### nnInteractive Server 

There are multiple ways to run the server. In the following instruction, we use `pip` install to deploy the server. 

First, we create a virtual environment using `venv`:
~~~~
$ cd         # Move to the home directory
$ mkdir nn   # Create a directory for the virtual environment
$ cd nn
$ python3 -m venv .
$ source ./bin/activate
~~~~

To install the server, run the following command. Please note that `"nnunetv2<2.7"` is added to avoid a compatibility issue with a newer `nnunetv2` library.  
~~~
$ pip install nninteractive-slicer-server "nnunetv2<2.7"
~~~

~~~
$ nninteractive-slicer-server --host 0.0.0.0 --port 1527
~~~


[Back to Prerequisites](prerequisites.html) | [Back to Workshop page](index.html)

