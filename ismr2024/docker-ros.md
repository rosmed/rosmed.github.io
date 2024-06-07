Running ROS on Docker
=====================

The easiest option to run this tutorial is using our Docker image. It allows you to have a ROS2 environment without a dedicated computer, and can be used on non-Linux environments, including Windows and macOS [^1]. The Docker image is available at [the Docker Hub](https://hub.docker.com/r/rosmed/docker-ubuntu-22.04-ros2-slicerros2-lw), and can be pulled and executed using the following commands. 


Installing the Docker image
---------------------------

Pull the image using the following command:
~~~~
$ docker pull rosmed/docker-ubuntu-22.04-ros2-slicerros2-lw:ismr2024
~~~~
The image is about 11GB, and it may take a while to complete the process. Alternatively, you can load the docker image from a file (NOTE: This is for in-person attendees in [the live tutorial at ISMR2024](https://rosmed.github.io/ismr2024/index)) 
~~~~
$ docker load < ismr2024-docker-slicerros2.tar.gz
~~~~

Running the Docker image
---------------------------

Once the image is downloaded or loaded, run the following command:
~~~~
$ docker run -it --rm -p 6080:80 rosmed/docker-ubuntu-22.04-ros2-slicerros2-lw:ismr2024
~~~~
In this example, the web port on the docker container will be mapped to port 6080 on the host computer. The desktop can be accessed at `https://127.0.0.1:6080/` or `https://localhost:6080/` from a web browser.

![ISMRPhoto](images/dockerRemoteDesktop.png){:class="img-responsive" width="800px"}


(Optional) Running the Docker image on a remote computer
--------------------------------------------------------

If you happen to have SSH access to a remote high-performance computer with a Docker environment, you may run the ROS Docker image on the remote computer and display the desktop on the browser on your local machine. Supposing the hostname of the remote computer is `remote.computer` and your username is `yourusername`, log in to the remote computer and run the Docker command as follows:

~~~~
$ ssh -l `yourusername` -L 6080:localhost:6080 remote.computer
$ screen                                  # To keep the Docker image running even after closing the terminal.
$ docker pull rosmed/docker-ubuntu-22.04-ros2-slicerros2-lw:ismr2024
$ docker run -it --rm -p 6080:80 rosmed/docker-ubuntu-22.04-ros2-slicerros2-lw:ismr2024
~~~~

The `-L` option for the ssh command is used to forward TCP port 6808 on `remote.computer` to TCP port 6808 of your local machine. Once the docker image has started, you can open a Web browser on your local machine, and access https://127.0.0.1:6080/ or https://localhost:6080 from there.


(Optional) Building the Docker image
------------------------------------

This tutorial's Docker image was built using  [Dockerfiles](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/). A Dockerfile contains a series of instructions to build an OS environment. Those instructions are typically commands to obtain an existing base Docker image, install software packages, download files, and compile programs from source code. By using a Dockerfile, developers can easily modify, maintain, and reproduce the Docker image. The Dockerfiles for this tutorials are shared in [the GitHub repository](https://github.com/rosmed/docker-ubuntu-22.04-ros2-novnc/tree/ismr2024), so that the users can modify them and use the image for their purposes (e.g., new tutorials, building testing environments, etc). For the detailed build instruction, please refer to README.md in the Dockerfile repository.

[^1]: The Docker image provided for this tutorial is built for the x86_64 CPU architecture. A Mac with an Apple Silicon CPU will be able to run this Docker image using the emulator, though it might encounter some unexpected errors.


