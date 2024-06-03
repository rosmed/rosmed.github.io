Running ROS on Docker
---------------------

Pull the image using the following command:
~~~~
$ docker pull rosmed/docker-ubuntu-22.04-ros2-slicerros2-lw:ismr2024
~~~~
The image is about 11GB, and it may take a while to complete the process. Instead, you can load the docker image from a file. 
~~~~
$ docker load < ismr2024-docker-slicerros2.tar.gz
~~~~

Once the image is downloaded or loaded, run the following command:
~~~~
$ docker run -it --rm -p 6080:80 rosmed/docker-ubuntu-22.04-ros2-slicerros2-lw:ismr2024
~~~~
In this example, the web port on the docker container will be mapped to port 6080 on the host computer. The desktop can be accessed at http://127.0.0.1:6080/ from a web browser.


