Running ROS on Docker
---------------------

P{ull the image using the following command:
~~~~
$ docker pull rosmed/docker-ubuntu-22.04-ros2-slicerros2-lw:ismr2024
~~~~
The image is about 11GB, and it may take a while to complete the process. Once the image is downloaded, run the following command:
~~~~
$ docker run -it --rm -p 6080:80 rosmed/docker-ubuntu-22.04-ros2-slicerros2-lw:ismr2024
~~~~
In this example, the web port on the docker container will be mapped to port 6080 on the host computer. The desktop can be accessed at http://127.0.0.1:6080/ from a web browser.


