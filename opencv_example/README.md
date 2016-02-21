# Overview

There are many ways to improve this little program and make it more general, such as using the ROS CV Bridge, but there is some value in showing just how simple it is to take images from some other framework (in this case, OpenCV), manually populate the sensor_msgs::Image fields, and publish messages to ROS. All using a standalone Makefile, thanks to the pkg-config files from the ROS packages.

# Ubuntu Linux

~~~
sudo apt-get install ros-indigo-std-msgs ros-indigo-roslaunch ros-indigo-roscpp ros-indigo-sensor-msgs 
cd ros1_external_use/opencv_example
make
~~~

