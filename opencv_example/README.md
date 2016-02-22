# Overview

There are many ways to improve this little program and make it more general, such as using the ROS CV Bridge, but there is some value in showing just how simple it is to take images from some other framework (in this case, OpenCV), manually populate the sensor_msgs::Image fields, and publish messages to ROS. All using a standalone Makefile, thanks to the pkg-config files from the ROS packages.

# Ubuntu Linux

## Build
~~~
sudo apt-get install ros-indigo-std-msgs ros-indigo-roslaunch ros-indigo-roscpp ros-indigo-sensor-msgs
export CPATH=/opt/ros/indigo/include:$CPATH
export PKG_CONFIG_PATH=/opt/ros/indigo/lib/pkgconfig:$PKG_CONFIG_PATH
cd ros1_external_use/opencv_example
make
~~~

## Run
Terminal 1: ROS core
~~~
. /opt/ros/indigo/setup.sh
roscore
~~~

Terminal 2: image publisher
~~~
export LD_LIBRARY_PATH=/opt/ros/indigo/lib:$LD_LIBRARY_PATH
export ROS_MASTER_URI=http://localhost:11311
./publish_cv_cam
~~~

# Debian GNU/Linux

## Build
~~~
sudo apt-get install libroscpp-dev libsensor-msgs-dev libopencv-dev python-roslaunch
cd ros1_external_use/opencv_example/
make
~~~

## Run
Terminal 1: ROS core
~~~
roscore
~~~

Terminal 2: image publisher
~~~
./publish_cv_cam
~~~
