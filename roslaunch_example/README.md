# Overview

This directory demonstrates how to install and use roslaunch files with
make. To meet roslaunch's expectations about how our install layout looks,
we put things in particular places and also auto-generate some simple
meta-data files.

# Ubuntu Linux

## Build

~~~
sudo apt-get install ros-indigo-roslaunch ros-indigo-roscpp ros-indigo-std-msgs
export CPATH=/opt/ros/indigo/include:$CPATH
export PKG_CONFIG_PATH=/opt/ros/indigo/lib/pkgconfig:$PKG_CONFIG_PATH
cd ros1_external_use/roslaunch_example
make install
~~~

## Run
~~~
# Start by sourcing the ROS setup file to get several environment
# variables. If you prefer to set them yourself, you'll need:
#     ROS_PACKAGE_PATH=/opt/ros/indigo/share
#     CMAKE_PREFIX_PATH=/opt/ros/indigo:$CMAKE_PREFIX_PATH
#     PATH=/opt/ros/indigo/bin:$PATH
#     PYTHONPATH=/opt/ros/indigo/lib/python2.7/dist-packages:$PYTHONPATH
#     LD_LIBRARY_PATH=/opt/ros/indigo/lib:$LD_LIBRARY_PATH
#     ROS_MASTER_URI=http://localhost:11311 (or whatever you prefer)
. /opt/ros/indigo/setup.sh
# Now extend a couple of environment variables to let roslaunch find your
# project:
export ROS_PACKAGE_PATH=/tmp/myproject/share:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=/tmp/myproject:$CMAKE_PREFIX_PATH
# Launch your talker / listener pair:
roslaunch /tmp/myproject/share/myproject/launch/talker_listener.launch
# Or, shorthand:
# roslaunch myproject talker_listener.launch
~~~

