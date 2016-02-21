# Overview

This directory demonstrates how to use both pre-existing and custom ROS
message structures in C++ and Python via CMake and make.

# Ubuntu Linux (using binary ROS packages)

## Install ROS packages
Installing on Ubuntu is easy because we have pre-packaged binaries.
~~~
# Install the minimal prerequisites: catkin plus any message packages that
# you need.  We have to explicitly install the catkin package here because
# the message packages don't require it but we will need it to get the macros
# for doing code generation on our custom messages.
sudo apt-get install ros-indigo-catkin ros-indigo-sensor-msgs ros-indigo-geometry-msgs ros-indigo-map-msgs
~~~

## Build the example
Here's where it's the normal CMake routine, plus some initial
environment configuration to ensure that we can find the ROS packages that
are installed in `/opt/ros/indigo`.
~~~
cd ros1_external_use/ros1_msgs
# Set up some environment variables. If you don't want such fine-grained
# control, you could instead do `. /opt/ros/indigo/setup.sh`
export CMAKE_PREFIX_PATH=/opt/ros/indigo:$CMAKE_PREFIX_PATH
export PYTHONPATH=/opt/ros/indigo/lib/python2.7/dist-packages:$PYTHONPATH
export CPATH=/opt/ros/indigo/include:$CPATH
# Build with cmake as usual
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/tmp/myproject ..
make install
~~~

## Run
~~~
# Extend your python path to find both your own installation and the ROS installation
export PYTHONPATH=/tmp/myproject/lib/python2.7/dist-packages:/opt/ros/indigo/lib/python2.7/dist-packages:$PYTHONPATH
# Run the executables
/tmp/myproject/bin/use_existing_msg
/tmp/myproject/bin/use_custom_msg
/tmp/myproject/bin/use_msgs.py
~~~

# Mac OSX (from source)

## Install ROS packages
We don't supply binary packages of ROS for OSX, so we'll need to pull the
source and build it. The source lives in multiple repositories, so we'll
use `rosinstall_generator` to dynamically generate a custom recipe to get
the source for just the packages that we want, at the right versions.
~~~
# Make a place to work (could be anywhere)
mkdir ~/ros1_ws
cd ~/ros1_ws
# Download a custom install file for the ROS packages that we're going to
# use, in this case:
#   sensor_msgs : defines the sensor_msgs/LaserScan message that we're using
#   geometry_msgs : defines the geometry_msgs/Pose message that we're using
#   map_msgs : defines the map_msgs/ProjectedMap message that we're using
# Download a custom install file for the ROS packages that we're going to use
rosinstall_generator --deps --rosdistro indigo --tar --wet-only sensor_msgs geometry_msgs map_msgs > ros1.repos
# Pull the sources described in the file into a subdirectory `src`
wstool init -j8 src ros1.repos
# Use rosdep to further install any system dependencies required 
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
# Build the ROS packages, installing them locally into a subdirectory
# `install_isolated`
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
~~~

## Build the example
Like Ubuntu, this step is normal CMake, plus some environment configuration
to find the ROS packages installed in `~/ros1_ws`.
~~~
# Get the example code
cd ros1_external_use/ros1_msgs
# Set up some environment variables. If you don't want such fine-grained
# control, you could instead do `. ~/ros1_ws/setup.sh`
export CMAKE_PREFIX_PATH=$HOME/ros1_ws/install_isolated:$CMAKE_PREFIX_PATH
export PYTHONPATH=$HOME/ros1_ws/install_isolated/lib/python2.7/site-packages:$PYTHONPATH
export CPATH=$HOME/ros1_ws/install_isolated/include:$CPATH
# Build with cmake as usual
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/tmp/myproject ..
make install
~~~

## Run
~~~
# Extend your python path to find both your own installation and the ROS installation
export PYTHONPATH=/tmp/myproject/lib/python2.7/site-packages:$HOME/ros1_ws/install_isolated/lib/python2.7/site-packages:$PYTHONPATH
# Run the executables
/tmp/myproject/bin/use_existing_msg
/tmp/myproject/bin/use_custom_msg
/tmp/myproject/bin/use_msgs.py
~~~

# Using plain make (Ubuntu Linux or OSX)
Because ROS packages also provide `pkg-config` files, we can run the build
from plain make. To try it out, run through the same steps as above, but
when you get to the `mkdir build` step, stop and do the following instead
(to be more specific, you can skip the above step of setting
`CMAKE_PREFIX_PATH`, because we don't need it when using make):

~~~
# Ubuntu, using binary debs
export PKG_CONFIG_PATH=/opt/ros/indigo/lib/pkgconfig:$PKG_CONFIG_PATH
# OSX, using local build
#export PKG_CONFIG_PATH=$HOME/ros1_ws/install_isolated/lib/pkgconfig:$PKG_CONFIG_PATH
make install
~~~

You should get the same result as with CMake, installed to /tmp/myproject.
