# Overview

This directory demonstrates how to install and use roslaunch files with
make. To meet roslaunch's expectations about how our install layout looks,
we put things in particular places and also auto-generate some simple
meta-data files.

# Mac OSX

## Install ROS packages
~~~
# Make a place to work (could be anywhere)
mkdir ~/ros1_ws
cd ~/ros1_ws
# Download a custom install file for the ROS packages that we're going to
# use, in this case:
#   roslaunch : provides the `roscore` name server that we'll need at run-time
#   roscpp : the C++ client library that we're compiling and linking against
#   std_msgs : the package that defines the std_msgs/String message type that we're using
rosinstall_generator --deps --rosdistro indigo --tar --wet-only roslaunch roscpp std_msgs > ros1.repos
# Pull the sources described in the file into a subdirectory `src`
wstool init -j8 src ros1.repos
# Use rosdep to further install any system dependencies required 
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
# Build the ROS packages, installing them locally into a subdirectory
# `install_isolated`
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
~~~

## Build
~~~
export CPATH=$HOME/ros1_ws/install_isolated/include:$CPATH
export PKG_CONFIG_PATH=$HOME/ros1_ws/install_isolated/lib/pkgconfig:$PKG_CONFIG_PATH
cd ros1_external_use/roslaunch_example
make install
~~~

## Run
~~~
# Start by sourcing the ROS setup file to get several environment
# variables. If you prefer to set them yourself, you'll need:
#     ROS_PACKAGE_PATH=$HOME/ros1_ws/install_isolated/share
#     CMAKE_PREFIX_PATH=$HOME/ros1_ws/install_isolated:$CMAKE_PREFIX_PATH
#     PATH=$HOME/ros1_ws/install_isolated/bin:$PATH
#     PYTHONPATH=$HOME/ros1_ws/install_isolated/lib/python2.7/dist-packages:$PYTHONPATH
#     LD_LIBRARY_PATH=$HOME/ros1_ws/install_isolated/lib:$LD_LIBRARY_PATH
#     ROS_MASTER_URI=http://localhost:11311 (or whatever you prefer)
. $HOME/ros1_ws/install_isolated/setup.sh
# Now extend a couple of environment variables to let roslaunch find your
# project:
export ROS_PACKAGE_PATH=/tmp/myproject/share:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=/tmp/myproject:$CMAKE_PREFIX_PATH
# Launch your talker / listener pair:
roslaunch /tmp/myproject/share/myproject/launch/talker_listener.launch
# Or, shorthand:
# roslaunch myproject talker_listener.launch
~~~
