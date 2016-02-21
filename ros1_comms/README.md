# Overview

This directory demonstrates building a simple C++ publisher / subscriber
pair (taken from the standard tutorial:
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
with CMake and make. The goal is to show that using the middleware aspects
of ROS is no different from using any other software dependency, with no
unusual impact on develompent.

# Ubuntu Linux

# Mac OSX

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

## Build the example
Like Ubuntu, this step is normal CMake, plus some environment configuration
to find the ROS packages installed in `~/ros1_ws`.
~~~
cd ros1_external_use/ros1_comms
# Set up some environment variables. If you don't want such fine-grained
# control, you could instead do `. ~/ros1_ws/setup.sh`
export CMAKE_PREFIX_PATH=$HOME/ros1_ws/install_isolated:$CMAKE_PREFIX_PATH
export CPATH=$HOME/ros1_ws/install_isolated/include:$CPATH
# Build with cmake as usual
mkdir build
cd build
cmake ..
make
~~~

## Run
Terminal 1:
Run a ROS core, which is necessary to allows ROS nodes to discover each
other:
~~~
# Load the environment configuration needed by roscore
. ~/ros1_ws/install_isolated/setup.sh
roscore
~~~

Terminal 2:
Run the listener:
~~~
# Set ROS_MASTER_URI to point to the roscore that you started (adjust if
# you started roscore on a different machine and/or port):
export ROS_MASTER_URI=http://localhost:11311
# Extend your library path to find the shared objects installed in
# ~/ros1_ws:
export DYLD_LIBRARY_PATH=$HOME/ros1_ws/install_isolated/lib:$DYLD_LIBRARY_PATH
cd ros1_external_use/ros1_comms/build
./listener
~~~

Terminal 3:
Run the talker:
~~~
# Set ROS_MASTER_URI to point to the roscore that you started (adjust if
# you started roscore on a different machine and/or port):
export ROS_MASTER_URI=http://localhost:11311
# Extend your library path to find the shared objects installed in
# ~/ros1_ws:
export DYLD_LIBRARY_PATH=$HOME/ros1_ws/install_isolated/lib:$DYLD_LIBRARY_PATH
cd ros1_external_use/ros1_comms/build
./talker
~~~

You should see the talker printing `hello world` messages and the listener
printing `I heard [hello world]` messages.

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

# Notes

The procedure described above is intended to be as minimal and as explicit
as possible. For example, when running your executables, instead of
manually working with environment variables, you could simply source the
auto-generated `~/ros1_ws/install_isolated/setup.sh` file. We're specifying
the environment variables manually here to show exactly what is required to
make the system work.

The `rosinstall_generator` line prescribed above can be tuned to your
needs. If your application requires a different set of ROS packages, simply
change that line to bring in more or less stuff (but if you're going to be
doing publishing or subscribing, then do keep `roslaunch`, to ensure that
you have the necessary `roscore` service).

