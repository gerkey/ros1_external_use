# Overview

This repository demonstrates how to use ROS software in C++ and Python via
CMake and make. The goal is to show that, while ROS itself is **developed**
using catkin, package.xml meta-data files, and other tools that help with
managing a workspace drawn from multiple repositories, you don't have to
use those tools when developing your software that **uses** ROS.

It should be easy to use ROS just as you would any other software
dependency, without giving consideration to the tools that are used
internal to the development of ROS (though you might also find those tools
useful for your own projects). We can do this because (thanks to catkin)
ROS packages all provide CMake configuration files that allow you to
`find_package()` and use them in your own CMake build just like most modern
software.

ROS packages also provide `pkg-config` files, which opens the door to doing
everything directly with make, not CMake.

Each directory demonstrates a different use case. Please explore and report
any problems that you encounter.

# Pre-installation on Ubuntu Linux

On Ubuntu, we provide binary packages for ROS software. To configure your
system to use them, you need to add our repository and key to your system,
like so (assuming that you're using ROS Indigo on Ubuntu trusty):

~~~
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
~~~

Now you're ready to `sudo apt-get install` ROS packages as indicated in the
detailed instructions in each example in this repository.

# Pre-installation on Mac OSX

We need to install a few system dependencies (on Ubuntu this step is done
automatically by the dependencies encoded in the binary packages). We get
those dependencies from a combination of brew and pip:

~~~
# <Install brew and configure your environment to include /usr/local (http://brew.sh)>
# Update brew and add a tap that will provide some ROS-specific dependencies
brew update
brew tap ros/deps
# brew install some stuff
brew install cmake libyaml console_bridge
# Also get pip
sudo easy_install pip
# pip install some stuff
sudo -H pip install -U wstool rosinstall rosinstall_generator rospkg catkin-pkg Distribute
# Initialize rosdep, which we'll use later to make sure that
# package-specific system dependencies are satisfied
sudo rosdep init
rosdep update
~~~

# Get the code (all platforms)

Whatever platform you're on, you'll need to clone this repo:

~~~
git clone https://github.com/gerkey/ros1_external_use
~~~

# TODO

Examples yet to be done include:

* using services (pre-existing and custom)
* using actions (pre-existing and custom)
* making run-time tools like roslaunch, rostopic, and rosmsg interact with your installed software (maybe auto-generate a `package.xml` as part of the install step)
