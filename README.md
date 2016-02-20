# Ubuntu Linux (using binary ROS packages)

## Build and install

### Install prerequisites
~~~
# Install the minimal prerequisites: catkin plus any message packages that you need
sudo apt-get install ros-indigo-catkin ros-indigo-sensor-msgs ros-indigo-geometry-msgs ros-indigo-map-msgs
~~~

### Build the example
~~~
# Get the example code
git clone https://github.com/gerkey/ros1_msg_reuse
cd ros1_msg_reuse
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

# Ubuntu Linux (from source)
TODO

# Mac OSX (from source)

## Build and install

### Install prerequisites
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
# (Note: this step avoids using rosdep to resolve system dependencies and is therefore somewhat brittle.)
sudo -H pip install -U wstool rosinstall rosinstall_generator rospkg catkin-pkg Distribute PyYAML empy argparse
# Make a place to work (could be anywhere)
mkdir ~/ros1_ws
cd ~/ros1_ws
# Download a custom install file for the ROS packages that we're going to use
rosinstall_generator --deps --rosdistro indigo --tar --wet-only sensor_msgs geometry_msgs map_msgs > ros1.repos
# Pull the sources described in the file into a subdirectory `src`
wstool init -j8 src ros1.repos
# Build the ROS packages, installing them into a subdirectory `install_isolated`
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
~~~

### Build the example
~~~
# Get the example code
git clone https://github.com/gerkey/ros1_msg_reuse
cd ros1_msg_reuse
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
