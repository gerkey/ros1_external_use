# Ubuntu Linux (using binary ROS packages)

## Build and install
~~~
# Install the minimal prerequisites: catkin plus any message packages that you need
sudo apt-get install ros-indigo-catkin ros-indigo-sensor-msgs ros-indigo-geometry-msgs ros-indigo-map-msgs
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
TODO
