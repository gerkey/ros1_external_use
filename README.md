# Ubuntu Linux

## Build and install
~~~
sudo apt-get install ros-indigo-sensor-msgs ros-indigo-geometry-msgs ros-indigo-map-msgs
cd ros1_msg_reuse
# Set up some environment variables. If you don't want such fine-grained
# control, you could instead do `. /opt/ros/indigo/setup.sh`
export CMAKE_PREFIX_PATH=/opt/ros/indigo:$CMAKE_PREFIX_PATH
export PYTHONPATH=/opt/ros/indigo/lib/python2.7/site-packages:$PYTHONPATH
export CPATH=/opt/ros/indigo/include:$CPATH
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/tmp/myproject ..
make install
~~~

## Run
~~~
export PYTHONPATH=/tmp/myproject/lib/python2.7/site-packages:/opt/ros/indigo/lib/python2.7/site-packages:$PYTHONPATH
/tmp/myproject/bin/foo
/tmp/myproject/bin/bar
python /tmp/myproject/bin/use_msgs.py
~~~

