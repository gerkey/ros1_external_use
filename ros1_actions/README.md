# Overview
This directory demonstrates doing code generation for actions and then using the
result in combination with `actionlib`. Each `.action` file is processed into a set of `.msg` files, which are then processed into language-specific (C++ or Python) code. Those messages are used by the libraries from `actionlib` to implement clients and servers. 

# Installing ROS packages
Following the other examples, you need to install `actionlib`, `actionlib_msgs`, and `sensor_msgs`.

# Building
Following the other examples, you can either use CMake or make to build. You need your environment configured to find your ROS installation.

# Running
Following the other examples, you need three terminals: (i) run `roscore`; (ii) run `./actionserver`; and (iii) run `./actionclient`. You need your environment configured to find your ROS installation.

**TODO:**

* document the detailed steps
