# To use this file, you need to comment out one line in a file in your ROS installation. In:
#   /opt/ros/<DISTRO>/share/rmw_implementation/cmake/rmw_implementation-extras.cmake
# Find this line near the bottom and comment it out:
#   #find_package(Threads REQUIRED)
# In compensation, we added -lpthread explicitly on the link line below.
# 
# This shouldn't be necessary, but cmake's --find-package support is kind of brittle.

deps = rclcpp std_msgs
cflags = $(foreach dep, $(deps), $(shell cmake --find-package -DNAME=$(dep) -DCOMPILER_ID=GNU -DLANGUAGE=C -DMODE=COMPILE))
lflags = $(foreach dep, $(deps), $(shell cmake --find-package -DNAME=$(dep) -DCOMPILER_ID=GNU -DLANGUAGE=C -DMODE=LINK))

all: pub

%: %.cpp
	$(CXX) -Wall -o $@ $(cflags) $< $(lflags) -lpthread
