project = myproject
install_prefix ?= /tmp/$(project)

all: talker listener

# Get flags for the packages we're using
roscpp_cflags = $(shell pkg-config --cflags roscpp)
std_msgs_cflags = $(shell pkg-config --cflags std_msgs)
roscpp_libs = $(shell pkg-config --libs roscpp)
std_msgs_libs = $(shell pkg-config --libs std_msgs)

# There are some incompatibility issues with some versions of pkg-config
# and some linkers regarding the -l:/absolute/path syntax. As a quick
# workaround, we'll remove the leading -l: prefix, leaving just the
# absolute path.
roscpp_libs_nocolon = $(subst -l:,,$(roscpp_libs))
std_msgs_libs_nocolon = $(subst -l:,,$(std_msgs_libs))

# Build the executables
%: %.cpp
	$(CXX) -Wall -o $@ $(roscpp_cflags) $(std_msgs_cflags) $< $(roscpp_libs_nocolon) $(std_msgs_libs_nocolon)

# Install stuff
install: talker listener
	# Install the executables to <prefix>/lib/<project>, so that
	# they can be found by rosrun/roslaunch.
	mkdir -p $(install_prefix)/lib/$(project)
	cp -a talker listener $(install_prefix)/lib/$(project)
	# Install the .launch file
	mkdir -p $(install_prefix)/share/$(project)/launch
	cp -a talker_listener.launch $(install_prefix)/share/$(project)/launch
	# Create a simple package.xml, which is necessary to be treated as
	# a ROS package.
	echo "<package><name>$(project)</name></package>" > $(install_prefix)/share/$(project)/package.xml
	# Create an empty .catkin marker file, which causes
	# rosrun/roslaunch to look in <prefix>/lib/project for executables
	# for each <prefix> in CMAKE_PREFIX_PATH
	touch $(install_prefix)/.catkin

uninstall:
	rm -f $(install_prefix)/lib/$(project)/talker $(install_prefix)/lib/$(project)/listener $(install_prefix)/share/$(project)/talker_listener.launch $(install_prefix)/share/$(project)/package.xml $(install_prefix)/.catkin

# Clean up
clean:
	rm -f talker listener
