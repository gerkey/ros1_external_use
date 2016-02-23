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
any problems that you encounter. Also, if there are use cases that you'd like to see demonstrated, please open an issue.

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

# Preliminary documentation
Here we begin to document in a definitive manner the tools and techniques that
are being demonstrated in the examples.

## Build system choices
We recommend using either CMake or make. But you should be able to port from
make to pretty much any other build system.

## Environment configuration for building
If your ROS packages are installed to a system standard location, such as `/usr`
(this is where the new upstream Debian packages get installed), then you
probably don't need any special configuration. However, if your packages are
installed to somewhere like `/opt/ros` (using the Ubuntu packages from OSRF) or
in your home directory (if you built from source), then you'll need to modify
some environment variables to allow all the various CMake, C++, and Python
assets to be located.

To get your environment configured, you have two choices:

1. Source the setup file that's provided with your installation. This is the
easiest way to get the environment configuration. E.g., if you are using ROS
Indigo from the OSRF packages, then you would do:

        . /opt/ros/indigo/setup.sh

1. Set the required variables manually. If your packages are installed to
`<prefix>` (e.g., `/opt/ros/indigo`, or `$HOME/ros1_ws/install_isolated`), then
the following commands will get you configured for building:

        # To find_package() things from CMake, you need your ROS installation on
        # your CMAKE_PREFIX_PATH.
        export CMAKE_PREFIX_PATH=<prefix>:$CMAKE_PREFIX_PATH
        # To pkg-config things from make, you need your ROS installation on
        # your PKG_CONFIG_PATH.
        export PKG_CONFIG_PATH=<prefix>/lib/pkgconfig:$PKG_CONFIG_PATH
        # To find headers installed to <prefix>/include, you need to modify
        # CPATH.
        export CPATH=<prefix>/include:$CPATH
        # The following line may require modification depending on your Python
        # version and your system type (`dist-packages` vs. `site-packages).
        export PYTHONPATH=<prefix>/lib/python2.7/dist-packages:$PYTHONPATH
        # Add the shared library location. This isn't actually needed for
        # building, but it's easier to list it here.
        # Linux version: 
        export LD_LIBRARY_PATH=<prefix>/lib:$LD_LIBRARY_PATH
        # OSX version:
        #export DYLD_LIBRARY_PATH=<prefix>/lib:$DYLD_LIBRARY_PATH

## Environment configuration for running
After building, you normally install your software somewhere before executing
it. This isn't always necessary, but it usually eventualy is as you start do
more complex things, like code generation. The requirements for run-time
environment configuration are:

1. Start with a subset of the build-time environment configuration.
Specifically, you need: `CMAKE_PREFIX_PATH` and `PYTHONPATH` (again, you could
just source the ROS setup file to get all the variables).
1. Add environment configuration for your installed software, assuming that it's
installed at `<install_prefix>`. A good start is:

        export CMAKE_PREFIX_PATH=<install_prefix>:$CMAKE_PREFIX_PATH
        # Modify this line as needed for your installation choices:
        export PYTHONPATH=<install_prefix>/lib/python2.7/dist-packages:$PYTHONPATH

## Building C++ programs
To build your C++ application code against ROS packages, you need to assemble
the right flags to pass to the compiler and linker.

### CMake
ROS packages follow the CMake configuration protocol, which means that you can
just `find_package()` each one and then using the resulting variables that are
defined. To build an executable that relies on package `foo`:
`foo_INCLUDE_DIRS`, `foo_LIBRARIES`, 
E.g., if you're going to use `roscpp`:
~~~
find_package(foo REQUIRED)
include_directories(${foo_INCLUDE_DIRS})
add_executable(myprogram myprogram.cpp)
target_link_libraries(myprogram ${foo_LIBRARIES})
~~~
Notes:
* You don't have to call `link_directories(${foo_LIBRARY_DIRS})` because ROS
packages follow the recommended practice of returning absolute paths in
`${foo_LIBRARIES}`.
* Another variable of interst in `${foo_DIR}`, which points to
`<prefix>/share/foo`; you might start there when locating package-specific
assets.

### make
ROS packages provide `pkg-config` files that let you get build flags from make
(or from a shell script or the command line, or wherever). The equivalent of the
CMake example using package `foo` looks like this in make:
~~~
foo_cflags = $(shell pkg-config --cflags foo)
foo_libs = $(shell pkg-config --libs foo)
# Work around a known issue with new linkers that don't accept -l:/path/to/lib
foo_libs_nocolon = $(subst -l:,,$(foo_libs))
myprogram: myprogram.cpp
	$(CXX) -Wall -o $@ $(foo_cflags) $< $(foo_libs_nocolon)
~~~
Notes:
* You can get just the include dirs by calling `pkg-config --cflags-only-I foo`.
* You might also call `pkg-config --variable=prefix foo` if you need to get the
directory containing the package to locate other assets.

## Using pre-existing ROS messages in C++
If you're using messages defined in an installed ROS package from your C++
program, then you just need to follow the steps from the previous section to
build your program with the flags provided by that package.

## Using pre-existing ROS messages in Python
Using messages defined in an installed ROS package from your Python program
doesn't require any special configuration. As long as your `PYTHONPATH` points
to your ROS installation, things like `from std_msgs.msg import String` will
just work.

## Doing code generation for custom messages
If you define your own ROS messages by writing `.msg` files in your own
application, then you'll need to run the code generator(s) to produce the code
required to instantiate the corresponding structures in memory.

Notes:
* If you're using custom messages in Python, you should always `make install`
and get your runtime environment configuration set up before trying to run any
programs. Otherwise, you won't be able to import your generated Python classes.

### CMake
Calling message code generators from CMake is simplified because the `gencpp`
package provides macros to help. Let's say that you have two custom messages,
`Foo.msg` and `Bar.msg`, and that they in turn use messages from two installed
ROS packages `foo_msgs` and `bar_msgs`. Then you would do message generation like so:
~~~
# Give the project a name (good practice in general and required by the code
# generators to decide how to namespace and where to produce their outputs).
project(myproject)
# This line is temporarily required because of a bug in genmsg that will be fixed soon:
# https://github.com/ros/genmsg/pull/61
find_package(catkin REQUIRED)
# The genmsg package gives us the code generator macros
find_package(genmsg REQUIRED)
# We need to find_package() each ROS package that defines messages that our
# message use.
find_package(foo_msgs REQUIRED)
find_package(bar_msgs REQUIRED)
# Enumerate our custom message files
add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} FILES Foo.msg Bar.msg)
# Do code generation, specifying which other message packages we depend on.
generate_messages(DEPENDENCIES foo_msgs bar_msgs)
~~~
That's code generation all set up. Now let's build a program that uses the
output:
~~~
# Build a program that uses the custom messages
add_executable(use_custom_msg use_custom_msg.cpp)
# Ensure that code generation happens before building use_custom_msg by
# depending on a special target created by generate_messages().
add_dependencies(use_custom_msg ${PROJECT_NAME}_generate_messages)
~~~
Notes:
* The C++ code for `Foo.msg` will be in `devel/include/myproject/Foo.h` and will define a
message class `myproject::Foo`. Your include directories are automatically
modified to allow your code to `#include <myproject/Foo.h>`.
* The Python code for `Foo.msg` will be in
`devel/lib/python2.7/dist-packages/myproject` (or similar) and will define a
Python class `Foo` that can be imported from the module `myproject.msg`.
* Depending on what ROS client libraries you have installed, code generators for
other languages might run (e.g., for LISP). We don't cover those here.
* All the generated code is automatically registered for installation when you
later do `make install`.

### make
Compared to CMake, it takes a bit more effort to call the message code
generators from make, but it's definitely doable. Taking the example from above
(`Foo.msg` and `Bar.msg` depend on messages from `foo_msgs` and `bar_msgs`),
here's what you would do:
~~~
# Give the project a name (good practice in general and required by the code
# generators to decide how to namespace and where to produce their outputs).
project = myproject
# Get flags for the message packages that we depend on
foo_msgs_includes = `pkg-config --cflags-only-I foo_msgs`
bar_msgs_includes = `pkg-config --cflags-only-I bar_msgs`
# Enumerate our custom messages
msgs = Foo.msg Bar.msg
# Compute the expected outputs from the code generators
msgs_cpp = $(foreach msg, $(msgs), $(project)/$(basename $(msg)).h)
msgs_py = $(foreach msg, $(msgs), $(project)/msg/_$(basename $(msg)).py)
msgs_py_init = $(project)/msg/__init__.py
# Find C++ and Python code generators
gencpp_dir = $(shell pkg-config --variable=prefix gencpp)
genpy_dir = $(shell pkg-config --variable=prefix genpy)
# Compute include directories for all message packages. This is an overinclusive
# hack because it would be painful to recursively gather the message package
# dependencies (though in principle it can be done by calling
# `pkg-config --print-requires <pkg>` for each one). The goal is to end up with
# a set of strings of the form:
#  -Ifoo_msgs:/path/to/foo_msgs/msg -Ibar_msgs:/path/to/bar_msgs/msg
pkg_msg_dirs = $(wildcard $(gencpp_dir)/share/*/msg)
pkg_msg_includes = $(foreach dir, $(pkg_msg_dirs), -I$(shell basename `dirname $(dir)`):$(dir))
# General rule for doing C++ code generation. Args:
#  $(pkg_msg_includes) : collection of args of the form `-I:foo_msgs:/path/to/foo_msgs`
#  -p $(project) : the "package" name; determines the namespace used in the code
#  -o $(project) : the output directory name
#  -e $(gencpp_dir)/share/gencpp : location of templates needed by gencpp
$(project)/%.h: %.msg
	$(gencpp_dir)/lib/gencpp/gen_cpp.py $(pkg_msg_includes) -p $(project) -o $(project) -e $(gencpp_dir)/share/gencpp $<
# General rule for doing Python code generation. Args:
#  $(pkg_msg_includes) : collection of args of the form `-I:foo_msgs:/path/to/foo_msgs`
#  -p $(project) : the "package" name; determines the namespace used in the code
#  -o $(project) : the output directory name
$(project)/msg/_%.py: %.msg
	$(gencpp_dir)/lib/genpy/genmsg_py.py $(pkg_msg_includes) -p $(project) -o $(project)/msg $<
# Extra rule for generating the __init__.py module file
$(msgs_py_init): $(msgs_py)
	$(gencpp_dir)/lib/genpy/genmsg_py.py --initpy -p $(project) -o $(project)/msg
~~~
That's code generation all set up. Now let's build a program that uses the
output:
~~~
# Build an executable that uses locally defined messages. Note that it depends
# on the output from the C++ generator.
use_custom_msg: use_custom_msg.cpp $(msgs_cpp)
	$(CXX) -Wall -o $@ $(foo_msgs_includes) $(bar_msgs_includes) -I. $<
~~~
Further, let's show how to install our custom messages, both the raw input and
the generated output:
~~~
install_prefix ?= /tmp/$(project)
install: all
	# As usual, the python install location might vary from platform to platform
	mkdir -p $(install_prefix)/include/$(project) $(install_prefix)/lib/python2.7/site-packages/$(project)/msg $(install_prefix)/bin
	# Install our executable that uses custom messages
	cp -a use_custom_msg $(install_prefix)/bin
	# Install the C++ generated code
	cp -a $(msgs_cpp) $(install_prefix)/include/$(project)
	# Install the Python generated code
	cp -a $(msgs_py) $(msgs_py_init) $(install_prefix)/lib/python2.7/site-packages/$(project)/msg
	# Drop an empty `__init__.py` file to make `myproject` into a Python module
	touch $(install_prefix)/lib/python2.7/site-packages/$(project)/__init__.py
~~~

## Doing code generation for custom services
**TODO: this should be very similar to code generation for messages.**

## Doing code generation for custom actions
**TODO: this should be similar to code generation for messages, but will be a
two-step process.**

## Installing for use by tools like roslaunch
If you've ever used ROS, you know how useful tools like `roslaunch`, `rosmsg`,
and `rostopic` are (maybe also `rosrun`, if you like to invoke package-specific
executables). These tools share a requirement, which is the ability to locate
package-specific assets at run time. To use them with your own software, you
need to make your code look a little like a ROS package. This is easy to do
during installation.

For example, let's say that you build two nodes, `talker`, and `listener`, and
you have a launch file, `talker_listener.launch` that refers to them like so:
~~~
<launch>
  <node pkg="myproject" type="talker" name="talker" output="screen"/>
  <node pkg="myproject" type="listener" name="listener" output="screen"/>
</launch>
~~~
What's required to run that launch file, so that `roslaunch` can find your
programs? We need to do a few things:

1. Have a simple `package.xml` file in `<install_prefix>/share/myproject`.
1. Create an empty marker file `<install_prefix>/.catkin`. If `<install_prefix>`
is included in the `CMAKE_PREFIX_PATH`, then this marker file will cause
`roslaunch` and friends to look in there for package-specific executables.
1. Install the executables to `<install_prefix>/lib/myproject`. That's where
`roslaunch` will look.

### make
Here's what the install rule could look like:
~~~
# Assume that you have earlier rules that will build the talker and listener
# executables
install: talker listener
	# Install the executables to <prefix>/lib/<project>, so that
	# they can be found by rosrun/roslaunch.
	mkdir -p $(install_prefix)/lib/$(project)
	cp -a talker listener $(install_prefix)/lib/$(project)
	# Install the .launch file; this could go anywhere, but `launch` is a
	# good pattern to follow.
	mkdir -p $(install_prefix)/share/$(project)/launch
	cp -a talker_listener.launch $(install_prefix)/share/$(project)/launch
	# Create a simple package.xml, which is necessary to be treated as
	# a ROS package.
	echo "<package><name>$(project)</name></package>" > $(install_prefix)/share/$(project)/package.xml
	# Create an empty .catkin marker file, which causes
	# rosrun/roslaunch to look in <prefix>/lib/project for executables
	# for each <prefix> in CMAKE_PREFIX_PATH
	touch $(install_prefix)/.catkin
~~~
Then, after you `make install`, if you have the run-time environment
configuration set up properly, you should be able to do:

    roslaunch myproject talker_listener.launch

Or, equivalently:

    roslaunch <install_prefix>/share/myproject/launch/talker_listener.launch
