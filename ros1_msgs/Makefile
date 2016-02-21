project = myproject
install_prefix ?= /tmp/$(project)

# Find all my custom messages and compute targets for C++ and Python generated code
msgs = $(wildcard *.msg)
msgs_cpp = $(foreach msg, $(msgs), $(project)/$(basename $(msg)).h)
msgs_py = $(foreach msg, $(msgs), $(project)/msg/_$(basename $(msg)).py)
msgs_py_init = $(project)/msg/__init__.py

all: use_existing_msg use_custom_msg $(msgs_py) $(msgs_py_init) $(msgs_cpp)

# Get flags for an existing message package
sensor_msgs_includes = $(shell pkg-config --cflags-only-I sensor_msgs)
# Build an executable using an existing message package
use_existing_msg: use_existing_msg.cpp
	$(CXX) -Wall -o $@ $(sensor_msgs_includes) $<

# Find C++ and Python code generators
gencpp_dir = $(shell pkg-config --variable=prefix gencpp)
genpy_dir = $(shell pkg-config --variable=prefix genpy)
# Compute include directories for all message packages. This is a overinclusive
# hack because it would be painful to recursively gather the message package
# dependencies (though in principle it can be done by calling
# `pkg-config --print-requires <pkg>` for each one). The goal is to end up with
# a set of strings of the form:
#  -Ifoo_msgs:/path/to/foo_msgs/msg -Ibar_msgs:/path/to/bar_msgs/msg
pkg_msg_dirs = $(wildcard $(gencpp_dir)/share/*/msg)
pkg_msg_includes = $(foreach dir, $(pkg_msg_dirs), -I$(shell basename `dirname $(dir)`):$(dir))
# General rule for doing C++ code generation
$(project)/%.h: %.msg
	$(gencpp_dir)/lib/gencpp/gen_cpp.py $(pkg_msg_includes) -p $(project) -o $(project) -e $(gencpp_dir)/share/gencpp $<
# General rule for doing Python code generation
$(project)/msg/_%.py: %.msg
	$(gencpp_dir)/lib/genpy/genmsg_py.py $(pkg_msg_includes) -p $(project) -o $(project)/msg $<
# Extra rule for generating the __init__.py module file
$(msgs_py_init): $(msgs_py)
	$(gencpp_dir)/lib/genpy/genmsg_py.py --initpy -p $(project) -o $(project)/msg

# Build an executable that uses locally defined messages
use_custom_msg: use_custom_msg.cpp $(msgs_cpp)
	$(CXX) -Wall -o $@ $(geometry_msgs_includes) $(map_msgs_includes) -I. $<

# Install everything
install: all
	mkdir -p $(install_prefix)/include/$(project) $(install_prefix)/lib/python2.7/site-packages/$(project)/msg $(install_prefix)/bin
	cp -a use_existing_msg use_custom_msg use_msgs.py $(install_prefix)/bin
	chmod a+x $(install_prefix)/bin/use_msgs.py
	cp -a $(msgs_cpp) $(install_prefix)/include/$(project)
	cp -a $(msgs_py) $(msgs_py_init) $(install_prefix)/lib/python2.7/site-packages/$(project)/msg
	touch $(install_prefix)/lib/python2.7/site-packages/$(project)/__init__.py

# Clean up
clean:
	rm -f use_existing_msg use_custom_msg $(msgs_cpp) $(msgs_py) $(msgs_py_init)
