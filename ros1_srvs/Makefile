project = myproject
install_prefix ?= /tmp/$(project)

# Find all my custom services and compute targets for C++ and Python generated code
srvs = $(wildcard *.srv)
srvs_cpp = $(foreach srv, $(srvs), $(project)/$(basename $(srv)).h $(project)/$(basename $(srv))Request.h $(project)/$(basename $(srv))Response.h)
srvs_py = $(foreach srv, $(srvs), $(project)/srv/_$(basename $(srv)).py)
srvs_py_init = $(project)/srv/__init__.py

all: use_existing_srv use_custom_srv $(srvs_py) $(srvs_py_init) $(srvs_cpp)

# Get flags for an existing message package
sensor_msgs_includes = $(shell pkg-config --cflags-only-I sensor_msgs)
# Build an executable using an existing message package
use_existing_srv: use_existing_srv.cpp
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
$(project)/%.h: %.srv
	$(gencpp_dir)/lib/gencpp/gen_cpp.py $(pkg_msg_includes) -p $(project) -o $(project) -e $(gencpp_dir)/share/gencpp $<
# General rule for doing Python code generation
$(project)/srv/_%.py: %.srv
	$(gencpp_dir)/lib/genpy/gensrv_py.py $(pkg_msg_includes) -p $(project) -o $(project)/srv $<
# Extra rule for generating the __init__.py module file
$(srvs_py_init): $(srvs_py)
	$(gencpp_dir)/lib/genpy/gensrv_py.py --initpy -p $(project) -o $(project)/srv

# Build an executable that uses locally defined messages
use_custom_srv: use_custom_srv.cpp $(srvs_cpp)
	$(CXX) -Wall -o $@ $(geometry_msgs_includes) $(map_msgs_includes) -I. $<

# Install everything
install: all
	mkdir -p $(install_prefix)/include/$(project) $(install_prefix)/lib/python2.7/site-packages/$(project)/srv $(install_prefix)/bin $(install_prefix)/share/$(project)/srv
	cp -a $(srvs) $(install_prefix)/share/$(project)/srv
	cp -a use_existing_srv use_custom_srv use_srvs.py $(install_prefix)/bin
	chmod a+x $(install_prefix)/bin/use_srvs.py
	cp -a $(srvs_cpp) $(install_prefix)/include/$(project)
	cp -a $(srvs_py) $(srvs_py_init) $(install_prefix)/lib/python2.7/site-packages/$(project)/srv
	touch $(install_prefix)/lib/python2.7/site-packages/$(project)/__init__.py

# Clean up
clean:
	rm -f use_existing_srv use_custom_srv $(srvs_cpp) $(srvs_py) $(srvs_py_init)
