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

# Clean up
clean:
	rm -f talker listener
