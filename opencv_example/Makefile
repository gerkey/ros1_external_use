all: publish_cv_cam

ros_libs = $(shell pkg-config --libs roscpp std_msgs sensor_msgs)
# There are some incompatibility issues with some versions of pkg-config
# and some linkers regarding the -l:/absolute/path syntax. As a quick
# workaround, we'll remove the leading -l: prefix, leaving just the
# absolute path.
ros_libs_nocolon = $(subst -l:,,$(ros_libs))

publish_cv_cam: publish_cv_cam.cpp
	$(CXX) -Wall -o $@ `pkg-config --cflags roscpp std_msgs sensor_msgs opencv` $< $(ros_libs_nocolon) `pkg-config --libs opencv`

clean:
	rm -f publish_cv_cam
