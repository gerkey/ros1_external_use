cmake_minimum_required(VERSION 2.8.3)
project(myproject)

# Generate code from our own custom actions, using genmsg.  Explicitly finding
# catkin is a temporary workaround for a bug in genmsg:
#   https://github.com/ros/genmsg/pull/61
find_package(catkin REQUIRED)
find_package(genmsg REQUIRED)

# Find packages containing messages that we use in our actions
find_package(sensor_msgs REQUIRED)
# We always need actionlib_msgs when building actions because the .msg files
# that are generated from the .action file will refer to messages from
# actionlib_msgs.
find_package(actionlib_msgs REQUIRED)

# Enumerate our custom messages files
add_action_files(DIRECTORY . FILES DoDishes.action)
# Do code generation, specifying which other message packages we depend on.
generate_messages(DEPENDENCIES sensor_msgs actionlib_msgs)

# We further need the libraries from actionlib to build our examples
find_package(actionlib REQUIRED)
include_directories(${actionlib_INCLUDE_DIRS})

# Build an executable that uses our messages, making it depend on the
# message-generation step.
add_executable(actionclient actionclient.cpp)
target_link_libraries(actionclient ${actionlib_LIBRARIES})
add_dependencies(actionclient ${PROJECT_NAME}_generate_messages)

add_executable(actionserver actionserver.cpp)
target_link_libraries(actionserver ${actionlib_LIBRARIES})
add_dependencies(actionserver ${PROJECT_NAME}_generate_messages)

# (optional) Install the executables. The generated code will also
# automatically be installed.
install(TARGETS actionclient actionserver
        DESTINATION bin)

