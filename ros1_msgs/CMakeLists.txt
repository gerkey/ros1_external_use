cmake_minimum_required(VERSION 2.8.3)
project(myproject)

# Build an executable that uses an existing ROS msg: find_package() the ROS
# package that contains the message, then use its reported include directories.
find_package(sensor_msgs REQUIRED)
include_directories(${sensor_msgs_INCLUDE_DIRS})
add_executable(use_existing_msg use_existing_msg.cpp)

# Generate code from our own custom messages
# find_package catkin and genmsg, required for message generation macros
find_package(catkin REQUIRED)
find_package(genmsg REQUIRED)
# find_package the other message packages that our custom messages use
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
# Enumerate our custom messages files
add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} FILES Foo.msg Bar.msg)
# Do code generation, specifying which other message packages we depend on.
generate_messages(DEPENDENCIES geometry_msgs map_msgs)

# Build an executable that uses our messages, making it depend on the
# message-generation step.
add_executable(use_custom_msg use_custom_msg.cpp)
add_dependencies(use_custom_msg ${PROJECT_NAME}_generate_messages)

# (optional) Install the executables. The generated code will also
# automatically be installed.
install(TARGETS use_existing_msg use_custom_msg
        DESTINATION bin)
install(PROGRAMS use_msgs.py
        DESTINATION bin)
