cmake_minimum_required(VERSION 3.5)
project(pub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pub pub.cpp)
target_include_directories(pub PRIVATE ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(pub ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
