cmake_minimum_required(VERSION 3.5)
project(talker_listener)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker talker.cpp)
target_include_directories(talker PRIVATE ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(talker ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

add_executable(listener listener.cpp)
target_include_directories(listener PRIVATE ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(listener ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
