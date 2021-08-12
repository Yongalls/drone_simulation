cmake_minimum_required(VERSION 2.8.3)
project(drone_simulation)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs mavros_msgs)


## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS mavros_msgs
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rect_path_position_control src/rect_path_position_control.cpp)
target_link_libraries(rect_path_position_control ${catkin_LIBRARIES})
add_dependencies(rect_path_position_control mavros_msgs_generate_messages_cpp)

add_executable(rect_path_velocity_control src/rect_path_velocity_control.cpp)
target_link_libraries(rect_path_velocity_control ${catkin_LIBRARIES})
add_dependencies(rect_path_velocity_control mavros_msgs_generate_messages_cpp)
