cmake_minimum_required(VERSION 2.8.3)
project(synthetic_task_generator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES rubis_pkg
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)


add_executable(synthetic_task_generator
  src/synthetic_task_generator_node.cpp
  src/synthetic_task_generator.cpp
)

add_dependencies(synthetic_task_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(synthetic_task_generator
  ${catkin_LIBRARIES}
)
