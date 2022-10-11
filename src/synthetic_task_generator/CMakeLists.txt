cmake_minimum_required(VERSION 2.8.3)
project(synthetic_task_generator)

add_compile_options(-std=c++11)

# Enable following line if you need instance mode
add_definitions(-DINSTANCE)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp  
  std_msgs
  message_generation  
)

add_message_files(FILES
  SyntheticTaskMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(synthetic_task_generator
  src/synthetic_task_generator_node.cpp
  src/synthetic_task_generator.cpp
  src/sched_instance.cpp

)

add_dependencies(synthetic_task_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(synthetic_task_generator
  ${catkin_LIBRARIES}
)
