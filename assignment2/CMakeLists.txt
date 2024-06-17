cmake_minimum_required(VERSION 3.0.2)
project(assignment2)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  apriltag
  apriltag_ros
  control_msgs
  gazebo_ros_link_attacher
  geometry_msgs
  message_generation
  message_runtime
  move_base_msgs
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
  std_msgs
  tf
  tiago_iaslab_simulation
  navigation
)

add_message_files(
  FILES position.msg
)

add_action_files(
  DIRECTORY action FILES PickPlace.action Detection.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs geometry_msgs  
)

catkin_package(
  CATKIN_DEPENDS 
  actionlib 
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(node_a src/NodeA.cpp)
target_link_libraries(node_a ${catkin_LIBRARIES})
add_dependencies(node_a ${assignment2_EXPORTED_TARGETS})

add_executable(node_b src/NodeB.cpp)
target_link_libraries(node_b ${catkin_LIBRARIES})
add_dependencies(node_b ${assignment2_EXPORTED_TARGETS})

add_executable(node_c src/NodeC.cpp)
target_link_libraries(node_c ${catkin_LIBRARIES})
add_dependencies(node_c ${assignment2_EXPORTED_TARGETS})