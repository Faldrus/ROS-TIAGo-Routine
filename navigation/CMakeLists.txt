cmake_minimum_required(VERSION 3.0.2)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  move_base_msgs
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES position.msg
)

add_action_files(
  DIRECTORY action FILES Custom.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(user src/user_main.cpp src/UserNode.cpp)
target_link_libraries(user ${catkin_LIBRARIES})
add_dependencies(user ${navigation_EXPORTED_TARGETS})

add_executable(proxy src/proxy_main.cpp src/ProxyAction.cpp)
target_link_libraries(proxy ${catkin_LIBRARIES})
add_dependencies(proxy ${navigation_EXPORTED_TARGETS})