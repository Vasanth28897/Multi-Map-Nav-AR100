cmake_minimum_required(VERSION 3.0.2)
project(multi_map_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  move_base_msgs
  roscpp
  roslib
  rospack
  std_msgs
)

find_package(SQLite3 REQUIRED)

add_action_files(
  DIRECTORY action
  FILES NavToGoal.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multi_map_nav
 CATKIN_DEPENDS actionlib actionlib_msgs message_runtime move_base_msgs roscpp roslib rospack std_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${SQLite3_INCLUDE_DIRS}
)


add_executable(navigation_server src/navigation_server.cpp src/wormhole_manager.cpp src/map_switcher.cpp)

add_dependencies(navigation_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(navigation_server 
  ${catkin_LIBRARIES} sqlite3 
)


