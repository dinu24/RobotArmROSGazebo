cmake_minimum_required(VERSION 2.8.3)
project(collision_detector_node)

add_compile_options(-std=c++11)


# Convert the Qt Signals and Slots for QWidget events


find_package(catkin REQUIRED COMPONENTS
  roscpp
	rospy
	std_msgs
	geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)


catkin_package(
  CATKIN_DEPENDS
  moveit_core
  moveit_ros_planning_interface
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(collision_detector_node src/collision_detector_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_detector_node ${catkin_LIBRARIES})
