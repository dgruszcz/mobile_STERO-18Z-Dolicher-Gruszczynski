cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  nav_core
  global_planner
  tf
  tf2
  tf2_ros
  nav_msgs
  costmap_2d
  base_local_planner
)
add_service_files(
  FILES
  ElektronSrv.srv
) 
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)
catkin_package(
	#INCLUDE_DIRS include
	#LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS message_runtime
)

include_directories(include
${catkin_INCLUDE_DIRS}
)



install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")

add_executable(costmap src/costmap.cpp)
target_link_libraries(costmap ${catkin_LIBRARIES})
add_dependencies(costmap stero_mobile_init_generate_messages_cpp)
