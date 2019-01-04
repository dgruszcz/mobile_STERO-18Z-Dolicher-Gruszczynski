cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
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
	DEPENDS elektron
)

include_directories(include
${catkin_INCLUDE_DIRS})



install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
install (DIRECTORY srv/ DESTINATION share/${CMAKE_PROJECT_NAME}/srv
         FILES_MATCHING PATTERN "*.srv")