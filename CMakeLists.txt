cmake_minimum_required(VERSION 2.8.3)

project(project_1)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	message_filters
	dynamic_reconfigure
	tf
)

add_message_files(
	FILES
	floatStamped.msg
)

generate_messages(
	DEPENDENCIES std_msgs
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(odometry
  src/odometry.cpp
)

add_dependencies(
	odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
	odometry ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
	odometry
	${catkin_LIBRARIES}
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

add_dependencies(odometry ${PROJECT_NAME}_gencfg)

catkin_package(CATKIN_DEPENDS
	message_filters
	message_runtime
	std_msgs
)
