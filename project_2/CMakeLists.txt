cmake_minimum_required(VERSION 2.8.3)
project(project_2)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_filters roscpp sensor_msgs std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
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

target_link_libraries(odometry ${catkin_LIBRARIES})

install(TARGETS odometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
