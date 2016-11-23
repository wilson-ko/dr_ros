cmake_minimum_required(VERSION 2.8.3)
project(dr_ros)

find_package(catkin REQUIRED COMPONENTS
	dr_base
	dr_log
	dr_param
	roscpp
	roslib
	tf
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS dr_log dr_param
)

dr_include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/node.cpp
	src/signal.cpp
	src/resource.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

dr_add_gtest(resource test/resource.cpp)
target_link_libraries(${PROJECT_NAME}_test_resource   ${PROJECT_NAME} ${catkin_LIBRARIES})

install(
	TARGETS "${PROJECT_NAME}"
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
	DIRECTORY "include/${PROJECT_NAME}/"
	DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)

install(
	DIRECTORY bin/
	DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
	DIRECTORY launch
	DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)
