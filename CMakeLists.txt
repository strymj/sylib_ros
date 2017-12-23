cmake_minimum_required(VERSION 2.8.3)

project(sylib_ros)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	# image_transport
	# cv_bridge
	)

find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	# LIBRARIES mylib
	# CATKIN_DEPENDS roscpp rospy
	# DEPENDS system_lib
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	)


add_library(${PROJECT_NAME}
	src/point.cpp
	src/lidar2d.cpp
	src/move_base_goal.cpp
	)

add_library(sylib_image_ros
	src/image_proc.cpp
	src/text_detection.cpp
	)

add_executable(scan_fixer
	sample_src/scan_fixer.cpp
	)
target_link_libraries(scan_fixer
	${catkin_LIBRARIES}
	${PROJECT_NAME}
	)


add_executable(ocr
	sample_src/ocr.cpp
	)
target_link_libraries(ocr
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PROJECT_NAME}
	sylib_image_ros
	lept
	tesseract
	)	
