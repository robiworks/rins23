cmake_minimum_required(VERSION 2.8.3)
project(task1)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf2
	tf2_geometry_msgs
    message_generation
    rospy
)

add_message_files(
    FILES
    FacePositionMessage.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

find_package(OpenCV)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(map_goals src/map_goals.cpp)
add_executable(navigator src/navigator.cpp)

target_link_libraries(map_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${message_runtime_LIBRARIES})
target_link_libraries(navigator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${message_runtime_LIBRARIES})

catkin_install_python(
    PROGRAMS
    scripts/face_localizer_dnn
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
