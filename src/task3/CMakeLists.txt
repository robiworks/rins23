cmake_minimum_required(VERSION 3.0.2)
project(task3)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  genmsg
  nodelet
  nodelet_topic_tools
  pcl_conversions
  pcl_msgs
  pluginlib
  rosbag
  rosconsole
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

link_libraries(${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_message_files(
  FILES
  RingPoseMsg.msg
  ColorMsg.msg
  PosterContentMsg.msg
  FacePositionMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  LIBRARIES
    pcl_ros_filters
    pcl_ros_io
    pcl_ros_tf
  CATKIN_DEPENDS
    message_runtime
    cv_bridge
    dynamic_reconfigure
    pcl_conversions
    pcl_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
  DEPENDS
    Boost
    EIGEN3
    PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(task3_navigator src/navigator.cpp)
target_link_libraries(task3_navigator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${message_runtime_LIBRARIES})

add_executable(task3_ring_detection src/ring_detection.cpp)
add_dependencies(task3_ring_detection ${catkin_EXPORTED_TARGETS})
target_link_libraries(task3_ring_detection ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_executable(task3_ground_ring_detection src/ground_ring_detection.cpp)
add_dependencies(task3_ground_ring_detection ${catkin_EXPORTED_TARGETS})

add_executable(task3_cylinder_detection src/cylinder_detection.cpp)
target_link_libraries(task3_cylinder_detection ${catkin_LIBRARIES} ${message_runtime_LIBRARIES} ${PCL_LIBRARIES})

if(NOT CMAKE_CXX_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()
