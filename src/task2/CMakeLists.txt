cmake_minimum_required(VERSION 2.8.6)
project(task2)

## Find system dependencies
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

## Find catkin packages
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

## Add include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

link_libraries(${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Add message files
add_message_files(
  FILES
  RingPoseMsg.msg
  ColorMsg.msg
)

## Generate message files
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare the catkin package
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

add_executable(task2_navigator src/navigator.cpp)
target_link_libraries(task2_navigator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${message_runtime_LIBRARIES})

add_executable(circle src/ring_detection.cpp)
add_dependencies(circle ${catkin_EXPORTED_TARGETS})

add_executable(ground src/ground_ring.cpp)
add_dependencies(ground ${catkin_EXPORTED_TARGETS})

target_link_libraries(circle ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

add_executable(task2_cylinder src/cylinder.cpp)
target_link_libraries(task2_cylinder ${catkin_LIBRARIES} ${message_runtime_LIBRARIES} ${PCL_LIBRARIES})

if(NOT CMAKE_CXX_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()
