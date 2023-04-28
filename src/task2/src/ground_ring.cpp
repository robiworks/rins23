#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <task2/RingPoseMsg.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace message_filters;
using namespace sensor_msgs;
using namespace cv_bridge;

bool debug;
ros::Publisher marker_pub;
ros::Publisher ground_ring_pub;

void green_callback(task2::RingPoseMsg pose){
  ROS_WARN("Green callback");

  // Move the arm 90 degrees

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_ring_detection");
  ros::NodeHandle nh("~");

  std::string depth_topic, rgb_topic, cam_info;

  bool debug_param = false;

  nh.getParam("depth", depth_topic);
  nh.getParam("rgb", rgb_topic);
  nh.getParam("cam_info", cam_info);
  nh.getParam("debug", debug_param);

  debug = debug_param;

  // TODO when adding topics and params, add error checking
  if (depth_topic.empty() || rgb_topic.empty()) {
    ROS_ERROR("No depth or rgb topic specified");
    return -1;
  }

  ROS_INFO("Ground Ring Node Detectgor started");
  ROS_INFO("Depth topic: %s", depth_topic.c_str());
  ROS_INFO("RGB topic: %s", rgb_topic.c_str());
  ROS_INFO("Debug: %s", debug ? "true" : "false");

  // Create a ROS subscriber for rgb and depth images
  Subscriber<Image> rgb_sub(nh, rgb_topic, 1);
  Subscriber<Image> depth_sub(nh, depth_topic, 1);
  // Create a marker publisher
  
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("ground_ring_marker", 10000);

    ros::Subscriber green_sub = nh.subscribe("/custom_msgs/nav/green_ring_detected", 1, &green_callback);

    ground_ring_pub = nh.advertise<task2::RingPoseMsg>("/custom_msgs/ground_ring_detection", 1000);

    ros::spin();

    return 0;
  }