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
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace message_filters;
using namespace sensor_msgs;
using namespace cv_bridge;

bool debug;
bool search = false;

ros::Publisher marker_pub;
ros::Publisher ground_ring_pub;
ros::Publisher arm_pub;

std::vector<cv::Vec4f> detectCircles(cv::Mat input_img, cv::Mat output_img){
    std::vector<cv::Vec4f> circles, validCircles;

      // Arugments for hough transform
  int   minRadius            = 160;
  int   maxRadius            = 300;
  int   minDist              = 100;
  float imageScale           = 2;
  int   cannyThreshold       = 100;
  int   accumulatorThreshold = 75;

  int centerThreshold = 100;

  // Apply the Hough Transform to find the circles
  cv::HoughCircles(
      input_img,
      circles,
      cv::HOUGH_GRADIENT,
      imageScale,
      minDist,
      cannyThreshold,
      accumulatorThreshold,
      minRadius,
      maxRadius
  );

  for (size_t i = 0; i < circles.size(); i++) {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int       radius = cvRound(circles[i][2]);

    // Store valid circles
    validCircles.insert(validCircles.end(), circles[i]);

    // TODO: Play with paramaters
    // TODO: Maybe get mean center value?
    // TODO publish the location of the radius

    // Draw the circle outline
    if (debug) {
      ROS_WARN("Circle radius: %d", radius);

      // Draw the center of the circle
      cv::circle(output_img, center, 3, cv::Scalar(0, 255, 0), -1);

      // Draw the circle outline
      cv::circle(output_img, center, radius, cv::Scalar(0, 0, 255), 1);
    }
  }

  return validCircles;
}


void image_callback(const sensor_msgs::Image::ConstPtr &rgb_image){
    if(search){
        ROS_WARN("Image callback");
        cv_bridge::CvImagePtr cv_rgb;
        try {
            cv_rgb = cv_bridge::toCvCopy(rgb_image);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray_img = cv::Mat(cv_rgb->image.rows, cv_rgb->image.cols, CV_8UC1);
        cv::cvtColor(cv_rgb->image, gray_img, CV_BGR2GRAY);

        cv::Mat rgb_img = cv::Mat(cv_rgb->image.rows, cv_rgb->image.cols, CV_8UC3);
        cv::cvtColor(cv_rgb->image, rgb_img, CV_BGR2RGB);

        std::vector<cv::Vec4f> circles = detectCircles(gray_img, rgb_img);

        // Show image with cv2
        cv::imshow("Image", rgb_img);
        cv::waitKey(1);
    }
}

void green_callback(task2::RingPoseMsg pose){
  ROS_WARN("Green callback");

    // Move the arm to the position
    trajectory_msgs::JointTrajectory trajectory;
    trajectory_msgs::JointTrajectoryPoint point;

    trajectory.joint_names.push_back("arm_shoulder_pan_joint");
    trajectory.joint_names.push_back("arm_shoulder_lift_joint");
    trajectory.joint_names.push_back("arm_elbow_flex_joint");
    trajectory.joint_names.push_back("arm_wrist_flex_joint");

    point.positions.push_back(0);
    point.positions.push_back(0.3);
    point.positions.push_back(0.9);
    point.positions.push_back(0);

    point.time_from_start = ros::Duration(1.0);
    
    trajectory.points.push_back(point);

    arm_pub.publish(trajectory);
  
    //wait 2 sec
    ros::Duration(2.0).sleep();
  search = true;
}

void stop_green_callback(task2::RingPoseMsg pose){
  ROS_WARN("Stop green callback");
  search = false;
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
//   rgb_sub(nh, rgb_topic, 1);
//   Subscriber<Image> depth_sub(nh, depth_topic, 1);
  // Create a marker publisher
  
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("ground_ring_marker", 10000);

    ros::Subscriber green_sub = nh.subscribe("/custom_msgs/nav/green_ring_detected", 1, &green_callback);

    ground_ring_pub = nh.advertise<task2::RingPoseMsg>("/custom_msgs/ground_ring_detection", 1000);
    ros::Subscriber rgb_sub = nh.subscribe("/arm_camera/rgb/image_raw", 1, &image_callback);
    arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/turtlebot_arm/arm_controller/command", 1000);

    ros::spin();

    return 0;
  }