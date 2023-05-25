#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <task3/ArmExtendSrv.h>
#include <task3/RingPoseMsg.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace message_filters;
using namespace sensor_msgs;
using namespace cv_bridge;

typedef sync_policies::ApproximateTime<Image, Image> ApproxSync;

visualization_msgs::MarkerArray marker_array;

// Array of detections
std::vector<geometry_msgs::PointStamped> detections;
int                                      detection_count = 0;

bool debug;
bool search = false;

bool arm_extended = false;

geometry_msgs::Pose final_pose;

ros::Publisher marker_pub;
ros::Publisher ground_ring_pub;
ros::Publisher arm_pub;

tf2_ros::Buffer*            tfBuffer   = NULL;
tf2_ros::TransformListener* tfListener = NULL;

void moveArmDefault() {
  // Move the arm to the position
  trajectory_msgs::JointTrajectory      trajectory;
  trajectory_msgs::JointTrajectoryPoint point;

  trajectory.joint_names.push_back("arm_shoulder_pan_joint");
  trajectory.joint_names.push_back("arm_shoulder_lift_joint");
  trajectory.joint_names.push_back("arm_elbow_flex_joint");
  trajectory.joint_names.push_back("arm_wrist_flex_joint");

  // 0, -1.3, 2.2, 1
  point.positions.push_back(0);
  point.positions.push_back(-1.3);
  point.positions.push_back(2.2);
  point.positions.push_back(1);

  point.time_from_start = ros::Duration(1.0);

  trajectory.points.push_back(point);

  arm_pub.publish(trajectory);

  //wait 2 sec
  ros::Duration(2.0).sleep();
}

void moveArmScanRing() {
  // Move the arm to the position
  trajectory_msgs::JointTrajectory      trajectory;
  trajectory_msgs::JointTrajectoryPoint point;

  trajectory.joint_names.push_back("arm_shoulder_pan_joint");
  trajectory.joint_names.push_back("arm_shoulder_lift_joint");
  trajectory.joint_names.push_back("arm_elbow_flex_joint");
  trajectory.joint_names.push_back("arm_wrist_flex_joint");

  point.positions.push_back(0.1);
  point.positions.push_back(0.01);
  point.positions.push_back(1);
  point.positions.push_back(0);

  point.time_from_start = ros::Duration(1.0);

  trajectory.points.push_back(point);

  arm_pub.publish(trajectory);

  //wait 2 sec
  ros::Duration(2.0).sleep();
}

void getDepths(
    std::vector<cv::Vec4f>            circles,
    const cv_bridge::CvImageConstPtr &depth_f,
    const cv_bridge::CvImageConstPtr &rgb_image,
    cv::Mat                           output,
    std_msgs::Header                  depth_header
) {
  // ROS_INFO("Getting depths");

  if (debug) {
    cv::imshow("rgb", output);
    cv::waitKey(1);
  }

  // Get the depth image
  for (size_t i = 0; i < circles.size(); i++) {
    int minX = std::max(cvRound(circles[i][0] - circles[i][2]), 0);
    int maxX = std::min(cvRound(circles[i][0] + circles[i][2]), depth_f->image.cols);
    int minY = std::max(cvRound(circles[i][1] - circles[i][2]), 0);
    int maxY = std::min(cvRound(circles[i][1] + circles[i][2]), depth_f->image.rows);

    task3::RingPoseMsg pose;

    pose.color.r = 0;
    pose.color.g = 0;
    pose.color.b = 0;

    // Get the average depth

    cv::rectangle(output, cv::Point(minX, minY), cv::Point(maxX, maxY), cv::Scalar(122, 255, 0), 1);

    float accumulator = 0;
    int   count       = 0;

    for (int y = minY; y < maxY; y++) {
      for (int x = minX; x < maxX; x++) {
        float depth = depth_f->image.at<float>(y, x);

        if (depth > 0.1) {
          float dist = sqrt(pow(x - circles[i][0], 2) + pow(y - circles[i][1], 2));

          if (dist <= circles[i][2]) {
            accumulator += depth;
            count++;

            cv::Vec3b rgb_vals = rgb_image->image.at<cv::Vec3b>(y, x);

            pose.color.r += rgb_vals[2];
            pose.color.g += rgb_vals[1];
            pose.color.b += rgb_vals[0];

            output.at<cv::Vec3b>(y, x) = output.at<cv::Vec3b>(y, x) + cv::Vec3b(100, 100, 0);
          }
        }
      }
    }

    if (count < 20)
      return;

    float distance = accumulator / count;

    pose.color.r /= count;
    pose.color.g /= count;
    pose.color.b /= count;

    // Debug the color
    if (debug) {
      // ROS_INFO("Color: %f, %f, %f", pose.color.r, pose.color.g, pose.color.b);
    }

    // Calculate the distance with angles and the depth
    float kf = 554;

    double angle_to_target = atan2(depth_f->image.cols / 2 - circles[i][0], kf);

    float x_target = distance * cos(angle_to_target);
    float y_target = distance * sin(angle_to_target);

    geometry_msgs::PointStamped point;

    point.header.frame_id = "arm_camera_rgb_optical_frame";
    point.header.stamp    = depth_header.stamp;
    point.point.x         = -y_target;
    point.point.y         = 0;
    point.point.z         = x_target; //Try switching .y and .z

    // // Make a marker for the point
    // visualization_msgs::Marker marker;
    // marker.header.frame_id    = "camera_rgb_optical_frame";
    // marker.header.stamp       = depth_header.stamp;
    // marker.ns                 = "points_and_lines";
    // marker.id                 = rand();
    // marker.type               = visualization_msgs::Marker::SPHERE;
    // marker.action             = visualization_msgs::Marker::ADD;
    // marker.pose.position.x    = point.point.x;
    // marker.pose.position.y    = point.point.y;
    // marker.pose.position.z    = point.point.z;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    // marker.color.r            = pose.color.r / 255;
    // marker.color.g            = pose.color.g / 255;
    // marker.color.b            = pose.color.b / 255;
    // marker.color.a            = 1.0;
    // marker.scale.x            = 0.1;
    // marker.scale.y            = 0.1;
    // marker.scale.z            = 0.1;
    // marker.lifetime           = ros::Duration();

    // marker_array.markers.push_back(marker);

    // marker_pub.publish(marker_array);
    geometry_msgs::Pose pose_msg;

    try {
      if (detection_count < 15) {
        ROS_WARN("Not enough detections");
        detections.push_back(point);
        detection_count++;
      } else {
        // Calculate the mean
        float mean_x = 0;
        float mean_y = 0;
        float mean_z = 0;

        for (size_t i = 0; i < detections.size(); i++) {
          mean_x += detections[i].point.x;
          mean_y += detections[i].point.y;
          mean_z += detections[i].point.z;
        }

        mean_x /= detections.size();
        mean_y /= detections.size();
        mean_z /= detections.size();

        // Make a marker for the point
        visualization_msgs::Marker marker;
        marker.header.frame_id    = "arm_camera_rgb_optical_frame";
        marker.header.stamp       = depth_header.stamp;
        marker.ns                 = "points_and_lines";
        marker.id                 = rand();
        marker.type               = visualization_msgs::Marker::SPHERE;
        marker.action             = visualization_msgs::Marker::ADD;
        marker.pose.position.x    = mean_x;
        marker.pose.position.y    = mean_y;
        marker.pose.position.z    = mean_z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.r            = pose.color.r / 255;
        marker.color.g            = pose.color.g / 255;
        marker.color.b            = pose.color.b / 255;
        marker.color.a            = 1.0;
        marker.scale.x            = 0.1;
        marker.scale.y            = 0.1;
        marker.scale.z            = 0.1;
        marker.lifetime           = ros::Duration();

        marker_array.markers.push_back(marker);

        marker_pub.publish(marker_array);

        // Transform point from arm camera frame to map frame
        point.point.x = mean_x;
        point.point.y = mean_y;
        point.point.z = mean_z;

        geometry_msgs::PointStamped ps;
        ps = tfBuffer->transform(point, "map", ros::Duration(3.0));

        pose_msg.position.x = ps.point.x;
        pose_msg.position.y = ps.point.y;
        pose_msg.position.z = ps.point.z;

        pose.pose = pose_msg;

        ground_ring_pub.publish(pose);
        final_pose = pose.pose;

        search = false;
      }

    } catch (const std::exception &e) {
      ROS_ERROR("Transform error: %s", e.what());
      continue;
    }
  }
}

std::vector<cv::Vec4f> detectCircles(cv::Mat input_img, cv::Mat output_img) {
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
    // ROS_WARN("Circle radius: %d", radius);

    if (debug) {
      // Draw the center of the circle
      cv::circle(output_img, center, 3, cv::Scalar(0, 255, 0), -1);

      // Draw the circle outline
      cv::circle(output_img, center, radius, cv::Scalar(0, 0, 255), 1);
    }
  }

  return validCircles;
}

void image_callback(
    const sensor_msgs::Image::ConstPtr &rgb_image,
    const sensor_msgs::Image::ConstPtr &depth_image
) {
  if (!search)
    return;

  cv_bridge::CvImageConstPtr cv_ptr;
  cv_bridge::CvImageConstPtr cv_rgb;

  try {
    cv_ptr = cv_bridge::toCvCopy(depth_image);
    cv_rgb = cv_bridge::toCvCopy(rgb_image);
  } catch (const std::exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Convert the image to grayscale and to 8bit from 32bit
  cv::Mat gray_img = cv::Mat(cv_rgb->image.rows, cv_rgb->image.cols, CV_8UC1);
  cv::cvtColor(cv_rgb->image, gray_img, CV_BGR2GRAY);

  // Same for rgb for showing the result
  cv::Mat rgb_img = cv::Mat(gray_img.size(), CV_8UC3);
  cv::cvtColor(gray_img, rgb_img, cv::COLOR_GRAY2RGB);

  // Detect the circles
  std::vector<cv::Vec4f> circles = detectCircles(gray_img, rgb_img);

  getDepths(circles, cv_ptr, cv_rgb, rgb_img, depth_image->header);
}

// Fuunction that returns a pose named scanCallback
void scanCallback(const std_msgs::Bool::ConstPtr &doSearch) {
  ROS_WARN("Scan callback");
  search = doSearch->data;
  if (search) {
    moveArmScanRing();
  } else {
    moveArmDefault();
  }
}

bool extendArmCallback(task3::ArmExtendSrv::Request &req, task3::ArmExtendSrv::Response &res) {
  ROS_WARN("Extend arm callback");
  bool extend = req.extend;
  if (extend) {
    moveArmScanRing();
    arm_extended = true;
  } else {
    moveArmDefault();
    arm_extended = false;
  }

  res.extended = arm_extended;

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_ring_detection");
  ros::NodeHandle nh("~");

  std::string depth_topic, rgb_topic, cam_info;

  bool debug_param = false;

  // nh.getParam("depth", depth_topic);
  // nh.getParam("rgb", rgb_topic);
  depth_topic = "/arm_camera/depth/image_raw";
  rgb_topic   = "/arm_camera/rgb/image_raw";

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

  // Service for extending the arm
  ros::ServiceServer arm_service = nh.advertiseService("/arm_control/extend", extendArmCallback);

  // subscribwer for scanning the ground
  ros::Subscriber scan_sub = nh.subscribe("/arm_control/scan", 1, &scanCallback);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("ground_ring_marker", 10000);

  ground_ring_pub = nh.advertise<task3::RingPoseMsg>("/arm_control/parking_point", 1000);

  Subscriber<Image> rgb_sub(nh, rgb_topic, 1);
  Subscriber<Image> depth_sub(nh, depth_topic, 1);

  // Create a ApproximateTimeSynchronizer to synchronize the rgb and depth images
  Synchronizer<ApproxSync> sync(ApproxSync(5), rgb_sub, depth_sub);

  // Register a callback for the synchronizer. _1 and _2 are placeholders for the rgb and depth images
  sync.registerCallback(boost::bind(&image_callback, _1, _2));

  arm_pub =
      nh.advertise<trajectory_msgs::JointTrajectory>("/turtlebot_arm/arm_controller/command", 1000);

  // Create a buffer and listener for coordinate transforms
  tfBuffer   = new tf2_ros::Buffer;
  tfListener = new tf2_ros::TransformListener(*tfBuffer);

  ros::spin();

  return 0;
}
