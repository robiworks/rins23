#include "geometry_msgs/PointStamped.h"
#include "pcl/point_cloud.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

#include <iostream>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <task2/RingPoseMsg.h>
#include <visualization_msgs/Marker.h>

ros::Publisher cylinder_publisher;

tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZRGB PointT;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob) {
  // All the objects needed

  ros::Time time_rec, time_test;
  time_rec = ros::Time::now();

  pcl::PassThrough<PointT>                             pass;
  pcl::NormalEstimation<PointT, pcl::Normal>           ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PCDWriter                                       writer;
  pcl::ExtractIndices<PointT>                          extract;
  pcl::ExtractIndices<pcl::Normal>                     extract_normals;
  pcl::search::KdTree<PointT>::Ptr                     tree(new pcl::search::KdTree<PointT>());
  Eigen::Vector4f                                      centroid;

  // Datasets
  pcl::PointCloud<PointT>::Ptr      cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr      cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr      cloud_filtered2(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr       coefficients_plane(new pcl::ModelCoefficients),
      coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices),
      inliers_cylinder(new pcl::PointIndices);

  // Read in the cloud data
  pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0, 1.5);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-2, 2);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-2, 2);
  pass.filter(*cloud_filtered);


  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_filtered);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.03);
  seg.setInputCloud(cloud_filtered);
  seg.setInputNormals(cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment(*inliers_plane, *coefficients_plane);

  // Extract the planar inliers from the input cloud
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
  extract.filter(*cloud_plane);

  pcl::PCLPointCloud2 outcloud_plane;
  pcl::toPCLPointCloud2(*cloud_plane, outcloud_plane);

  // Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*cloud_filtered2);
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals2);

  // Check if there are any points left
  if (cloud_filtered2->points.empty() || cloud_normals2->points.empty()) {
    return;
  }

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.05);
  seg.setRadiusLimits(0.1, 0.3);
  seg.setInputCloud(cloud_filtered2);
  seg.setInputNormals(cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder, *coefficients_cylinder);

  // Write the cylinder inliers to disk
  extract.setInputCloud(cloud_filtered2);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
  extract.filter(*cloud_cylinder);

  if (cloud_cylinder->points.empty())
    goto missdetection;
  else {
    pcl::compute3DCentroid(*cloud_cylinder, centroid);
    //std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " << centroid[1]
    //          << " " << centroid[2] << " " << centroid[3] << std::endl;
    geometry_msgs::PointStamped     point_camera;
    geometry_msgs::PointStamped     point_map;
    geometry_msgs::TransformStamped tss;

    point_camera.header.frame_id = "camera_rgb_optical_frame";
    point_camera.header.stamp    = ros::Time::now();

    point_map.header.frame_id = "map";
    point_map.header.stamp    = ros::Time::now();

    point_camera.point.x = centroid[0];
    point_camera.point.y = centroid[1];
    point_camera.point.z = centroid[2];

    int r_total = 0, g_total = 0, b_total = 0;
    int num_points = cloud_cylinder->points.size();
    for (const auto &point : cloud_cylinder->points) {
      r_total += point.r;
      g_total += point.g;
      b_total += point.b;
    }

    int r_avg = r_total / num_points;
    int g_avg = g_total / num_points;
    int b_avg = b_total / num_points;

    if ((r_avg > 200 && g_avg > 200 && b_avg > 200) || (r_avg == g_avg && g_avg == b_avg))
      goto missdetection; // missdetection

    std::cout << "Average color of the cylinder (R, G, B): (" << r_avg << ", " << g_avg << ", "
              << b_avg << ")" << std::endl;

    try {
      time_test = ros::Time::now();
      tss       = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Transform warning: %s\n", ex.what());
    }
    tf2::doTransform(point_camera, point_map, tss);

    std::cerr << "point_map: " << point_map.point.x << " " << point_map.point.y << " "
              << point_map.point.z << std::endl;

    task2::RingPoseMsg  cylinder_msg;
    geometry_msgs::Pose pose;

    pose.position.x = point_map.point.x;
    pose.position.y = point_map.point.y;
    pose.position.z = point_map.point.z;

    cylinder_msg.color.r = r_avg;
    cylinder_msg.color.g = g_avg;
    cylinder_msg.color.b = b_avg;
    cylinder_msg.pose   = pose;

    cylinder_publisher.publish(cylinder_msg);
  }
missdetection:
  std::cerr << "Can't find the cylindrical component." << std::endl;
  return;
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "cylinder_segment");
  ros::NodeHandle nh;

  // For transforming between coordinate frames
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  cylinder_publisher = nh.advertise<task2::RingPoseMsg>("/custom_msgs/cylinder_detection", 1);
  // Spin
  ros::spin();
}
