#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>


typedef pcl::PointXYZ PointT;

class CylinderDetection
{
public:
  CylinderDetection() : nh_("~")
  {
    sub_ = nh_.subscribe("/camera/depth/points", 1, &CylinderDetection::cloudCallback, this);
    pub_ = nh_.advertise<geometry_msgs::PointStamped>("cylinder_position", 1);
  }

  void cloudCallback(const pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    // All the objects needed
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);
    pass.filter (*cloud_filtered);
    ROS_INFO_STREAM("PointCloud after filtering has: " << cloud_filtered->size () << " data points.");

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    //ROS_INFO_STREAM("Plane coefficients: " << *coefficients_plane);

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

    extract.filter (*cloud_plane);
    //ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->size () << " data points.");
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Check if there are any points left
    if (cloud_filtered2->points.empty () || cloud_normals2->points.empty ())
    {
      ROS_WARN("Could not find any points remaining after planar extraction.");
      return;
    }
    
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.01);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0.2, 0.7);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);
    
    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    //ROS_INFO_STREAM("Cylinder coefficients: " << *coefficients_cylinder);
    //
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder-> points.empty() == 0)
    {
      ROS_WARN("No cylinder found");
      return;
    
    }
  }

private:
  ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    pcl::PCDWriter writer;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cylinder_detection_node");
  CylinderDetection cylinder_detection;
  ros::spin();
  return 0;
}
