#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>


using namespace message_filters;
using namespace sensor_msgs;
using namespace cv_bridge;

typedef sync_policies::ApproximateTime<Image, Image> ApproxSync;

bool debug;

void image_callback(const sensor_msgs::Image::ConstPtr &rgb_image, const sensor_msgs::Image::ConstPtr &depth_image)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_bridge::CvImageConstPtr cv_rgb;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_image);
        cv_rgb = cv_bridge::toCvCopy(rgb_image);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert the image to grayscale and to 8bit from 32bit
    cv::Mat gray_img = cv::Mat(cv_ptr->image.size(), CV_8UC1);
    cv::convertScaleAbs(cv_ptr->image, gray_img, 100, 0.0);

    // Same for rgb for showing the result
    cv::Mat rgb_img = cv::Mat(gray_img.size(), CV_8UC3);
    cv::cvtColor(gray_img, rgb_img, cv::COLOR_GRAY2RGB);
    
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ring_detection");
    ros::NodeHandle nh("~");

    // Get the topic names TODO: add more
    std::string depth_topic, rgb_topic;

    bool debug_param = false;

    nh.getParam("depth", depth_topic);
    nh.getParam("rgb", rgb_topic);
    nh.getParam("debug", debug_param);

    debug = debug_param;

    // TODO when adding topics and params, add error checking
    if (depth_topic.empty() || rgb_topic.empty())
    {
        ROS_ERROR("No depth or rgb topic specified");
        return -1;
    }

    ROS_INFO("Cirlce detection node started");
    ROS_INFO("Depth topic: %s", depth_topic.c_str());
    ROS_INFO("RGB topic: %s", rgb_topic.c_str());
    ROS_INFO("Debug: %s", debug ? "true" : "false");

    // Create a ROS subscriber for rgb and depth images
    Subscriber<Image> rgb_sub(nh, rgb_topic, 1);
    Subscriber<Image> depth_sub(nh, depth_topic, 1);

    // Create a ApproximateTimeSynchronizer to synchronize the rgb and depth images
    Synchronizer<ApproxSync> sync(ApproxSync(5), rgb_sub, depth_sub);

    // Register a callback for the synchronizer. _1 and _2 are placeholders for the rgb and depth images
    sync.registerCallback(boost::bind(&image_callback, _1, _2));

    // Spin
    ros::spin();

    return 0;
}
