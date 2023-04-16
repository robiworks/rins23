// Create a ros node that subscribes to the rgb/image_raw and using opencv cv2 finds all cylinders in the image. The cylinders should be visualized using a marker in rviz.

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO("Received image");
    
//   // Convert the image to OpenCV format
//   cv_bridge::CvImagePtr cv_ptr;
//   try
//   {
//     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//     return;
//   }

//   // Convert the image to grayscale
//   cv::Mat gray;
//   cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

//   // Blur the image to reduce noise
//   cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

//   // Detect edges using the Canny algorithm
//   cv::Mat edges;
//   cv::Canny(gray, edges, 50, 150);

//   // Find contours in the image
//   std::vector<std::vector<cv::Point> > contours;
//   cv::findContours(edges.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

//   // Find the rotated rectangles and ellipses for each contour
//   std::vector<cv::RotatedRect> minRect( contours.size() );
//   std::vector<cv::RotatedRect> minEllipse( contours.size() );

//   for( int i = 0; i < contours.size(); i++ )
//   {
//     minRect[i] = cv::minAreaRect( cv::Mat(contours[i]) );
//     if( contours[i].size() > 5 )
//     {
//       minEllipse[i] = cv::fitEllipse( cv::Mat(contours[i]) );
//     }
//   }

//   // Draw contours + rotated rects + ellipses
//   cv::Mat drawing = cv::Mat::zeros( edges.size(), CV_8UC3 );
//   for( int i = 0; i< contours.size(); i++ )
//   {
//     cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//     // contour
//     cv::drawContours( drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
//     // ellipse
//     cv::ellipse( drawing, minEllipse[i],
//                     color, 2, 8 );
//     // rotated rectangle
//     cv::Point2f rect_points[4];
//     minRect[i].points( rect_points );
//     for( int j = 0; j < 4; j++ )
//     {
//       cv::line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
//     }
//     }

//     // Show the image
//     cv::imshow("Image window", drawing);
//     cv::waitKey(3);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cylinder_detection");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the raw rgb image
    // ros::Subscriber sub = nh.subscribe ("/camera/rgb/image_raw", 1, image_callback);

  // Get the rgb image from the camera every 1sec




  // // Create a ApproximateTimeSynchronizer to synchronize the rgb and depth images
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_callback);
  // Spin
  ros::spin ();
}