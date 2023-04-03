#include "ros/ros.h"

#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

using namespace std;
using namespace cv;

struct TransformedPoint {
    float x;
    float y;
};

class Navigator {
  private:
    // RINS-provided initialization
    Mat                             cv_map;
    float                           map_resolution = 0;
    geometry_msgs::TransformStamped map_transform;

    // Navigation-related
  public:
    bool map_ready = false;

  private:
    TransformedPoint           current_goal;
    const double               NAVIGATION_THRESHOLD = 0.2;
    bool                       navigation_completed = false;
    tf2_ros::Buffer            tfBuffer;
    tf2_ros::TransformListener tfListener;

    bool reachedGoal(const geometry_msgs::TransformStamped ts) {
      const double diff_x = abs(ts.transform.translation.x - this->current_goal.x);
      const double diff_y = abs(ts.transform.translation.y - this->current_goal.y);

      return diff_x <= this->NAVIGATION_THRESHOLD && diff_y <= this->NAVIGATION_THRESHOLD;
    }

  public:
    void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map) {
      int size_x = msg_map->info.width;
      int size_y = msg_map->info.height;

      if ((size_x < 3) || (size_y < 3)) {
        ROS_INFO(
            "Map size is only x: %d,  y: %d . Not running map to image conversion",
            size_x,
            size_y
        );
        return;
      }

      // resize cv image if it doesn't have the same dimensions as the map
      if ((this->cv_map.rows != size_y) && (this->cv_map.cols != size_x)) {
        this->cv_map = cv::Mat(size_y, size_x, CV_8U);
      }

      this->map_resolution                        = msg_map->info.resolution;
      this->map_transform.transform.translation.x = msg_map->info.origin.position.x;
      this->map_transform.transform.translation.y = msg_map->info.origin.position.y;
      this->map_transform.transform.translation.z = msg_map->info.origin.position.z;
      this->map_transform.transform.rotation      = msg_map->info.origin.orientation;

      const std::vector<int8_t> &map_msg_data(msg_map->data);
      unsigned char*             cv_map_data = (unsigned char*) cv_map.data;

      //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
      int size_y_rev = size_y - 1;

      for (int y = size_y_rev; y >= 0; --y) {
        int idx_map_y = size_x * (size_y - y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {
          int idx = idx_img_y + x;

          switch (map_msg_data[idx_map_y + x]) {
            case -1:
              cv_map_data[idx] = 127;
              break;

            case 0:
              cv_map_data[idx] = 255;
              break;

            case 100:
              cv_map_data[idx] = 0;
              break;
          }
        }
      }

      ROS_INFO("Map is available");
      this->map_ready = true;
    }

    void statusCallback(const actionlib_msgs::GoalStatusArrayConstPtr &msg) {
      // Exit and wait for next goal if navigation is completed
      if (this->navigation_completed) {
        return;
      }

      // http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29
      geometry_msgs::TransformStamped ts;

      for (int i = 0; i < msg->status_list.size(); i++) {
        try {
          // Get current position relative to map origin
          ts = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
          ROS_DEBUG(
              "Currently at x: %f, y: %f",
              ts.transform.translation.x,
              ts.transform.translation.y
          );

          bool goal_status = reachedGoal(ts);

          if (msg->status_list[i].status == actionlib_msgs::GoalStatus::SUCCEEDED && goal_status) {
            ROS_INFO("Navigation completed!");
            this->navigation_completed = true;
            break;
          }

          if (msg->status_list[i].status == actionlib_msgs::GoalStatus::ABORTED) {
            ROS_WARN("Unable to reach goal!");
            this->navigation_completed = true;
            break;
          }
        } catch (tf2::TransformException &ex) {
          ROS_WARN("%s", ex.what());
          continue;
        }
      }
    }

    Navigator() : tfBuffer(), tfListener(tfBuffer) {
      ROS_INFO("Initialized Navigator");
    }

    void navigateThroughPoints(ros::Publisher goal_publisher, TransformedPoint* arr) {
      geometry_msgs::PoseStamped goal;
      goal.header.frame_id    = "map";
      goal.pose.orientation.w = 1;

      for (int i = 0; i < 5; i++) {
        goal.pose.position.x = arr[i].x;
        goal.pose.position.y = arr[i].y;
        goal.header.stamp    = ros::Time::now();

        ROS_INFO("Navigating to hardcoded point: (x: %f, y: %f)", arr[i].x, arr[i].y);

        goal_publisher.publish(goal);
        this->current_goal.x = arr[i].x;
        this->current_goal.y = arr[i].y;

        while (!this->navigation_completed) {
          ros::spinOnce();
        }

        this->navigation_completed = false;
      }

      ROS_INFO("Finished navigating through array of predetermined points!");
    }
};

static bool NAVIGATOR_IMPLEMENTED = false;

int main(int argc, char** argv) {
  if (!NAVIGATOR_IMPLEMENTED) {
    cout << "Not implemented yet\n";
    return 0;
  }

  ros::init(argc, argv, "navigator");
  ros::NodeHandle nh;

  Navigator       nav;
  ros::Subscriber map_sub  = nh.subscribe("map", 10, &Navigator::mapCallback, &nav);
  ros::Publisher  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  ros::Subscriber status_sub =
      nh.subscribe("/move_base/status", 10, &Navigator::statusCallback, &nav);

  // Wait until the map is ready
  while (!nav.map_ready) {
    ros::spinOnce();
  }

  // Five hardcoded points
  TransformedPoint hardcodedPoints[5];
  hardcodedPoints[0].x = 0.2;
  hardcodedPoints[0].y = -1.3;
  hardcodedPoints[1].x = 3.2;
  hardcodedPoints[1].y = -0.15;
  hardcodedPoints[2].x = 1.65;
  hardcodedPoints[2].y = 0.8;
  hardcodedPoints[3].x = 0.6;
  hardcodedPoints[3].y = 2.5;
  hardcodedPoints[4].x = -0.65;
  hardcodedPoints[4].y = 0.15;

  nav.navigateThroughPoints(goal_pub, hardcodedPoints);

  return 0;
}
