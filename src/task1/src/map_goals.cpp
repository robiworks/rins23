#include "ros/ros.h"

#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sound_play/sound_play.h>
#include <stdlib.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

using namespace std;
using namespace cv;

// RINS-provided initialization
Mat                             cv_map;
float                           map_resolution = 0;
geometry_msgs::TransformStamped map_transform;

// Our initalization
ros::Publisher  goal_pub;
ros::Subscriber map_sub;
ros::Subscriber status_sub;
ros::Subscriber cancel_sub;

tf2_ros::Buffer*            tfBuffer   = NULL;
tf2_ros::TransformListener* tfListener = NULL;
sound_play::SoundClient*    sc         = NULL;

struct TransformedPoint {
    float x;
    float y;
};

bool             map_ready            = false;
bool             navigation_completed = false;
bool             navigation_paused    = false;
double           NAV_THRESHOLD        = 0.2;
double           RATE                 = 10;
TransformedPoint currentGoal;

// Provided by RINS for map initialization
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
  if ((cv_map.rows != size_y) && (cv_map.cols != size_x)) {
    cv_map = cv::Mat(size_y, size_x, CV_8U);
  }

  map_resolution                        = msg_map->info.resolution;
  map_transform.transform.translation.x = msg_map->info.origin.position.x;
  map_transform.transform.translation.y = msg_map->info.origin.position.y;
  map_transform.transform.translation.z = msg_map->info.origin.position.z;

  map_transform.transform.rotation = msg_map->info.origin.orientation;

  //tf2::poseMsgToTF(msg_map->info.origin, map_transform);

  const std::vector<int8_t> &map_msg_data(msg_map->data);

  unsigned char* cv_map_data = (unsigned char*) cv_map.data;

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
  map_ready = true;
}

bool reachedGoal(const geometry_msgs::TransformStamped ts) {
  const double diff_x = abs(ts.transform.translation.x - currentGoal.x);
  const double diff_y = abs(ts.transform.translation.y - currentGoal.y);

  return diff_x <= NAV_THRESHOLD && diff_y <= NAV_THRESHOLD;
}

void statusCallback(const actionlib_msgs::GoalStatusArrayConstPtr &msg) {
  // If navigation is completed, exit and wait for next goal
  if (navigation_paused || navigation_completed)
    return;

  // http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29
  geometry_msgs::TransformStamped ts;

  for (int i = 0; i < msg->status_list.size(); i++) {
    try {
      ts = tfBuffer->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
      ROS_DEBUG(
          "Currently at x: %f, y: %f",
          ts.transform.translation.x,
          ts.transform.translation.y
      );

      if (msg->status_list[i].status == actionlib_msgs::GoalStatus::SUCCEEDED && reachedGoal(ts)) {
        ROS_INFO("Navigation completed!");
        navigation_completed = true;
        break;
      }

      if (msg->status_list[i].status == actionlib_msgs::GoalStatus::ABORTED) {
        ROS_WARN("Unable to reach goal!");
        navigation_completed = true;
        break;
      }
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      continue;
    }
  }
}

void navigateTo(TransformedPoint point) {
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id    = "map";
  goal.pose.orientation.w = 1;

  // Set goal position to point coordinates
  goal.pose.position.x = point.x;
  goal.pose.position.y = point.y;
  goal.header.stamp    = ros::Time::now();

  ROS_INFO("Received navigate command, navigating to: (x: %f, y: %f)", point.x, point.y);
  goal_pub.publish(goal);

  currentGoal.x = point.x;
  currentGoal.y = point.y;
}

void cancelCallback(const actionlib_msgs::GoalIDConstPtr &msg) {
  ROS_WARN("Received CANCEL message, goal ID: %s", msg->id.c_str());
  navigation_paused = true;

  ROS_INFO("Playing sound ...");
  sc->say("Kosha Mona!", "voice_kal_diphone", 1.0);
  ros::Duration(2.0).sleep();

  // Continue navigation and resend goal
  navigation_paused = false;
  navigateTo(currentGoal);
}

void navigateThroughPoints(TransformedPoint* arr) {
  for (int i = 0; i < 5; i++) {
    navigateTo(arr[i]);

    while (!navigation_completed) {
      ros::spinOnce();
    }

    navigation_completed = false;
  }

  ROS_INFO("Finished navigating through interest points!");
}

int main(int argc, char** argv) {
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

  ros::init(argc, argv, "map_goals");
  ros::NodeHandle n;

  tfBuffer   = new tf2_ros::Buffer;
  tfListener = new tf2_ros::TransformListener(*tfBuffer);
  sc         = new sound_play::SoundClient;

  map_sub    = n.subscribe("map", 10, &mapCallback);
  goal_pub   = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5);
  status_sub = n.subscribe("/move_base/status", 5, &statusCallback);
  cancel_sub = n.subscribe("/move_base/cancel", 2, &cancelCallback);

  // Wait until map is ready
  while (!map_ready) {
    ros::spinOnce();
  }

  navigateThroughPoints(hardcodedPoints);

  return 0;
}
