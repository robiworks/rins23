#include "ros/ros.h"

#include <stdlib.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <actionlib_msgs/GoalStatusArray.h>

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0;
geometry_msgs::TransformStamped map_transform;

ros::Publisher goal_pub;
ros::Subscriber map_sub;
ros::Subscriber status_sub;

struct TransformedPoint {
    float x;
    float y;
};

bool map_ready = false;

TransformedPoint currentGoal;

double RATE = 4;

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3) ) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
    map_transform.transform.translation.x = msg_map->info.origin.position.x;
    map_transform.transform.translation.y = msg_map->info.origin.position.y;
    map_transform.transform.translation.z = msg_map->info.origin.position.z;

    map_transform.transform.rotation = msg_map->info.origin.orientation;

    //tf2::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y-1;

    for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y -y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
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

void mouseCallback(int event, int x, int y, int, void* data) {

    if( event != EVENT_LBUTTONDOWN || cv_map.empty())
        return;

    int v = (int)cv_map.at<unsigned char>(y, x);

	if (v != 255) {
		ROS_WARN("Unable to move to (x: %d, y: %d), not reachable", x, y);
		return;
	}

    geometry_msgs::Point pt;
    geometry_msgs::Point transformed_pt;

    pt.x = (float)x * map_resolution;
    pt.y = (float)(cv_map.rows - y) * map_resolution;
    pt.z = 0.0;
    
    tf2::doTransform(pt, transformed_pt, map_transform);
    
    //geometry_msgs::Point transformed = map_transform * pt;

    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.pose.orientation.w = 1;
    goal.pose.position.x = transformed_pt.x;
    goal.pose.position.y = transformed_pt.y;
    goal.header.stamp = ros::Time::now();

    ROS_INFO("Moving to (x: %f, y: %f)", transformed_pt.x, transformed_pt.y);

    goal_pub.publish(goal);
}

bool navigation_completed = false;

bool reachedGoal(const geometry_msgs::TransformStamped transformStamped) {
    double threshold = 0.2;

    return abs(transformStamped.transform.translation.x - currentGoal.x) <= threshold && abs(transformStamped.transform.translation.y - currentGoal.y) <= threshold;
}

void statusCallback(const actionlib_msgs::GoalStatusArrayConstPtr& msg) {
    // If navigation is completed, exit and wait for next goal
    if (navigation_completed) return;

    // http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;

    for (int i = 0; i < msg->status_list.size(); i++) {
        try {
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
            ROS_INFO("Currently at x: %f, y: %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

            if (msg->status_list[i].status == actionlib_msgs::GoalStatus::SUCCEEDED && reachedGoal(transformStamped)) {
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

void navigateThroughPoints(TransformedPoint* arr) {
    ros::Rate rate(RATE);

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.orientation.w = 1;

    for (int i = 0; i < 5; i++) {
        goal.pose.position.x = arr[i].x;
        goal.pose.position.y = arr[i].y;
        goal.header.stamp = ros::Time::now();

        ROS_INFO("Navigating to hardcoded point: (x: %f, y: %f)", arr[i].x, arr[i].y);

        goal_pub.publish(goal);
        currentGoal.x = arr[i].x;
        currentGoal.y = arr[i].y;

        while (!navigation_completed) {
            rate.sleep();
            ros::spinOnce();
        }

        navigation_completed = false;
    }

    ROS_INFO("Finished navigating through array of predetermined points!");
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

    map_sub = n.subscribe("map", 10, &mapCallback);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    status_sub = n.subscribe("/move_base/status", 10, &statusCallback);

    ros::Rate rate(RATE);
    while (!map_ready) {
        rate.sleep();
        ros::spinOnce();
    }

    navigateThroughPoints(hardcodedPoints);

    return 0;

}
