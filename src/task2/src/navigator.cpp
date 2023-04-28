#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <signal.h>
#include <sound_play/sound_play.h>
#include <stdlib.h>
#include <task2/ColorMsg.h>
#include <task2/RingPoseMsg.h>

using namespace std;
using namespace cv;

// Typedef for convenience of communication to the MoveBaseAction action interface
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* ------------------------------------------------------------------------- */
/*   Navigator structs, enums                                                */
/* ------------------------------------------------------------------------- */

struct NavigatorPoint {
    float x;
    float y;
    bool  spin;
};

enum NavigatorState {
  IDLE,       // Navigation is idle, only applicable at the start
  PREPARING,  // Navigation is preparing to start
  NAVIGATING, // Robot is navigating around the map
  FINISHED,   // Navigation finished successfully
  FAILED      // Navigation failed
};

/* ------------------------------------------------------------------------- */
/*   Navigator class                                                         */
/* ------------------------------------------------------------------------- */

class Navigator {
  public:
    NavigatorState currentState;

    Navigator() {
      ROS_INFO("[Navigator] Initializing");

      // Initialize MoveBaseClient
      client = new MoveBaseClient("move_base", true);
      client->waitForServer();

      // Initialize SoundClient
      soundClient = new sound_play::SoundClient;
      ROS_INFO("[Navigator] Waiting for SoundClient initialization");
      ros::Duration(2.0).sleep();

      // Everything initialized, set current state to idle
      currentState = NavigatorState::IDLE;
    }

    Navigator(ros::Publisher* cmdvelPub) : Navigator() {
      cmdvelPublisher = cmdvelPub;
    }

    // Navigate to a given point
    void navigateTo(NavigatorPoint point) {
      move_base_msgs::MoveBaseGoal goal;

      // Set header
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp    = ros::Time::now();
      // Set target position
      goal.target_pose.pose.position.x = point.x;
      goal.target_pose.pose.position.y = point.y;
      // TODO Set target orientation
      goal.target_pose.pose.orientation.w = 1;

      ROS_INFO("[Navigator] Navigating to: (x: %.3f, y: %.3f)", point.x, point.y);
      client->sendGoal(goal);
      currentGoal = point;

      // Start monitoring navigation
      monitorNavigation();

      // Spin 360 degrees if the interest points wants us to do so
      if (point.spin) {
        spin(360.0);
      }
    }

    // Navigate through a list (vector) of points
    void navigateList(vector<NavigatorPoint> points) {
      int len = points.size();

      for (int i = 0; i < len; i++) {
        navigateTo(points[i]);
      }
    }

    // Callback to handle:
    // - /custom_msgs/nav/green_ring_detected
    // - /custom_msgs/nav/ring_detected
    void ringCallback(const task2::RingPoseMsgConstPtr &msg) {
      ROS_INFO(
          "[Navigator] Received new ring detected message: (x: %f, y: %f, z: %f)",
          msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z
      );
      ROS_INFO("[Navigator] Expecting to process color callback soon");
    }

    // Callback to handle /custom_msgs/sound/new_color
    void colorCallback(const task2::ColorMsgConstPtr &msg) {
      ROS_INFO(
          "[Navigator] Received color message. Stopping and playing sound of ring color: %s",
          msg->color.c_str()
      );
      client->cancelGoal();

      soundClient->say(msg->color);
      ros::Duration(3.0).sleep();

      navigateTo(currentGoal);
    }

    // Clean up (used on SIGINT)
    void cleanUp() {
      // Cancel all goals on the client and stop playing sounds
      client->cancelAllGoals();
      soundClient->stopAll();
      isKilled = true;
    }

  private:
    MoveBaseClient*          client;
    NavigatorPoint           currentGoal;
    sound_play::SoundClient* soundClient;
    ros::Publisher*          cmdvelPublisher;
    bool                     isKilled = false;

    void monitorNavigation() {
      // Monitor navigation until it reaches a terminal state
      actionlib::SimpleClientGoalState goalState = client->getState();
      while (!goalState.isDone() && !isKilled) {
        ROS_DEBUG("[Navigator] Current goal state: %s", goalState.toString().c_str());

        // Update currentState according to goal state
        switch (goalState.state_) {
          // The goal has been sent to the action server but has not yet been processed
          case actionlib::SimpleClientGoalState::PENDING:
            currentState = NavigatorState::PREPARING;
            break;
          // The goal is currently being worked on by the action server
          case actionlib::SimpleClientGoalState::ACTIVE:
            currentState = NavigatorState::NAVIGATING;
            break;
        }

        // Handle incoming messages to stop/explore etc.
        // They will be processed as callbacks
        ros::spinOnce();

        ros::Duration(0.5).sleep();
        goalState = client->getState();
      }

      // Handle terminal states
      currentState = NavigatorState::FINISHED;
      switch (goalState.state_) {
        // The client cancels a goal before the action server has started working on it
        case actionlib::SimpleClientGoalState::RECALLED:
          ROS_WARN("[Navigator] Client cancelled a goal before the server started processing it!");
          break;
        // The action server rejected the goal for some reason
        case actionlib::SimpleClientGoalState::REJECTED:
          ROS_WARN("[Navigator] Action server rejected the goal!");
          currentState = NavigatorState::FAILED;
          break;
        // The client cancels a goal that is currently being worked on by the action server
        case actionlib::SimpleClientGoalState::PREEMPTED:
          ROS_WARN("[Navigator] Client cancelled a goal currently being worked on!");
          break;
        // The action server completed the goal but encountered an error in doing so
        case actionlib::SimpleClientGoalState::ABORTED:
          ROS_WARN("[Navigator] Goal completed but an error was encountered!");
          break;
        // The action server successfully completed the goal
        case actionlib::SimpleClientGoalState::SUCCEEDED:
          ROS_INFO("[Navigator] Successfully completed goal!");
          break;
        // The client lost contact with the action server
        case actionlib::SimpleClientGoalState::LOST:
          ROS_WARN("[Navigator] Client lost contact with the action server!");
          currentState = NavigatorState::FAILED;
          break;
      }
    }

    static constexpr float SPIN_RATE        = 4;
    static constexpr float SPIN_ANGULAR_VEL = 0.85;
    static constexpr float DEGREE_RATIO     = 29.75 / 360;

    void spin(float degrees) {
      ros::Rate            rate(SPIN_RATE);
      geometry_msgs::Twist msg;

      // Set parameters for rotation
      msg.linear.x  = 0.0;
      msg.angular.z = SPIN_ANGULAR_VEL;

      // Calculate number of iterations required to spin given degrees
      int iterations = round((degrees * DEGREE_RATIO) / SPIN_ANGULAR_VEL);
      for (int i = 0; i < iterations; i++) {
        cmdvelPublisher->publish(msg);
        rate.sleep();

        // Process any callbacks that might have arrived while spinning
        // TODO Ideally this should be removed from here and we should get face location
        // TODO Then rotate accordingly, approach and greet
        ros::spinOnce();
      }

      // Stop the robot after it finishes rotating
      msg.angular.z = 0;
      cmdvelPublisher->publish(msg);
    }
};

/* ------------------------------------------------------------------------- */
/*   Map initialization                                                      */
/* ------------------------------------------------------------------------- */

bool mapReady = false;

void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map) {
  Mat                             cv_map;
  float                           map_resolution = 0;
  geometry_msgs::TransformStamped map_transform;

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
  map_transform.transform.rotation      = msg_map->info.origin.orientation;

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
  mapReady = true;
}

/* ------------------------------------------------------------------------- */
/*   Main                                                                    */
/* ------------------------------------------------------------------------- */

Navigator* navigator;

void sigintHandler(int sig) {
  ROS_WARN("Received SIGINT, cleaning up and stopping node");

  if (navigator != NULL) {
    navigator->cleanUp();
  }

  // Call ROS shutdown
  ros::shutdown();
}

int main(int argc, char* argv[]) {
  // Initialize node
  ros::init(argc, argv, "navigator");
  ros::NodeHandle nh;

  // Register SIGINT handler
  signal(SIGINT, sigintHandler);

  // Wait for map initialization
  ros::Subscriber mapSub = nh.subscribe("map", 10, &mapCallback);
  while (!mapReady) {
    ros::spinOnce();
  }

  // Vector of interest points in the map
  vector<NavigatorPoint> interestPoints {
    {-0.9712396264076233,  0.3039016127586365, false},
    {  -0.81903076171875,  1.5590908527374268,  true},
    {0.07623038440942764,   2.691239356994629, false},
    { 2.1217899322509766,  2.7076127529144287, false},
    { 2.5573792457580566,  1.3039171695709229, false},
    {  1.286680817604065,  0.5794230103492737,  true},
    {  3.354393482208252, 0.27303266525268555, false},
    {  3.716524362564087, -0.4115545153617859,  true},
    { 2.0537025928497314,  -1.007872462272644, false},
    {-0.1095728874206543, -1.1081676483154297,  true},
  };

  // Initialize publisher for robot rotation
  ros::Publisher cmdvelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);

  // Initialize Navigator
  navigator = new Navigator(&cmdvelPub);

  // Initialize ring detection subscribers
  ros::Subscriber greenRingSub =
      nh.subscribe("/custom_msgs/nav/green_ring_detected", 1, &Navigator::ringCallback, navigator);
  ros::Subscriber ringSub =
      nh.subscribe("/custom_msgs/nav/ring_detected", 1, &Navigator::ringCallback, navigator);
  ros::Subscriber colorSub =
      nh.subscribe("/custom_msgs/sound/new_color", 1, &Navigator::colorCallback, navigator);

  // Navigate through interest points
  navigator->navigateList(interestPoints);
}
