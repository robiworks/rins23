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
#include <task2/RingPoseMsg.h>

using namespace std;
using namespace cv;

// Typedef for convenience of communication to the MoveBaseAction action interface
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* ------------------------------------------------------------------------- */
/*   Navigator structs, enums                                                */
/* ------------------------------------------------------------------------- */

struct NavigatorPoint {
    double x;
    double y;
    bool   spin;
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

    Navigator(ros::Publisher* cmdvelPub, int numberOfRings, int numberOfCylinders) : Navigator() {
      cmdvelPublisher = cmdvelPub;
      NUMBER_OF_RINGS = numberOfRings;
      NUMBER_OF_CYLINDERS = numberOfCylinders;
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
    }

    // Navigate through a list (vector) of points
    void navigateList(vector<NavigatorPoint> points) {
      int len = points.size();

      for (int i = 0; i < len; i++) {
        if (ringsFound < NUMBER_OF_RINGS || cylindersFound < NUMBER_OF_CYLINDERS) {
          navigateTo(points[i]);
        }
      }
    }

    // Callback to handle /custom_msgs/nav/ring_detected
    void ringCallback(const task2::RingPoseMsgConstPtr &msg) {
      ROS_INFO(
          "[Navigator] Received new ring detected message: (x: %f, y: %f, z: %f, color: %s)",
          msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z,
          msg->color_name.c_str()
      );
      sayRingColor(msg->color_name);
      ringsFound++;
    }

    // Callback to handle /custom_msgs/nav/green_ring_detected
    void greenRingCallback(const task2::RingPoseMsgConstPtr &msg) {
      ROS_WARN(
          "[Navigator] Green ring detected at (x: %f, y: %f, z: %f)",
          msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z
      );
      sayRingColor(msg->color_name);
      ringsFound++;

      // Activate parking spot search
      NavigatorPoint parkingPoint { msg->pose.position.x + 0.215, msg->pose.position.y, false };
      ROS_WARN(
          "[Navigator] Navigating to parking point: (x: %f, y: %f)",
          parkingPoint.x,
          parkingPoint.y
      );

      isParking = true;
      client->cancelGoal();
      navigateTo(parkingPoint);
    }

    // Callback to handle /custom_msgs/nav/cylinder_detected
    void cylinderCallback(const task2::RingPoseMsgConstPtr &msg) {
      ROS_INFO(
          "[Navigator] Received new cylinder detected message: (x: %f, y: %f, z: %f, color: %s)",
          msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z,
          msg->color_name.c_str()
      );
      cylindersFound++;
      sayCylinderColor(msg->color_name);
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
    bool                     isKilled        = false;
    bool                     goalCancelled   = false;
    bool                     isParking       = false;
    int                      NUMBER_OF_RINGS = 3;
    int                      NUMBER_OF_CYLINDERS = 3;
    int                      ringsFound      = 0;
    int                      cylindersFound  = 0;

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

          // Check if we cancelled the goal
          if (goalCancelled) {
            goalCancelled = false;
            navigateTo(currentGoal);
          }

          break;
        // The action server completed the goal but encountered an error in doing so
        case actionlib::SimpleClientGoalState::ABORTED:
          ROS_WARN("[Navigator] Goal completed but an error was encountered!");
          break;
        // The action server successfully completed the goal
        case actionlib::SimpleClientGoalState::SUCCEEDED:
          ROS_INFO("[Navigator] Successfully completed goal!");

          // Check if this was a parking maneuver
          if (isParking) {
            soundClient->say("I have finished parking myself!");
            ros::Duration(4.0).sleep();
            currentState = NavigatorState::FINISHED;
            isParking    = false;
            break;
          }

          // Spin 360 degrees if the interest points wants us to do so
          if (currentGoal.spin) {
            spin(360.0);
          }

          break;
        // The client lost contact with the action server
        case actionlib::SimpleClientGoalState::LOST:
          ROS_WARN("[Navigator] Client lost contact with the action server!");
          currentState = NavigatorState::FAILED;
          break;
      }
    }

    static constexpr float SPIN_RATE        = 4;
    static constexpr float SPIN_ANGULAR_VEL = 0.75;
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
        ros::spinOnce();
      }

      // Stop the robot after it finishes rotating
      msg.angular.z = 0;
      cmdvelPublisher->publish(msg);
    }

    void sayRingColor(std::string color_name) {
      // Stop the robot temporarily
      goalCancelled = true;
      client->cancelGoal();

      // Say the color of the ring
      std::string speak = "I see a " + color_name + " ring!";
      soundClient->say(speak);
      ros::Duration(2.0).sleep();
    }

    void sayCylinderColor(std::string color_name) {
      // Stop the robot temporarily
      goalCancelled = true;
      client->cancelGoal();

      // Say the color of the cylinder
      std::string speak = "I see a " + color_name + " cylinder!";
      soundClient->say(speak);
      ros::Duration(2.0).sleep();
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
  // vector<NavigatorPoint> interestPoints {
  //   { 0.3815518319606781,  -1.021867036819458, false},
  //   { 2.1781322956085205,  -1.012013554573059, false},
  //   { 3.1676177978515625,  0.2698194980621338,  true},
  //   { 1.1551456451416016,  0.9515640139579773, false},
  //   { 1.0135111808776855,   2.656785011291504, false},
  //   {-0.6259070038795471,   2.225338935852051,  true},
  //   {-1.4852330684661865, 0.15332327783107758,  true},
  // };
  vector<NavigatorPoint> interestPoints {
    {  0.3815518319606781,  -1.021867036819458, false},
    {  2.1781322956085205,  -1.012013554573059, false},
    {  2.6001322956085205,  -0.3131423993110657, true},
    {   3.067493640899658,  0.7811423993110657,  true},
    {  1.4856997728347778,  0.9578039050102234,  true},
    {  2.3523111808776855,   2.510785011291504,  true},
    {-0.39255309104919434,  2.8976528644561768,  true},
    { -0.9696336388587952,  1.8396551609039307,  true},
    { -1.4852330684661865, 0.15332327783107758,  true},
  };

  // Initialize publisher for robot rotation
  ros::Publisher cmdvelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);

  // Initialize Navigator
  navigator = new Navigator(&cmdvelPub, 4, 4);

  // Initialize ring detection subscribers
  ros::Subscriber greenRingSub = nh.subscribe(
      "/custom_msgs/nav/green_ring_detected",
      1,
      &Navigator::greenRingCallback,
      navigator
  );
  ros::Subscriber ringSub =
      nh.subscribe("/custom_msgs/nav/ring_detected", 1, &Navigator::ringCallback, navigator);

  // Initialize cylinder detection subscriber
  ros::Subscriber cylinderSub = nh.subscribe(
      "/custom_msgs/nav/cylinder_detected",
      1,
      &Navigator::cylinderCallback,
      navigator
  );

  // Navigate through interest points
  navigator->navigateList(interestPoints);
}
