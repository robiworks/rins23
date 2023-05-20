#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <cmath>
#include <ctime>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <signal.h>
#include <sound_play/sound_play.h>
#include <stdlib.h>
#include <task2/RingPoseMsg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

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

struct DoublePoint
{
    double x;
    double y;
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

    vector<NavigatorPoint> generateInterestPoints(int wallMargin, int pointMargin, int numClusters, ros::Publisher* marker_pub, ros::ServiceClient* proxy){
      // Read the map using cv2
      Mat map = imread("./src/task3/config/map2_edited.pgm", IMREAD_GRAYSCALE);
  
      // Map information position and resolution
      double positionX = -12.2;
      double positionY = -12.2;
      double resolution = 0.05000000074505806;

      // Create binary map where 254 is 1 and else is 0
      Mat binaryMap = map.clone();
      binaryMap = binaryMap == 254;

      // Erode the map
      Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
      erode(binaryMap, binaryMap, kernel);

      // Create a list of all (x, y) coordinates of the map. binarymap where values are 1
      vector<DoublePoint> mapPoints;
      int totalPoints = 0;
      for (int i = 0; i < map.rows; i++) {
        for (int j = 0; j < map.cols; j++) {
          totalPoints++;
          if (map.at<uchar>(i, j) == 254) {
            // Create a double point
            DoublePoint point;
            point.x = j;
            point.y = i;
            mapPoints.push_back(point);
          }
        }
      }

      // List of points
      vector<DoublePoint> points;
      while (points.size() < numClusters)
      {
        bool addPoint = true;

        // Select random point
        int randomIndex = rand() % mapPoints.size();
        DoublePoint selectedPoint = mapPoints[randomIndex];
        // printf("Selected point: %f, %f\n", selectedPoint.x, selectedPoint.y);

        // Check if point is too close to wall by checking if all values in circle around point is 0
        for (int i = -wallMargin; i < wallMargin; i++) {
          for (int j = -wallMargin; j < wallMargin; j++) {
            if (binaryMap.at<uchar>(selectedPoint.y + i, selectedPoint.x + j) == 0) {
              addPoint = false;
              break;
            }
          }
        }

        // Check if point is too close to other points
        if (points.size() > 0){
          for (int i = 0; i < points.size(); i++) {
            if (sqrt(pow(selectedPoint.x - points[i].x, 2) + pow(selectedPoint.y - points[i].y, 2)) < pointMargin) {
              addPoint = false;
              break;
            }
          }
        }

        // Add point if it is not too close to wall or other points
        if (addPoint) {
          points.push_back(selectedPoint);
          // SHow point on map
          // cv::circle(map, Point(selectedPoint.x, selectedPoint.y), 1, Scalar(0, 0, 255), 4);
        }
      }
      // cv::imshow("Map", map);
      // cv::waitKey(0);

      // Create markerarray
      visualization_msgs::MarkerArray marker_array;

      // Go through points and calculate points.x and points.y
      for (int i = 0; i < points.size(); i++) {
 
      }

      // Rotate all points 90 degrees
      for (int i = 0; i < points.size(); i++) {
        // Calculate x and y
        double x = positionX + points[i].x * resolution;
        double y = positionY + points[i].y * resolution;

        points[i].x = y;
        points[i].y = x;    

        x = points[i].x;
        y = points[i].y;


        float rad = -90 * (M_PI / 180);
        // printf("Radians: %f\n", rad);
        points[i].x = x * cos(rad) - y * sin(rad);
        points[i].y = y * cos(rad) + x * sin(rad);

        points[i].y = points[i].y - 0.25;
      }

      // Create a poseStamped at 0, 0 
      geometry_msgs::PoseStamped origin = createPoseStamped(0, 0);

      vector<geometry_msgs::PoseStamped> poseStampedPoints;
      for (int i = 0; i < points.size(); i++) {
        geometry_msgs::PoseStamped pose = createPoseStamped(points[i].x, points[i].y);
        poseStampedPoints.push_back(pose);
      }

      // Closest point
      geometry_msgs::PoseStamped startPoint = getClosest(origin, poseStampedPoints, proxy);

      // Ordered points
      vector<geometry_msgs::PoseStamped> orderedPoints;
      orderedPoints.push_back(startPoint);

      // Remove start point from poseStampedPoints
      for (int i = 0; i < poseStampedPoints.size(); i++) {
        if (poseStampedPoints[i].pose.position.x == startPoint.pose.position.x && poseStampedPoints[i].pose.position.y == startPoint.pose.position.y) {
          poseStampedPoints.erase(poseStampedPoints.begin() + i);
          break;
        }
      }

      // Go through all points and find the closest one
      while (poseStampedPoints.size() > 0)
      {
        //  Prev point
        geometry_msgs::PoseStamped prevPoint = orderedPoints[orderedPoints.size() - 1];

        // Closest point
        geometry_msgs::PoseStamped closestPoint = getClosest(prevPoint, poseStampedPoints, proxy);

        // Add closest point to orderedPoints
        orderedPoints.push_back(closestPoint);

        // Remove closest point from poseStampedPoints
        for (int i = 0; i < poseStampedPoints.size(); i++) {
          if (poseStampedPoints[i].pose.position.x == closestPoint.pose.position.x && poseStampedPoints[i].pose.position.y == closestPoint.pose.position.y) {
            poseStampedPoints.erase(poseStampedPoints.begin() + i);
            break;
          }
        }
      }

      // Final points
      vector<geometry_msgs::PoseStamped> finalPoints = calculateAngles(orderedPoints);

      // Create markerarray for visualization
      vector<NavigatorPoint> navigatorPoints;

      for (int i = 0; i < finalPoints.size(); i++) {
        NavigatorPoint point;
        point.x = finalPoints[i].pose.position.x;
        point.y = finalPoints[i].pose.position.y;
        point.spin = true;

        navigatorPoints.push_back(point);


        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "points";
        marker.id = i;

        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = finalPoints[i].pose.position.x;
        marker.pose.position.y = finalPoints[i].pose.position.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = finalPoints[i].pose.orientation.w;
        marker.pose.orientation.z = finalPoints[i].pose.orientation.z;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        if (i == 0) {
          marker.type = visualization_msgs::Marker::SPHERE;
        } else if (i == finalPoints.size() - 1) {
          marker.type = visualization_msgs::Marker::CUBE;
        } else {
          marker.type = visualization_msgs::Marker::ARROW;
          marker.scale.x = 0.1;
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;
        }

        marker_array.markers.push_back(marker);
      }

      marker_pub->publish(marker_array);

      return navigatorPoints;
    }

    geometry_msgs::PoseStamped createPoseStamped(double x, double y) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.w = 1.0;

      return pose;
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

    vector<geometry_msgs::PoseStamped> calculateAngles(vector<geometry_msgs::PoseStamped> points) {

      // Final points
      vector<geometry_msgs::PoseStamped> finalPoints;

      // Go through all points and calculate the angle
      for (int i = 0; i < points.size(); i++) {
        // Point
        geometry_msgs::PoseStamped point = points[i];
        // Next point using %
        geometry_msgs::PoseStamped nextPoint = points[(i + 1) % points.size()];

        // Calculate angle
        double angle = atan2(nextPoint.pose.position.y - point.pose.position.y, nextPoint.pose.position.x - point.pose.position.x);

        // Quaternion
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(angle);

        // Get rotations z and w 
        double z = q.z;
        double w = q.w;

        // Create a new point
        geometry_msgs::PoseStamped newPoint;
        newPoint.pose.position.x = point.pose.position.x;
        newPoint.pose.position.y = point.pose.position.y;
        newPoint.pose.position.z = point.pose.position.z;
        newPoint.pose.orientation.z = z;
        newPoint.pose.orientation.w = w;

        // Add point to finalPoints
        finalPoints.push_back(newPoint);
      }

      return finalPoints;
    }

    float calculateDistance(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2, ros::ServiceClient* proxy) {
      // Create a service proxy to get plan
      nav_msgs::GetPlan srv;
      srv.request.start = pose1;
      srv.request.goal = pose2;
      srv.request.tolerance = 0;

      // Call the service
      if (proxy->call(srv)) {
        return srv.response.plan.poses.size();
      } else {
        ROS_ERROR("Failed to call service get_plan");
        return 0;
      }
    }

    geometry_msgs::PoseStamped getClosest(geometry_msgs::PoseStamped goal, vector<geometry_msgs::PoseStamped> points, ros::ServiceClient* proxy) {
      geometry_msgs::PoseStamped closestPoint = points[0];
      float closestDistance = calculateDistance(goal, closestPoint, proxy);

      // Go through all points and find the closest one
      for (int i = 1; i < points.size(); i++) {
        float distance = calculateDistance(goal, points[i], proxy);

        if (distance < closestDistance) {
          closestPoint = points[i];
          closestDistance = distance;
        }
      }

      return closestPoint;
    }

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


  // vector<NavigatorPoint> interestPoints {
  //   {  0.3815518319606781,  -1.021867036819458, false},
  //   {  2.1781322956085205,  -1.012013554573059, false},
  //   {  2.6001322956085205,  -0.3131423993110657, true},
  //   {   3.067493640899658,  0.7811423993110657,  true},
  //   {  1.4856997728347778,  0.9578039050102234,  true},
  //   {  2.3523111808776855,   2.510785011291504,  true},
  //   {-0.39255309104919434,  2.8976528644561768,  true},
  //   { -0.9696336388587952,  1.8396551609039307,  true},
  //   { -1.4852330684661865, 0.15332327783107758,  true},
  // };

  // Initialize publisher for robot rotation
  ros::Publisher cmdvelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);




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

    // Marker publisher
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/marker/points", 10);

    // Get plan proxy
    ros::ServiceClient plan_proxy = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

    // Initialize Navigator
    navigator = new Navigator(&cmdvelPub, 4, 4);
    vector<NavigatorPoint> interestPoints = navigator->generateInterestPoints(10, 18, 10, &marker_pub, &plan_proxy);

  // Navigate through interest points
  navigator->navigateList(interestPoints);
}
