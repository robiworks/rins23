#include "navigator_fsm.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <signal.h>
#include <sound_play/sound_play.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <string>
#include <task3/ArmExtendSrv.h>
#include <task3/CylinderFaceSrv.h>
#include <task3/FaceDialogueSrv.h>
#include <task3/FacePositionMsg.h>
#include <task3/FineApproachSrv.h>
#include <task3/PosterExplorationSrv.h>
#include <task3/RingPoseMsg.h>

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

struct PosterData {
    int         prize;
    std::string ringColor;
};

/* ------------------------------------------------------------------------- */
/*   Navigator class                                                         */
/* ------------------------------------------------------------------------- */

class Navigator {
  public:
    // Main FSM state
    FSMMainState currentMainState;

    // Child FSM states depending on current main state
    FSMExploringState currentExploringState;
    FSMSearchingState currentSearchingState;
    FSMParkingState   currentParkingState;

    /* --------------------------------------------------------------------- */
    /*   Constructors and init                                               */
    /* --------------------------------------------------------------------- */

    Navigator() {
      ROS_INFO("Initializing");

      // Initialize MoveBaseClient
      client = new MoveBaseClient("move_base", true);
      client->waitForServer();

      // Initialize SoundClient
      soundClient = new sound_play::SoundClient;
      ROS_INFO("Waiting for SoundClient initialization");
      ros::Duration(2.0).sleep();

      // The FSM starts in Exploring.Exploring state
      currentMainState      = FSMMainState::EXPLORING;
      currentExploringState = FSMExploringState::EXPLORING;
    }

    Navigator(
        ros::Publisher*     cmdvelPub,
        ros::ServiceClient* posterExplorationSrv,
        ros::ServiceClient* faceDialogueSrv,
        ros::ServiceClient* armExtendSrv,
        ros::ServiceClient* cylinderFaceSrv,
        ros::Publisher*     parkingPub,
        ros::ServiceClient* fineApproachSrv
    )
        : Navigator() {
      cmdvelPublisher          = cmdvelPub;
      posterExplorationService = posterExplorationSrv;
      faceDialogueService      = faceDialogueSrv;
      armExtendService         = armExtendSrv;
      cylinderFaceService      = cylinderFaceSrv;
      parkingPublisher         = parkingPub;
      fineApproachService      = fineApproachSrv;
    }

    /* --------------------------------------------------------------------- */
    /*   Public navigator functions                                          */
    /* --------------------------------------------------------------------- */

    // Approach a point
    // Used when approaching faces, posters, cylinders, rings, etc.
    void approachPoint(geometry_msgs::Pose point) {
      move_base_msgs::MoveBaseGoal goal;

      // Set header
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp    = ros::Time::now();
      // Set target pose
      goal.target_pose.pose = point;

      ROS_INFO(
          "Approaching: (x: %.3f, y: %.3f, rot_z: %.3f)",
          point.position.x,
          point.position.y,
          point.orientation.z
      );
      client->sendGoal(goal);

      // Start monitoring navigation
      monitorNavigation();
    }

    // Navigate to a given point
    // Used when exploring through interest points
    void navigateTo(NavigatorPoint point) {
      move_base_msgs::MoveBaseGoal goal;

      // Set header
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp    = ros::Time::now();
      // Set target position
      goal.target_pose.pose.position.x    = point.x;
      goal.target_pose.pose.position.y    = point.y;
      goal.target_pose.pose.orientation.w = 1;

      ROS_INFO("Navigating to: (x: %.3f, y: %.3f, spin: %d)", point.x, point.y, point.spin);
      client->sendGoal(goal);

      // Start monitoring navigation
      monitorNavigation();
    }

    // Navigate through a list (vector) of points
    void navigateList(vector<NavigatorPoint> points) {
      ROS_INFO("-------- Starting EXPLORING phase");

      int len = points.size();
      for (int i = 0; i < len; i++) {
        currentGoal = points[i];
        navigateTo(points[i]);

        // Spin 360 degrees if the interest points wants us to do so
        if (points[i].spin) {
          spin(360.0);
        }
      }

      if (currentMainState == FSMMainState::EXPLORING) {
        // Finished exploring the polygon
        currentExploringState = FSMExploringState::FINISHED;

        // Transition to searching state
        currentMainState = FSMMainState::SEARCHING;
        ROS_INFO("-------- Starting SEARCHING phase");
        startSearchingPhase();
      } else {
        ROS_ERROR(
            "Invalid main state after finishing navigation through interest points! %d",
            static_cast<int>(currentMainState)
        );
      }
    }

    // Searching phase (main state) of FSM
    void startSearchingPhase() {
      if (dialogueCylinderColors.size() < 2) {
        ROS_ERROR("No cylinder colors saved, aborting searching phase!");
        cleanUp();
        return;
      }

      // Find cylinder colors and their locations
      std::string col1 = toUpperCase(dialogueCylinderColors.at(0));
      std::string col2 = toUpperCase(dialogueCylinderColors.at(1));
      ROS_INFO("Searching for cylinders %s, %s", col1.c_str(), col2.c_str());
      task3::RingPoseMsg cyl1, cyl2;

      for (int i = 0; i < savedCylinders.size(); i++) {
        task3::RingPoseMsgConstPtr item = savedCylinders.at(i);
        std::string                col  = toUpperCase(item->color_name);

        printf("Cylinder %d: %s\n", i, col.c_str());

        if (col1 == col) {
          cyl1.color      = item->color;
          cyl1.color_name = item->color_name;
          cyl1.pose       = item->pose;
          continue;
        }
        if (col2 == col) {
          cyl2.color      = item->color;
          cyl2.color_name = item->color_name;
          cyl2.pose       = item->pose;
          continue;
        }
      }

      // if (!cyl1 || !cyl2) {
      //   ROS_ERROR("Could not find the cylinders with the required colors");
      //   return;
      // }

      // Try cylinder 1 first
      std::string result = approachCylinder(cyl1);
      if (result != "null") {
        // Robber found on 1st cylinder
        takeRobberToPrison(result);
      } else {
        // Robber is on 2nd cylinder
        result = approachCylinder(cyl2);
        takeRobberToPrison(result);
      }
    }

    // Approach a cylinder, look on top of it and continue depending on result
    std::string approachCylinder(task3::RingPoseMsg cylinder) {
      // Approach the cylinder
      currentSearchingState = FSMSearchingState::DRIVING;
      ROS_INFO("Approaching cylinder: %s", cylinder.color_name.c_str());
      approachPoint(cylinder.pose);

      // Transition to AT_CYLINDER state
      currentSearchingState   = FSMSearchingState::AT_CYLINDER;
      std::string prisonColor = "null";

      // Extend arm
      task3::ArmExtendSrv extend;
      extend.request.extend = true;

      currentSearchingState = FSMSearchingState::LOOKING;
      if (armExtendService->call(extend)) {
        task3::CylinderFaceSrv look;
        look.request.cylinder_color = cylinder.color_name;

        // Look on top of cylinder
        task3::FineApproachSrv fineApproach;
        fineApproach.request.action = "approach";
        if (fineApproachService->call(fineApproach)) {
          if (fineApproach.response.success) {
            soundClient->say("Cylinder approached");
            ros::Duration(5.0).sleep();
          } else {
            ROS_ERROR("Fine approach service failed");
          }

        } else {
          ROS_ERROR("Failed to call fine approach service");
        }

        if (cylinderFaceService->call(look)) {
          if (look.response.correct_robber) {
            // Found the robber
            ROS_INFO("Robber found, ring color: %s", look.response.ring_color.c_str());
            currentSearchingState = FSMSearchingState::ROBBER_FOUND;
            prisonColor           = look.response.ring_color;
          } else {
            // Didn't find the robber here
            ROS_INFO("Robber not found");
            currentSearchingState = FSMSearchingState::ROBBER_NOT_FOUND;
          }
        }
        // Move arm back to default position
        fineApproach.request.action = "retreat";
        if (fineApproachService->call(fineApproach)) {
          if (fineApproach.response.success) {
            soundClient->say("Cylinder retreated");
            ros::Duration(5.0).sleep();
          } else {
            ROS_ERROR("Fine approach service failed");
          }

        } else {
          ROS_ERROR("Failed to call fine approach service");
        }
        task3::ArmExtendSrv retract;
        retract.request.extend = false;
        armExtendService->call(retract);
      } else {
        ROS_ERROR("Failed to call arm extend service");
      }

      return prisonColor;
    }

    // Take the robber to assigned prison
    void takeRobberToPrison(std::string prisonColor) {
      ROS_INFO("-------- Starting PARKING phase");

      // Transition to parking main state
      currentMainState    = FSMMainState::PARKING;
      currentParkingState = FSMParkingState::DRIVING;

      // Tell the robot to enter the car
      soundClient->say("Hey mister robber, take a spin in our robot car!");
      ros::Duration(5.0).sleep();

      for (int i = 0; i < savedRings.size(); i++) {
        task3::RingPoseMsgConstPtr item = savedRings.at(i);

        if (item->color_name == prisonColor) {
          // Drive to approach point of ring
          approachPoint(item->pose);
          break;
        }
      }

      // Should be at the ring approach point here
      currentParkingState = FSMParkingState::FINDING_SPOT;

      // Extend arm
      task3::ArmExtendSrv extend;
      extend.request.extend = true;
      if (armExtendService->call(extend)) {
        // Publish "request" to find parking spot
        std_msgs::Bool parking;
        parking.data = true;
        parkingPublisher->publish(parking);

        // Wait for parking position callback to fire
        // Parking execution is moved into that callback
        spin(180.0, 0.5);
      } else {
        ROS_ERROR("Failed to call arm extend service");
      }
    }

    // Clean up (used on SIGINT)
    void cleanUp() {
      // Cancel all goals on the client and stop playing sounds
      client->cancelAllGoals();
      soundClient->stopAll();
      isKilled = true;
    }

    /* --------------------------------------------------------------------- */
    /*   Callback handlers                                                   */
    /* --------------------------------------------------------------------- */

    // RING DETECTION
    // Callback to handle /custom_msgs/nav/ring_detected
    void ringCallback(const task3::RingPoseMsgConstPtr &msg) {
      if (currentMainState == FSMMainState::EXPLORING &&
          currentExploringState == FSMExploringState::EXPLORING) {
        ROS_INFO(
            "Ring detected: (x: %f, y: %f, z: %f, color: %s)",
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->color_name.c_str()
        );

        // Transition state and optionally say the ring's color
        currentExploringState = FSMExploringState::RING_DETECTED;
        client->cancelGoal();
        describeObject("ring", msg->color_name);

        // Save the ring's color and location
        savedRings.push_back(msg);
        ringsFound++;

        // Go back to exploring state after saying color
        currentExploringState = FSMExploringState::EXPLORING;
        navigateTo(currentGoal);
      } else {
        warnInvalidState("Cannot process ring detections outside Exploring.Exploring!");
      }
    }

    // CYLINDER DETECTION
    // Callback to handle /custom_msgs/nav/cylinder_detected
    void cylinderCallback(const task3::RingPoseMsgConstPtr &msg) {
      if (currentMainState == FSMMainState::EXPLORING &&
          currentExploringState == FSMExploringState::EXPLORING) {
        ROS_INFO(
            "Cylinder detected: (x: %f, y: %f, z: %f, color: %s)",
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->color_name.c_str()
        );

        // Transition state and say the cylinder's color
        currentExploringState = FSMExploringState::CYLINDER_DETECTED;
        client->cancelGoal();
        describeObject("cylinder", msg->color_name);

        // Save the cylinder's color and location
        savedCylinders.push_back(msg);
        cylindersFound++;

        // Go back to exploring state after saying color
        currentExploringState = FSMExploringState::EXPLORING;
        navigateTo(currentGoal);
      } else {
        warnInvalidState("Cannot process cylinder detections outside Exploring.Exploring!");
      }
    }

    // FACE DETECTION
    // Callback to handle /custom_msgs/face_detected
    void faceDetectedCallback(const task3::FacePositionMsgConstPtr &msg) {
      if (currentMainState == FSMMainState::EXPLORING &&
          currentExploringState == FSMExploringState::EXPLORING) {
        ROS_INFO(
            "Face detected: (x: %f, y: %f, z: %f)",
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        // Send the robot towards the detected face
        currentExploringState = FSMExploringState::APPROACHING_FACE;

        // Approach face
        std_msgs::Bool approachMsg;
        approachMsg.data = true;

        // Get navigator point for approaching
        geometry_msgs::Pose approachPose;
        approachPose = msg->pose;

        ROS_INFO("Approaching face");
        client->cancelGoal();
        // isCancelled = true;
        approachPoint(approachPose);

        describeObject("face", "clever");
        ros::Duration(5.0).sleep();

        // Finished approaching, transition state
        currentExploringState = FSMExploringState::AT_FACE;

        task3::FaceDialogueSrv srv;
        // Dialogue performed in this step
        currentExploringState = FSMExploringState::DIALOGUE;
        if (faceDialogueService->call(srv)) {
          ROS_INFO("Face dialogue service called successfully");
          ROS_INFO(
              "useful: %d, color 1: %s, color 2: %s",
              srv.response.useful,
              srv.response.color1.c_str(),
              srv.response.color2.c_str()
          );
          bool kurac = srv.response.useful;

          if (kurac) {
            // Save colors from dialogue in this step
            currentExploringState = FSMExploringState::SAVE_DIALOGUE;
            ROS_INFO("Saving colors from dialogue");

            dialogueCylinderColors.push_back(srv.response.color1);
            dialogueCylinderColors.push_back(srv.response.color2);
          }
        } else {
          ROS_ERROR("Failed to call dialogue service");
        }

        // Go back to exploring
        currentExploringState = FSMExploringState::EXPLORING;
      } else {
        warnInvalidState("Cannot process face detections outside Exploring.Exploring!");
      }
    }

    // POSTER DETECTION
    // Callback to handle /custom_msgs/poster_detected
    void posterDetectedCallback(const task3::FacePositionMsgConstPtr &msg) {
      if (currentMainState == FSMMainState::EXPLORING &&
          currentExploringState == FSMExploringState::EXPLORING) {
        ROS_INFO(
            "Poster detected: (x: %f, y: %f, z: %f)",
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        // Send the robot towards the detected face
        currentExploringState = FSMExploringState::APPROACHING_POSTER;

        // Approach poster
        std_msgs::Bool approachMsg;
        approachMsg.data = true;

        // Get navigator point for approaching
        geometry_msgs::Pose approachPose;
        approachPose = msg->pose;

        ROS_INFO("Approaching poster");
        client->cancelGoal();
        // isCancelled = true;
        approachPoint(approachPose);

        describeObject("poster", "funny");
        ros::Duration(5.0).sleep();

        // Finished approaching, transition state
        currentExploringState = FSMExploringState::AT_POSTER;

        task3::PosterExplorationSrv srv;
        if (posterExplorationService->call(srv)) {
          // OCR performed in this step
          currentExploringState = FSMExploringState::POSTER_OCR;

          ROS_INFO("Poster exploration service called successfully");
          ROS_INFO("%d prize, %s ring_color", srv.response.prize, srv.response.ring_color.c_str());

          char buffer[128];
          sprintf(
              buffer,
              "I have detected %d BTC prize with %s ring_color",
              srv.response.prize,
              srv.response.ring_color.c_str()
          );
          describeObject(buffer, ".");

          // Save poster data in this step
          currentExploringState = FSMExploringState::SAVE_POSTER;
          PosterData poster     = { srv.response.prize, srv.response.ring_color };
          savedPosters.push_back(poster);
        } else {
          ROS_ERROR("Failed to call poster exploration service");
        }

        // Go back to exploring
        currentExploringState = FSMExploringState::EXPLORING;
      } else {
        warnInvalidState("Cannot process poster detections outside Exploring.Exploring!");
      }
    }

    // PARKING POINT DETECTION
    // Callback to handle /arm_control/parking_point
    void parkingPointCallback(geometry_msgs::Pose pose) {
      if (currentMainState == FSMMainState::PARKING &&
          currentParkingState == FSMParkingState::FINDING_SPOT) {
        ROS_INFO("Received parking point: (x: %f, y: %f)", pose.position.x, pose.position.y);

        // Park into spot
        currentParkingState = FSMParkingState::MANEUVERING;
        approachPoint(pose);

        // Parking should be finished at this point
        currentParkingState = FSMParkingState::FINISHED;
        soundClient->say("I'm done!");
        ros::Duration(3.0).sleep();

        // Wave with the manipulator
        task3::ArmExtendSrv srv;
        srv.request.extend = false;
        armExtendService->call(srv);
        srv.request.extend = true;
        armExtendService->call(srv);

        // Move to END state
        currentMainState = FSMMainState::END;
        ROS_INFO("-------- Transitioned to END phase");
      } else {
        warnInvalidState("Cannot process parking point detection outside Parking.FindingSpot!");
      }
    }

  private:
    // Navigation client
    MoveBaseClient*          client;
    sound_play::SoundClient* soundClient;
    NavigatorPoint           currentGoal;

    // Publishers
    ros::Publisher* cmdvelPublisher;
    ros::Publisher* parkingPublisher;

    // Services
    ros::ServiceClient* posterExplorationService;
    ros::ServiceClient* faceDialogueService;
    ros::ServiceClient* armExtendService;
    ros::ServiceClient* cylinderFaceService;
    ros::ServiceClient* fineApproachService;

    // Status booleans
    bool isKilled    = false;
    bool isCancelled = false;

    // Default: 4 rings and 4 cylinders in task 3
    int NUMBER_OF_RINGS     = 4;
    int NUMBER_OF_CYLINDERS = 4;
    int ringsFound          = 0;
    int cylindersFound      = 0;

    // Vectors for saving data
    vector<task3::RingPoseMsgConstPtr> savedRings;
    vector<task3::RingPoseMsgConstPtr> savedCylinders;
    vector<PosterData>                 savedPosters;
    vector<std::string>                dialogueCylinderColors; // Should have exactly 2 items

    /* --------------------------------------------------------------------- */
    /*   Navigation monitoring                                               */
    /* --------------------------------------------------------------------- */

    void monitorNavigation() {
      // Monitor navigation at 5 Hz
      ros::Rate rate(5);

      // Monitor navigation until it reaches a terminal state
      actionlib::SimpleClientGoalState goalState = client->getState();
      while (!goalState.isDone() && !isKilled && !isCancelled) {
        ROS_DEBUG("Current goal state: %s", goalState.toString().c_str());

        // Update currentState according to goal state
        switch (goalState.state_) {
          // The goal has been sent to the action server but has not yet been processed
          case actionlib::SimpleClientGoalState::PENDING:
            break;
          // The goal is currently being worked on by the action server
          case actionlib::SimpleClientGoalState::ACTIVE:
            break;
        }

        if (currentMainState == FSMMainState::EXPLORING &&
            currentExploringState == FSMExploringState::EXPLORING) {
          // Process queued callbacks if the robot is just driving around, exploring
          ros::spinOnce();
        }

        rate.sleep();
        goalState = client->getState();
      }

      // if (isCancelled) {
      //   // Reset cancelled state (happens when cancelling an interest goal to approach face/poster)
      //   ROS_INFO("Cancelling interest point navigation monitoring to approach face or poster");
      //   isCancelled = false;
      //   return;
      // }

      // Handle terminal states
      switch (goalState.state_) {
        // The client cancels a goal before the action server has started working on it
        case actionlib::SimpleClientGoalState::RECALLED:
          ROS_WARN("Client cancelled a goal before the server started processing it!");
          break;
        // The action server rejected the goal for some reason
        case actionlib::SimpleClientGoalState::REJECTED:
          ROS_ERROR("Action server rejected the goal!");
          break;
        // The client cancels a goal that is currently being worked on by the action server
        case actionlib::SimpleClientGoalState::PREEMPTED:
          ROS_WARN("Client cancelled a goal currently being worked on!");
          break;
        // The action server completed the goal but encountered an error in doing so
        case actionlib::SimpleClientGoalState::ABORTED:
          ROS_ERROR("Goal completed but an error was encountered!");
          break;
        // The action server successfully completed the goal
        case actionlib::SimpleClientGoalState::SUCCEEDED:
          if (currentMainState == FSMMainState::EXPLORING &&
              (currentExploringState == FSMExploringState::APPROACHING_FACE ||
               currentExploringState == FSMExploringState::APPROACHING_POSTER)) {
            ROS_INFO("Finished approaching face or poster!");
          } else if (currentMainState == FSMMainState::SEARCHING && currentSearchingState == FSMSearchingState::DRIVING) {
            ROS_INFO("Finished approaching cylinder!");
          } else if (currentMainState == FSMMainState::PARKING && currentParkingState == FSMParkingState::DRIVING) {
            ROS_INFO("Finished approaching parking ring!");
          } else if (currentMainState == FSMMainState::PARKING && currentParkingState == FSMParkingState::MANEUVERING) {
            ROS_INFO("Finished maneuvering into parking spot!");
          } else {
            ROS_INFO("Successfully navigated to interest point!");
          }
          break;
        // The client lost contact with the action server
        case actionlib::SimpleClientGoalState::LOST:
          ROS_ERROR("Client lost contact with the action server!");
          break;
      }
    }

    /* --------------------------------------------------------------------- */
    /*   Fine robot maneuvering                                              */
    /* --------------------------------------------------------------------- */

    static constexpr float SPIN_RATE        = 4;
    static constexpr float SPIN_ANGULAR_VEL = 0.75;
    static constexpr float DEGREE_RATIO     = 29.75 / 360;

    void spin(float degrees, float angularVel = SPIN_ANGULAR_VEL) {
      ros::Rate            rate(SPIN_RATE);
      geometry_msgs::Twist msg;

      // Set parameters for rotation
      msg.linear.x  = 0.0;
      msg.angular.z = angularVel;

      // Calculate number of iterations required to spin given degrees
      int iterations = round((degrees * DEGREE_RATIO) / angularVel);
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

    /* --------------------------------------------------------------------- */
    /*   Functions utilizing sound node                                      */
    /* --------------------------------------------------------------------- */

    void describeObject(std::string objectType, std::string colorName) {
      // Say the type of the object and its color
      std::string sentence = "I see a " + colorName + " " + objectType + "!";
      soundClient->say(sentence);
      ros::Duration(2.0).sleep();
    }

    /* --------------------------------------------------------------------- */
    /*   Utilities                                                           */
    /* --------------------------------------------------------------------- */

    void warnInvalidState(std::string reason) {
      ROS_WARN("Invalid state: %s", reason.c_str());
      ROS_WARN("-- Current main state: %d", static_cast<int>(currentMainState));
      ROS_WARN("-- Current exploring state: %d", static_cast<int>(currentExploringState));
      ROS_WARN("-- Current searching state: %d", static_cast<int>(currentSearchingState));
      ROS_WARN("-- Current parking state: %d", static_cast<int>(currentParkingState));
    }

    std::string toUpperCase(const std::string &str) {
      std::string result = str;
      std::transform(result.begin(), result.end(), result.begin(), ::toupper);
      return result;
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
    {0.20354169607162476, -0.7123628854751587, true},
    { 3.3873140811920166, 0.23788361251354218, true},
    {  1.096898078918457,    0.75606369972229, true},
    {  2.399085521697998,  2.0735414028167725, true},
    { 1.0957181453704834,  2.8578972816467285, true},
    {-1.3266278505325317,  2.2552082538604736, true},
    { -0.964529275894165,  0.7039206027984619, true},
  };

  // Initialize publisher for robot rotation
  ros::Publisher cmdvelPub  = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
  ros::Publisher parkingPub = nh.advertise<std_msgs::Bool>("/arm_control/scan", 10);

  // Initialize services
  ros::ServiceClient posterExplorationService =
      nh.serviceClient<task3::PosterExplorationSrv>("/poster_exploration");
  ros::ServiceClient faceDialogueService =
      nh.serviceClient<task3::FaceDialogueSrv>("/face_dialogue");
  ros::ServiceClient armExtendService =
      nh.serviceClient<task3::ArmExtendSrv>("/arm_control/extend");
  ros::ServiceClient cylinderFaceService = nh.serviceClient<task3::CylinderFaceSrv>("/is_robber");
  ros::ServiceClient fineApproachService =
      nh.serviceClient<task3::FineApproachSrv>("/fine_approach");

  // Initialize Navigator
  navigator = new Navigator(
      &cmdvelPub,
      &posterExplorationService,
      &faceDialogueService,
      &armExtendService,
      &cylinderFaceService,
      &parkingPub,
      &fineApproachService
  );

  // Initialize subscribers
  ros::Subscriber ringSub =
      nh.subscribe("/custom_msgs/nav/ring_detected", 1, &Navigator::ringCallback, navigator);
  ros::Subscriber cylinderSub = nh.subscribe(
      "/custom_msgs/nav/cylinder_detected",
      1,
      &Navigator::cylinderCallback,
      navigator
  );
  ros::Subscriber faceSub =
      nh.subscribe("/custom_msgs/face_detected", 1, &Navigator::faceDetectedCallback, navigator);
  ros::Subscriber posterSub = nh.subscribe(
      "/custom_msgs/poster_detected",
      1,
      &Navigator::posterDetectedCallback,
      navigator
  );
  ros::Subscriber parkingSub =
      nh.subscribe("/arm_control/parking_point", 1, &Navigator::parkingPointCallback, navigator);

  // Navigate through interest points
  navigator->navigateList(interestPoints);
}
