#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <mutex>
#include <string>
#include <vector>
#include <cmath>

class PandaCliController
{
public:
  PandaCliController();

  // CLI input processing
  void handleCommands(const std::string& line);

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  // -------------------- Types --------------------
  struct PickJob
  {
    geometry_msgs::Point pos{};      // x,y,z
    double tcp_yaw_deg{45.0};        // yaw around Z [deg]
    bool has_tcp_yaw{false};

    double width_mm{0.0};
    bool has_width{false};
  };

  // -------------------- ROS & MoveIt --------------------
  ros::NodeHandle nh_;
  MoveGroupInterface arm_;
  MoveGroupInterface hand_;

  ros::Publisher  feedback_pub_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_width_;
  ros::Subscriber sub_goalpose_;
  ros::Subscriber sub_state_;

  std::string feedback_topic_        = "/rueckmeldung";
  std::string sweet_pose_topic_      = "/sweet_pose_translated";
  std::string sweet_width_topic_     = "/robot/status";
  std::string sweet_goalpose_topic_  = "/sp4/goalpose";
  std::string state_topic_           = "/current_state";
  std::string last_sp_state_         = "";

  // -------------------- Configuration --------------------
  double grap_z_  = 0.015; // Greifhöhe über Tisch zum Testen
  double hover_z_ = 0.265; // Hoverhöhe für die Bildhöhe und Pick-Bewegungen

  geometry_msgs::Quaternion default_orientation_;

  enum class MotionMode { PTP, LIN };
  MotionMode cli_mode_ = MotionMode::PTP;
  // -------------------- State --------------------
  // GoalPose-ID: 1 = Standard-Ablage, 2 = Jackal-Ablage
  std::mutex goalpose_mtx_;
  int current_goalpose_id_ = 1;

  std::mutex object_width_mtx_;
  double object_width_mm_ = 0.0;
  bool has_object_width_ = false;

  bool pick_running_ = false;

  // -------------------- Math helpers --------------------
  double deg2rad(double deg) { return deg * (M_PI / 180.0); }
  double rad2deg(double rad) { return rad * (180.0 / M_PI); }

  // -------------------- Core helpers --------------------
  void initializeDefaultOrientation();

  bool planAndExecute(MoveGroupInterface& group, MoveGroupInterface::Plan& plan);
  void setPTP();
  void setLIN();

  bool moveArmToPose(const geometry_msgs::Pose& pose);

  void openGripper();
  void closeGripper();
  void closeGripperWithWidth(double width_mm);

  void stopMovement();

  void moveToObserverPosition();
  void moveToStartPosition();
  void moveToPlacePosition();

  bool moveToAbovePlaceJoints(int goalpose_id);
  bool moveToCarryJoints();

  geometry_msgs::Pose makePlacePose(int goalpose_id, bool above) const;

  bool setWristAngle(double angle_deg);

  void printPose();
  void printJoints();

  void publishFeedback();
  void pickRoutine(const PickJob& object_data);

  // -------------------- ROS callbacks --------------------
  void sp4PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void robotStatusCallback(const std_msgs::String::ConstPtr& msg);
  void goalPoseCallback(const std_msgs::Int32::ConstPtr& msg);
  void stateCallback(const std_msgs::String::ConstPtr& msg);

  // -------------------- CLI help --------------------
  void printHelp();
};
