#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <mutex>
#include <queue>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <cmath>

using moveit::planning_interface::MoveGroupInterface;

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define PANDA_SIM_OFFSET_Z 0.79

// Umrechnungsfunktionen Grad <-> Radiant (Namen beibehalten)
inline double deg2rad(double deg) { return deg * DEG2RAD; }
inline double rad2deg(double rad) { return rad * RAD2DEG; }

class PandaCliControl
{
public:
  PandaCliControl()
    : nh_(),
      arm_("panda_arm"),
      hand_("panda_hand")
  {
    // MoveIt-Basiskonfiguration
    arm_.setPlanningTime(10.0);
    arm_.setMaxVelocityScalingFactor(1.0);
    hand_.setMaxVelocityScalingFactor(1.0);

    initializeDefaultOrientation();
    moveToStartPosition();
    moveToObserverPosition();
    printHelp();

    // Publisher / Subscriber / Timer
    feedback_pub = nh_.advertise<std_msgs::String>(feedback_topic, 1, true);
    sub_pose_    = nh_.subscribe(sweet_pose_topic,  1, &PandaCliControl::sp4PoseCallback, this);
    sub_width_   = nh_.subscribe(sweet_width_topic, 1, &PandaCliControl::robotStatusCallback, this);
    sub_receiver_ = nh_.subscribe(sweet_receiver_topic, 1, &PandaCliControl::receiverCallback, this);
    queue_timer_ = nh_.createTimer(ros::Duration(0.1), &PandaCliControl::processQueueTimerCb, this);
  }

  // Verarbeitet die eingegebene CLI-Zeile zum Ausführen von Befehlen
  void handleCommands(const std::string& line)
  {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "help")
    {
      printHelp();
    }
    else if (cmd == "move_to")
    {
      double x, y, z;
      if (!(iss >> x >> y >> z))
      {
        ROS_WARN("INPUT_ERROR: move_to x y z");
        return;
      }

      geometry_msgs::Pose target;
      target.position.x = x;
      target.position.y = y;
      target.position.z = z;
      auto current_orientation = arm_.getCurrentPose().pose.orientation;
      target.orientation = current_orientation;

      if (!moveArmToPose(target, "move_to"))
      {
        ROS_WARN("Planning failed");
      }
    }
    else if (cmd == "set_orientation")
    {
      double qw, qx, qy, qz;
      if (!(iss >> qw >> qx >> qy >> qz))
      {
        ROS_WARN("INPUT_ERROR: set_orientation qw qx qy qz");
        return;
      }
      default_orientation.w = qw;
      default_orientation.x = qx;
      default_orientation.y = qy;
      default_orientation.z = qz;
      ROS_INFO("Default orientation set.");
    }
    else if (cmd == "turn_hand")
    {
      double angle_deg;
      if (!(iss >> angle_deg))
      {
        ROS_WARN("INPUT_ERROR: turn_hand angle_deg");
        return;
      }
      setWristAngle(angle_deg);
    }
    else if (cmd == "set_joints")
    {
      std::vector<double> joint_deg(7);
      for (int i = 0; i < 7; ++i)
      {
        if (!(iss >> joint_deg[i]))
        {
          ROS_WARN("INPUT_ERROR: set_joints j1...j7 (deg)");
          return;
        }
      }

      std::vector<double> joint_rad(7);
      for (int i = 0; i < 7; ++i)
      {
        joint_rad[i] = deg2rad(joint_deg[i]);
      }

      arm_.setJointValueTarget(joint_rad);
      MoveGroupInterface::Plan plan;
      if (!planAndExecute(arm_, plan, "set_joints"))
      {
        ROS_WARN("Planning failed");
      }
    }
    else if (cmd == "print_pose")
    {
      printPose();
    }
    else if (cmd == "print_joints")
    {
      printJoints();
    }
    else if (cmd == "open")
    {
      openGripper();
    }
    else if (cmd == "close")
    {
      closeGripper();
    }
    else if (cmd == "stop")
    {
      stopMovement();
    }
    else if (cmd == "observerPos")
    {
      moveToObserverPosition();
    }
    else if (cmd == "placePos")
    {
      moveToPlacePosition();
    }
    else if (cmd == "startPos")
    {
      moveToStartPosition();
    }
    else if (cmd == "pick")
    {
      double x, y;
      double angle_deg;
      bool got_angle = false;

      if (!(iss >> x >> y))
      {
        ROS_WARN("INPUT_ERROR: pick x y [angle_deg]");
        return;
      }
      if (iss >> angle_deg)
      {
        got_angle = true;
      }

      PickJob object_data;
      object_data.pos.x = x;
      object_data.pos.y = y;
      object_data.pos.z = grap_z;   // gleicher Name wie vorher

      if (got_angle)
      {
        object_data.tcp_yaw_deg = angle_deg;
        object_data.has_tcp_yaw = true;
      }
      else
      {
        object_data.has_tcp_yaw = false;
      }

      enqueuePick(object_data);
    }
    else if (cmd == "quit")
    {
      ROS_INFO("Shutdown");
      ros::shutdown();
    }
    else if (!cmd.empty())
    {
      ROS_WARN("Unknown cmd '%s'. Type 'help'", cmd.c_str());
    }
  }

private:
  // Pick-Auftrag Struct
  struct PickJob
  {
    geometry_msgs::Point pos{};   // x, y, z
    double tcp_yaw_deg  {45.0};   // Greifer-Winkel um die Z-Achse
    bool   has_tcp_yaw  {false};  // true, wenn Winkel gesetzt werden soll
    double width_mm     {0.0};    // Breite der Süßigkeit in mm
    std::string receiver{"default"}; // "default" oder "jackal"

  };

  //--------------------------------------------------- ROS & MoveIt Member --------------------------------------------------------

  ros::NodeHandle   nh_;
  MoveGroupInterface arm_;
  MoveGroupInterface hand_;

  ros::Publisher  feedback_pub;                             // Publisher für die "Fertig" Nachricht
  ros::Subscriber sub_pose_;                                // Subscriber für die Süßigkeiten-Position
  ros::Subscriber sub_width_;                               // Subscriber für die Süßigkeiten-Breite
  ros::Subscriber sub_receiver_;                            // Subscriber für den Empfänger
  ros::Timer      queue_timer_;

  std::string feedback_topic    = "/rueckmeldung";          // Topic für die "Fertig" Nachricht
  std::string sweet_pose_topic  = "/sweet_pose_translated"; // Informationen über die übersetzten Koordinaten
  std::string sweet_width_topic = "/robot/status";          // Informationen über die Süßigkeiten-Breite
  std::string sweet_receiver_topic = "/sp4/receiver";       // Informationen über den Empfänger

  //--------------------------------------------------- Konfiguration --------------------------------------------------------

  // --- Höhen für Greif- und Hover-Positionen ---
  const double grap_z  = PANDA_SIM_OFFSET_Z + 0.15;
  const double hover_z = PANDA_SIM_OFFSET_Z + 0.22;

  // Default-Orientierung
  geometry_msgs::Quaternion default_orientation;
  //--------------------------------------------------- Zustands-Variablen --------------------------------------------------------

  // Thread-sichere Pick-Queue
  std::queue<PickJob> pick_queue;
  // Mutexes für Thread-Sicherheit
  std::mutex  pick_queue_mtx;
  std::mutex  receiver_mtx;
  std::mutex  object_width_mtx;
  // Standardwerte für die Süßigkeiten-Breite und Empfänger
  double      object_width_mm  = 0.0;
  bool        has_object_width = false;
  std::string current_receiver = "default";
  //--------------------------------------------------- Hilfsfunktionen ---------------------------------------------------------

  // Job in die Queue einreihen
  void enqueuePick(const PickJob& job)
  {
    std::lock_guard<std::mutex> lk(pick_queue_mtx);
    pick_queue.push(job);
  }

  // Nächsten Job holen (falls vorhanden)
  bool tryDequeuePick(PickJob& out)
  {
    std::lock_guard<std::mutex> lk(pick_queue_mtx);
    if (pick_queue.empty())
    {
      return false;
    }
    out = pick_queue.front();
    pick_queue.pop();
    return true;
  }

  // Timer Callback zur Abarbeitung der Pick-Queue
  void processQueueTimerCb(const ros::TimerEvent&)
  {
    PickJob job;
    if (tryDequeuePick(job))
    {
      ROS_INFO("[QUEUE] Start pick job at x=%.3f y=%.3f", job.pos.x, job.pos.y);
      pickRoutine(job);
    }
  }

  // Initialisiert die Default-Orientierung so dass der Greifer nach unten zeigt
  void initializeDefaultOrientation()
  {
    default_orientation.w = 0.0;
    default_orientation.x = 1.0;
    default_orientation.y = 0.0;
    default_orientation.z = 0.0;
  }

  // Plant und führt den gegebenen Plan aus
  bool planAndExecute(MoveGroupInterface& group, MoveGroupInterface::Plan& plan, const char* description)
  {
    if (!(group.plan(plan) ==
              moveit::planning_interface::MoveItErrorCode::SUCCESS &&
          group.execute(plan)))
    {
      ROS_WARN("Failed: %s", description);
      return false;
    }
    return true;
  }

  // Wrapper: Pose-Ziel setzen und ausführen
  bool moveArmToPose(const geometry_msgs::Pose& pose, const char* description)
  {
    arm_.setPoseTarget(pose);
    MoveGroupInterface::Plan plan;
    if (!planAndExecute(arm_, plan, description))
    {
      ROS_WARN("Failed: %s", description);
      return false;
    }
    return true;
  }

  // Öffnet den Greifer komplett
  void openGripper()
  {
    hand_.setNamedTarget("open");
    hand_.move();
  }

  // Schließt den Greifer komplett
  void closeGripper()
  {
    hand_.setNamedTarget("close");
    hand_.move();
  }

  // Schließt den Greifer mit berücksichtigung der Objektbreite in mm
  void closeGripperWithWidth(double width_mm)
  {
    double object_width_m = width_mm / 1000.0;
    const double safety_margin = 0.002; // 2 mm
    double target_gap   = object_width_m + safety_margin;
    double finger_joint = target_gap / 2.0;

    std::vector<double> joints;
    joints.push_back(finger_joint);
    joints.push_back(finger_joint);

    hand_.setJointValueTarget(joints);
    hand_.move();

    ROS_INFO("[SP4] closeGripperWithWidth: width=%.1f mm -> gap=%.2f mm, joint=%.3f rad",
             width_mm, target_gap * 1000.0, finger_joint);
  }

  // Stoppt alle Bewegungen von Arm und Hand
  void stopMovement()
  {
    arm_.stop();
    hand_.stop();
    ROS_INFO("Stopped");
  }

  // Bewegt den Roboter in die Beobachterposition damit ein Bild der Szene gemacht werden kann
  void moveToObserverPosition()
  {
    geometry_msgs::Pose observer_p;
    observer_p.position.x = 0.4;
    observer_p.position.y = 0.0;
    observer_p.position.z = hover_z;
    observer_p.orientation.x = 0.92388;
    observer_p.orientation.y = -0.38268;
    observer_p.orientation.z = 0.0;
    observer_p.orientation.w = 0.0;

    moveArmToPose(observer_p, "move to observer position");
  }

  // Bewegt den Roboter in die Startposition
  void moveToStartPosition()
  {
    std::vector<double> start_joints =
    {
      0.0,
      deg2rad(-45.0),
      0.0,
      deg2rad(-135.0),
      0.0,
      deg2rad(90.0),
      deg2rad(45.0)
    };
    arm_.setJointValueTarget(start_joints);
    MoveGroupInterface::Plan plan;
    if (!planAndExecute(arm_, plan, "move to start position"))
    {
      ROS_WARN("Failed moving to start Position");
    }
  }

  // Bewegt den Roboter zur Ablageposition
  void moveToPlacePosition()
  {
    geometry_msgs::Pose place_pose;
    place_pose.position.x = 0.0;
    place_pose.position.y = -0.6;
    place_pose.position.z = grap_z;
    place_pose.orientation = default_orientation;
    if (!moveArmToPose(place_pose, "move to place position"))
    {
      ROS_WARN("Failed moving to place Position");
    }
  }

  // Erstellt eine Ablagepose basierend auf dem Empfänger
  geometry_msgs::Pose makePlacePose(const std::string& receiver, bool above) const
{
    geometry_msgs::Pose pose;
    pose.orientation = default_orientation;

    const bool is_jackal = (receiver == "jackal");

    if (is_jackal)
    {
      // TODO: Anpassen dass die Position des Ablagerohrs angefahren wird
      pose.position.x = 0.0;
      pose.position.y = -0.7;
    }
    else
    {
      // bisherige Standard-Ablage
      pose.position.x = 0.0;
      pose.position.y = -0.6;
    }

    if(above)
    {
      pose.position.z = hover_z;
    }
    else
    {
      pose.position.z = grap_z;
    }
    return pose;
  }

  // Ändert Joint 7 auf den angegebenen Winkel in Grad
  bool setWristAngle(double angle_deg)
  {
    auto st = arm_.getCurrentState();
    const auto* jmg = st->getJointModelGroup("panda_arm");
    std::vector<double> joints;
    st->copyJointGroupPositions(jmg, joints);
    double target_deg = std::fmod(angle_deg + 180.0, 360.0) - 180.0;
    joints[6] = deg2rad(target_deg);
    arm_.setJointValueTarget(joints);
    MoveGroupInterface::Plan plan;
    if (!planAndExecute(arm_, plan, "set wrist yaw"))
    {
      ROS_WARN("Failed setting wrist yaw to %.1f°", angle_deg);
      return false;
    }

    ROS_INFO("Wrist yaw set to %.1f deg", angle_deg);
    return true;
  }

  // Gibt die aktuellen Koordinaten und Orientierung des Endeffektors aus
  void printPose()
  {
    auto ps = arm_.getCurrentPose();
    const auto& p = ps.pose.position;
    const auto& o = ps.pose.orientation;
    ROS_INFO("Pose: x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z);
    ROS_INFO("Ori:  w=%.3f x=%.3f y=%.3f z=%.3f", o.w, o.x, o.y, o.z);
  }

  // Gibt die aktuellen Gelenkwinkel in Grad aus
  void printJoints()
  {
    auto st = arm_.getCurrentState();
    const auto* jmg = st->getJointModelGroup("panda_arm");
    std::vector<double> joints;
    st->copyJointGroupPositions(jmg, joints);
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
      ROS_INFO("Joint %zu: %.1f°", i + 1, rad2deg(joints[i]));
    }
  }

  // Publisht die SP4 "Fertig" Nachricht auf dem Feedback Topic
  void publishFeedback()
  {
    std_msgs::String msg;
    msg.data = "Süßigkeit dargereicht";
    feedback_pub.publish(msg);
  }
  // Führt die Pick and Place Routine für ein einzelnes Objekt durch
  void pickRoutine(const PickJob& object_data)
  {
    const double x = object_data.pos.x;
    const double y = object_data.pos.y;

    geometry_msgs::Pose above_pose;
    above_pose.position.x = x;
    above_pose.position.y = y;
    above_pose.position.z = hover_z;
    above_pose.orientation = default_orientation;

    geometry_msgs::Pose above_place_pose = makePlacePose(object_data.receiver, true);
    geometry_msgs::Pose place_pose       = makePlacePose(object_data.receiver, false);

    geometry_msgs::Pose grasp_pose = above_pose;
    grasp_pose.position.z = grap_z;

    ROS_INFO("[auto_pick] Move above target (%.2f, %.2f)", x, y);
    if (!moveArmToPose(above_pose, "move above target"))
    {
      ROS_WARN("[auto_pick] Failed move above");
      return;
    }

    if (object_data.has_tcp_yaw)
    {
      ROS_INFO("[auto_pick] Applying TCP yaw %.1f deg", object_data.tcp_yaw_deg);
      if (!setWristAngle(object_data.tcp_yaw_deg))
      {
        ROS_WARN("[auto_pick] Failed to apply TCP yaw");
      }
    }

    ROS_INFO("[auto_pick] Opening gripper...");
    openGripper();

    auto c_orientation = arm_.getCurrentPose().pose.orientation;
    grasp_pose.orientation = c_orientation;
    ROS_INFO("[auto_pick] Lowering to grasp...");
    if (!moveArmToPose(grasp_pose, "lower to grasp"))
    {
      ROS_WARN("[auto_pick] Failed lowering");
      return;
    }

    double width_mm = object_data.width_mm;
    ROS_INFO("[auto_pick] Closing gripper for candy width %.1f mm ...", width_mm);
    closeGripperWithWidth(width_mm);

    above_pose.orientation = c_orientation;
    ROS_INFO("[auto_pick] Lifting back to hover...");
    if (!moveArmToPose(above_pose, "lift back to hover"))
    {
      ROS_WARN("[auto_pick] Failed to lift back to hover");
      return;
    }

    above_place_pose.orientation = c_orientation;
    ROS_INFO("[auto_pick] Moving to above place position...");
    if (!moveArmToPose(above_place_pose, "move to above place"))
    {
      ROS_WARN("[auto_pick] Failed move to above place");
      return;
    }

    place_pose.orientation = c_orientation;
    ROS_INFO("[auto_pick] Moving to place position...");
    if (!moveArmToPose(place_pose, "move to place"))
    {
      ROS_WARN("[auto_pick] Failed move to place");
      return;
    }

    ROS_INFO("[auto_pick] Opening gripper...");
    openGripper();

    above_place_pose.orientation = c_orientation;
    ROS_INFO("[auto_pick] Moving to above place position...");
    if (!moveArmToPose(above_place_pose, "move to above place (after open)"))
    {
      ROS_WARN("[auto_pick] Failed move to above place");
      return;
    }

    ROS_INFO("[auto_pick] Returning to observer position");
    moveToObserverPosition();

    publishFeedback();
  }
  // --------------------------------------------------- SP4 Schnittstellen ---------------------------------------------------------

  // Abonnieren des /sweet_pose_translated Topic zum Starten der Pick-Routine
  void sp4PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    double width_mm = 0.0;

    {
      std::lock_guard<std::mutex> lk(object_width_mtx);
      if (!has_object_width)
      {
        ROS_WARN("[SP4] Pose empfangen, aber keine aktuelle Breite vorhanden -> Pick wird ignoriert");
        return;
      }
      width_mm = object_width_mm;
      has_object_width = false;
    }

    geometry_msgs::Pose target_xy;
    target_xy.position.x = msg->pose.position.x;
    target_xy.position.y = msg->pose.position.y;
    target_xy.position.z = grap_z;

    target_xy.orientation = msg->pose.orientation;

    double angle_rad = std::atan2(msg->pose.orientation.z,
                                  msg->pose.orientation.w) * 2.0;
    double angle_deg = rad2deg(angle_rad);

    ROS_INFO(
      "[SP4] Received /robot/pose -> robot (m): x=%.3f y=%.3f; Orientation angle=%.1f°, width=%.1f mm",
      target_xy.position.x, target_xy.position.y, angle_deg, width_mm);

    PickJob object_data;
    object_data.pos         = target_xy.position;
    object_data.tcp_yaw_deg = angle_deg;
    object_data.has_tcp_yaw = true;
    object_data.width_mm    = width_mm;
    {
      std::lock_guard<std::mutex> lk(receiver_mtx);
      object_data.receiver = current_receiver;
    }

    enqueuePick(object_data);
  }

  // Abonnieren des /robot/status Topic zum Aktualisieren der Süßigkeiten-Breite
  void robotStatusCallback(const std_msgs::String::ConstPtr& msg)
  {
    std::istringstream iss(msg->data);
    double width_mm = 0.0;

    if (!(iss >> width_mm))
    {
      ROS_WARN("[SP4] /robot/status: konnte Breite nicht aus '%s' lesen", msg->data.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lk(object_width_mtx);
      object_width_mm  = width_mm;
      has_object_width = true;
    }

    ROS_INFO("[SP4] /robot/status: Breite aktualisiert auf %.1f mm", width_mm);
  }

  // Abonnieren des /sp4/receiver Topic zum Aktualisieren des Empfängers
  void receiverCallback(const std_msgs::String::ConstPtr& msg)
  {
    std::string receiver;

    if (msg->data.empty())
    {
      receiver = "default";
    }
    else
    {
      receiver = msg->data;
    }

    {
      std::lock_guard<std::mutex> lk(receiver_mtx);
      current_receiver = receiver;
    }

    ROS_INFO("[SP4] /sp4/receiver: receiver updated to '%s'", receiver.c_str());
  }

  // --------------------------------------------------- CLI Hilfe ---------------------------------------------------------
  void printHelp()
  {
    ROS_INFO_STREAM(R"EOS(

============================= Panda CLI Control Help ==========================

COMMAND GROUPS
---------------------------------------------------------------------------
  [General]
    help                        - Show this help
    quit                        - Shutdown node
    stop                        - Stop any current motion

  [Move Commands]
    move_to x y z               - Move end-effector to (x, y, z) [m]
    set_joints j1 ... j7        - Set all 7 joints [deg]
    pick x y [angle_deg]        - Pick object at (x, y) on table, optional yaw

  [TCP Commands]
    open                        - Open the panda hand
    close                       - Close the panda hand
    turn_hand angle             - Rotate wrist joint [deg]
    set_orientation qw qx qy qz - Set default end-effector orientation (quaternion)

  [Status Commands]
    print_pose                  - Print current end-effector pose
    print_joints                - Print current joint angles [deg]

  [Predefined Poses]
    startPos                    - Move to predefined start pose
    observerPos                 - Move to predefined observer pose
    placePos                    - Move to predefined placement pose

================================================================================

)EOS");
  }
};

// --------------------------------------------------- Main ---------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_cli_control");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  PandaCliControl node;

  // ---- CLI Loop ----
  std::string line;
  while (ros::ok())
  {
    std::cout << "\n> " << std::flush;
    if (!std::getline(std::cin, line))
    {
      break;
    }
    node.handleCommands(line);
  }

  ros::shutdown();
  return 0;
}
