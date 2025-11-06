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
using moveit::planning_interface::PlanningSceneInterface;
using namespace std;
// ----------------- Globals -----------------
MoveGroupInterface *arm_ptr = nullptr;
MoveGroupInterface *hand_ptr = nullptr;

// Sweet Picker Standard Topics welche beim OMx verwendet wurde  
ros::Publisher feedback_pub;             // publishes the "Fertig" Nachricht
string feedback_topic = "/rueckmeldung";
string pose_input_topic = "/robot/pose";

// Tischoberfläche mit berücksichtigung der Greiferfinger
double table_z = 0.79 + 0.118;  
// Sicherer Anfahrtshöhe zwischen den Punkten 20cm über dem Tisch
double hover_z = table_z + 0.2;

// ----------------- Vordefinierte Posen und Orientierungen -----------------
geometry_msgs::Quaternion default_orientation;
geometry_msgs::Pose place_pose;

vector<double> start_joints =
    {
        0.0,
        -45.0 * M_PI / 180.0,
        0.0,
        -135.0 * M_PI / 180.0,
        0.0,
        90.0 * M_PI / 180.0,
        45.0 * M_PI / 180.0};
vector<double> observer_joints =
    {
        0.0 * M_PI / 180.0,
        -50.0 * M_PI / 180.0,
        0.0 * M_PI / 180.0,
        -100.0 * M_PI / 180.0,
        0.0 * M_PI / 180.0,
        60.0 * M_PI / 180.0,
        45.0 * M_PI / 180.0};

// ----------------- Task Queue  -----------------
struct PickJob
{
  geometry_msgs:: Point pos; //x,y,z 
  double tcp_yaw_deg;   //Winkel des Greifers um die Z-Achse 
  bool has_tcp_yaw;  //True wenn Winkel angepasst werden soll 
};

queue<PickJob> pick_queue;
mutex pick_queue_mtx;

// Thread-sichere Warteschlange für Pick-Aufgaben
void enqueuePick(const PickJob &pt)
{
  lock_guard<mutex> lk(pick_queue_mtx);
  pick_queue.push(pt);
}

// Versucht eine Pick-Aufgabe aus der Warteschlange zu verarbeiten
bool tryDequeuePick(PickJob &out)
{
  lock_guard<mutex> lk(pick_queue_mtx);
  if (pick_queue.empty())
  {
    return false;
  }
  out = pick_queue.front();
  pick_queue.pop();
  return true;
}

// ----------------- Utility -----------------
void printHelp()
{
  ROS_INFO_STREAM(R"EOS(

============================= Panda CLI Control Help =============================

COMMAND GROUPS
---------------------------------------------------------------------------
  [General]
    help                        - Show this help
    quit                        - Shutdown node
    stop                        - Stop any current motion

  [Move Commands]
    move_to x y z               - Move end-effector to (x, y, z) [m]
    set_joints j1 ... j7        - Set all 7 joints [deg]
    pick x y                    - Pick object at (x, y) on table 

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

===================================================================================

)EOS");
}

// Nach unten gerichtete Greifer-Orientierung
void initializeDefaultOrientation()
{
  default_orientation.w = 0.0;
  default_orientation.x = 1.0;
  default_orientation.y = 0.0;
  default_orientation.z = 0.0;
}

// Vordefinierte Platzierpose auf der linken Seite des Tisches
void initializePredefinedPlacePose()
{
  place_pose.position.x = 0.0;
  place_pose.position.y = -0.6;
  place_pose.position.z = table_z;
  place_pose.orientation = default_orientation;
}

// Bewegt den Roboter in die Startposition
void moveToStartPosition()
{
  arm_ptr->setJointValueTarget(start_joints);
  MoveGroupInterface::Plan plan;
  if (!(arm_ptr->plan(plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS &&
        arm_ptr->execute(plan)))
  {
    ROS_WARN("Failed moving to start Position");
  }
}

// Publisht eine Nachricht auf dem Feedback Topic
void publishFeedback(const string &text)
{
  std_msgs::String msg;
  msg.data = text;
  feedback_pub.publish(msg);
}

bool setWristYawDeg(double angle_deg)
{
  auto st = arm_ptr->getCurrentState();
  const auto *jmg = st->getJointModelGroup("panda_arm");
  vector<double> joints;
  st->copyJointGroupPositions(jmg, joints);
  joints[6] = angle_deg * M_PI / 180.0; // Set wrist joint
  arm_ptr->setJointValueTarget(joints);
  MoveGroupInterface::Plan plan;
  if (!(arm_ptr->plan(plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS &&
        arm_ptr->execute(plan)))
  {
    ROS_WARN("Failed setting wrist yaw to %.1f°", angle_deg);
    return false;
  }
  ROS_INFO("Wrist yaw set to %.1f deg", angle_deg);
  return true;
}
// ----------------- Pick Routine -----------------
void pickRoutine(const PickJob &p_xy)
{
  const double x = p_xy.pos.x;
  const double y = p_xy.pos.y;

  // Erhöhte Postion des zugreifenden Objekts
  geometry_msgs::Pose above_pose;
  above_pose.position.x = x;
  above_pose.position.y = y;
  above_pose.position.z = hover_z;
  above_pose.orientation = default_orientation;

  // Erhöhte Positiion der Platzierpose - Anpassen für die Schnittstelle zum
  // Jackal: geplant ist aktuelle ein Rohr in der die Süßigkeiten platziert werden
  geometry_msgs::Pose above_place_pose;
  above_place_pose.position.x = 0.0;
  above_place_pose.position.y = -0.6;
  above_place_pose.position.z = hover_z;
  above_place_pose.orientation = default_orientation;

  // Eigentliche Position des zu greifenden Objekts
  geometry_msgs::Pose grasp_pose = above_pose;
  grasp_pose.position.z = table_z;

  // ---- Pick Sequence 1 ----
  ROS_INFO("[auto_pick] Move above target (%.2f, %.2f)", x, y);
  arm_ptr->setPoseTarget(above_pose);
  MoveGroupInterface::Plan plan1;
  if (!(arm_ptr->plan(plan1) && arm_ptr->execute(plan1)))
  {
    ROS_WARN("[auto_pick] Failed move above");
    //publishFeedback("FAILED: move_above");
    return;
  }
  if (p_xy.has_tcp_yaw)
  {
    ROS_INFO("[auto_pick] Applying TCP yaw %.1f deg", p_xy.tcp_yaw_deg);
    if (!setWristYawDeg(p_xy.tcp_yaw_deg))
    {
      ROS_WARN("[auto_pick] Failed to apply TCP yaw");
      // we still try to continue
    }
  }
  //  ---- Pick Sequence 2 ----
  ROS_INFO("[auto_pick] Opening gripper...");
  hand_ptr->setNamedTarget("open");
  hand_ptr->move();

  ROS_INFO("[auto_pick] Lowering to grasp...");
  arm_ptr->setPoseTarget(grasp_pose);
  MoveGroupInterface::Plan plan2;
  if (!(arm_ptr->plan(plan2) && arm_ptr->execute(plan2)))
  {
    ROS_WARN("[auto_pick] Failed lowering");
    //publishFeedback("FAILED: lower");
    return;
  }
  //  ---- Pick Sequence 3 ----
  ROS_INFO("[auto_pick] Closing gripper...");
  hand_ptr->setNamedTarget("close");
  hand_ptr->move();

  ROS_INFO("[auto_pick] Lifting back to hover...");
  arm_ptr->setPoseTarget(above_pose);
  MoveGroupInterface::Plan plan_lift;
  if (!(arm_ptr->plan(plan_lift) && arm_ptr->execute(plan_lift)))
  {
    ROS_WARN("[auto_pick] Failed to lift back to hover");
    //publishFeedback("FAILED: lift");
    return;
  }
  // ---- Place Sequence 1 ----
  ROS_INFO("[auto_pick] Moving to place position...");
  arm_ptr->setPoseTarget(place_pose);
  MoveGroupInterface::Plan plan3;
  if (!(arm_ptr->plan(plan3) && arm_ptr->execute(plan3)))
  {
    ROS_WARN("[auto_pick] Failed move to place");
    //publishFeedback("FAILED: move_place");
    return;
  }

  ROS_INFO("[auto_pick] Opening gripper...");
  hand_ptr->setNamedTarget("open");
  hand_ptr->move();

  // ---- Place Sequence 2 ----
  ROS_INFO("[auto_pick] Moving to above place position...");
  arm_ptr->setPoseTarget(above_place_pose);
  MoveGroupInterface::Plan plan4;
  if (!(arm_ptr->plan(plan4) && arm_ptr->execute(plan4)))
  {
    ROS_WARN("[auto_pick] Failed move to above place");
    //publishFeedback("FAILED: move_above_place");
    return;
  }

  // ---- Return to Start Position ----
  ROS_INFO("[auto_pick] Returning to start configuration...");
  arm_ptr->setJointValueTarget(start_joints);
  MoveGroupInterface::Plan plan5;
  if (!(arm_ptr->plan(plan5) && arm_ptr->execute(plan5)))
  {
    ROS_WARN("[auto_pick] Failed to return to start");
    //publishFeedback("FAILED: return_start");
    return;
  }
  // Publish Fertig Nachricht auf feedback_topic
  publishFeedback("Süßigkeit dargereicht"); //Süigkeit dargereicht laut SP3 Architektur
}

// ----------------- SP4 Schnittstellen -----------------
// Abbonieren des /robot/pose topic zum starten der Pick_Routine nach auslesen der Nachricht
void sp4PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  //Funktionale Erweiterung: Header verwenden um zu unterscheiden ob der Auftrag vom
  //Jackal kommt: Entweder mit Pick Vorgang warten bis der Roboter an der Abholposition ist
  //oder Pick Vorgang starten und Süßigkeit halten solange der Roboter sich zur Abholposition bewegt
  geometry_msgs::Pose target_xy;
  target_xy.position.x = msg->pose.position.x;
  target_xy.position.y = msg->pose.position.y;
  target_xy.position.z = table_z;
  // Nach der Intergration der Kamera - Übergebene Orientierung verwenden
  target_xy.orientation.w = msg->pose.orientation.w;
  target_xy.orientation.x = msg->pose.orientation.x;
  target_xy.orientation.y = msg->pose.orientation.y;
  target_xy.orientation.z = msg->pose.orientation.z;

  // Auslesen der Orientierung des zu greifenden Objektes um  Joint[6] entsprechend anzupassen
  double angle_rad = atan2(msg->pose.orientation.z, msg->pose.orientation.w) * 2.0;
  double angle_deg = angle_rad * 180.0 / M_PI;
  //TODO: PoseCallbasck anpassen um den Winkel zu berücksichtigen - evtl. in pickRoutine integrieren

  ROS_INFO(
      "[SP4] Received /robot/pose -> robot (m): x=%.3f y=%.3f ; Orientation angle=%.1f°",
      target_xy.position.x, target_xy.position.y, angle_deg);
      PickJob p_xy;
      p_xy.pos = target_xy.position;
      p_xy.tcp_yaw_deg = angle_deg;
      p_xy.has_tcp_yaw = true;
  // Direkt in die Pick-Queue einreihen
  enqueuePick(p_xy);
}

// Timer Callback zur Abarbeitung der Pick-Queue
void processQueueTimerCb(const ros::TimerEvent &)
{
  PickJob job;
  if (tryDequeuePick(job))
  {
    ROS_INFO("[QUEUE] Start pick job at x=%.3f y=%.3f", job.pos.x, job.pos.y);
    publishFeedback("Verarbeitung des Pick-Auftrags gestartet");
    pickRoutine(job);
  }
}

// ----------------- main -----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "panda_cli_control");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh;

  MoveGroupInterface arm("panda_arm");
  MoveGroupInterface hand("panda_hand");
  PlanningSceneInterface planning_scene_interface;

  arm_ptr = &arm;
  hand_ptr = &hand;

  arm.setPlanningTime(15.0);
  arm.setMaxVelocityScalingFactor(0.5);
  hand.setMaxVelocityScalingFactor(0.5);

  initializeDefaultOrientation();
  initializePredefinedPlacePose();
  moveToStartPosition();
  printHelp();

  // ---- Sweet Picker Schnittstellen ----
  ros::Subscriber sub_pose = nh.subscribe(pose_input_topic, 1, sp4PoseCallback);
  feedback_pub = nh.advertise<std_msgs::String>(feedback_topic, 1, true);

  // Timer zur Abarbeitung der Pick-Queue (10 Hz)
  ros::Timer queue_timer = nh.createTimer(ros::Duration(0.1), processQueueTimerCb);

  // ---- CLI Loop ----
  string line;
  while (ros::ok())
  {
    // Eingabeaufforderung
    cout << "\n> " << flush;
    // Einlesen der Eingabezeile in
    if (!getline(cin, line))
    {
      break;
    }
    // Parsen der Eingabezeile
    istringstream iss(line);
    string cmd;
    iss >> cmd;
    // Verarbeiten der Befehle
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
        continue;
      }
      geometry_msgs::Pose target;
      target.position.x = x;
      target.position.y = y;
      target.position.z = z;
      target.orientation = default_orientation;
      arm.setPoseTarget(target);
      MoveGroupInterface::Plan plan;
      if (!(arm.plan(plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS &&
            arm.execute(plan)))
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
        continue;
      }
      default_orientation.w = qw;
      default_orientation.x = qx;
      default_orientation.y = qy;
      default_orientation.z = qz;
      ROS_INFO("Default orientation set.");
    }
    else if (cmd == "turn_hand")
    {
      double angele_deg;
      if (!(iss >> angele_deg))
      {
        ROS_WARN("INPUT_ERROR: turn_hand angle_deg");
        continue;
      }
     setWristYawDeg(angele_deg );
    }
    else if (cmd == "set_joints")
    {
      vector<double> joint_deg(7);
      for (int i = 0; i < 7; ++i)
      {
        if (!(iss >> joint_deg[i]))
        {
          ROS_WARN("INPUT_ERROR: set_joints j1...j7 (deg)");
          joint_deg.clear();

          break;
        }
      }
      // Umrechnung der Gelenkwinkel von Grad in Radiant
      if (!joint_deg.empty())
      {
        vector<double> joint_rad(7);
        for (int i = 0; i < 7; ++i)
        {
          joint_rad[i] = joint_deg[i] * M_PI / 180.0;
        }
        arm.setJointValueTarget(joint_rad);
        MoveGroupInterface::Plan plan;
        if (!(arm.plan(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS &&
              arm.execute(plan)))
        {
          ROS_WARN("Planning failed");
        }
      }
    }
    else if (cmd == "print_pose")
    {
      auto ps = arm.getCurrentPose();
      const auto &p = ps.pose.position;
      const auto &o = ps.pose.orientation;
      ROS_INFO("Pose: x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z);
      ROS_INFO("Ori:  w=%.3f x=%.3f y=%.3f z=%.3f", o.w, o.x, o.y, o.z);
    }
    else if (cmd == "print_joints")
    {
      auto st = arm.getCurrentState();
      const auto *jmg = st->getJointModelGroup("panda_arm");
      vector<double> joints;
      st->copyJointGroupPositions(jmg, joints);
      for (size_t i = 0; i < joints.size(); ++i)
      {
        ROS_INFO("Joint %zu: %.1f°", i + 1, joints[i] * 180.0 / M_PI);
      }
    }
    else if (cmd == "open")
    {
      hand.setNamedTarget("open");
      hand.move();
    }
    else if (cmd == "close")
    {
      hand.setNamedTarget("close");
      hand.move();
    }
    else if (cmd == "stop")
    {
      arm.stop();
      hand.stop();
      ROS_INFO("Stopped");
    }
    else if (cmd == "observerPos")
    {
      arm.setJointValueTarget(observer_joints);
      MoveGroupInterface::Plan plan;
      if (!(arm.plan(plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS &&
            arm.execute(plan)))
      {
        ROS_WARN("Failed moving to observer Position");
      }
    }
    else if (cmd == "placePos")
    {
      arm.setPoseTarget(place_pose);
      MoveGroupInterface::Plan plan;
      if (!(arm.plan(plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS &&
            arm.execute(plan)))
      {
        ROS_WARN("Failed moving to place Position");
      }
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
        continue;
      }
      if (iss >> angle_deg)
      {
        got_angle = true;
      }
      PickJob p_xy;
      p_xy.pos.x = x;
      p_xy.pos.y = y;
      p_xy.pos.z = table_z;
      if (got_angle)
      {
        p_xy.tcp_yaw_deg = angle_deg;
        p_xy.has_tcp_yaw = true;
      }
      else
      {
        p_xy.has_tcp_yaw = false;
      }

      enqueuePick(p_xy);
    }
    else if (cmd == "quit")
    {
      ROS_INFO("Shutdown");
      break;
    }
    else if (!cmd.empty())
    {
      ROS_WARN("Unknown cmd '%s'. Type 'help'", cmd.c_str());
    }
  }

  ros::shutdown();
  return 0;
}
