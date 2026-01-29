#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <mutex>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <cmath>

using moveit::planning_interface::MoveGroupInterface;

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)


// Umrechnungsfunktionen Grad <-> Radiant (Namen beibehalten)
inline double deg2rad(double deg) { return deg * DEG2RAD; }
inline double rad2deg(double rad) { return rad * RAD2DEG; }

// Feste Gelenkpose "über Ablage" (abovePlace) – hier musst du ggf. einmal anlernen
std::vector<double> above_place_joints_ =
{
  deg2rad(-90.0),
  deg2rad(-10.0),
  0.0,
  deg2rad(-135.0),
  0.0,
  deg2rad(125.0),
  deg2rad(45.0)
};
std::vector<double> carry_joints_ =
{
  deg2rad(-45.0),
  deg2rad(-10.0),
  0.0,
  deg2rad(-135.0),
  0.0,
  deg2rad(125.0),
  deg2rad(45.0)
};


class PandaCliControl
{
public:
  PandaCliControl()
    : nh_(),
      arm_("panda_arm"),
      hand_("panda_hand")
  {
    // MoveIt-Basiskonfiguration
    arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_.setPlannerId("PTP");
    arm_.setPlanningTime(10.0);

    arm_.setMaxVelocityScalingFactor(0.1);
    arm_.setMaxAccelerationScalingFactor(0.05);

    hand_.setMaxVelocityScalingFactor(0.2);
    hand_.setMaxAccelerationScalingFactor(0.2);

    initializeDefaultOrientation();
    loadZOffset();
    moveToStartPosition();
    moveToObserverPosition();
    printHelp();

    // Publisher / Subscriber / Timer
    feedback_pub = nh_.advertise<std_msgs::String>(feedback_topic, 1, true);
    sub_pose_    = nh_.subscribe(sweet_pose_topic,  1, &PandaCliControl::sp4PoseCallback, this);
    sub_width_   = nh_.subscribe(sweet_width_topic, 1, &PandaCliControl::robotStatusCallback, this);
    sub_receiver_ = nh_.subscribe(sweet_receiver_topic, 1, &PandaCliControl::receiverCallback, this);
    sub_state_ = nh_.subscribe(state_topic, 1, &PandaCliControl::stateCallback, this);
}

  // Verarbeitet die eingegebene CLI-Zeile zum Ausführen von Befehlen
  void handleCommands(const std::string& line)
  {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    // "stop" immer erlauben, auch während der Pick-Routine
    if (cmd == "stop")
    {
      stopMovement();
      return;
    }
    //Blockiern von Eingaben während Pick-Routine
    if (pick_running)
    {
      ROS_WARN("[CLI] Command ignored: pick routine already running");
      return;
    }

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

      pick_running = true;
      pickRoutine(object_data);
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
    bool   has_width    {false};  // true, wenn Breite gesetzt wurde
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
  ros::Subscriber sub_state_;                               // Subscriber für den SP4-Zustand um die Beobachtungsposition zu setzen

  std::string feedback_topic    = "/rueckmeldung";          // Topic für die "Fertig" Nachricht
  std::string sweet_pose_topic  = "/sweet_pose_translated"; // Informationen über die übersetzten Koordinaten
  std::string sweet_width_topic = "/robot/status";          // Informationen über die Süßigkeiten-Breite
  std::string sweet_receiver_topic = "/sp4/receiver";       // Informationen über den Empfänger
  std::string state_topic = "/current_state";               // Topic für den SP4-Zustand
  std::string last_sp_state = "";                           // Letzter SP4-Zustand

  //--------------------------------------------------- Konfiguration --------------------------------------------------------

  // --- Höhen für Greif- und Hover-Positionen ---
  double panda_offset_z = 0.79; //Default (Simualtion)
  double grap_z  = panda_offset_z + 0.0;
  double hover_z = panda_offset_z + 0.0;

  // Default-Orientierung
  geometry_msgs::Quaternion default_orientation;
  //--------------------------------------------------- Zustands-Variablen --------------------------------------------------------

  // Empfänger-Management
  std::mutex  receiver_mtx;
  std::string current_receiver = "default";
  // Objektbreiten-Management
  std::mutex  object_width_mtx;
  double      object_width_mm  = 0.0;
  bool        has_object_width = false;

  bool pick_running = false;
  //--------------------------------------------------- Hilfsfunktionen ---------------------------------------------------------
  // Lädt den Z-Offset aus dem Parameter-Server für die Verwendeung in der Simulation und realen Roboter
  void loadZOffset()
  {
    ros::NodeHandle pnh("~");  // private namespace: ~panda_offset_z
    pnh.param("panda_offset_z", panda_offset_z, 0.79);

    grap_z  = panda_offset_z + 0.2;
    hover_z = panda_offset_z + 0.4;

    ROS_INFO("[INIT] panda_offset_z=%.3f m -> grap_z=%.3f m hover_z=%.3f m",
            panda_offset_z, grap_z, hover_z);
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
    arm_.setStartStateToCurrentState();
    arm_.setPoseTarget(pose);
    MoveGroupInterface::Plan plan;
    if (!planAndExecute(arm_, plan, description))
    {
      ROS_WARN("Failed: %s", description);
      return false;
    }
    return true;
  }
  // Wrapper noch bei Bewegung über die Z-Achse Geschwindigkeit minimieren

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
    std::vector<double> observer_joints =
    {
      0.0,
      deg2rad(-10.0),
      0.0,
      deg2rad(-135.0),
      0.0,
      deg2rad(125.0),
      deg2rad(45.0)
    };
    arm_.setJointValueTarget(observer_joints);
    MoveGroupInterface::Plan plan;
    if (!planAndExecute(arm_, plan, "move to observer position"))
    {
      ROS_WARN("Failed moving to observer Position");
    }
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
  bool moveToAbovePlaceJoints()
  {
    arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_.setPlannerId("PTP");
    arm_.setMaxVelocityScalingFactor(0.10);
    arm_.setMaxAccelerationScalingFactor(0.05);

    arm_.setStartStateToCurrentState();
    arm_.setJointValueTarget(above_place_joints_);

    MoveGroupInterface::Plan plan;
    return planAndExecute(arm_, plan, "move to above place (fixed joints)");
  }
  bool moveToCarryJoints()
  {
    arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_.setPlannerId("PTP");
    arm_.setMaxVelocityScalingFactor(0.10);
    arm_.setMaxAccelerationScalingFactor(0.05);

    arm_.setStartStateToCurrentState();
    arm_.setJointValueTarget(carry_joints_);

    MoveGroupInterface::Plan plan;
    return planAndExecute(arm_, plan, "move to carry pose (fixed joints)");
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
        // aktuelle Standard-Ablage um 20cm  nach hinten versetzt
        pose.position.x = -0.2;
        pose.position.y = -0.6;
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

  // Ändert Joint 7 auf den angegebenen Winkel in Grad (nur 0..166, negative -> +180 äquivalent)
  bool setWristAngle(double angle_deg)
  {
    auto st = arm_.getCurrentState();
    const auto* jmg = st->getJointModelGroup("panda_arm");
    std::vector<double> joints;
    st->copyJointGroupPositions(jmg, joints);

    // 1) Finger-Äquivalenz: negative Werte auf +180 schieben
    double target_deg = angle_deg;
    if (target_deg < 0.0)
      target_deg += 180.0;   // -116 -> 64

    // 2) Nur 0..166 erlauben
    if (target_deg < 0.0)   target_deg = 0.0;
    if (target_deg > 166.0) target_deg = 166.0;

    joints[6] = deg2rad(target_deg);

    arm_.setStartStateToCurrentState();
    arm_.setJointValueTarget(joints);

    MoveGroupInterface::Plan plan;
    if (!planAndExecute(arm_, plan, "set wrist joint7 (0..166)"))
    {
      ROS_WARN("Failed setting wrist yaw request %.1f° (mapped to %.1f°)", angle_deg, target_deg);
      return false;
    }

    auto st_after = arm_.getCurrentState();
    std::vector<double> j_after;
    st_after->copyJointGroupPositions(st_after->getJointModelGroup("panda_arm"), j_after);

    ROS_INFO("Wrist yaw request %.1f deg -> mapped %.1f deg -> joint7 now %.1f deg",
            angle_deg, target_deg, rad2deg(j_after[6]));

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

    // --- Pose-Ziele (nur für LIN Z-Moves und "über Objekt" Pose, falls du das weiterhin willst) ---
    geometry_msgs::Pose above_pose;
    above_pose.position.x = x;
    above_pose.position.y = y;
    above_pose.position.z = hover_z;
    above_pose.orientation = default_orientation;

    geometry_msgs::Pose grasp_pose = above_pose;
    grasp_pose.position.z = grap_z;

    // Ablage-Posen (nur Z-Teil wird linear gemacht)
    geometry_msgs::Pose above_place_pose = makePlacePose(object_data.receiver, true);
    geometry_msgs::Pose place_pose       = makePlacePose(object_data.receiver, false);

    // // -------------------- 1) Über Objekt (PTP) --------------------
    ROS_INFO("[auto_pick] move to above target (%.2f, %.2f, %.2f)", x, y, hover_z);
    if (!moveArmToPose(above_pose, " above target"))
    {
      ROS_WARN("[auto_pick] Failed  above target");
      pick_running = false;
      return;
    }

    // Optional: TCP Yaw setzen (PTP Joint change)
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

    // Orientation fixieren (damit LIN nur Z fährt)
    geometry_msgs::Quaternion c_orientation = arm_.getCurrentPose().pose.orientation;
    above_pose.orientation = c_orientation;
    grasp_pose.orientation = c_orientation;

    // -------------------- 2) Linear runter (LIN) --------------------
    arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_.setPlannerId("LIN");
    arm_.setMaxVelocityScalingFactor(0.05);
    arm_.setMaxAccelerationScalingFactor(0.03);

    ROS_INFO("[auto_pick] LIN down to grasp...");
    if (!moveArmToPose(grasp_pose, "LIN down to grasp"))
    {
      ROS_WARN("[auto_pick] Failed LIN down");
      pick_running = false;
      return;
    }

    // -------------------- 3) Greifen --------------------
    if (object_data.has_width)
    {
      ROS_INFO("[auto_pick] Closing gripper for candy width %.1f mm ...", object_data.width_mm);
      closeGripperWithWidth(object_data.width_mm);
    }
    else
    {
      ROS_WARN("[auto_pick] No width provided -> closing gripper with default close");
      closeGripper();
    }

    // -------------------- 4) Linear hoch (LIN) --------------------
    ROS_INFO("[auto_pick] LIN up to hover...");
    if (!moveArmToPose(above_pose, "LIN up to hover"))
    {
      ROS_WARN("[auto_pick] Failed LIN up");
      pick_running = false;
      return;
    }

    // -------------------- 5) CarryPose (PTP fixed joints) --------------------
    ROS_INFO("[auto_pick] PTP to carry pose (fixed joints)...");
    if (!moveToCarryJoints())
    {
      ROS_WARN("[auto_pick] Failed PTP to carry pose");
      pick_running = false;
      return;
    }

    // -------------------- 6) Über Ablage (PTP fixed joints) --------------------
    ROS_INFO("[auto_pick] PTP to above place using fixed joints...");
    if (!moveToAbovePlaceJoints())
    {
      ROS_WARN("[auto_pick] Failed PTP to fixed abovePlace joints");
      pick_running = false;
      return;
    }

    // Nach fixed-joint move die aktuelle Orientation nehmen, damit LIN sauber nur Z fährt
    c_orientation = arm_.getCurrentPose().pose.orientation;
    above_place_pose.orientation = c_orientation;
    place_pose.orientation       = c_orientation;

    // -------------------- 7) Linear runter (LIN) --------------------
    arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_.setPlannerId("LIN");
    arm_.setMaxVelocityScalingFactor(0.05);
    arm_.setMaxAccelerationScalingFactor(0.03);

    ROS_INFO("[auto_pick] LIN down to place...");
    if (!moveArmToPose(place_pose, "LIN down to place"))
    {
      ROS_WARN("[auto_pick] Failed LIN down to place");
      pick_running = false;
      return;
    }

    // -------------------- 8) Ablegen + linear hoch (LIN) --------------------
    ROS_INFO("[auto_pick] Opening gripper...");
    openGripper();

    ROS_INFO("[auto_pick] LIN up from place...");
    if (!moveArmToPose(above_place_pose, "LIN up from place"))
    {
      ROS_WARN("[auto_pick] Failed LIN up from place");
      pick_running = false;
      return;
    }

    // -------------------- 9) Zurück zur ObserverPos (PTP feste Gelenke) --------------------
    arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_.setPlannerId("PTP");
    arm_.setMaxVelocityScalingFactor(0.10);
    arm_.setMaxAccelerationScalingFactor(0.05);

    ROS_INFO("[auto_pick] Returning to observer position");
    moveToObserverPosition();

    publishFeedback();
    pick_running = false;
  }

  // --------------------------------------------------- SP4 Schnittstellen ---------------------------------------------------------

  // Abonnieren des /sweet_pose_translated Topic zum Starten der Pick-Routine
  void sp4PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    if (pick_running)
    {
      ROS_WARN("[SP4] Ignoring /sweet_pose_translated: pick routine already running");
      return;
    }

    PickJob job;

    // Pose übernehmen
    job.pos = msg->pose.position;
    job.pos.z = grap_z;

    // Yaw berechnen (wie bisher)
    double angle_rad = std::atan2(msg->pose.orientation.z,
                                  msg->pose.orientation.w) * 2.0;
    job.tcp_yaw_deg = rad2deg(angle_rad);
    job.has_tcp_yaw = true;

    // Breite holen (falls vorhanden)
    {
      std::lock_guard<std::mutex> lk(object_width_mtx);
      if (has_object_width)
      {
        job.width_mm = object_width_mm;
        job.has_width = true;
        // Zurücksetzen nach dem Holen
        has_object_width = false;
      }
      else
      {
        job.has_width = false;
      }
    }

    // Receiver holen
    {
      std::lock_guard<std::mutex> lk(receiver_mtx);
      job.receiver = current_receiver;
    }

    pick_running = true;
    pickRoutine(job);
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

  // Abonnieren des /current_state Topic zum Bewegen in die Beobachterposition zum aufnehmen der aktuellen Szene
  void stateCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string new_state = msg->data;

    // Flanke: irgendwas -> WAITING_FOR_SELECTION
    if (new_state == "WAITING_FOR_SELECTION" && last_sp_state != "WAITING_FOR_SELECTION")
    {
      if (pick_running)
      {
        ROS_WARN("[STATE] WAITING_FOR_SELECTION received but pick is running -> observer move skipped");
      }
      else
      {
        ROS_INFO("[STATE] -> WAITING_FOR_SELECTION: moving to observer position");
        moveToObserverPosition();
      }
    }

    last_sp_state = new_state;
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
