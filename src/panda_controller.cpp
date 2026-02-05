#include <panda_cli_controller/panda_controller.hpp>

#include <vector>
#include <cmath>

double deg2rad(double deg) { return deg * (M_PI / 180.0); }

// Feste Gelenkposen für definierte Übergabepunkte (file-local)
static const std::vector<double> abovePlaceJoints =
{
  deg2rad(-90.0),  deg2rad(-10.0),  0.0,  deg2rad(-135.0),  0.0,  deg2rad(125.0),  deg2rad(45.0)
};
static const std::vector<double> abovePlaceJointsJackal =
{
  deg2rad(-115.0),  deg2rad(-10.0),  0.0,  deg2rad(-135.0),  0.0,  deg2rad(125.0),  deg2rad(45.0)
};

static const std::vector<double> carryJoints =
{
  deg2rad(-45.0),  deg2rad(-10.0),  0.0,  deg2rad(-135.0),  0.0,  deg2rad(125.0),  deg2rad(45.0)
};

static const std::vector<double> startJoints =
{
  0.0,  deg2rad(-45.0),  0.0,  deg2rad(-135.0),  0.0,  deg2rad(90.0),  deg2rad(45.0)
};

// Wenn diese Gelenkpossen angepasst werden, muss der Coordinate_Translator auch angerpasst werden!
// Die Markieungen auf dem Labortisch sind auf diese Pose kalibriert.
static const std::vector<double> observerJoints =
{
  0.0,  deg2rad(-20.0),  0.0,  deg2rad(-150.0),  0.0,  deg2rad(130.0),  deg2rad(45.0)
};


PandaCliController::PandaCliController()
  : nh_(),
    arm_("panda_arm"),
    hand_("panda_hand")
{
  arm_.setPoseReferenceFrame("panda_link0");
  // Expliziter TCP-Link für alle kartesischen Pose-Ziele
  arm_.setEndEffectorLink("panda_hand_tcp");

  // --- MoveIt Basis-Konfiguration (Pilz Planner, PTP als Standard) ---
  arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm_.setPlannerId("PTP");
  arm_.setPlanningTime(20.0); // Längere Planungszeit für komplexe Bewegungen

  // Konservatives Scaling für stabile, reproduzierbare Bewegungen
  arm_.setMaxVelocityScalingFactor(0.05);
  arm_.setMaxAccelerationScalingFactor(0.05);

  hand_.setMaxVelocityScalingFactor(0.1);
  hand_.setMaxAccelerationScalingFactor(0.1);

  // --- Initialwerte / Parameter laden ---
  initializeDefaultOrientation();

  // --- Startposition einnehmen (für definierte Ausgangslage) ---
  moveToStartPosition();
  //moveToObserverPosition();

  // --- Publisher/Subscriber (SP4 Schnittstellen + Zustands-Observer) ---
  feedback_pub_ = nh_.advertise<std_msgs::String>(feedback_topic_, 1, true);
  sub_pose_     = nh_.subscribe(sweet_pose_topic_,      1, &PandaCliController::sp4PoseCallback, this);
  sub_width_    = nh_.subscribe(sweet_width_topic_,     1, &PandaCliController::robotStatusCallback, this);
  sub_goalpose_ = nh_.subscribe(sweet_goalpose_topic_,  1, &PandaCliController::goalPoseCallback, this);
  sub_state_    = nh_.subscribe(state_topic_,           1, &PandaCliController::stateCallback, this);

  // CLI-Hilfe ausgeben
  printHelp();
}

void PandaCliController::initializeDefaultOrientation()
{
  // Default TCP-Orientierung (wird später ggf. durch aktuelle Pose überschrieben)
  default_orientation_.w = 0.0;
  default_orientation_.x = 1.0;
  default_orientation_.y = 0.0;
  default_orientation_.z = 0.0;
}

bool PandaCliController::planAndExecute(MoveGroupInterface& group,
                                        MoveGroupInterface::Plan& plan)
{
  const bool ok_plan =
    (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  const bool ok_exec = ok_plan && static_cast<bool>(group.execute(plan));

  if (!ok_exec)
    return false;

  return true;
}

void PandaCliController::setPTP()
{
  arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm_.setPlannerId("PTP");
}

void PandaCliController::setLIN()
{
  arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm_.setPlannerId("LIN");
}

bool PandaCliController::moveArmToPose(const geometry_msgs::Pose& pose)
{
  arm_.setStartStateToCurrentState();
  arm_.setPoseTarget(pose);

  MoveGroupInterface::Plan plan;
  return planAndExecute(arm_, plan);
}

void PandaCliController::openGripper()
{
  // Greifer in definierte "open"-Konfiguration fahren
  hand_.setNamedTarget("open");
  hand_.move();
}

void PandaCliController::closeGripper()
{
  // Greifer in definierte "close"-Konfiguration fahren
  hand_.setNamedTarget("close");
  hand_.move();
}

void PandaCliController::closeGripperWithWidth(double width_mm)
{
  // Ziel-Spaltmaß aus Objektbreite
  const double object_width_m = width_mm / 1000.0;
  const double target_gap = object_width_m;
  const double finger_joint = target_gap / 2.0;

  // Beide Finger symmetrisch setzen
  std::vector<double> joints;
  joints.push_back(finger_joint);
  joints.push_back(finger_joint);

  hand_.setJointValueTarget(joints);
  hand_.move();

  ROS_INFO("[SP4] closeGripperWithWidth: width=%.1f mm -> gap=%.2f mm, joint=%.3f rad",
           width_mm, target_gap * 1000.0, finger_joint);
}

void PandaCliController::stopMovement()
{
  // Sofortiges Stoppen laufender Bewegungen (Arm + Hand)
  arm_.stop();
  hand_.stop();
  ROS_INFO("Stopped");
}

void PandaCliController::moveToObserverPosition()
{
  arm_.setStartStateToCurrentState();
  arm_.setJointValueTarget(observerJoints);

  MoveGroupInterface::Plan plan;
  if (!planAndExecute(arm_, plan))
    ROS_WARN("Failed moving to observer Position");
}

void PandaCliController::moveToStartPosition()
{
  arm_.setStartStateToCurrentState();
  arm_.setJointValueTarget(startJoints);

  MoveGroupInterface::Plan plan;
  if (!planAndExecute(arm_, plan))
    ROS_WARN("Failed moving to start Position");
}

void PandaCliController::moveToPlacePosition()
{
  int goalpose_id = 1;
  {
    std::lock_guard<std::mutex> lk(goalpose_mtx_);
    goalpose_id = current_goalpose_id_;
  }

  setPTP();
  if (!moveToAbovePlaceJoints(goalpose_id))
  {
    ROS_WARN("Failed moving to abovePlaceJoints");
    return;
  }

  geometry_msgs::Quaternion place_ori = arm_.getCurrentPose().pose.orientation;
  geometry_msgs::Pose above_place_pose = makePlacePose(goalpose_id, true);
  geometry_msgs::Pose place_pose       = makePlacePose(goalpose_id, false);

  above_place_pose.orientation = place_ori;
  place_pose.orientation       = place_ori;

  setLIN();
  if (!moveArmToPose(place_pose))
  {
    ROS_WARN("Failed moving LIN down to place");
    return;
  }

}

bool PandaCliController::moveToAbovePlaceJoints(int goalpose_id)
{
  setPTP();
  arm_.setStartStateToCurrentState();

  const bool is_jackal = (goalpose_id == 2);
  arm_.setJointValueTarget(is_jackal ? abovePlaceJointsJackal : abovePlaceJoints);

  MoveGroupInterface::Plan plan;
  return planAndExecute(arm_, plan);
}

bool PandaCliController::moveToCarryJoints()
{
  setPTP();
  arm_.setStartStateToCurrentState();
  arm_.setJointValueTarget(carryJoints);

  MoveGroupInterface::Plan plan;
  return planAndExecute(arm_, plan);
}

geometry_msgs::Pose PandaCliController::makePlacePose(int goalpose_id, bool above) const
{
  // GoalPose-abhängige Ablagepose (Position), z je nach "above"/"place"
  geometry_msgs::Pose pose;
  pose.orientation = default_orientation_;

  const bool is_jackal = (goalpose_id == 2);

  if (is_jackal)
  {
    // TODO: Werte für die spätere Jackal-Ablage anpassen
    pose.position.x = -0.3;
    pose.position.y = -0.6;
  }
  else
  {
    pose.position.x = 0.0;
    pose.position.y = -0.6;
  }

  pose.position.z = above ? hover_z_ : grap_z_;
  return pose;
}

bool PandaCliController::setWristAngle(double angle_deg)
{
  // Aktuelle Gelenkwerte lesen und nur Joint7 (Handgelenk-Yaw) anpassen
  auto st = arm_.getCurrentState();
  const auto* jmg = st->getJointModelGroup("panda_arm");
  std::vector<double> joints;
  st->copyJointGroupPositions(jmg, joints);

  // Mapping/Clamp: gewünschter Winkel in zulässigen Bereich bringen
  double target_deg = angle_deg;
  if (target_deg < 0.0)
    target_deg += 180.0;

  if (target_deg < 0.0)   target_deg = 0.0;
  if (target_deg > 166.0) target_deg = 166.0;

  joints[6] = deg2rad(target_deg);

  arm_.setStartStateToCurrentState();
  arm_.setJointValueTarget(joints);

  MoveGroupInterface::Plan plan;
  if (!planAndExecute(arm_, plan))
  {
    ROS_WARN("Failed setting wrist yaw");
    return false;
  }

  // Logging: tatsächlichen Joint7-Wert nach Ausführung ausgeben
  auto st_after = arm_.getCurrentState();
  std::vector<double> j_after;
  st_after->copyJointGroupPositions(st_after->getJointModelGroup("panda_arm"), j_after);
  return true;
}

void PandaCliController::printPose()
{
  // Aktuelle TCP-Pose ausgeben (Debug)
  auto ps = arm_.getCurrentPose();
  const auto& p = ps.pose.position;
  const auto& o = ps.pose.orientation;
  const double z_table = p.z; // Bei der Simulation muss die Tischhöhe abgezogen werden (0.79 m)
  ROS_INFO("Pose: x=%.3f y=%.3f z=%.3f", p.x, p.y, z_table);
  ROS_INFO("Ori:  w=%.3f x=%.3f y=%.3f z=%.3f", o.w, o.x, o.y, o.z);
}

void PandaCliController::printJoints()
{
  // Aktuelle Gelenkwinkel ausgeben (Debug)
  auto st = arm_.getCurrentState();
  const auto* jmg = st->getJointModelGroup("panda_arm");
  std::vector<double> joints;
  st->copyJointGroupPositions(jmg, joints);

  for (std::size_t i = 0; i < joints.size(); ++i)
    ROS_INFO("Joint %zu: %.1f deg", i + 1, rad2deg(joints[i]));
}

void PandaCliController::publishFeedback()
{
  // Status/Feedback an das SweetPicker-System publizieren
  std_msgs::String msg;
  msg.data = "Süßigkeit dargereicht";
  feedback_pub_.publish(msg);
}

void PandaCliController::pickRoutine(const PickJob& object_data)
{
  pick_running_ = true;

  int goalpose_id = 1;
  {
    std::lock_guard<std::mutex> lk(goalpose_mtx_);
    goalpose_id = current_goalpose_id_;
  }

  const double x = object_data.pos.x;
  const double y = object_data.pos.y;

  geometry_msgs::Pose above_pose;
  above_pose.position.x = x;
  above_pose.position.y = y;
  above_pose.position.z = hover_z_;
  above_pose.orientation = default_orientation_;

  geometry_msgs::Pose grasp_pose = above_pose;
  grasp_pose.position.z = grap_z_;

  geometry_msgs::Pose above_place_pose = makePlacePose(goalpose_id, true);
  geometry_msgs::Pose place_pose       = makePlacePose(goalpose_id, false);

  // 1) Über Objekt (PTP)
  setPTP();
  if (!moveArmToPose(above_pose))
  {
    ROS_WARN("[auto_pick] Failed PTP above target");
    pick_running_ = false;
    return;
  }

  // Optional: TCP-Yaw
  if (object_data.has_tcp_yaw)
  {
    if (!setWristAngle(object_data.tcp_yaw_deg))
      ROS_WARN("[auto_pick] Failed to apply TCP yaw");
  }

  openGripper();

  geometry_msgs::Quaternion c_orientation = arm_.getCurrentPose().pose.orientation;
  above_pose.orientation = c_orientation;
  grasp_pose.orientation = c_orientation;

  // 2) Runter zum Objekt (LIN)
  setLIN();
  if (!moveArmToPose(grasp_pose))
  {
    ROS_WARN("[auto_pick] Failed LIN down to grasp");
    pick_running_ = false;
    return;
  }

  if (object_data.has_width)
    closeGripperWithWidth(object_data.width_mm);
  else
  closeGripperWithWidth(22.0); // Milyway und Snicker 22 Maoam 23

  // 3) Hoch (LIN)
  if (!moveArmToPose(above_pose))
  {
    ROS_WARN("[auto_pick] Failed LIN up to above");
    pick_running_ = false;
    return;
  }

  // 4) Transport: nur über feste Joint-Stützpunkte zur Ablage
  setPTP();

  if (!moveToCarryJoints())
  {
    ROS_WARN("[auto_pick] Failed PTP to carryJoints");
    pick_running_ = false;
    return;
  }

  if (!moveToAbovePlaceJoints(goalpose_id))
  {
    ROS_WARN("[auto_pick] Failed PTP to abovePlaceJoints");
    pick_running_ = false;
    return;
  }

  // Orientierung aus der tatsächlich erreichten AbovePlace-Pose übernehmen
  geometry_msgs::Quaternion place_ori = arm_.getCurrentPose().pose.orientation;
  above_place_pose.orientation = place_ori;
  place_pose.orientation       = place_ori;

  // 5) LIN runter zur Ablage
  setLIN();
  if (!moveArmToPose(place_pose))
  {
    ROS_WARN("[auto_pick] Failed LIN down to place");
    pick_running_ = false;
    return;
  }

  openGripper();

  // 6) LIN hoch zurück auf "über Ablage" (Pose)
  if (!moveArmToPose(above_place_pose))
  {
    ROS_WARN("[auto_pick] Failed LIN up from place");
    pick_running_ = false;
    return;
  }

  // 7) Rückweg: wieder über feste Joints (PTP)
  (void)moveToAbovePlaceJoints(goalpose_id);
  (void)moveToCarryJoints();

  moveToObserverPosition();

  publishFeedback();
  pick_running_ = false;
}
