#include <panda_cli_controller/panda_controller.hpp>

#include <sstream>
#include <iostream>

/**
 * Zentrale Kommandoverarbeitung der CLI.
 *
 * Parst eine eingegebene Textzeile, erkennt den Befehl
 * und führt die zugehörige Roboter- oder Greiferaktion aus.
 * Während einer aktiven Pick-Routine werden Befehle (außer "stop")
 * blockiert.
 */
void PandaCliController::handleCommands(const std::string& line)
{
  std::istringstream iss(line);
  std::string cmd;
  iss >> cmd;

  if (cmd == "stop")
  {
    stopMovement();
    return;
  }

  if (pick_running_)
  {
    ROS_WARN("[CLI] Command ignored: pick routine already running");
    return;
  }

  if (cmd == "help")
  {
    printHelp();
    return;
  }

  if (cmd == "mode")
  {
    std::string m;
    if (!(iss >> m))
    {
      ROS_WARN("INPUT_ERROR: mode ptp|lin");
      return;
    }

    if (m == "ptp")
    {
      cli_mode_ = MotionMode::PTP;
      ROS_INFO("[CLI] Motion mode set to PTP");
    }
    else if (m == "lin")
    {
      cli_mode_ = MotionMode::LIN;
      ROS_INFO("[CLI] Motion mode set to LIN");
    }
    else
    {
      ROS_WARN("INPUT_ERROR: mode ptp|lin");
    }
    return;
  }

  if (cmd == "move_to")
  {
    double x, y, z;
    if (!(iss >> x >> y >> z))
    {
      ROS_WARN("INPUT_ERROR: move_to x y z");
      return;
    }

    if (cli_mode_ == MotionMode::PTP)
    { setPTP();}
    else
    { setLIN();}

    geometry_msgs::Pose target;
    target.position.x = x;
    target.position.y = y;
    target.position.z = z;
    target.orientation = arm_.getCurrentPose().pose.orientation;

    if (!moveArmToPose(target))
      ROS_WARN("Planning failed");
    return;
  }

  if (cmd == "set_orientation")
  {
    double qw, qx, qy, qz;
    if (!(iss >> qw >> qx >> qy >> qz))
    {
      ROS_WARN("INPUT_ERROR: set_orientation qw qx qy qz");
      return;
    }
    default_orientation_.w = qw;
    default_orientation_.x = qx;
    default_orientation_.y = qy;
    default_orientation_.z = qz;
    ROS_INFO("Default orientation set.");
    return;
  }

  if (cmd == "turn_hand")
  {
    double angle_deg;
    if (!(iss >> angle_deg))
    {
      ROS_WARN("INPUT_ERROR: turn_hand angle_deg");
      return;
    }
    setWristAngle(angle_deg);
    return;
  }

  if (cmd == "set_joints")
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
      joint_rad[i] = deg2rad(joint_deg[i]);

    arm_.setJointValueTarget(joint_rad);
    MoveGroupInterface::Plan plan;
    if (!planAndExecute(arm_, plan))
      ROS_WARN("Planning failed");
    return;
  }

  if (cmd == "print_pose")
  {
    printPose();
    return;
  }

  if (cmd == "print_joints")
  {
    printJoints();
    return;
  }

  if (cmd == "open")
  {
    openGripper();
    return;
  }

  if (cmd == "close")
  {
    closeGripper();
    return;
  }

  if (cmd == "close_w")
  {
    double width_mm;
    if (!(iss >> width_mm))
    {
      ROS_WARN("INPUT_ERROR: close_w width_mm");
      return;
    }

    if (width_mm <= 0.0)
    {
      ROS_WARN("INPUT_ERROR: close_w width_mm (must be > 0)");
      return;
    }

    closeGripperWithWidth(width_mm);
    return;
  }

  if (cmd == "observerPos")
  {
    moveToObserverPosition();
    return;
  }

  if (cmd == "placePos")
  {
    moveToPlacePosition();
    return;
  }

  if (cmd == "startPos")
  {
    moveToStartPosition();
    return;
  }

  if (cmd == "pick")
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
      got_angle = true;

    PickJob job;
    job.pos.x = x;
    job.pos.y = y;
    job.pos.z = grap_z_;

    if (got_angle)
    {
      job.tcp_yaw_deg = angle_deg;
      job.has_tcp_yaw = true;
    }

    pick_running_ = true;
    pickRoutine(job);
    return;
  }

  if (cmd == "quit")
  {
    ROS_INFO("Shutdown");
    ros::shutdown();
    return;
  }

  if (!cmd.empty())
    ROS_WARN("Unknown cmd '%s'. Type 'help'", cmd.c_str());
}

void PandaCliController::printHelp()
{
  ROS_INFO_STREAM(R"EOS(

============================= Panda CLI Controller Help =======================

  [General]
    help                        - Show this help
    quit                        - Shutdown node
    stop                        - Stop any current motion

  [Move]
    mode ptp|lin                - Select planner mode for move_to
    move_to x y z               - Move end-effector to (x, y, z) [m]
    set_joints j1 ... j7        - Set all 7 joints [deg]
    pick x y [angle_deg]        - Pick object at (x, y), optional yaw

  [TCP / Gripper]
    open                        - Open the panda hand
    close                       - Close the panda hand
    close_w width_mm            - Close the panda hand to specified width [mm]
    turn_hand angle             - Rotate wrist joint [deg]
    set_orientation qw qx qy qz - Set default end-effector orientation (quaternion)

  [Status]
    print_pose                  - Print current end-effector pose
    print_joints                - Print current joint angles [deg]

  [Predefined Poses]
    startPos                    - Move to predefined start pose
    observerPos                 - Move to predefined observer pose
    placePos                    - Move to predefined placement pose

===============================================================================

)EOS");
}
