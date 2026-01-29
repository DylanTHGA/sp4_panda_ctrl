#include <panda_cli_controller/panda_controller.hpp>

#include <sstream>
#include <cmath>

/**
 * Callback für /sweet_pose_translated.
 * Startet eine Pick-Routine anhand der übergebenen Zielpose,
 * sofern aktuell keine Pick-Routine aktiv ist.
 */
void PandaCliController::sp4PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Verhindert parallele Pick-Routinen
  if (pick_running_)
  {
    ROS_WARN("[SP4] Ignoring /sweet_pose_translated: pick routine already running");
    return;
  }

  PickJob job;

  // Zielposition übernehmen, Greifhöhe fest vorgeben
  job.pos = msg->pose.position;
  job.pos.z = grap_z_;

  // TCP-Yaw aus Quaternion berechnen
  const double angle_rad =
      std::atan2(msg->pose.orientation.z, msg->pose.orientation.w) * 2.0;
  job.tcp_yaw_deg = rad2deg(angle_rad);
  job.has_tcp_yaw = true;

  // Objektbreite übernehmen (falls vorhanden, einmalig)
  {
    std::lock_guard<std::mutex> lk(object_width_mtx_);
    if (has_object_width_)
    {
      job.width_mm = object_width_mm_;
      job.has_width = true;
      has_object_width_ = false;
    }
  }

  // Pick-Routine starten
  pick_running_ = true;
  pickRoutine(job);
}

/**
 * Callback für /robot/status.
 * Liest die erkannte Objektbreite und speichert sie für den nächsten Pick.
 */
void PandaCliController::robotStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  std::istringstream iss(msg->data);
  double width_mm = 0.0;

  // Breite aus Status-String parsen
  if (!(iss >> width_mm))
  {
    ROS_WARN("[SP4] /robot/status: konnte Breite nicht aus '%s' lesen",
             msg->data.c_str());
    return;
  }

  // Objektbreite thread-sicher ablegen
  {
    std::lock_guard<std::mutex> lk(object_width_mtx_);
    object_width_mm_ = width_mm;
    has_object_width_ = true;
  }

  ROS_INFO("[SP4] /robot/status: Breite aktualisiert auf %.1f mm", width_mm);
}

/**
 * Callback für /sp4/goalpose.
 * Aktualisiert die aktuell gewählte Ablage-Zielkennung:
 * 1 = Standard-Ablageposition, 2 = Jackal-Ablageposition
 */
void PandaCliController::goalPoseCallback(const std_msgs::Int32::ConstPtr& msg)
{
  const int id = msg->data;

  if (id != 1 && id != 2)
  {
    ROS_WARN("[SP4] /sp4/goalpose: invalid id %d (expected 1 or 2)", id);
    return;
  }

  {
    std::lock_guard<std::mutex> lk(goalpose_mtx_);
    current_goalpose_id_ = id;
  }

  ROS_INFO("[SP4] /sp4/goalpose: goalpose_id updated to %d", id);
}

/**
 * Callback für den SweetPicker-Zustand.
 * Bewegt den Roboter bei Eintritt in WAITING_FOR_SELECTION
 * in die Beobachterposition.
 */
void PandaCliController::stateCallback(const std_msgs::String::ConstPtr& msg)
{
  const std::string new_state = msg->data;

  // Flankenerkennung: Wechsel zu WAITING_FOR_SELECTION
  if (new_state == "WAITING_FOR_SELECTION" &&
      last_sp_state_ != "WAITING_FOR_SELECTION")
  {
    if (pick_running_)
    {
      ROS_WARN("[STATE] WAITING_FOR_SELECTION received but pick is running, observer move skipped");
    }
    else
    {
      ROS_INFO("[STATE] -> WAITING_FOR_SELECTION: moving to observer position");
      moveToObserverPosition();
    }
  }

  // Zustand merken
  last_sp_state_ = new_state;
}
