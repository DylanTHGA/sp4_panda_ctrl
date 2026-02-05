#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

static const double MM_TO_M = 1.0 / 1000.0;

class SP4CoordinateTranslator
{
public:
  SP4CoordinateTranslator()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    subscribed_topic_name_ = "/robot/pose";
    published_topic_name_  = "/sweet_pose_translated";

    pnh.param<std::string>("output_frame", output_frame_, std::string("panda_link0"));

    // Vision origin (0,0) in robot frame (panda_link0) approx (0.57, 0.185)
    pnh.param("origin_x_m", origin_x_m_, 0.57);
    pnh.param("origin_y_m", origin_y_m_, 0.185);

    sub_ = nh.subscribe(subscribed_topic_name_, 10, &SP4CoordinateTranslator::poseCallback, this);
    pub_ = nh.advertise<geometry_msgs::PoseStamped>(published_topic_name_, 10);

    ROS_INFO_STREAM("[coordinate_translator] sub=" << subscribed_topic_name_
                    << " pub=" << published_topic_name_
                    << " frame=" << output_frame_);
    ROS_INFO_STREAM("[coordinate_translator] vision origin (0,0) -> panda_link0 = ("
                    << origin_x_m_ << ", " << origin_y_m_ << ")");
    ROS_INFO_STREAM("[coordinate_translator] mapping: X = origin_x + y_mm/1000, Y = origin_y - x_mm/1000");
  }

private:
  ros::Subscriber sub_;
  ros::Publisher  pub_;
  std::string subscribed_topic_name_;
  std::string published_topic_name_;
  std::string output_frame_;

  double origin_x_m_{0.57};
  double origin_y_m_{0.185};

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    
    const double x_mm = msg->pose.position.x;   
    const double y_mm = msg->pose.position.y;  
    const double x_m = x_mm * MM_TO_M;
    const double y_m = y_mm * MM_TO_M;

    geometry_msgs::PoseStamped out;
    out.header = msg->header;
    out.header.frame_id = output_frame_;

    out.pose.position.x = origin_x_m_ + y_m;
    out.pose.position.y = origin_y_m_ - x_m;
    out.pose.position.z = 0.0;

    ROS_INFO_STREAM_THROTTLE(0.2,
      "[coordinate_translator] img(mm)=(" << x_mm << "," << y_mm << ")"
      << " -> panda_link0(m)=(" << out.pose.position.x << "," << out.pose.position.y << ",0)");

    pub_.publish(out);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coordinate_translator");
  SP4CoordinateTranslator translator;
  ros::spin();
  return 0;
}
