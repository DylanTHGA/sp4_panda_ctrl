#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

static const double MM_TO_M = 1.0 / 1000.0;

// Bildbreite = 295 mm (0.295 m), Bildhöhe = 220 mm (0.220 m)
// Bildmitte soll bei (0.4, 0.0) im Panda-Frame liegen.
// -> X_LEFT = 0.4 - 0.295/2 = 0.2525
// -> Y_TOP  = 0.0 + 0.220/2 = 0.11
static const double X_LEFT_ROBOT = 0.2525;
static const double Y_TOP_ROBOT  = 0.11;

class SP4CoordinateTranslator
{
public:
    SP4CoordinateTranslator()
    {
        ros::NodeHandle nh;

        subscribed_topic_name_ = "/robot/pose";
        // panda_cli_control erwartet PoseStamped auf diesem Topic:
        published_topic_name_  = "/sweet_pose_translated";

        sub_ = nh.subscribe(subscribed_topic_name_, 10,
                            &SP4CoordinateTranslator::poseCallback, this);

        pub_ = nh.advertise<geometry_msgs::PoseStamped>(published_topic_name_, 10);

        ROS_INFO_STREAM(ros::this_node::getName()
                        << ": listening on " << subscribed_topic_name_
                        << ", publishing on " << published_topic_name_
                        << ", top-left at ("
                        << X_LEFT_ROBOT << ", " << Y_TOP_ROBOT << ") in robot frame");
    }

private:
    ros::Subscriber sub_;
    ros::Publisher  pub_;
    std::string subscribed_topic_name_;
    std::string published_topic_name_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // Bildverarbeitung liefert: x,y in mm relativ zur oberen linken Bildecke
        double x_mm = msg->pose.position.x;
        double y_mm = msg->pose.position.y;

        double x_m = x_mm * MM_TO_M;
        double y_m = y_mm * MM_TO_M;

        geometry_msgs::PoseStamped out;

        // Header (Zeitstempel etc.) übernehmen
        out.header = msg->header;

        // Position: Top-Left -> Panda-Koordinaten
        out.pose.position.x = X_LEFT_ROBOT + x_m;
        out.pose.position.y = Y_TOP_ROBOT  - y_m;
        out.pose.position.z = 0.0;

        // Orientierung unverändert übernehmen
        out.pose.orientation = msg->pose.orientation;

        ROS_INFO_STREAM(ros::this_node::getName()
                        << ": img(mm)=(" << x_mm << ", " << y_mm << ")"
                        << " -> robot(m)=(" << out.pose.position.x
                        << ", " << out.pose.position.y
                        << ", " << out.pose.position.z << ")");

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
