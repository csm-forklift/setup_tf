#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class Setup_tf
{
private:
    ros::NodeHandle nh;
    ros::Subscriber odom_filtered_sub;
    tf::TransformBroadcaster base_link_broadcaster;
public:
    Setup_tf()
    {
        odom_filtered_sub = nh.subscribe<nav_msgs::Odometry>("odom/filtered", 10, &Setup_tf::odom_filteredCallback, this);
    }

    void odom_filteredCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Broadcast transform for odom to base_link
        base_link_broadcaster.sendTransform(tf::StampedTransform(
            tf::Transform(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w),
                          tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)),
            ros::Time::now(),
            "odom",
            "base_link"
        ));
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "setup_tf");
    Setup_tf setup_tf;
    ros::spin();

    return 0;
}
