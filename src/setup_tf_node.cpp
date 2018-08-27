#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

class Setup_tf
{
private:
    ros::NodeHandle nh;
    ros::Subscriber odom_filtered_sub;
    ros::Subscriber enc_pose_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber camera_pose_sub;
    tf::TransformBroadcaster base_link_broadcaster;
    tf::TransformBroadcaster enc_base_link_broadcaster;
    tf::TransformBroadcaster imu_base_link_broadcaster;
    struct Position {double x, y, z;}; // used at IMU position in transform since IMU only gives orientation
    Position imu_base_link_position;
    tf::TransformBroadcaster camera_base_link_broadcaster;
public:
    Setup_tf()
    {
        odom_filtered_sub = nh.subscribe<nav_msgs::Odometry>("odom/filtered", 10, &Setup_tf::odom_filteredCallback, this);
        enc_pose_sub = nh.subscribe<nav_msgs::Odometry>("enc/odom", 10, &Setup_tf::enc_poseCallback, this);
        imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data", 10, &Setup_tf::imuCallback, this);
        camera_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("camera_pose/pose", 10, &Setup_tf::camera_poseCallback, this);
        imu_base_link_position.x = 0;
        imu_base_link_position.y = 0;
        imu_base_link_position.z = 0;
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

        // Store base_link position as imu_base_link_position
        imu_base_link_position.x = msg->pose.pose.position.x;
        imu_base_link_position.y = msg->pose.pose.position.y;
        imu_base_link_position.z = msg->pose.pose.position.z;
    }

    void enc_poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Broadcast transform for odom to encoder's predicted base_link
        enc_base_link_broadcaster.sendTransform(tf::StampedTransform(
            tf::Transform(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w),
                          tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)),
            ros::Time::now(),
            "odom",
            "enc/base_link"
        ));
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        // Broadcast transform for odom to encoder's predicted base_link
        imu_base_link_broadcaster.sendTransform(tf::StampedTransform(
            tf::Transform(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w),
                          tf::Vector3(imu_base_link_position.x, imu_base_link_position.y, imu_base_link_position.z)),
            ros::Time::now(),
            "odom",
            "imu/base_link"
        ));
    }

    void camera_poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        // Broadcast transform for odom to camera's predicted base_link

        // // FIXME: Checking orientation
        // std::cout << "Orientation\n";
        // std::cout << "x: " << msg->pose.pose.orientation.x << "\n";
        // std::cout << "y: " << msg->pose.pose.orientation.y << "\n";
        // std::cout << "z: " << msg->pose.pose.orientation.z << "\n";
        // std::cout << "w: " << msg->pose.pose.orientation.w << std::endl;

        camera_base_link_broadcaster.sendTransform(tf::StampedTransform(
            tf::Transform(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w),
                          tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)),
            ros::Time::now(),
            "odom",
            "camera/base_link"
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
