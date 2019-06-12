/*
 * This node subscribes to the relevant odometry data and publishes the
 * associated dynamic tranforms.
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class Setup_tf
{
private:
    ros::NodeHandle nh;

    // This struct is used for storing position for transforms which maintain a
    // constant relative position to their parent frame
    struct Position {double x, y, z;};

    //===== Odom Filtered =====//
    ros::Subscriber odom_filtered_sub;
    tf::TransformBroadcaster base_footprint_broadcaster;
    tf::TransformBroadcaster base_link_broadcaster;
    tf::Transform odom_T_base_link;
    tf::Transform odom_T_base_footprint;
    tf::Transform base_footprint_T_base_link;

    //===== base_link from Encoders =====//
    ros::Subscriber enc_pose_sub;
    tf::TransformBroadcaster enc_base_link_broadcaster;

    //===== Cameras =====//
    ros::Subscriber camera_pose_sub;
    tf::TransformBroadcaster camera_base_link_broadcaster;

    // Ground clearance for base_footprint
    double ground_clearance;

public:
    Setup_tf()
    {
        odom_filtered_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Setup_tf::odom_filteredCallback, this);
        enc_pose_sub = nh.subscribe<nav_msgs::Odometry>("enc/odom", 10, &Setup_tf::enc_poseCallback, this);
        camera_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("camera_pose/pose", 10, &Setup_tf::camera_poseCallback, this);

        // Get ground clearance for base_footprint
        nh.param<double>("/forklift/ground_clearance", ground_clearance, 0.000);
    }

    void odom_filteredCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Get the transform from /odom to /base_link
        odom_T_base_link.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        odom_T_base_link.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

        // Get the transform from /base_footprint to /base_link
        base_footprint_T_base_link.setOrigin(tf::Vector3(0,0,ground_clearance));
        base_footprint_T_base_link.setRotation(tf::Quaternion(0,0,0,1));

        // Convert into transform from /odom to /base_footprint
        odom_T_base_footprint = odom_T_base_link * base_footprint_T_base_link.inverse();

        // Broadcast transforms for odom to base_footprint and base_footprint to /base_link
        // base_footprint_broadcaster.sendTransform(tf::StampedTransform(odom_T_base_footprint,
        //     ros::Time::now(),
        //     "odom",
        //     "base_footprint"
        // ));
        // base_link_broadcaster.sendTransform(tf::StampedTransform(base_footprint_T_base_link,
        //     ros::Time::now(),
        //     "base_footprint",
        //     "base_link"
        // ));
        base_link_broadcaster.sendTransform(tf::StampedTransform(odom_T_base_link,
            ros::Time::now(),
            "odom",
            "base_link"
        ));
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
            msg->header.frame_id
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
