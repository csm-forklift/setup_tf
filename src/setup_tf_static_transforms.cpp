/*
    This node handles publishing static transforms. ROS's 'tf' package has a
    node that can be run in launch files called 'static_transform_publisher'
    which this code replaces for our system. This way we can load parameters
    using '.yaml' files and get the transform information from there. The
    default period suggested by the 'static_transform_publisher' documentation
    is 100ms, so a rate value of 10Hz should be sufficient for this node.

    *** Adding Transforms ***
    Here are instructions on adding additional transforms.

    1) Add the transform parameters to a '.yaml' file
    First create the transform in a '.yaml' file. It can be already created or a
    new one. It is best to keep common sensors/parts together, e.g. the
    'camera3' frame transform parameters should be placed in the 'cameras.yaml'
    file with 'camera1' and 'camera2' etc. The format should follow the template
    below (all values should be in SI units):

    [frame_name]:
        pose:
            x: 0.0
            y: 0.0
            z: 0.0
            qx: 0.0
            qy: 0.0
            qz: 0.0
            qw: 1.0
        frames:
            frame_id: "[parent_frame_name]"
            child_frame_id: "[child_frame name]"

    i.e.
    camera3:
        pose:
            x: 0.200
            y: -0.015
            z: 0.045
            qx: 0.0
            qy: 0.0
            qz: -0.7071068
            qw: 0.7071068
        frames:
            frame_id: "base_link"
            child_frame_id: "camera3_link"

    2) Edit this code to include your new transform
    Once the yaml parameters are created you can now add the frame name and file
    name to this code and it will look for the transform. There are two sections
    within this code that require alterations. Each is marked within headers
    denoting "OPERATOR TRANSFORM DEFINITIONS (#)".
    In the section marked with "(1)", you must create a vector of strings to
    hold your frame name and file name. This is done as follows:

    std::vector<std::string> [frame_name]_transform;

    i.e.
    std::vector<std::string camera1_transform;

    Next in the section marked with "(2)", you must add the frame name and file
    name to that vector and then add that vector to the catalog vector. This is
    done as follows:

    [frame_name]_transform.push_back("[frame_name]");
    [frame_name]_transform.push_back("[file_name]");
    frame_catalog.push_back([frame_name]_transform);

    i.e.
    camera1_transform.push_back("camera1");
    camera1_transform.push_back("cameras.yaml");
    frame_catalog.push_back(camera1_transform);

    Finally, recompile the 'setup_tf' package using catkin so the changes take
    affect.
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>
class StaticTransforms
{
private:
    ros::NodeHandle nh_;

    // This is the only variable the operator needs to change. When adding a new
    // sensor, add the frame name to this vector along with the filename for
    // where the transform parameters are located.
    // frame_catalog = [frame_name[], file[]]
    std::vector<std::vector <std::string> > frame_catalog;
    // Parameters that define the transform
    struct Parameters {
        double x, y, z, qx, qy, qz, qw;
        std::string frame_id, child_frame_id;
    };
    // Set of variables needed for publishing a transform
    struct Transform {
        std::string frame_name;
        Parameters parameters;
        tf::TransformBroadcaster broadcaster;
    };
    std::vector<Transform> transforms;

    // Default parameters for when no value is provided from ROS param
    Parameters default_parameters;

    //========================================================================//
    //******************** OPERATOR TRANSFORM DEFINITIONS (1) ****************//
    //========================================================================//

    // List transforms needing to be published
    std::vector<std::string> odom_transform;
    std::vector<std::string> front_axel_middle_transform;
    std::vector<std::string> wheel_left_transform;
    std::vector<std::string> wheel_right_transform;
    std::vector<std::string> laser_scanner1_transform;
    std::vector<std::string> imu_transform;
    std::vector<std::string> camera1_transform;
    std::vector<std::string> marker1_transform;

    //========================================================================//
    //******************** OPERATOR TRANSFORM DEFINITIONS (1) ****************//
    //========================================================================//

public:
    StaticTransforms() : nh_("~")
    {
        // Set default parameter values
        default_parameters.x = 0;
        default_parameters.y = 0;
        default_parameters.z = 0;
        default_parameters.qx = 0;
        default_parameters.qy = 0;
        default_parameters.qz = 0;
        default_parameters.qw = 1;
        default_parameters.frame_id = "base_link";
        default_parameters.child_frame_id = "default_link";

        //====================================================================//
        //****************** OPERATOR TRANSFORM DEFINITIONS (2) **************//
        //====================================================================//

        // Add frame names and file names to catalog
        //====================================================================//
        // World Frames
        //====================================================================//
        //===== World to Odom (world) =====//
        /*
           this transform allows the forklift to turn off in a pose that is not
           at the origin of the world frame and then start up again with odom
           set to that pose
        */
        odom_transform.push_back("odom");
        odom_transform.push_back("world.yaml");
        frame_catalog.push_back(odom_transform);
        //===== World to Marker1 =====//
        marker1_transform.push_back("marker1");
        marker1_transform.push_back("world.yaml");
        frame_catalog.push_back(marker1_transform);

        //====================================================================//
        // Forklift Frames
        //====================================================================//
        //===== Front axel middle (base_link) =====//
        front_axel_middle_transform.push_back("front_axel_middle");
        front_axel_middle_transform.push_back("forklift.yaml");
        frame_catalog.push_back(front_axel_middle_transform);

        // FIXME: The wheel transforms should actually be dynamic transforms with
        // orientation provided by encoder node
        //===== Wheel right (base_link) =====//
        wheel_right_transform.push_back("wheel_right");
        wheel_right_transform.push_back("forklift.yaml");
        frame_catalog.push_back(wheel_right_transform);
        //===== Wheel left (base_link) =====//
        wheel_left_transform.push_back("wheel_left");
        wheel_left_transform.push_back("forklift.yaml");
        frame_catalog.push_back(wheel_left_transform);

        //====================================================================//
        // Sensor Frames
        //====================================================================//
        //===== IMU (base_link) =====//
        imu_transform.push_back("imu");
        imu_transform.push_back("imu.yaml");
        frame_catalog.push_back(imu_transform);
        //===== Laser Scanners (base_link) =====//
        laser_scanner1_transform.push_back("laser_scanner1");
        laser_scanner1_transform.push_back("laser_scanners.yaml");
        frame_catalog.push_back(laser_scanner1_transform);
        //===== Cameras (base_link) =====//
        camera1_transform.push_back("camera1");
        camera1_transform.push_back("cameras.yaml");
        frame_catalog.push_back(camera1_transform);

        //====================================================================//
        //****************** OPERATOR TRANSFORM DEFINITIONS (2) **************//
        //====================================================================//

        //===== Generate Transforms for Publishing =====//
        for (int i = 0; i < frame_catalog.size(); ++i) {
            transforms.push_back(generateTransform(frame_catalog[i][0], frame_catalog[i][1]));
        }
    }

    const Transform generateTransform(std::string frame_name, std::string file)
    {
        // Print debugging information
        ROS_INFO("Expecting %s transform in file: %s", frame_name.c_str(), file.c_str());

        Transform transform;
        transform.frame_name = frame_name;
        transform.broadcaster = tf::TransformBroadcaster();

        // Get parameters from loaded yaml file, use defaults if load fails
        std::map<std::string, double> pose_map;
        std::map<std::string, std::string> frame_map;
        if (!nh_.getParam((frame_name + "/pose").c_str(), pose_map)) {
            transform.parameters = default_parameters;
            ROS_INFO("Could not load pose info for %s transform, using defaults:\nposition: [%0.0f, %0.0f, %0.0f]\norientation: [%0.0f, %0.0f, %0.0f, %0.0f]",
                     frame_name.c_str(), transform.parameters.x, transform.parameters.y, transform.parameters.z,
                     transform.parameters.qx, transform.parameters.qy, transform.parameters.qz, transform.parameters.qw);
        }
        else {
            transform.parameters.x = pose_map["x"];
            transform.parameters.y = pose_map["y"];
            transform.parameters.z = pose_map["z"];
            transform.parameters.qx = pose_map["qx"];
            transform.parameters.qy = pose_map["qy"];
            transform.parameters.qz = pose_map["qz"];
            transform.parameters.qw = pose_map["qw"];
        }
        if (!nh_.getParam((frame_name + "/frames").c_str(), frame_map)) {
            transform.parameters.frame_id = default_parameters.frame_id;
            transform.parameters.child_frame_id = frame_name + "_link";
            ROS_INFO("Could not load frame info for %s transform, using defaults:\nframe_id: %s\nchild_frame:id: %s",
                     frame_name.c_str(), transform.parameters.frame_id.c_str(), transform.parameters.child_frame_id.c_str());
        }
        else {
            transform.parameters.frame_id = frame_map["frame_id"];
            transform.parameters.child_frame_id = frame_map["child_frame_id"];
        }

        return transform;
    }

    void sendStaticTransform(tf::TransformBroadcaster &broadcaster, Parameters &parameters)
    {
        broadcaster.sendTransform(tf::StampedTransform(
            tf::Transform(tf::Quaternion(parameters.qx, parameters.qy, parameters.qz, parameters.qw),
                          tf::Vector3(parameters.x, parameters.y, parameters.z)),
            ros::Time::now(),
            parameters.frame_id,
            parameters.child_frame_id
        ));
    }

    void sendAllTransforms()
    {
        for (int i = 0; i < transforms.size(); ++i) {
            sendStaticTransform(transforms[i].broadcaster, transforms[i].parameters);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "setup_tf_static_transforms");
    StaticTransforms transforms;

    ros::Rate rate(10); // publish at 10Hz = 100ms period
    while (ros::ok()) {
        transforms.sendAllTransforms();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
