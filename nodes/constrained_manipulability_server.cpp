#include <random>
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <constrained_manipulability/constrained_manipulability.hpp>

sensor_msgs::JointState joint_state;
bool joint_state_received(false);

void jointSensorCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_state = *msg;
    joint_state_received = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "constrained_manipulability_server",ros::init_options::AnonymousName);
    std::srand(std::time(nullptr));

    ros::NodeHandle nh; // Create a node handle and start the node

    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 10, &jointSensorCallback);

    std::string kdl_chain_filename, joint_state_topic_name, root, tip, robot_desc;
    bool fetch_param_server;
    std::string robot_namespace_suffix;

    constrained_manipulability::getParameter("~/robot_namespace_suffix", robot_namespace_suffix);
    constrained_manipulability::getParameter("~/fetch_param_server", fetch_param_server);
    constrained_manipulability::getParameter("~/kdl_chain_filename", kdl_chain_filename);
    constrained_manipulability::getParameter("~/root", root);
    constrained_manipulability::getParameter("~/tip", tip);
    constrained_manipulability::getParameter("~/joint_state_topic_name", joint_state_topic_name);
    constrained_manipulability::getParameter("~/robot_desc", robot_desc);

    constrained_manipulability::ConstrainedManipulability constrained_manip(nh,
                                                                        robot_namespace_suffix,
                                                                        fetch_param_server,
                                                                        kdl_chain_filename,
                                                                        root, tip,
                                                                        joint_state_topic_name,robot_desc);

    // Loop with 100 Hz rate
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        if (joint_state_received == true)
        {
            joint_state_received = false;

            ROS_ERROR_COND(constrained_manip.checkCollision(joint_state), "Robot in collision");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
