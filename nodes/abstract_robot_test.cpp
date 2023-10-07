#include <string>
#include <vector>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>

#include <urdf/model.h>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <shape_msgs/SolidPrimitive.h>

#include <robot_collision_checking/fcl_interface.h>

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
    ros::init(argc, argv, "abstract_robot_test");

    ros::NodeHandle nh; // Create a node handle and start the node

    // ros::Subscriber joint_sub = nh.subscribe("/joint_states", 1, &jointSensorCallback);

    std::string kdl_chain_filename, root, tip,joint_state_topic_name;
    std::vector<int> object_primitive;
    std::vector<std::vector<double>> obj_dimensions;
    std::vector<std::vector<double>> obj_poses;
    std::vector<shape_msgs::SolidPrimitive> shapes_in;
    constrained_manipulability::TransformVector shapes_pose;
    robot_collision_checking::FCLObjectSet objects;
    bool fetch_param_server,show_mp, show_cmp;


    constrained_manipulability::getParameter("~/fetch_param_server", fetch_param_server);
    constrained_manipulability::getParameter("~/kdl_chain_filename", kdl_chain_filename);
    constrained_manipulability::getParameter("~/root", root);
    constrained_manipulability::getParameter("~/tip", tip);
    constrained_manipulability::getParameter("~/joint_state_topic_name", joint_state_topic_name);
    constrained_manipulability::getParameter("~/show_mp", show_mp);
    constrained_manipulability::getParameter("~/show_cmp", show_cmp);
    constrained_manipulability::getVectorParam("~/object_primitive", object_primitive);
    constrained_manipulability::getVectorVectorParam("~/object_dimensions", obj_dimensions);
    constrained_manipulability::getVectorVectorParam("~/object_poses", obj_poses);
    constrained_manipulability::getCollisionShapes(object_primitive,
                                                   obj_dimensions,
                                                   obj_poses,
                                                   shapes_in,
                                                   shapes_pose);

    ros::Subscriber joint_sub = nh.subscribe(joint_state_topic_name, 1, &jointSensorCallback);

    constrained_manipulability::ConstrainedManipulability constrained_manip(nh,
                                                                            fetch_param_server,
                                                                            kdl_chain_filename,
                                                                            root, tip,
                                                                            joint_state_topic_name);

    bool use_static_functions(true);

    // TEST FOR STATIC FUNCTIONS
    KDL::Tree my_tree_;
    KDL::Chain chain_;
    urdf::Model model_;
    std::string robot_desc_string;

    if (use_static_functions)
    {
        if (fetch_param_server)
        {
        nh.param("robot_description", robot_desc_string, std::string());
        model_.initParamWithNodeHandle("robot_description", nh);
        if (!kdl_parser::treeFromString(robot_desc_string, my_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
        }
        else
        {
            ROS_INFO("Success");
        }

        }
        else
        {
            ROS_INFO("Loading URDF From File");
            if (!kdl_parser::treeFromFile(kdl_chain_filename, my_tree_))
            {
                ROS_ERROR("Failed to construct kdl tree");
            }
            else
            {
                ROS_INFO("Success");
            }
            
        }
        my_tree_.getChain(root,
                    tip,
                    chain_);
        
    }
    objects.resize(shapes_in.size());
    for (int i = 0; i < shapes_in.size(); ++i)
    {
        constrained_manip.addCollisionObject(shapes_in[i], shapes_pose[i], i);
        objects[i].object_shape = shapes_in[i];
        objects[i].object_transform = shapes_pose[i];
    }
    ros::Duration(2.0).sleep();
    constrained_manip.displayObjects();

    while (ros::ok())
    {
        if (joint_state_received == true)
        {
            joint_state_received = false;
            
            constrained_manip.checkCollision(joint_state);
            ROS_INFO("Trying here");
            constrained_manipulability::Polytope allowable_poly = constrained_manip.getAllowableMotionPolytope(
                joint_state,
                show_mp,
                {0.0, 0.0, 0.5, 0.0},
                {0.0, 0.0, 1.0, 0.4});
            
            constrained_manipulability::Polytope constrained_motion_poly = constrained_manip.getConstrainedAllowableMotionPolytope(
                joint_state,
                show_cmp,
                {0.0, 0.0, 0.5, 0.0},
                {1.0, 0.0, 0.0, 0.4});
            constrained_manipulability::Polytope vel_poly = constrained_manip.getVelocityPolytope(
                joint_state, false);
            constrained_manipulability::Polytope constrained_vel_poly = constrained_manip.getConstrainedVelocityPolytope(
                joint_state, false);

            if (use_static_functions)
            {

                Eigen::MatrixXd A;
                Eigen::VectorXd b;
                allowable_poly = constrained_manipulability::ConstrainedManipulability::
                    getAllowableMotionPolytope(chain_, model_, joint_state, A, b);
                constrained_motion_poly = constrained_manipulability::ConstrainedManipulability::
                    getConstrainedAllowableMotionPolytope(chain_, model_, objects, joint_state, A, b);
                vel_poly = constrained_manipulability::ConstrainedManipulability::
                    getVelocityPolytope(chain_, model_, joint_state, A, b);
                constrained_vel_poly = constrained_manipulability::ConstrainedManipulability::
                    getConstrainedVelocityPolytope(chain_, model_, objects, joint_state, A, b);
            }
        }

        ros::spinOnce();
        ros::Duration(0.0001).sleep();
    }
    return 0;
}
