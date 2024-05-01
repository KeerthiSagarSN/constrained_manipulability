#include <string>
#include <vector>
#include <boost/bind.hpp>
// #include <boost/scoped_ptr.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>

#include <urdf/model.h>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <shape_msgs/SolidPrimitive.h>

#include <robot_collision_checking/fcl_interface.h>

#include <constrained_manipulability/constrained_manipulability.hpp>
#include <std_msgs/Float64MultiArray.h>
// #include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <mutex>

sensor_msgs::JointState joint_state;
constrained_manipulability::PolytopeHyperplanes PH_allowable0;
constrained_manipulability::PolytopeHyperplanes PH_constrained0;
constrained_manipulability::PolytopeHyperplanes PH_allowable1;
constrained_manipulability::PolytopeHyperplanes PH_constrained1;
constrained_manipulability::PolytopeHyperplanes jacobian_offset_int0;
constrained_manipulability::PolytopeHyperplanes jacobian_offset_int1;
Eigen::MatrixXd AHrep_MP0;
Eigen::MatrixXd AHrep_MP1;
Eigen::VectorXd bHrep_MP0;
Eigen::VectorXd bHrep_MP1;



Eigen::MatrixXd AHrep_CMP0;
Eigen::MatrixXd AHrep_CMP1;
Eigen::VectorXd bHrep_CMP0;
Eigen::VectorXd bHrep_CMP1;

Eigen::MatrixXd AHrep_MPout;
Eigen::MatrixXd AHrep_CMPout;





Eigen::Vector3d offset_position0;
Eigen::Vector3d offset_position1;
// Eigen::VectorXd bHrep_CMPout;
std::vector<double> color_pts0 = {0.0, 0.0, 0.5, 0.0};
std::vector<double> color_line0 = {1.0, 0.0, 0.0, 0.4};

std::vector<double> color_pts1 = {0.0, 0.5, 0.0, 1.0};
std::vector<double> color_line1 = {0.0, 1.0, 0.0, 0.8};

bool joint_state_received(true);
bool marker_success(false);
bool MP0_received;
bool MP1_received;

bool CMP0_received;
bool CMP1_received;

bool JMP0_received;
bool JMP1_received;
// Eigen::Matrix<double, 3, Eigen::Dynamic> base_J_ee_int;
// Eigen::Affine3d base_T_ee_int;
Eigen::MatrixXd base_J_ee_int;
Eigen::MatrixXd base_J_ee_int0;
Eigen::MatrixXd base_J_ee_int1;
std::mutex mtx; // Create a mutex

KDL::Tree my_tree;

KDL::Chain chain;
urdf::Model model;
// boost::scoped_ptr<KDL::ChainJntToJacSolver> kdl_dfk_solver;
// boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver;


std::shared_ptr<KDL::ChainJntToJacSolver> kdl_dfk_solver;
std::shared_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver;
Eigen::Affine3d base_T_ee;
// Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;



void jointSensorCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_state = *msg;
    joint_state_received = true;
}

Eigen::MatrixXd float64MultiArrayToEigenMatrix(const std_msgs::Float64MultiArray &msg) {
    int rows = msg.layout.dim[0].size;
    int cols = msg.layout.dim[1].size;
    
    Eigen::MatrixXd matrix(rows, cols);
    
    if (msg.data.size() != rows * cols) {
        ROS_ERROR("Invalid size of Float64MultiArray data.");
        return matrix; // Return an empty matrix or handle the error as needed.
    }
    
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            matrix(i, j) = msg.data[i * cols + j];
        }
    }
    
    return matrix;
}


// Eigen::Matrix<double, 6, Eigen::Dynamic> float64MultiArrayToEigenMatrixDynamic(const std_msgs::Float64MultiArray &msg) {
//     int rows = msg.layout.dim[0].size;
//     int cols = msg.layout.dim[1].size;
    
//     Eigen::Matrix<double, 6, Eigen::Dynamic> matrix(rows, cols);
    
//     if (msg.data.size() != rows * cols) {
//         ROS_ERROR("Invalid size of Float64MultiArray data.");
//         return matrix; // Return an empty matrix or handle the error as needed.
//     }
    
//     for (int i = 0; i < rows; i++) {
//         for (int j = 0; j < cols; j++) {
//             matrix(i, j) = msg.data[i * cols + j];
//         }
//     }
    
//     return matrix;
// }
Eigen::VectorXd float64ArrayToEigenVector(const std::vector<double> &data) {
    Eigen::VectorXd eigenVector(data.size());
    
    for (size_t i = 0; i < data.size(); ++i) {
        eigenVector(i) = data[i];
    }

    return eigenVector;
}


void allowableMPCallback0(const constrained_manipulability::PolytopeHyperplanes::ConstPtr &msg1)
{
    

    PH_allowable0 = *msg1;
    ROS_INFO("INside allowableMP0");
    MP0_received = true;
    AHrep_MP0 = float64MultiArrayToEigenMatrix(PH_allowable0.A);
    bHrep_MP0 = float64ArrayToEigenVector(PH_allowable0.b);


    

}

void allowableMPCallback1(const constrained_manipulability::PolytopeHyperplanes::ConstPtr &msg2)
{
    

    PH_allowable1 = *msg2;
    ROS_INFO("INside allowableMP1");
    MP1_received = true;
    AHrep_MP1 = float64MultiArrayToEigenMatrix(PH_allowable1.A);
    bHrep_MP1 = float64ArrayToEigenVector(PH_allowable1.b);
    

}


void CMPCallback0(const constrained_manipulability::PolytopeHyperplanes::ConstPtr &msg3)
{
    

    PH_constrained0 = *msg3;
    // std::cout<<"CMP0"<<PH_constrained0<<std::endl;
    ROS_INFO("INside CMP0");
    CMP0_received = true;
    AHrep_CMP0 = float64MultiArrayToEigenMatrix(PH_constrained0.A);
    // std::cout<<"AHrep_CMP0"<<AHrep_CMP0<<std::endl;
    bHrep_CMP0 = float64ArrayToEigenVector(PH_constrained0.b);



}

void CMPCallback1(const constrained_manipulability::PolytopeHyperplanes::ConstPtr &msg4)
{
    PH_constrained1 = *msg4;
    // ROS_INFO("INside CMP1");
    CMP1_received = true;
    AHrep_CMP1 = float64MultiArrayToEigenMatrix(PH_constrained1.A);
    bHrep_CMP1 = float64ArrayToEigenVector(PH_constrained1.b);

}



void jacobian_offsetCallback0(const constrained_manipulability::PolytopeHyperplanes::ConstPtr &msg5)
{
    jacobian_offset_int0 = *msg5;
    ROS_INFO("INside JOF0");
    JMP0_received = true;
    // base_J_ee_intXD0 = float64MultiArrayToEigenMatrix(jacobian_offset_int0.A)
    base_J_ee_int0 = float64MultiArrayToEigenMatrix(jacobian_offset_int0.A);
    offset_position0 = float64ArrayToEigenVector(jacobian_offset_int0.b);

}


void jacobian_offsetCallback1(const constrained_manipulability::PolytopeHyperplanes::ConstPtr &msg6)
{
    jacobian_offset_int1 = *msg6;
    ROS_INFO("INside JOF1");
    JMP1_received = true;
    base_J_ee_int1 = float64MultiArrayToEigenMatrix(jacobian_offset_int1.A);
    offset_position1 = float64ArrayToEigenVector(jacobian_offset_int1.b);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intersect_polytope_test");

    ros::NodeHandle nh; // Create a node handle and start the node

    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 1, &jointSensorCallback);
    //ros::Subscriber 

    // std::string joint_state_topic_name2;

    bool show_mp, show_cmp;

    //constrained_manipulability::getParameter("~/robot_namespace_suffix", robot_namespace_suffix);

    // Create multiple subscribers for different topics, each with a different extra argument
    std::string allowableMP0 = "constrained_manipulability/PolytopeHyperplaneAllowable0";
    std::string allowableMP1 =  "constrained_manipulability/PolytopeHyperplaneAllowable1";

    std::string constraintMP0 = "constrained_manipulability/PolytopeHyperplaneConstraint0";
    std::string constraintMP1 =  "constrained_manipulability/PolytopeHyperplaneConstraint1";

    std::string jacobian_offset0 = "constrained_manipulability/jacobian_offset0";
    std::string jacobian_offset1 = "constrained_manipulability/jacobian_offset1";


    ros::Subscriber sub1 = nh.subscribe(allowableMP0,1,&allowableMPCallback0);
    ros::Subscriber sub2 = nh.subscribe(allowableMP1,1,&allowableMPCallback1);
    ros::Subscriber sub3 = nh.subscribe(constraintMP0,1,&CMPCallback0);
    ros::Subscriber sub4 = nh.subscribe(constraintMP1,1,&CMPCallback1);


    ros::Subscriber sub5 = nh.subscribe(jacobian_offset0,1,&jacobian_offsetCallback0);
    ros::Subscriber sub6 = nh.subscribe(jacobian_offset1,1,&jacobian_offsetCallback1);

    ros::Publisher mkr_intersection_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker_intersection", 1);

    

    // constrained_manipulability::getParameter("~/fetch_param_server", fetch_param_server);
    // constrained_manipulability::getParameter("~/kdl_chain_filename", kdl_chain_filename);
    // constrained_manipulability::getParameter("~/root", root);
    // constrained_manipulability::getParameter("~/tip", tip);
    // constrained_manipulability::getParameter("~/joint_state_topic_name2", joint_state_topic_name2);
    // constrained_manipulability::getParameter("~/show_mp", show_mp);
    // constrained_manipulability::getParameter("~/show_cmp", show_cmp);


    // ros::Subscriber joint_sub = nh.subscribe(joint_state_topic_name2, 1, &jointSensorCallback);
    

    // constrained_manipulability::ConstrainedManipulability constrained_manip(nh,
    //                                                                         robot_namespace_suffix,
    //                                                                         fetch_param_server,
    //                                                                         kdl_chain_filename,
    //                                                                         root, tip,
    //                                                                         joint_state_topic_name);

    bool use_static_functions(false);

    // TEST FOR STATIC FUNCTIONS
    KDL::Tree my_tree;
    KDL::Chain chain;
    urdf::Model model;
    std::string robot_desc_string;

    std::string robot_description = "robot_description";
    nh.param(robot_description, robot_desc_string, std::string());
    model.initParamWithNodeHandle(robot_description, nh);


    bool fetch_param_server = true;
    if(fetch_param_server)
    {
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }
    }
    else
    {
        {
            ROS_INFO("Success");
        }
    }
    std::string root = "base_link";
    std::string tip = "tool0";
    std::string base_link = root;
    my_tree.getChain(root,
                        tip,
                        chain);
    unsigned int ndof;
    ndof = chain.getNrOfJoints();
    Eigen::Affine3d base_T_ee;
    // Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;

    KDL::JntArray kdl_joint_positions(ndof);
    ros::Duration(0.01).sleep();

    kdl_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    kdl_dfk_solver.reset(new KDL::ChainJntToJacSolver(chain));



    //constrained_manip.displayObjects();

    while (ros::ok())
    {
        //std::cout << "MP0_received" << MP0_received << std::endl;

        if (MP0_received == true && MP1_received == true && CMP0_received == true && CMP1_received == true && JMP0_received == true && JMP1_received == true )
        {
            MP0_received = false;
            MP1_received =false;
            CMP0_received = false;
            CMP1_received =false;
            JMP0_received = false;
            JMP1_received = false;
            //ROS_INFO("Trying here");
            Eigen::Vector3d offset_position;

            offset_position = offset_position0 + (offset_position1 - offset_position0)/2.0;
            ROS_INFO("Trying here");



            // Stack matrix2 below matrix1
            AHrep_MPout = Eigen::MatrixXd::Zero(AHrep_MP0.rows() + AHrep_MP1.rows(), AHrep_MP0.cols());
            AHrep_MPout << AHrep_MP0, AHrep_MP1;

                // Create a new vector to store the stacked vectors
            Eigen::VectorXd bHrep_MPout(bHrep_MP0.size() + bHrep_MP1.size());

            // Copy data from vector1 and vector2 into the stackedVector
            bHrep_MPout << bHrep_MP0, bHrep_MP1;


            // // Stack matrix2 below matrix1
            AHrep_CMPout = Eigen::MatrixXd::Zero(AHrep_CMP0.rows() + AHrep_CMP1.rows(), AHrep_CMP0.cols());
            AHrep_CMPout << AHrep_CMP0, AHrep_CMP1;

            //     // Create a new vector to store the stacked vectors
            Eigen::VectorXd bHrep_CMPout(bHrep_CMP0.size() + bHrep_CMP1.size());

            // // Copy data from vector1 and vector2 into the stackedVector
            bHrep_CMPout << bHrep_CMP0, bHrep_CMP1;

            // std::cout << "AHrep0 out before is" <<AHrep_MP0<<std::endl;
            // std::cout << "AHrep1 out before is" <<AHrep_MP1<<std::endl;
            // std::cout << "AHrep out after is" <<AHrep_MPout<<std::endl;

            // constrained_manipulability::ConstrainedManipulability::jointStatetoKDLJointArrayStatic(chain,joint_state, kdl_joint_positions);
            // constrained_manipulability::ConstrainedManipulability::getKDLKinematicInformationStatic(kdl_joint_positions, base_T_ee, base_J_ee,ndof,kdl_dfk_solver,kdl_fk_solver);

            // offset_position = base_T_ee.translation();



            // Stack jacobian matrices
           
            // base_J_ee_int << base_J_ee_int0, base_J_ee_int1;
            int numRows = base_J_ee_int0.rows();
            int numCols = base_J_ee_int0.cols() + base_J_ee_int1.cols();

            base_J_ee_int = Eigen::MatrixXd::Zero(base_J_ee_int0.rows()+ base_J_ee_int1.rows(), base_J_ee_int1.cols());
            base_J_ee_int << base_J_ee_int0, base_J_ee_int1;
            // // Use the block operation to concatenate the matrices horizontally
            // base_J_ee_int.block(0, 0, numRows, base_J_ee_int0.cols()) =base_J_ee_int0;
            // base_J_ee_int.block(0, base_J_ee_int0.cols(), numRows, base_J_ee_int1.cols()) = base_J_ee_int1;
            std::cout << "jacobian matrix is" <<base_J_ee_int <<std::endl;
            Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee(6, base_J_ee_int.cols());
            for (int i = 0; i < 6; ++i)
             {
                for (int j = 0; j < base_J_ee_int.cols(); ++j) {
                    base_J_ee(i, j) = base_J_ee_int(i, j);
                }
             }
             std::cout << "jacobian matrix is" << base_J_ee <<std::endl;
            // base_J_ee = base_J_ee_int;
            // base_J_ee = base_J_ee_int.cast<double>();
            // bHrep_CMPout(bHrep_CMP0.size() + bHrep_CMP1.size());

            
            // bHrep_CMPout << bHrep_CMP0, bHrep_CMP1;
            // bHrep_MPout = Eigen::MatrixXd::Zero(bHrep_MP0.rows() + bHrep_MP1.rows(), bHrep_MP0.cols());
            // bHrep_MPout << bHrep_MP0, bHrep_MP1;
            // To get intersection we simply stack the polytopes
            // AHrep_MPout.resize(AHrep_MP0.rows() + AHrep_MP1.rows(), AHrep_MP0.cols());
            // bHrep_MPout.resize(bHrep_MP0.rows() + bHrep_MP0.rows());
            // AHrep_MPout.topRows(AHrep_MP0.rows()) = AHrep_MP0;
            // AHrep_MPout.bottomRows(AHrep_MP1.rows()) = AHrep_MP1;
            // bHrep_MPout.head(bHrep_MP0.rows()) = bHrep_MP0;
            // bHrep_MPout.tail(bHrep_MP1.rows()) = bHrep_MP1;

            //std::cout << "stacked AHrep out before is" <<AHrep_MPout<<std::endl;
        
            constrained_manipulability::Polytope vrep_polytope_mp("allowable_motion_polytope", AHrep_MPout, bHrep_MPout);

            vrep_polytope_mp.transformCartesian(base_J_ee.topRows(3),offset_position);
            marker_success = constrained_manipulability::ConstrainedManipulability::staticplotPolytope(vrep_polytope_mp,offset_position, color_pts0, color_line0,mkr_intersection_pub);




            constrained_manipulability::Polytope vrep_polytope_cmp("constrained_motion_polytope", AHrep_CMPout, bHrep_CMPout);





            vrep_polytope_cmp.transformCartesian(base_J_ee.topRows(3),offset_position);
            marker_success = constrained_manipulability::ConstrainedManipulability::staticplotPolytope(vrep_polytope_cmp,offset_position, color_pts1, color_line1,mkr_intersection_pub);


            // Transform to Cartesian Space
            
            // vrep_polytope_mp.transformCartesian(base_J_ee.topRows(3), offset_position);
            // ROS_INFO("Managed to come here 11");
            // if (show_polytope)
            // {
                
            //     plotPolytope(vrep_polytope, offset_position, color_pts, color_line);
            //     ROS_INFO("Managed to come here 12");
            // }


            // AHrep_CMPout.resize(AHrep_CMP0.rows() + AHrep_CMP1.rows(), AHrep_CMP0.cols());
            // bHrep_CMPout.resize(bHrep_CMP0.rows() + bHrep_CMP0.rows());
            // AHrep_CMPout.topRows(AHrep_CMP0.rows()) = AHrep_CMP0;
            // AHrep_CMPout.bottomRows(AHrep_CMP1.rows()) = AHrep_CMP1;
            // bHrep_CMPout.head(bHrep_CMP0.rows()) = bHrep_CMP0;
            // bHrep_CMPout.tail(bHrep_CMP1.rows()) = bHrep_CMP1;


            // constrained_manipulability::Polytope vrep_polytope_cmp("constrained_allowable_motion_polytope", AHrep_MP0, bHrep_MP0);

            // marker_success = constrained_manipulability::ConstrainedManipulability::staticplotPolytope(vrep_polytope_cmp,offset_position, color_pts1, color_line1,mkr_intersection_pub);



            // Polytope vrep_polytope_cmp("allowable_motion_polytope_int", AHrep_MPout, bHrep_MPout);
        
            //     // Transform to Cartesian Space
        
            // vrep_polytope_cmp.transformCartesian(base_J_ee.topRows(3), offset_position);
            // ROS_INFO("Managed to come here 11");
            // if (show_polytope)
            // {
                
            //     plotPolytope(vrep_polytope, offset_position, color_pts, color_line);
            //     ROS_INFO("Managed to come here 12");
            // }
        
            // constrained_manipulability::Polytope allowable_poly_intersection = constrained_manip.getAllowableMotionPolytope(
            //     joint_state,
            //     show_mp,
            //     {0.0, 0.0, 0.5, 0.0},
            //     {0.0, 0.0, 1.0, 0.4});
            
            // constrained_manipulability::Polytope constrained_motion_poly_intersection = constrained_manip.getConstrainedAllowableMotionPolytope(
            //     joint_state,
            //     show_cmp,
            //     {0.0, 0.0, 0.5, 0.0},
            //     {1.0, 0.0, 0.0, 0.4});
                
            // constrained_manipulability::Polytope vel_poly = constrained_manip.getVelocityPolytope(
            //     joint_state, false);
            // constrained_manipulability::Polytope constrained_vel_poly = constrained_manip.getConstrainedVelocityPolytope(
            //     joint_state, false);

            // if (use_static_functions)
            // {

            //     Eigen::MatrixXd A;
            //     Eigen::VectorXd b;
            //     allowable_poly = constrained_manipulability::ConstrainedManipulability::
            //         getAllowableMotionPolytope(chain_, model_, joint_state, A, b);
            //     constrained_motion_poly = constrained_manipulability::ConstrainedManipulability::
            //         getConstrainedAllowableMotionPolytope(chain_, model_, objects, joint_state, A, b);
            //     // vel_poly = constrained_manipulability::ConstrainedManipulability::
            //     //     getVelocityPolytope(chain_, model_, joint_state, A, b);
            //     // constrained_vel_poly = constrained_manipulability::ConstrainedManipulability::
            //     //     getConstrainedVelocityPolytope(chain_, model_, objects, joint_state, A, b);
            // }
        }

        ros::spinOnce();
        ros::Duration(0.0001).sleep();
    }
    return 0;
}
