#!/usr/bin/env python3
## 
###

#### Old Dependencies


#!/usr/bin/env python
# Library import
############# ROS Dependencies #####################################
import rospy
import os
from geometry_msgs import msg
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped, WrenchStamped, PointStamped
from std_msgs.msg import Bool, Float32,Int16,String

from sensor_msgs.msg import Joy, JointState, PointCloud
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial import distance as dist_scipy
from numpy import sum,eye

from numpy import size


import tf_conversions as tf_c
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import Pose, Point
#### Polytope Plot - Dependency #####################################

# Polygon plot for ROS - Geometry message
from jsk_recognition_msgs.msg import PolygonArray, SegmentArray
import matplotlib.pyplot as plt

from geometry_msgs.msg import Polygon, PolygonStamped, Point32, Pose
import time
import threading


from rospygradientpolytope.visual_polytope import velocity_polytope, desired_polytope, velocity_polytope_with_estimation
from rospygradientpolytope.polytope_ros_message import create_polytopes_msg, create_polygon_msg, create_capacity_vertex_msg, create_segment_msg
from rospygradientpolytope.polytope_functions import get_polytope_hyperplane, get_capacity_margin
#from rospygradientpolytope.polytope_gradient_functions_optimized import Gamma_hat_gradient
from rospygradientpolytope.polytope_gradient_functions import Gamma_hat_gradient,Gamma_hat_gradient_dq

from rospygradientpolytope.sawyer_functions import jacobianE0, position_70
from rospygradientpolytope.robot_functions import getHessian, getJ_pinv
from rospygradientpolytope.linearalgebra import check_ndarray

#################### Linear Algebra ####################################################

from numpy.core.numeric import cross
from numpy import matrix, matmul, transpose, isclose, array, rad2deg, abs, vstack, hstack, shape, eye, zeros, random, savez, load
from numpy import polyfit, poly1d, count_nonzero

from numpy import float64, average,matmul,dot

from numpy.linalg import norm, det,pinv
import multiprocessing as mp
from math import atan2, pi, asin, acos
from geometry_msgs.msg import Pose, Point, Quaternion

from numpy import sum,matmul
############# ROS Dependencies #####################################
import rospy
import os
from geometry_msgs import msg
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped, WrenchStamped, PointStamped
from std_msgs.msg import Bool, Float32,Int16,String,MultiArrayDimension,MultiArrayLayout

from std_msgs.msg import Header,Float64

from sensor_msgs.msg import Joy, JointState, PointCloud
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial import distance as dist_scipy
from numpy import sum,average,mean,ones,dot,multiply
from tf.transformations import quaternion_matrix
#from pytransform3d.urdf import UrdfTransformManager
from ipaddress import collapse_addresses
from itertools import chain
import queue
from re import T
from numpy.core.numeric import cross
from geometry_msgs import msg
import rospy
from numpy import matrix, matmul, transpose, isclose, array, rad2deg, abs, vstack, hstack, shape, eye, zeros

from threading import Thread, Lock

import threading

import multiprocessing

from example_robot_data import load



import scipy.optimize as sco
## Mutex operator
from threading import Lock
import time
import threading
## Multiprocessing toolbox for Jacobian function

from multiprocessing import Process

# Import services here

from rospygradientpolytope.srv import IKopt, IKoptResponse


from tf.transformations import quaternion_matrix


mutex = Lock()




from numpy.linalg import norm, det, pinv, solve
from math import atan2, pi, asin, acos
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped, WrenchStamped, PointStamped
from std_msgs.msg import Bool,Float64
from sensor_msgs.msg import Joy, JointState, PointCloud,Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import copy

import tf_conversions as tf_c
from tf2_ros import TransformBroadcaster

# from kuka_rsi_hw_interface.srv import *



# Polygon plot for ROS - Geometry message
from jsk_recognition_msgs.msg import PolygonArray, SegmentArray
from geometry_msgs.msg import Polygon, PolygonStamped, Point32


## Do not distribute this library
from rospygradientpolytope.linearalgebra import proj_point_plane,V_unit


# Old library components - Refer to catkin_telebot_ws for the Rviz plane messages
#from rospygradientpolytope.visual_polytope import velocity_polytope, desired_polytope, velocity_polytope_with_estimation
#from rospygradientpolytope.polytope_ros_message import create_plane_msg,create_polytopes_msg, create_polygon_msg, create_capacity_vertex_msg, create_segment_msg



## Do not distribute this library
from rospygradientpolytope.visual_polytope import velocity_polytope, desired_polytope, velocity_polytope_with_estimation,cartesian_velocity_polytope
from rospygradientpolytope.visual_polytope import cartesian_cmp_polytope, cartesian_velocity_with_joint_limit_polytope, cartesian_cmp_hsm_polytope
from rospygradientpolytope.polytope_ros_message import create_polytopes_msg,create_fast_polytopes_msg, create_polygon_msg, create_capacity_vertex_msg, create_segment_msg
from rospygradientpolytope.polytope_functions import get_polytope_hyperplane, get_capacity_margin, get_constraint_obstacle_jacobian
#from rospygradientpolytope.polytope_gradient_functions_optimized import Gamma_hat_gradient
from rospygradientpolytope.polytope_gradient_functions import Gamma_hat_gradient,Gamma_hat_gradient_dq

from rospygradientpolytope.sawyer_functions import jacobianE0, position_70
from rospygradientpolytope.robot_functions import getHessian, getJ_pinv
from rospygradientpolytope.linearalgebra import check_ndarray

import PyKDL

from urdf_parser_py.urdf import URDF

# from kdl_parser_py import KDL
# from kdl_parser_py import urdf

#import open3d as o3d
### For service - Fixture line detection
from std_srvs.srv import Trigger, TriggerRequest

from std_msgs.msg import Float64MultiArray,Int32

# from pykdl_utils.kdl_kinematics import KDLKinematics
# from pykdl_utils.joint_kinematics import JointKinematics

from example_robot_data import load


### Import Pinnochio for Kinematics and Dynamics instead of KDL here
import pinocchio as pin


import time

from os.path import dirname, join, abspath
import rospkg






# getting the node namespace
namespace = rospy.get_namespace()

# For joint angle in URDF to screw



mutex = Lock()

mutex3 = Lock()

eps    = 1e-5
IT_MAX = 1000
DT     = 1e-1
damp   = 1e-12




# Load the urdf model
urdf_model_filename = '/home/imr/.local/lib/python3.8/site-packages/cmeel.prefix/share/example-robot-data/robots/dual_robot_description/urdf/kuka_meca.urdf'




robot_suffix = "_dualarm"




# get panda robot usinf example_robot_data
robot = load('kuka_meca')

# Load the urdf model
#robot.model = pin.buildModelFromUrdf(urdf_model_filename)

# get joint position ranges
q_max = robot.model.upperPositionLimit.T
q_min = robot.model.lowerPositionLimit.T
q_mean = (q_max+q_min)/2.0
print('q_mean',q_mean)
#input('stop q-mean')
# get max velocity
dq_max = robot.model.velocityLimit
dq_min = -dq_max

# Use robot configuration
# q0 = np.random.uniform(q_min,q_max)
q0 = (q_min+q_max)/2



# calculate the jacobian
data = robot.model.createData()

pin.framesForwardKinematics(robot.model,data,q0)
pin.computeJointJacobians(robot.model,data, q0)

# end-effector pose
Xee = data.oMf[robot.model.getFrameId(robot.model.frames[-1].name)]


urdf_model_path = '/home/imr/.local/lib/python3.8/site-packages/cmeel.prefix/share/example-robot-data/robots/dual_robot_description/urdf/kuka_meca.urdf'
mesh_dir = '/home/imr/.local/lib/python3.8/site-packages/cmeel.prefix/share/example-robot-data/robots/dual_robot_description/meshes/'
#geom_model = pin.buildGeomFromUrdf(robot.model,urdf_model_path,mesh_dir,pin.GeometryType.COLLISION)



geom_model = robot.collision_model

geom_data = pin.GeometryData(geom_model)
# geom_data.collisionRequest.enable_contact=True

print('geom_data',geom_data)

# Compute all the collisions
pin.computeCollisions(robot.model,data,geom_model,geom_data,q0,False)





a = pin.computeDistances(robot.model,data,geom_model,geom_data,q0)
b = geom_data.distanceResults[0]

#print('a result',a)
#print('b result',b)
geom_model = robot.collision_model
geom_data = pin.GeometryData(geom_model)
# Compute all the collisions
 

 
# Compute for a single pair of collision
pin.updateGeometryPlacements(robot.model,data,robot.collision_model,geom_data,q0)



J = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
# use only position jacobian
J = J[:3,:]

# end-effector pose
Xee = data.oMf[robot.model.getFrameId(robot.model.frames[-1].name)]

# ## visualise the robot
from pinocchio.visualize import MeshcatVisualizer

viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
# # Start a new MeshCat server and client.
viz.initViewer(open=True)
# Load the robot in the viewer.
viz.loadViewerModel()
viz.display(q0)


class Geomagic2KUKA():
    def __init__(self):
        rospy.init_node('Geo2KUKA', anonymous=True)



        self.rmodel = robot.model
        self.rdata = data
        self.geom_data = geom_data
        self.geom_model = geom_model
        self.no_of_joints = self.rmodel.njoints -1 ## Not including a fixed frame

        self.active_joints = 6



        # get joint position ranges
        self.q_max = robot.model.upperPositionLimit.T
        self.q_min = robot.model.lowerPositionLimit.T
        self.q_mean = (self.q_max+self.q_min)/2.0

        print('self.q_mean is',self.q_mean)
        #input('stop q-mean')
        self.psi_max = 1.0*zeros(self.no_of_joints)
        self.psi_min = 1.0*zeros(self.no_of_joints)

        self.qdot_max = dq_max
        self.qdot_min = -1.0*self.qdot_max


        self.q_upper_limit = self.q_max
        self.q_lower_limit = self.q_min

        self.damp = 1e-12
        self.DT = 0.004




        

        self.p_Hrep_A = Float64MultiArray()
        self.p_Hrep_b = Float64MultiArray()

        self.p_Hrep_size = Float64MultiArray()
        self.p_Hrep_size_dim = MultiArrayLayout()

        self.publish_velocity_polytope = rospy.Publisher(
            "/available_velocity_polytope"+robot_suffix, PolygonArray, queue_size=100)
        

        # Polytope in its H-rep from the polytope module
        '''
        self.publish_velocity_polytope_Hrep_A = rospy.Publisher(
            "/available_velocity_polytope_p_H_A"+robot_suffix, Float64MultiArray, queue_size=100)
        
        self.publish_velocity_polytope_Hrep_b = rospy.Publisher(
            "/available_velocity_polytope_p_H_b"+robot_suffix, Float64MultiArray, queue_size=100)
        
        self.publish_velocity_polytope_Hrep_size = rospy.Publisher(
            "/available_velocity_polytope_p_H_size"+robot_suffix, Float64MultiArray, queue_size=100)

        
        self.publish_desired_polytope = rospy.Publisher(
            "/desired_velocity_polytope"+robot_suffix, PolygonArray, queue_size=100)
        self.publish_capacity_margin_polytope = rospy.Publisher(
            "/capacity_margin_polytope"+robot_suffix, PolygonArray, queue_size=100)
        self.publish_vertex_capacity = rospy.Publisher(
            "/capacity_margin_vertex"+robot_suffix, PointStamped, queue_size=1)
        self.publish_vertex_proj_capacity = rospy.Publisher(
            "/capacity_margin_proj_vertex"+robot_suffix, PointStamped, queue_size=1)
        self.publish_capacity_margin_actual = rospy.Publisher(
            "/capacity_margin_actual"+robot_suffix, SegmentArray, queue_size=1)
        '''
        # publish plytope --- Estimated Polytope - Publisher
        '''
        self.publish_velocity_polytope_est = rospy.Publisher(
            "/available_velocity_polytope_est"+robot_suffix, PolygonArray, queue_size=100)
        self.publish_capacity_margin_polytope_est = rospy.Publisher(
            "/capacity_margin_polytope_est"+robot_suffix, PolygonArray, queue_size=100)
        self.publish_vertex_proj_capacity_est = rospy.Publisher(
            "/capacity_margin_proj_vertex_est"+robot_suffix, PointStamped, queue_size=1)
        self.publish_capacity_margin_actual_est = rospy.Publisher(
            "/capacity_margin_actual_est"+robot_suffix, SegmentArray, queue_size=1)

        self.publish_vertex_pose = rospy.Publisher(
            "/ef_pose_vertex"+robot_suffix, PointStamped, queue_size=1)
        self.publish_vertex_desired_pose = rospy.Publisher(
            "/ef_desired_pose_vertex"+robot_suffix, PointStamped, queue_size=1)
        '''
        self.polytope_display = False

        self.polytope_display_on_sub = rospy.Subscriber("polytope_show"+robot_suffix,Bool,self.polytope_show_on_callback)
       
        self.q_in_numpy = zeros(self.no_of_joints)

        self.qdot_out = zeros(self.no_of_joints)
        self.qdot_out_meca = zeros(self.no_of_joints)
        




        self.EnterFlag = True

        self.kuka_joint_states_subscriber = rospy.Subscriber(
            "/joint_states", JointState, self.kuka_callback, queue_size=1)




        self.end_effector_pos = rospy.Publisher("/end_effector_pos",PoseStamped,queue_size=1)
        self.end_effector_position = rospy.Publisher("/end_effector_position",PointStamped,queue_size=1)

        self.end_effector_position_kuka = rospy.Publisher("/grasp_frame_kuka",PointStamped,queue_size=1)
        self.end_effector_position_meca = rospy.Publisher("/grasp_frame_meca",PointStamped,queue_size=1)

        

        
        self.robot_joint_states = JointState()
        self.robot_joint_states.position = zeros(self.no_of_joints-1,dtype=float)

        self.kuka_joint_states = JointState()
        self.kuka_joint_states.position = zeros(6,dtype=float)    # [0.0,0.0,0.0,0.0,0.0,0.0]

        self.meca_joint_states = JointState()
        self.meca_joint_states.position = zeros(6,dtype=float)


        self.start_linearvelocity_state = False
        self.start_angularvelocity_state = False
        self.geo_pose_orientation_prev = array([0.0, 0.0, 0.0])

        self.pub_rate = 500  # Hz
        self.fixed_magnitude = 30
        #self.vel_scale = 3.5*self.fixed_magnitude*array([1.0, 1.0, 1.0])

        #self.vel_scale =1.0*self.fixed_magnitude*array([1.0, 1.0, 1.0])
        self.vel_scale =0.2*self.fixed_magnitude*array([1.0, 1.0, 1.0])

        
        self.angular_vel_scale = 10.0*array([1.0, 1.0, 1.0])
        
        self.angular_velocity_vector = matrix([[0.0], [0.0], [0.0]])
        self.previous_msg_state = False
        self.previous_msg = matrix([[0, 0, 0, 0, 0, 0]])
        self.button_robot_state = [0, 0]
        self.change_gripper_state = False
        self.changing_state = False

        



        self.base_line_counter = 0
        

        
        ## I dont know wtf we do the log to get to twist 
        ## Need to read about Lie Algebra
        
        self.cartesian_twist = pin.log(pin.SE3.Identity()).vector

        

        # Limits of all jointts are here

        self.robot_joint_names = ['joint_a1', 'joint_a2',
            'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6','meca_axis_1_joint','meca_axis_2_joint','meca_axis_3_joint','meca_axis_4_joint'\
                                  ,'meca_axis_5_joint','meca_axis_6_joint']

        self.robot_joint_names_pub = self.robot_joint_names

        self.gripper_state_msg = Bool()

        self.plane_verts = []
        self.pose_verts = []

        ## Distance between virtual guide planes
        self.dist_plane = 0.005 # 3 mm 
        self.dist_tol = 0.002

        self.qdot_limit = self.qdot_max

        # self.qdot_limit = [
        #     robot_urdf.joint_map[i].limit.velocity for i in self.robot_joint_names]

        # self.qdot_max = array(self.qdot_limit)
        # self.qdot_min = -1*self.qdot_max
        self.fun_iter = Int16()
        self.fun_iter.data = 0
        self.start_optimization_bool = False

        self.msg_status_ik = String()




        self.plot_polytope_thread = None
        self.thread_is_running = False


        self.force_baseline_arr_meca = zeros(shape=(500,3))
        self.torque_baseline_arr_meca = zeros(shape=(500,3))

        self.force_baseline_meca = zeros(shape=(3))
        self.torque_baseline_meca = zeros(shape=(3))
        self.baseline_record_once = True


        self.force_norm = 0
        self.torque_norm = 0


        ## Hard-coded value

        self.obstacle_link_vector = zeros(shape = (12,3))
        self.obstacle_dist_vector = zeros(shape = (12))
        self.scaled_maximum_vector = ones(shape=(12))
        self.danger_threshold = 0.1

        self.polytope_verts_cmp = array([])
        self.polytope_faces_cmp = array([])

        





        
        self.fun_counter = 0

        self.color_array_cm = ['g','r']
        
        self.time_arr = zeros(shape=(2))

        ############3 Python Attributes ####################################

        # self.joints_name = list(tm._joints)

        pin.forwardKinematics(self.rmodel,self.rdata, self.q_in_numpy)
        pin.updateFramePlacements(self.rmodel,self.rdata)
        

        self.publish_velocity_polytope = rospy.Publisher(
            "/available_velocity_polytope"+robot_suffix, PolygonArray, queue_size=100)
        
        self.publish_velocity_cmp = rospy.Publisher(
            "/available_cmp_polytope"+robot_suffix, PolygonArray, queue_size=100)
        

        self.publish_velocity_cmp_hsm = rospy.Publisher(
            "/available_cmp_hsm_polytope"+robot_suffix, PolygonArray, queue_size=100)
        

        self.cp1_publisher_msg = rospy.Publisher(
            "/cp1_point"+robot_suffix, PointStamped, queue_size=100)
        
        self.cp2_publisher_msg = rospy.Publisher(
            "/cp2_point"+robot_suffix, PointStamped, queue_size=100)
        
        self.publish_velocity_polytope = rospy.Publisher(
            "/available_velocity_polytope", PolygonArray, queue_size=100)
        self.publish_desired_polytope = rospy.Publisher(
            "/desired_velocity_polytope", PolygonArray, queue_size=100)
        self.publish_capacity_margin_polytope = rospy.Publisher(
            "/capacity_margin_polytope", PolygonArray, queue_size=100)
        self.publish_vertex_capacity = rospy.Publisher(
            "/capacity_margin_vertex", PointStamped, queue_size=1)
        self.publish_vertex_proj_capacity = rospy.Publisher(
            "/capacity_margin_proj_vertex", PointStamped, queue_size=1)
        self.publish_capacity_margin_actual = rospy.Publisher(
            "/capacity_margin_actual", SegmentArray, queue_size=1)

        # publish plytope --- Estimated Polytope - Publisher

        self.publish_velocity_polytope_est = rospy.Publisher(
            "/available_velocity_polytope_est", PolygonArray, queue_size=100)
        self.publish_capacity_margin_polytope_est = rospy.Publisher(
            "/capacity_margin_polytope_est", PolygonArray, queue_size=100)
        self.publish_vertex_proj_capacity_est = rospy.Publisher(
            "/capacity_margin_proj_vertex_est", PointStamped, queue_size=1)
        self.publish_capacity_margin_actual_est = rospy.Publisher(
            "/capacity_margin_actual_est", SegmentArray, queue_size=1)

        self.publish_vertex_pose = rospy.Publisher(
            "/ef_pose_vertex", PointStamped, queue_size=1)
        self.publish_vertex_desired_pose = rospy.Publisher(
            "/ef_desired_pose_vertex", PointStamped, queue_size=1)

        # Subscribe joints of Robot --- Joint State subscriber

        self.robot_joint_state_publisher = rospy.Publisher(
            "/joint_states", JointState, queue_size=1)
                
        self.polytope_display = False
        
        self.polytope_display_on_sub = rospy.Subscriber("polytope_show",Bool,self.polytope_show_on_callback)
        self.start_interactive_ik_sub = rospy.Subscriber("run_ik",Bool,self.start_interactive_ik)
        self.pub_end_ik = rospy.Publisher("ik_progress",Int16,queue_size=1)
        self.pub_status_ik = rospy.Publisher("status_ik",String,queue_size=1)
        self.sub_ik_pos = rospy.Subscriber("interactive_sphere",Pose,self.ik_pose_callback)


        #self.robot_joint_state_subscriber = rospy.Subscriber("/joint_states",JointState,self.joint_state_callback,queue_size=1)



        self.cartesian_desired_vertices = 0.05*array([[0.20000, 0.50000, 0.50000],
                                                     [0.50000, -0.10000, 0.50000],
                                                     [0.50000, 0.50000, -0.60000],
                                                     [0.50000, -0.10000, -0.60000],
                                                     [-0.30000, 0.50000, 0.50000],
                                                     [-0.30000, -0.10000, 0.50000],
                                                     [-0.30000, 0.50000, -0.60000],
                                                     [-0.30000, -0.10000, -0.60000]])
        


        
        self.desired_pose = Pose()


        self.desired_vertices = zeros(
            shape=(len(self.cartesian_desired_vertices), 3))

        self.desired_vertices = self.cartesian_desired_vertices

        self.pub_rate = 500  # Hz

        self.sigmoid_slope = 150

        self.sigmoid_slope_input = 5

        self.fun_iter = Int16()
        self.fun_iter.data = 0
        self.start_optimization_bool = False

        self.msg_status_ik = String()

        print('self.qdot_max', self.qdot_max)
        print('self.qdot_min', self.qdot_min)
        self.q_in = zeros(12)

        jac_output = mp.Array("f",[0,0,0,0,0,0,0,0,0,0,0,0])

        self.plot_polytope_thread = None
        self.thread_is_running = False
       

        

        self.q_upper_limit = array([self.q_max]).T
        #self.q_upper_limit = self.pykdl_util_kin.joint_limits_upper
        self.q_lower_limit = array([self.q_min]).T
        #self.q_lower_limit = self.pykdl_util_kin.joint_limits_lower

        self.q_bounds = hstack((self.q_lower_limit, self.q_upper_limit))
        sigmoid_slope_test = array([50, 100, 150, 200, 400])

        self.sigmoid_slope_array = array([50, 100, 150, 200, 400])

        self.cm_est = None

        self.time_counter = 0
        self.fun_counter = 0

        self.color_array_cm = ['g','r']
        self.cm_est_arr = zeros(shape=(2))
        self.cm_est_arr[:] = -10000
        self.time_arr = zeros(shape=(2))
    
        #self.plot_polytope()
    

    def ik_pose_callback(self,desired_ik_pose):
        print('this is what i am ')
        self.desired_ik_pose = desired_ik_pose.position
        print('desired ik pose is',self.desired_ik_pose)
    def start_interactive_ik(self,start_optimization):
        print('start_IK',start_optimization.data)
        self.start_optimization_bool = start_optimization.data
        if self.start_optimization_bool:
            print('Start IK')            
            self.fun_iter.data = 0
            self.fun_counter = 0
            self.compute_pose_ik(pos_ik=array([self.desired_ik_pose.x,self.desired_ik_pose.y,self.desired_ik_pose.z]))  # Picture - FEasible pose - 1 - Good
    def start_plot_thread(self):
        if self.thread_is_running:
            print("Thread already running!")
            return
        self.plot_polytope_thread = threading.Thread(target=self.plot_polytope)
        self.thread_is_running = True
        self.plot_polytope_thread.start()

        #input('I have started thread')
    def stop_thread(self):
        self.thread_is_running = False
        print('Stopping thread')

    def processfeedback(self, feedback):
        self.desired_pose.position.x = feedback.pose.position.x
        self.desired_pose.position.y = feedback.pose.position.y
        self.desired_pose.position.z = feedback.pose.position.z

    def polytope_show_on_callback(self,show_bool):
        self.polytope_display = show_bool.data

        if self.polytope_display:
            self.start_plot_thread()
            #self.start_cm_plot_thread()
        else:
            self.stop_thread()
            #self.stop_cm_thread()

    def plot_capacity_margin_est(self,cm_est):

        '''
        while self.thread_cm_is_running:
            if self.polytope_display:

                #print('plotting here')
        '''
        self.cm_est = cm_est
        if self.cm_est != None:
            
            if self.cm_est > 0:
                color_arr_cm = 'g'
            else:
                color_arr_cm = 'r'                
        

            if self.cm_est_arr[0] != -10000 and self.cm_est_arr[1] == -10000:

                self.cm_est_arr[1] = self.cm_est
                self.ax_cm.scatter(self.time_counter,self.cm_est,color=color_arr_cm)
                
            if self.cm_est_arr[0] == -10000:
                self.cm_est_arr[0] = self.cm_est     
            
                self.ax_cm.scatter(self.time_counter,self.cm_est,color=color_arr_cm)

            else:
                x = [self.time_counter-1,self.time_counter]
                
                self.cm_est_arr[0] =  self.cm_est_arr[1]
                self.cm_est_arr[1] =  self.cm_est
                self.ax_cm.plot(x,self.cm_est_arr,color=color_arr_cm)
                #self.cm_est_arr



            


            #self.fig_cm.canvas.flush_events()
            self.time_counter += 1

            # drawing updated values
            if self.polytope_display:
                self.fig_cm.canvas.draw()
            else:
                for artist in plt.gca().lines + plt.gca().collections:
                    artist.remove()
                    self.time_counter = 0



    def compute_pose_ik(self, pos_ik):
        import time
        from numpy.linalg import det
        from numpy import sum, mean, average, linspace
        import matplotlib.pyplot as plt

        
        q0 = zeros(self.active_joints)
        for j in range(6,self.no_of_joints):
            q0[6-j] = random.uniform(
                self.q_lower_limit[6-j], self.q_upper_limit[6-j])
        

        #for i in range(0, 1000):

        st = time.time()
        using_pinnochio = False

        if using_pinnochio:
            q_opt = self.fmin_opt_ik(q0, pos_ik, True)
            q0 = q_opt
        else:
            q_opt = self.fmin_opt_ik(q0, pos_ik, True)
            print('q_opt is',q_opt)
            q_opt = self.fmin_opt(q_opt[6:], pos_ik, True)
            q0 = q_opt.x

        # To publish the joint states to Robot
        #self.joint_state_publisher_robot(q_opt.x)

        ex_time = time.time() - st
        print('execution time is',ex_time)




        
        #print('self.polytope_display',self.polytope_display)
    def joint_state_callback(self, robot_joints):

        # Get Joint angles of the Sawyer Robot
        # Interchanged joint positions here
        ## Be careful for UR robot the index is changed 0 and 2 are interchanged

        for i in range(self.no_of_joints):
            self.q_in[i] = robot_joints.position[i]

    # def check_gradient(self,q_in,step_size:int):
    def joint_state_publisher_robot(self, q_joints):

        q_in = q_joints
        #print('q_in joints are',q_in)
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'
        msg.velocity = []
        msg.effort = []

        for i in range(self.no_of_joints):
            msg.name.append(self.robot_joint_names_pub[i])
            msg.position.append(q_in[i])
        #msg.position = [q_in[6],q_in[7],q_in[8],q_in[9],q_in[10],q_in[11],q_in[0],q_in[1],q_in[2],q_in[3],q_in[4],q_in[5]]
            
        self.robot_joint_state_publisher.publish(msg)

        '''
        if self.polytope_display:
            self.plot_polytope_thread = mp.Process(target=self.plot_polytope,args=(q_joints))
            self.plot_polytope_thread.start()
            #self.plot_polytope_thread.join()
        '''
        self.q_in = q_in
        

    def plot_polytope(self):                                                                                                                                                                                                                 
        
        while self.thread_is_running:
            if self.polytope_display:

                
                #print('I am plotting')
                viz.display(self.q_in_numpy)


                mutex.acquire()
                pin.computeFrameJacobian(self.rmodel, self.rdata,self.q_in_numpy,33)
                pin.forwardKinematics(self.rmodel,self.rdata, self.q_in_numpy)
                pin.updateFramePlacements(self.rmodel,self.rdata)


                pos_act1 = self.rdata.oMf[self.rmodel.getFrameId('tcp_kuka')].translation
                pos_act2 = self.rdata.oMf[self.rmodel.getFrameId('tcp_meca')].translation



                pos_act = pos_act1 + (pos_act2 - pos_act1)*0.5

                
                pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)

                
                J_Hess1 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_kuka'), pin.LOCAL_WORLD_ALIGNED)
                J_Hess2 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_meca'), pin.LOCAL_WORLD_ALIGNED)
                
                J_Hess = hstack((J_Hess1[:,:6],J_Hess2[:,6:]))


                

                scaling_factor = 10.0
                
                ### Polytope plot with estimation
                
                #pykdl_kin_jac = pykdl_util_kin.jacobian(self.q_in_numpy)
                polytope_verts, polytope_faces, facet_vertex_idx, capacity_faces, capacity_margin_proj_vertex, \
                    polytope_verts_est, polytope_faces_est, capacity_faces_est, capacity_margin_proj_vertex_est,cm_index = \
                                                velocity_polytope_with_estimation(J_Hess,self.qdot_max,self.qdot_min,self.desired_vertices,self.sigmoid_slope_input)
                desired_polytope_verts, desired_polytope_faces = desired_polytope(self.desired_vertices)


                self.cm_est = cm_index


                ef_pose = pos_act


                ########### Actual POlytope plot ###########################################################################
                # Publish polytope faces
                polyArray_message = self.publish_velocity_polytope.publish(create_polytopes_msg(polytope_verts, polytope_faces, \
                                                                                                    ef_pose,"base_link", scaling_factor))
                
                
                ### Desired polytope set - Publish

                DesiredpolyArray_message = self.publish_desired_polytope.publish(create_polytopes_msg(desired_polytope_verts, desired_polytope_faces, \
                                                                                                    ef_pose,"base_link", scaling_factor))


                ### Vertex for capacity margin on the Desired Polytope
                #print('facet_vertex_idx',facet_vertex_idx)
                closest_vertex = self.cartesian_desired_vertices[facet_vertex_idx[0,1]]
                #print('closest_vertex',closest_vertex)

                CapacityvertexArray_message = self.publish_vertex_capacity.publish(create_capacity_vertex_msg(closest_vertex, \
                                                                                            ef_pose, "base_link", scaling_factor))

                
                
                ### Vertex for capacity margin on the Available Polytope
                CapacityprojvertexArray_message = self.publish_vertex_proj_capacity.publish(create_capacity_vertex_msg(capacity_margin_proj_vertex, \
                                                                                            ef_pose, "base_link", scaling_factor))


                ### Vertex for capacity margin on the Available Polytope
                ActualposevertexArray_message = self.publish_vertex_pose.publish(create_capacity_vertex_msg(ef_pose, \
                                                                                            array([0,0,0]), "base_link", 1))
                
                ### Vertex for capacity margin on the Available Polytope
                '''
                DesiredposevertexArray_message = self.publish_vertex_desired_pose.publish(create_capacity_vertex_msg(self.pos_reference, \
                                                                                        array([0,0,0]), "base_link", 1))
                '''
                ### Plane for capacity margin 


                ### Vertex for capacity margin on the Available Polytope
                CapacitymarginactualArray_message = self.publish_capacity_margin_actual.publish(create_segment_msg(closest_vertex, \
                                                    capacity_margin_proj_vertex,ef_pose, "base_link", scaling_factor))
                
                capacityArray_message = self.publish_capacity_margin_polytope.publish(create_polytopes_msg(polytope_verts, capacity_faces, \
                                                                                                    ef_pose,"base_link", scaling_factor))


                ########### Estimated Polytope plot ###########################################################################
                
                # Publish polytope faces
                EstpolyArray_message = self.publish_velocity_polytope_est.publish(create_polytopes_msg(polytope_verts_est, polytope_faces_est, \
                                                                                                    ef_pose,"base_link", scaling_factor))
                
                

                ### Vertex for capacity margin on the Available Polytope
                EstCapacityprojvertexArray_message = self.publish_vertex_proj_capacity_est.publish(create_capacity_vertex_msg(capacity_margin_proj_vertex_est, \
                                                                                            ef_pose, "base_link", scaling_factor))


                ### Vertex for capacity margin on the Available Polytope
                EstCapacitymarginactualArray_message = self.publish_capacity_margin_actual_est.publish(create_segment_msg(closest_vertex, \
                                                    capacity_margin_proj_vertex_est,ef_pose, "base_link", scaling_factor))
                

                EstcapacityArray_message = self.publish_capacity_margin_polytope_est.publish(create_polytopes_msg(polytope_verts_est, capacity_faces_est, \
                                                                                                    ef_pose,"base_link", scaling_factor))

                


                ### Vertex 
                
                ##############################################################################################################
                
                
                #print('facet_vertex_idx',facet_vertex_idx)
                
                mutex.release()

    def ft_kuka_callback(self, ft_data_kuka):

        self.ft_sensor_kuka = ft_data_kuka
        torque = array([ft_data_kuka.wrench.torque.x, ft_data_kuka.wrench.torque.y, ft_data_kuka.wrench.torque.z ])
        forces = array([ft_data_kuka.wrench.force.x, ft_data_kuka.wrench.force.y, ft_data_kuka.wrench.force.z])
        wrench_arr = hstack([[torque,forces]])
        #print('wrench_arr - kuka',wrench_arr)
        pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)

                
        J_Hess1 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_kuka'), pin.LOCAL_WORLD_ALIGNED)
        force_polytope_verts = matmul(transpose(J_Hess1[0:3,:6]),transpose(wrench_arr))

    
    
    def collision_update_callback(self):
        
        #mutex.acquire()

        # # Compute for a single pair of collision        
        distance_results = self.geom_data.distanceResults
        #print('distanceresults',distance_results)
        # Print the status of collision for all collision pairs
        counter = 0

        for result in distance_results: 

            cp1 = result.getNearestPoint1()
            cp2 = result.getNearestPoint2()
            print('closest points 1',result.getNearestPoint1())
            print('closest points 2',result.getNearestPoint2())


            if (counter < 12):

                self.obstacle_link_vector[counter,0] = cp2[0] - cp1[0]
                self.obstacle_link_vector[counter,1] = cp2[1] - cp1[1]
                self.obstacle_link_vector[counter,2] = cp2[2] - cp1[2]


            counter += 1
    
    
        
    def polytope_show_on_callback(self,show_bool):
        self.polytope_display = show_bool.data

        if self.polytope_display:
            self.start_plot_thread()
            #self.start_cm_plot_thread()
        else:
            self.stop_thread()
            #self.stop_cm_thread()
    def start_plot_thread(self):
        if self.thread_is_running:
            print("Thread already running!")
            return
        self.plot_polytope_thread = threading.Thread(target=self.plot_polytope)
        self.thread_is_running = True
        self.plot_polytope_thread.start()

        #input('I have started thread')
    def stop_thread(self):
        self.thread_is_running = False
        print('Stopping thread')
    
    
    
    def plot_polytope(self):
        
        
               
        while not rospy.is_shutdown():
            pin.updateGeometryPlacements(self.rmodel,self.rdata,self.geom_model,self.geom_data,self.q_in_numpy)
            pin.computeDistances(self.rmodel, self.rdata, self.geom_model, self.geom_data, self.q_in_numpy)
            distance_results = self.geom_data.distanceResults

            counter = 0

            
            time_begin = rospy.Time.now()


            pin.computeFrameJacobian(self.rmodel, self.rdata,self.q_in_numpy,33)
            pin.forwardKinematics(self.rmodel,self.rdata, self.q_in_numpy)
            pin.updateFramePlacements(self.rmodel,self.rdata)


            pos_act1 = self.rdata.oMf[self.rmodel.getFrameId('tcp_kuka')].translation
            pos_act2 = self.rdata.oMf[self.rmodel.getFrameId('tcp_meca')].translation



            pos_act = pos_act1 + (pos_act2 - pos_act1)*0.5

            
            pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)

            
            J_Hess1 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_kuka'), pin.LOCAL_WORLD_ALIGNED)
            J_Hess2 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_meca'), pin.LOCAL_WORLD_ALIGNED)
            
            J_Hess = hstack((J_Hess1[:,:6],J_Hess2[:,6:]))

            

            scaling_factor = 80.0
            



            


            ef_pose = transpose(pos_act)


            distance_results = self.geom_data.distanceResults

            # Print the status of collision for all collision pairs
            counter = 0

            msg1 = PointStamped()
            msg1.header = Header()
            msg1.header.frame_id = 'telebot_cell_base_link'
            msg2 = PointStamped()
            msg2.header = Header()
            msg2.header.frame_id = 'telebot_cell_base_link'


            

            for result in distance_results: 
                #cr = geom_data.collisionResults[k].closestPoints
                #dr_result = distance_results[k]
                #print('result is',dir(result))
                #cp = robot.collision_model.collisionPairs[k]

                
                cp1 = result.getNearestPoint1()
                cp2 = result.getNearestPoint2()
                #print('closest points 1',result.getNearestPoint1())
                #print('closest points 2',result.getNearestPoint2())





                if (counter < 12):
                    self.obstacle_dist_vector[counter] = result.min_distance
                    if result.min_distance <= self.danger_threshold:

                        self.obstacle_link_vector[counter,0] = cp2[0] - cp1[0]
                        self.obstacle_link_vector[counter,1] = cp2[1] - cp1[1]
                        self.obstacle_link_vector[counter,2] = cp2[2] - cp1[2]


                        self.scaled_maximum_vector[counter] = result.min_distance**2

                    
                    if counter == 0:

                        msg1.point.x = cp1[0]
                        msg1.point.y = cp1[1]
                        msg1.point.z = cp1[2]

                        msg2.point.x = cp2[0]
                        msg2.point.y = cp2[1]
                        msg2.point.z = cp2[2]
                        self.cp1_publisher_msg.publish(msg1)
                        self.cp2_publisher_msg.publish(msg2)

                counter += 1

            
            ########### Actual POlytope plot ###########################################################################
            # Publish polytope faces
            # polyArray_message = self.publish_velocity_polytope.publish(create_polytopes_msg(polytope_verts, polytope_faces, \
            #                                                                                     ef_pose,"telebot_cell_base_link", scaling_factor))
            

            
            #print('obstacle_link_global is',self.obstacle_link_vector)
            #print('self.obstacle_dist_vector',self.obstacle_dist_vector)
            #input('stop and test')
            '''
            polytope_verts_cmp, polytope_faces_cmp,polytope_center,polytope_center_max = cartesian_cmp_polytope(J_Hess,self.q_in_numpy,self.qdot_min,self.qdot_max,\
                                                                                        self.q_min,self.q_max,self.q_mean,self.psi_max, self.psi_min, \
                                                                                            self.obstacle_link_vector, \
                                                                                                self.obstacle_dist_vector,self.danger_threshold)
            

            if polytope_verts_cmp.any():
                self.polytope_verts_cmp = polytope_verts_cmp
                self.polytope_faces_cmp = polytope_faces_cmp

            '''

            polytope_verts_cmp, polytope_faces_cmp,polytope_center,polytope_center_max = cartesian_cmp_hsm_polytope(J_Hess,self.q_in_numpy,self.qdot_min,self.qdot_max,\
                                                                                        self.q_min,self.q_max,self.q_mean,self.psi_max, self.psi_min, \
                                                                                            self.obstacle_link_vector, \
                                                                                                self.obstacle_dist_vector,self.danger_threshold)
            

            if polytope_verts_cmp.any():
                self.polytope_verts_cmp = polytope_verts_cmp
                self.polytope_faces_cmp = polytope_faces_cmp



            # polytope_verts_jpl, polytope_faces_jpl = cartesian_velocity_with_joint_limit_polytope(J_Hess,self.q_in_numpy,self.qdot_min,self.qdot_max,\
            #                                                                             self.q_min,self.q_max,self.q_mean,self.psi_max, self.psi_min)
            

            #J_coll = get_constraint_obstacle_jacobian(J_Hess,12,self.obstacle_link_vector,self.obstacle_dist_vector,self.danger_threshold)
            
            #print('J_coll is',J_coll)

            print('without scaling',self.qdot_max)
            qdot_max_scaled =multiply(self.qdot_max,self.scaled_maximum_vector)

            print('qdot_max_scaled',qdot_max_scaled)

            polytope_verts_hsm, polytope_faces_hsm = cartesian_velocity_polytope(J_Hess,self.qdot_min,qdot_max_scaled)


            

            ########### Obstacle with HSM POlytope plot ###########################################################################
            # Publish polytope faces
            polyArray_cmp_hsm_message = self.publish_velocity_cmp_hsm.publish(create_polytopes_msg(polytope_verts_hsm, polytope_faces_hsm, \
                                                                                                ef_pose,"telebot_cell_base_link", scaling_factor))
            


            polytope_point_msg = PointStamped()
            polytope_point_msg.header = Header()
            polytope_point_msg.header.frame_id = 'telebot_cell_base_link'
            polytope_point_msg.point.x = pos_act[0]+ polytope_center[0]/scaling_factor
            polytope_point_msg.point.y = pos_act[1]+polytope_center[1]/scaling_factor
            polytope_point_msg.point.z = pos_act[2]+polytope_center[2]/scaling_factor


            #self.chebychev_msg.publish(polytope_point_msg)

            ef_pose = transpose(pos_act)





            polyArray_cmp_message = self.publish_velocity_cmp.publish(create_polytopes_msg(self.polytope_verts_cmp, self.polytope_faces_cmp, \
                                                                                                ef_pose,"telebot_cell_base_link", scaling_factor))
            

            time_end = rospy.Time.now()
            duration1 = time_end - time_begin
            print('Duration for computation',duration1)


            end_ef_kuka_msg = PointStamped()
            end_ef_kuka_msg.header = Header()

            end_ef_kuka_msg.header.frame_id = 'telebot_cell_base_link'
            end_ef_kuka_msg.header.stamp = rospy.Time.now()

            
            end_ef_kuka_msg.point.x = pos_act[0]
            end_ef_kuka_msg.point.y = pos_act[1]
            end_ef_kuka_msg.point.z = pos_act[2]
            self.end_effector_position_kuka.publish(end_ef_kuka_msg)





    def collision_state_callback(self, collision_state):
        self.current_collision_state = collision_state.data
        # print('collision_state is', self.current_collision_state)

    

    def meca_callback(self, meca_qin_joints):
        # Callback for getting current joint states of MECA
        mutex.acquire()
        #self.q_in[0] = meca_qin_joints.position[0]
        self.q_in_numpy[6] = meca_qin_joints.position[0]
        #self.q_in[1] = meca_qin_joints.position[1]
        self.q_in_numpy[7] = meca_qin_joints.position[1]
        #self.q_in[2] = meca_qin_joints.position[2]
        self.q_in_numpy[8] = meca_qin_joints.position[2]
        #self.q_in[3] = meca_qin_joints.position[3]
        self.q_in_numpy[9] = meca_qin_joints.position[3]
        #self.q_in[4] = meca_qin_joints.position[4]
        self.q_in_numpy[10] = meca_qin_joints.position[4]
        #self.q_in[5] = meca_qin_joints.position[5]
        self.q_in_numpy[11] = meca_qin_joints.position[5]
        mutex.release()

    def kuka_callback(self, kuka_qin_joints):
        # Callback for getting current joint states of KUKA
        mutex.acquire()
        #self.q_in[0] = kuka_qin_joints.position[0]
        self.q_in_numpy[0] = kuka_qin_joints.position[0]
        #self.q_in[1] = kuka_qin_joints.position[1]
        self.q_in_numpy[1] = kuka_qin_joints.position[1]
        #self.q_in[2] = kuka_qin_joints.position[2]
        self.q_in_numpy[2] = kuka_qin_joints.position[2]
        #self.q_in[3] = kuka_qin_joints.position[3]
        self.q_in_numpy[3] = kuka_qin_joints.position[3]
        #self.q_in[4] = kuka_qin_joints.position[4]
        self.q_in_numpy[4] = kuka_qin_joints.position[4]
        #self.q_in[5] = kuka_qin_joints.position[5]
        self.q_in_numpy[5] = kuka_qin_joints.position[5]
        mutex.release()

    
    def fmin_opt_ik(self,x0_start,pose_desired,analytical_solver: bool):
        self.initial_x0 = float64(x0_start)
        self.pos_reference = float64(pose_desired)
        print('Reference position is', self.pos_reference)

        # input('self.pos_reference')

        # Desired vertex set

        # + self.pos_reference[0,0]
        self.desired_vertices[:, 0] = self.cartesian_desired_vertices[:, 0]
        # + self.pos_reference[0,1]
        self.desired_vertices[:, 1] = self.cartesian_desired_vertices[:, 1]
        # + self.pos_reference[0,2]
        self.desired_vertices[:, 2] = self.cartesian_desired_vertices[:, 2]
        # print('self.opt_polytope_gradient_model.d_gamma_hat',self.opt_polytope_gradient_model.d_gamma_hat)

        # Bounds created from the robot angles

        self.opt_bounds = float64(self.q_bounds)
        print('q_bounds are',self.opt_bounds)
        #q      = pin.neutral(self.rmodel)
        eps    = 1e-5
        IT_MAX = 1000
        DT     = 1e-1
        damp   = 1e-12

        i=0
        self.fun_iter.data = 100

        while True:
            pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)
            pin.framesForwardKinematics(self.rmodel, self.rdata, self.q_in_numpy)
            #pos_act_int1 = self.rdata.oMf[self.rmodel.getFrameId('tcp_kuka')].translation
            pos_act_int1 = pose_desired
            pos_act_int2 = self.rdata.oMf[self.rmodel.getFrameId('tcp_meca')].translation

            print('pos_act1',pos_act_int1)
            print('pos_act2',pos_act_int2)
            pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)
            #print('self.q_in_numpy is',self.q_in_numpy)
            J_Hess2 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_meca'), pin.LOCAL_WORLD_ALIGNED)
            J = J_Hess2                          
            oMdes = pin.SE3(eye(3), array([pos_act_int1[0], pos_act_int1[1], pos_act_int1[2]]))
            OMcurr = pin.SE3(eye(3), array([pos_act_int2[0], pos_act_int2[1], pos_act_int2[2]]))

            dMi = oMdes.actInv(OMcurr)
            err = pin.log(dMi).vector
            self.fun_counter += 0.5
            self.fun_iter.data = int(self.fun_counter)

            if norm(err) < eps:
                success = True
                self.msg_status_ik.data = 'Success'
                self.fun_iter.data = int(100)
                self.pub_end_ik.publish(self.fun_iter)
                break
            if i >= IT_MAX:
                success = False
                #self.fun_counter += 0.5
                self.fun_iter.data = int(100)
                self.msg_status_ik.data = 'Time limit'
                break

            v = - J.T.dot(solve(J.dot(J.T) + damp * eye(6), err))
            q = pin.integrate(self.rmodel,self.q_in_numpy,v*DT)

            self.q_in_numpy = q
            self.joint_state_publisher_robot(q)
            viz.display(q0)



            i += 1

            self.pub_status_ik.publish(self.msg_status_ik)
            self.pub_end_ik.publish(self.fun_iter)

        self.pub_status_ik.publish(self.msg_status_ik)
        self.pub_end_ik.publish(self.fun_iter)

        

        
        self.polytope_display = False

        q_joints_opt = q


        return q_joints_opt

    def fmin_opt(self, x0_start, pose_desired, analytical_solver: bool):
        ### Function - func
        # Initial point - x0
        # args -
        ## method - SLQSQ
        # jac = Jacobian - gradient of the


        self.initial_x0 = float64(x0_start)



        self.pos_reference = float64(pose_desired)
        print('Reference position is', self.pos_reference)

        # input('self.pos_reference')

        # Desired vertex set

        # + self.pos_reference[0,0]
        self.desired_vertices[:, 0] = self.cartesian_desired_vertices[:, 0]
        # + self.pos_reference[0,1]
        self.desired_vertices[:, 1] = self.cartesian_desired_vertices[:, 1]
        # + self.pos_reference[0,2]
        self.desired_vertices[:, 2] = self.cartesian_desired_vertices[:, 2]
        # print('self.opt_polytope_gradient_model.d_gamma_hat',self.opt_polytope_gradient_model.d_gamma_hat)

        # Bounds created from the robot angles

        self.opt_bounds = float64(self.q_bounds)
        print('q_bounds are',self.opt_bounds)


        # Constraints


        cons = ({'type': 'eq', 'fun': self.constraint_function, 'tol': 1e-4} )
               
         
        self.fun_iter.data = 0

        if analytical_solver:
            q_joints_opt = sco.minimize(fun=self.obj_function_gamma,  x0=self.initial_x0, bounds=self.opt_bounds[6:,:],
                                        jac=self.jac_func, constraints=cons, method='SLSQP',
                                        options={'disp': True, 'maxiter': 100})  # Paper maximum iterations is 3000
        else:

            q_joints_opt = sco.minimize(fun=self.obj_function_IK,  x0=self.initial_x0, bounds=self.opt_bounds,
                                        constraints=cons, tol=1e-6, method='COBYLA',
                                        options={'disp': True})

        self.fun_iter.data = 100

        self.pub_end_ik.publish(self.fun_iter)
        if q_joints_opt.success:

            self.msg_status_ik.data = 'Success'
        
        elif q_joints_opt.status == int(8):
            self.msg_status_ik.data = 'Directional search error'
        else:
            self.msg_status_ik.data = 'Time limit'            
            
        self.pub_status_ik.publish(self.msg_status_ik)
        print('q_joints_opt', q_joints_opt.x)
        self.polytope_display = False


        return q_joints_opt

    

    def obj_function_gamma(self, q_in):

        from numpy.linalg import det
        from numpy import sum

        self.q_in_numpy[6:] = q_in
        # To publish the joint states to Robot
        #self.joint_state_publisher_robot(q_in)


        pin.computeFrameJacobian(self.rmodel, self.rdata,self.q_in_numpy,33)
        pin.forwardKinematics(self.rmodel,self.rdata, self.q_in_numpy)
        pin.updateFramePlacements(self.rmodel,self.rdata)


        #pos_act1 = self.rdata.oMf[self.rmodel.getFrameId('tcp_kuka')].translation
        pos_act2 = self.rdata.oMf[self.rmodel.getFrameId('tcp_meca')].translation



        #pos_act = pos_act1 + (pos_act2 - pos_act1)*0.5
        pos_act = pos_act2

        
        pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)

        
        #J_Hess1 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_kuka'), pin.LOCAL_WORLD_ALIGNED)
        J_Hess2 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_meca'), pin.LOCAL_WORLD_ALIGNED)
        
        #J_Hess = hstack((J_Hess1[:,:6],J_Hess2[:,6:]))
        J_Hess = J_Hess2[:,6:]


        h_plus, h_plus_hat, h_minus, h_minus_hat, p_plus, p_minus, p_plus_hat, p_minus_hat, n_k, Nmatrix, Nnot = get_polytope_hyperplane(
            J_Hess, active_joints=self.active_joints, cartesian_dof_input=array([True, True, True, False, False, False]), qdot_min=self.qdot_min[6:],
            qdot_max=self.qdot_max[6:], cartesian_desired_vertices=self.desired_vertices, sigmoid_slope=self.sigmoid_slope_input)

        Gamma_minus, Gamma_plus, Gamma_total_hat, Gamma_min, Gamma_min_softmax, Gamma_min_index_hat, facet_pair_idx, hyper_plane_sign = get_capacity_margin(
            J_Hess, n_k, h_plus, h_plus_hat, h_minus, h_minus_hat,
            active_joints=self.active_joints, cartesian_dof_input=array([True, True, True, False, False, False]), qdot_min=self.qdot_min[6:],
            qdot_max=self.qdot_max[6:], cartesian_desired_vertices=self.desired_vertices, sigmoid_slope=self.sigmoid_slope_input)

        self.Gamma_min_softmax = Gamma_min_softmax

        print('Gamma now is',self.Gamma_min_softmax)



        return -1.0*self.Gamma_min_softmax

    def obj_function_IK(self, q_in):



        self.q_in_numpy = q_in
        pin.computeFrameJacobian(self.rmodel, self.rdata,self.q_in_numpy,33)
        pin.forwardKinematics(self.rmodel,self.rdata, self.q_in_numpy)
        pin.updateFramePlacements(self.rmodel,self.rdata)


        pos_act1 = self.rdata.oMf[self.rmodel.getFrameId('tcp_kuka')].translation
        pos_act2 = self.rdata.oMf[self.rmodel.getFrameId('tcp_meca')].translation



        pos_act = pos_act1 + (pos_act2 - pos_act1)*0.5

        
        pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)

        
        J_Hess1 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_kuka'), pin.LOCAL_WORLD_ALIGNED)
        J_Hess2 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_meca'), pin.LOCAL_WORLD_ALIGNED)
        
        J_Hess = hstack((J_Hess1[:,:6],J_Hess2[:,6:]))

        return_dist_error = norm(pos_act.flatten()-self.pos_reference)



        return float64(return_dist_error)

    def constraint_function(self, q_in):

        
        self.q_in_numpy[6:] = q_in


        pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)
        #print('self.q_in_numpy is',self.q_in_numpy)
        pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)
        pin.framesForwardKinematics(self.rmodel, self.rdata, self.q_in_numpy)
        #pos_act_int1 = self.rdata.oMf[self.rmodel.getFrameId('tcp_kuka')].translation
        pos_act_int2 = self.rdata.oMf[self.rmodel.getFrameId('tcp_meca')].translation
        J_Hess2 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_meca'), pin.LOCAL_WORLD_ALIGNED)
        J = J_Hess2                          
        #oMdes = pin.SE3(eye(3), array([pos_act_int1[0], pos_act_int1[1], pos_act_int1[2]]))
        oMdes = pin.SE3(eye(3), array([self.pos_reference[0], self.pos_reference[1], self.pos_reference[2]]))
        #oMdes = self.pos_reference
        OMcurr = pin.SE3(eye(3), array([pos_act_int2[0], pos_act_int2[1], pos_act_int2[2]]))

        self.joint_state_publisher_robot(self.q_in_numpy)
        #viz.display(q0)


        return -float64(pos_act_int2 - self.pos_reference)

    

    def constraint_function_Gamma(self, q_in):

        #J_Hess = array(self.pykdl_util_kin.jacobian(q_in))
        pin.computeFrameJacobian(self.rmodel, self.rdata,self.q_in_numpy,33)
        pin.forwardKinematics(self.rmodel,self.rdata, self.q_in_numpy)
        pin.updateFramePlacements(self.rmodel,self.rdata)


        #pos_act1 = self.rdata.oMf[self.rmodel.getFrameId('tcp_kuka')].translation
        pos_act2 = self.rdata.oMf[self.rmodel.getFrameId('tcp_meca')].translation



        #pos_act = pos_act1 + (pos_act2 - pos_act1)*0.5

        pos_act = pos_act2

        
        pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)

        
        #J_Hess1 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_kuka'), pin.LOCAL_WORLD_ALIGNED)
        J_Hess2 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_meca'), pin.LOCAL_WORLD_ALIGNED)
        
        #J_Hess = hstack((J_Hess1[:,:6],J_Hess2[:,6:]))
        J_Hess = J_Hess2[:,6:]

        h_plus, h_plus_hat, h_minus, h_minus_hat, p_plus, p_minus, p_plus_hat, p_minus_hat, n_k, Nmatrix, Nnot = get_polytope_hyperplane(
            J_Hess, active_joints=self.active_joints, cartesian_dof_input=array([True, True, True, False, False, False]), qdot_min=self.qdot_min[6:],
            qdot_max=self.qdot_max[6:], cartesian_desired_vertices=self.desired_vertices, sigmoid_slope=self.sigmoid_slope_input)

        Gamma_minus, Gamma_plus, Gamma_total_hat, Gamma_min, Gamma_min_softmax, Gamma_min_index_hat, facet_pair_idx, hyper_plane_sign = get_capacity_margin(
            J_Hess, n_k, h_plus, h_plus_hat, h_minus, h_minus_hat,
            active_joints=self.active_joints, cartesian_dof_input=array([True, True, True, False, False, False]), qdot_min=self.qdot_min[6:],
            qdot_max=self.qdot_max[6:], cartesian_desired_vertices=self.desired_vertices, sigmoid_slope=self.sigmoid_slope_input)

        #print('Current objective in optimization Gamma is',self.opt_polytope_model.Gamma_min_softmax)
        return float64(1.0*Gamma_min_softmax)

    def jac_func(self, q_in):

        self.fun_counter += 0.5
        self.fun_iter.data = int(self.fun_counter)
        
        pin.computeJointJacobians(self.rmodel,self.rdata, self.q_in_numpy)

        
        #J_Hess1 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_kuka'), pin.LOCAL_WORLD_ALIGNED)
        J_Hess2 = pin.getFrameJacobian(self.rmodel, self.rdata, self.rmodel.getFrameId('tcp_meca'), pin.LOCAL_WORLD_ALIGNED)
        
        #J_Hess = hstack((J_Hess1[:,:6],J_Hess2[:,6:]))
        J_Hess = J_Hess2[:,6:]

        Hess = getHessian(J_Hess)
        jac_output = mp.Array('f',zeros(shape=(self.active_joints)))


        h_plus, h_plus_hat, h_minus, h_minus_hat, p_plus, p_minus, p_plus_hat, p_minus_hat, n_k, Nmatrix, Nnot = get_polytope_hyperplane(
            J_Hess, active_joints=self.active_joints, cartesian_dof_input=array([True, True, True, False, False, False]), qdot_min=self.qdot_min[6:],
            qdot_max=self.qdot_max[6:], cartesian_desired_vertices=self.desired_vertices, sigmoid_slope=self.sigmoid_slope_input)

        Gamma_minus, Gamma_plus, Gamma_total_hat, Gamma_min, Gamma_min_softmax, Gamma_min_index_hat, facet_pair_idx, hyper_plane_sign = get_capacity_margin(
            J_Hess, n_k, h_plus, h_plus_hat, h_minus, h_minus_hat,
            active_joints=self.active_joints, cartesian_dof_input=array([True, True, True, False, False, False]), qdot_min=self.qdot_min[6:],
            qdot_max=self.qdot_max[6:], cartesian_desired_vertices=self.desired_vertices, sigmoid_slope=self.sigmoid_slope_input)



        # Create a new thread and start it
        threads = []
        for i_thread in range(self.active_joints):
            thread = mp.Process(target=Gamma_hat_gradient_dq,args=(J_Hess, Hess, n_k, Nmatrix, Nnot, h_plus_hat, h_minus_hat, p_plus_hat,\
                                        p_minus_hat, Gamma_total_hat, Gamma_min_index_hat,\
                                        self.qdot_min[6:], self.qdot_max[6:], self.desired_vertices,self.sigmoid_slope_input,i_thread,jac_output))
            thread.start()
            threads.append(thread)
        
        # now wait for them all to finish
        for thread in threads:
            thread.join()


        self.pub_end_ik.publish(self.fun_iter)
        return -float64(jac_output)


if __name__ == '__main__':
    print("Robot control start up v1 File\n")
    controller = Geomagic2KUKA()
    # controller.start()
    rospy.spin()