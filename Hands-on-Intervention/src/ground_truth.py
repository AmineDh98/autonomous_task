#!/usr/bin/python
 
import numpy as np
import math 
import rospy  
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState
from sensor_msgs.msg import NavSatFix 
from tf.broadcaster import TransformBroadcaster 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pymap3d import geodetic2ned
from visualization_msgs.msg import Marker, MarkerArray 
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float64MultiArray 

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped   
    
class DifferentialDrive:
    def __init__(self) -> None: 

        # robot constants 
        self.wheel_radius = 0.035 # meters      
        self.wheel_base_distance = 0.23 # meters    
 
        # initial pose of the robot  
        self.Xk = np.zeros([3,1])          
        
        # velocity and angular velocity of the robot
        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.left_wheel_received = False
 
        self.last_time = rospy.Time.now()

        # covariance details
        self.Pk = np.array([[0.04, 0, 0],    # 2,2 matrix  
                            [0, 0.04, 0], 
                            [0, 0, 0.04]])       # 3,3 matrix with diagonal 0.04            
        
        self.Qk = np.array([[0.2**2, 0],    # 2,2 matrix   
                             [0, 0.2**2]])     

        self.Rk = np.array([[2.0, 0],    # 2,2 matrix   
                             [0, 2.0]])   

  
        # joint state subscriber  
        self.js_sub = rospy.Subscriber("/turtlebot/stonefish_simulator/ground_truth_odometry", Odometry, self.joint_state_callback)    
        # odom publisher 

        # Joint Velocity publisher 
        self.gro_tru= rospy.Publisher("/my_ground_truth", Float64MultiArray, queue_size=10) 
        self.tf_br = TransformBroadcaster()
    
    def joint_state_callback(self, msg): 

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)  
        
        euler = euler_from_quaternion(quaternion)   
        print('odom', x, y, euler[2])         

        my_gro_tru = Float64MultiArray() 
        # converting linear and angular velocities to left and right wheel velocities  

        my_gro_tru.data = [float(x), float(y), float(euler[2])] # publishing for simulation and odometry(for RViz)    
        self.gro_tru.publish(my_gro_tru)    
        
        parent_frame_id = "world_ned"  
        child_frame_id = "turtlebot/kobuki/base_footprint"  
        self.tf_br.sendTransform((x, y, 0.0), quaternion, rospy.Time.now(), child_frame_id, parent_frame_id) 
             
 
if __name__ == '__main__':  

    rospy.init_node("differential_drive2")  
    robot = DifferentialDrive() 
    rospy.spin() 