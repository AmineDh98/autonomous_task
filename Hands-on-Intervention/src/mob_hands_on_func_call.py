#!/usr/bin/env python

# general libraries 
import time
import numpy as np  
import math  

# ros libraries 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry 
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray
import tf.transformations as tf
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from scipy.spatial.transform import Rotation as R
import transforms3d.axangles as t_ax   

# importing the functions  
from mob_hands_on_func_define import *    

class robot_move:
    def __init__(self):

        self.theta = np.array([[0, 0, 0, 0]])    
        self.state = [0, 0, 0]    

        # end-effector pose publisher only for RViz  
        self.pose_EE_pub = rospy.Publisher('pose_EE', PoseStamped, queue_size=10)   
        # joint states subscriber 
        self.joints_sub = rospy.Subscriber('/turtlebot/joint_states', JointState, self.callback)  

        self.joints_sub = rospy.Subscriber("/my_ground_truth", Float64MultiArray, self.ground_truth)  

    def ground_truth(self, gro_tru):

        # my_gro_tru.data = [float(x), float(y), float(euler[2])]
        self.x_r = gro_tru.data[0]  # x of the robot
        self.y_r = gro_tru.data[1]  # y of the robot 
        self.th = gro_tru.data[2]   # theta of the robot  
        self.state =np.array([gro_tru.data[0], gro_tru.data[1], gro_tru.data[2]])   

    def callback(self, data):    
        names = ['turtlebot/swiftpro/joint1', 'turtlebot/swiftpro/joint2', 'turtlebot/swiftpro/joint3', 'turtlebot/swiftpro/joint4']  
        if data.name == names:   

            self.theta =  np.array(data.position, ndmin=2).T     
            
            # Base kinematics 
            Tb = np.array([[math.cos(self.state[2]), -math.sin(self.state[2]), 0, self.state[0]],  
                        [math.sin(self.state[2]), math.cos(self.state[2]), 0, self.state[1]],  
                        [0,0,1,0], 
                        [0,0,0,1]])  
            
            # list of transformation of each link wih the base link, first is identity
            self.T = kinematics_tb(self.theta, Tb)           
             
            # publishing the pose of end-effector   
            pose_eef = pose_EE(self.T[-1])   
            self.pose_EE_pub.publish(pose_eef)      

if __name__ == '__main__':
    try:
        rospy.init_node('kinematics_node', anonymous=True) 
        K = robot_move()
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass