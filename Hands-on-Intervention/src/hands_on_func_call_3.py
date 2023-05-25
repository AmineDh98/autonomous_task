#!/usr/bin/env python

# general libraries 
import numpy as np 
import math  
 
# ros libraries 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Float64MultiArray
import tf.transformations as tf
  
# importing the functions 
from hands_on_func_define import * 
from  hands_on_func_call_2 import *
 
class robot_model:    
    def __init__(self): 
            
        self.revolute = [True, True , True , True] 
        self.dof = len(self.revolute)
             
        # desired goal  
        self.goal = [[0, [0.15, 0.2, -0.11]], [3, [math.radians(90)]]]    
        self.g_pose = goal_pose(self.goal[0][1], np.array(self.goal[1][1]))          

        # Task hierarchy definition 
        self.tasks = [ 
                    Position3D("End-effector position", np.array(self.goal[0][1]).reshape(3,1).reshape(3,1), 4),  
                    Orientation3D("end effector orientation", np.array(self.goal[1][1]), 4)       
                ]  

        # pose publishers only for RViz   
        self.pose_EE_pub = rospy.Publisher('pose_EE', PoseStamped, queue_size=10)  
        self.goal_check = rospy.Publisher('/goal_check', PoseStamped, queue_size=10)

        # Velocity publisher 
        self.joint_velocity= rospy.Publisher("/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)
        
        # joint states subscriber
        self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.JointState_callback) 

    # function to publish the velocities of the joints  
    def send_velocity(self, q):   
        p = Float64MultiArray() 
        p.data = [float(q[0,0]), float(q[1,0]), float(q[2,0]), float(q[3,0])] 
        self.joint_velocity.publish(p) 

    def JointState_callback(self, data):   
        self.goal_check.publish(self.g_pose)     
        names = ['swiftpro/joint1', 'swiftpro/joint2', 'swiftpro/joint3', 'swiftpro/joint4']
        if data.name == names: 

            self.theta = np.array(data.position , ndmin=2).T    
            self.robot = Manipulator(self.theta)          
 
            dt = 0.05  
            P = np.eye(self.dof) 
            dq = np.zeros((self.dof, 1)) 

            for i in range(len(self.tasks)):

                self.tasks[i].update(self.robot)  
                err = self.tasks[i].getError() 
                
                # Compute augmented Jacobian
                J =   self.tasks[i].getJacobian()  
                J_bar = J @ P
                
                # Inverse jacobian
                J_DLS = W_DLS(J_bar, 0.1)
                J_pinv = np.linalg.pinv(J_bar) 

                # Compute task velocity 
                dq = dq + J_DLS @ (err - J @ dq) 

                # Update null-space projector 
                P = P - (J_pinv @ J_bar) 
            self.send_velocity(dq)   
            self.robot.update(dq, dt)     
            
            # publishing the pose of end-effector  
            pose_eef = pose_EE(self.robot.getEETransform())    
            self.pose_EE_pub.publish(pose_eef) 

 
if __name__ == '__main__': 
    try:
        rospy.init_node('kinematics_node', anonymous=True) 
        a = robot_model()       
        rospy.spin() 

    except rospy.ROSInterruptException:
        pass