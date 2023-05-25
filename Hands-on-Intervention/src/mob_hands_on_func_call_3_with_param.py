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
from nav_msgs.msg import Odometry
import tf.transformations as tf 
  
# importing the functions 
from mob_hands_on_func_define import * 
from  mob_hands_on_func_call_2 import *  
 
class robot_model:    
    def __init__(self): 
 
        self.wheel_base_distance = 0.23 
        self.wheel_radius =  0.035  
        self.state = [0.0, 0.0, 0.0] 
        self.dt = 0.0 
        
        # pose publishers only for RViz   
        self.pose_EE_pub = rospy.Publisher('pose_EE', PoseStamped, queue_size=10)   
        self.goal_check = rospy.Publisher('/goal_check', PoseStamped, queue_size=10) 

        # 4 joint Velocity publisher 
        self.joint_velocity= rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10) 
        # Wheel Velocity publisher, left and right
        self.wheel_velocity= rospy.Publisher("/turtlebot/kobuki/commands/wheel_velocities", Float64MultiArray, queue_size=10) # Sending just velocity of float type
        # Those 2 publishers are doing the same thing just the message they are publishing is of different type
        self.J_wheel_velocity= rospy.Publisher("/velocities", JointState, queue_size=10) # Sending velocity as well as the current time used for computing delta t in odometry

        # self.wheel_velocity= rospy.Publisher("/turtlebot/kobuki/commands/wheel_velocities", Float64MultiArray, queue_size=10)          
         
        # joint states subscriber  
        self.goal_sub = rospy.Subscriber('/goal_set', Float64MultiArray,  self.goals_set) 
        self.joints_sub = rospy.Subscriber('/turtlebot/joint_states', JointState, self.JointState_callback)   

        # self.joints_sub = rospy.Subscriber("/my_ground_truth", Float64MultiArray, self.ground_truth) 
        self.joints_sub = rospy.Subscriber("kobuki/odom", Odometry, self.ground_truth)  
           
    
    def goals_set(self, goal_msg):
        # desired goal  
        self.goal = [[0, [goal_msg.data[0], goal_msg.data[1], goal_msg.data[2]]], [3, [goal_msg.data[3]]]]                    
        self.g_pose = goal_pose(self.goal[0][1], np.array(self.goal[1][1]))    
        self.goal_check.publish(self.g_pose) 
        self.thresholds = np.array([0.04, 0.06])
        self.max_angle = 0.5
        self.min_angle = -0.5 
        # Task hierarchy definition         
        self.tasks = [  
                    #Configuration3D("Configuration", np.array([np.hstack((self.goal[0][1],self.goal[1][1]))]).reshape(4, 1), 4),    
                    #OrientationBase("Base orientation", np.array(self.goal[1][1]), 1),
                    #JointLimit3D("Joint limit", self.thresholds, self.min_angle, self.max_angle , 3),
                    Position3D("End-effector position", np.array(self.goal[0][1]).reshape(3,1).reshape(3,1), 6),    
                    #OrientationBase("Base orientation", np.array(self.goal[1][1]), 1)
                    #Orientation3D("end effector orientation", np.array(self.goal[1][1]), 6)
                    #JointLimit3D("Joint limit", self.thresholds, self.min_angle, self.max_angle , 4), 
                    #JointPosition3D("Position", np.array(self.goal[1][1]), 3)             
                ]      

        # self.joints_sub = rospy.Subscriber("kobuki/odom", Odometry, self.ground_truth)  
 
    def ground_truth(self, gro_tru):

        # my_gro_tru.data = [float(x), float(y), float(euler[2])]
        # self.x_r = gro_tru.data[0]  # x of the robot 
        # self.y_r = gro_tru.data[1]  # y of the robot 
        # self.th = gro_tru.data[2]   # theta of the robot  
        # self.state =np.array([gro_tru.data[0], gro_tru.data[1], gro_tru.data[2]])    

        # with odometry
        self.dt = gro_tru.twist.twist.linear.x 
        quate = (gro_tru.pose.pose.orientation.x, gro_tru.pose.pose.orientation.y, gro_tru.pose.pose.orientation.z, gro_tru.pose.pose.orientation.w)
        eule = euler_from_quaternion(quate)
        self.state = np.array([gro_tru.pose.pose.position.x, gro_tru.pose.pose.position.y, eule[2]])       


    # function to publish the velocities of the joints   
    def send_velocity(self, q):   
        # publishing only the joint angles
        p = Float64MultiArray() 
        p.data = [float(q[2,0]), float(q[3,0]), float(q[4,0]), float(q[5,0])]  
        self.joint_velocity.publish(p) 

        
        # converting linear and angular velocities to left and right wheel velocities  
        w = q[0,0] 
        v = q[1,0] 
        v_r = (2 * v + w * self.wheel_base_distance) / (2 * self.wheel_radius)
        v_l = (2 * v - w * self.wheel_base_distance) / (2 * self.wheel_radius)  

        # publishing for simulation and odometry(for RViz)    
        m = Float64MultiArray()  
        m.data = [float(v_r), float(v_l)]  
        self.wheel_velocity.publish(m) 

        # publishing for oDometry  (in different format) 
        j_v = JointState()
        j_v.header.stamp = rospy.Time.now()   
        j_v.velocity = [float(v_r), float(v_l)] 
        self.J_wheel_velocity.publish(j_v)  

    def JointState_callback(self, data):    
             
        names = ['turtlebot/swiftpro/joint1', 'turtlebot/swiftpro/joint2', 'turtlebot/swiftpro/joint3', 'turtlebot/swiftpro/joint4'] 
        if data.name == names:  

            # self.theta = [4 by 1 vector] containing only the joints of the manipulator 
            self.theta =  np.array(data.position, ndmin=2).T   
            self.robot = Manipulator(self.theta)           
  
            # dt = 0.000005 
            dt = self.dt    
            P = np.eye(self.robot.getDOF())   
            # dq is a 6 by 1 vector [1st is angular velocity, 2nd linear velocity of robot and rest are four manipulator joint velocities]
            dq = np.zeros((self.robot.getDOF(), 1))   
            self.robot.update(dq, dt, self.state)   
 
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

                # Controlling the velocities of the joints (limiting the velocity of the robot base) 
                # Controlling the velocities of the joints
                # p_v and n_v are the upper and lower limits of all joints 
                p_v = 0.3 
                n_v = -0.3  
                for m in range(len(dq)):
                    if dq[m] < n_v:
                        dq = scale(dq, n_v, m)
                    if dq[m] > p_v:
                        dq = scale(dq, p_v, m)   
                dq = dq.reshape(6,1) 
                
                print(dq) 
 
            self.send_velocity(dq)    
            # publishing the pose of end-effector   
            pose_eef = pose_EE(self.robot.getEETransform())     
            self.pose_EE_pub.publish(pose_eef)   

            # updating all the parameters of the robot  
            self.robot.update(dq, dt, self.state)      
            

if __name__ == '__main__': 
    try:
        rospy.init_node('kinematics_node', anonymous=True)  
        a = robot_model()   

        rospy.spin() 

    except rospy.ROSInterruptException:
        pass
