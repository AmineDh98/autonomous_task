#!/usr/bin/python

import numpy as np
import rospy
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from std_msgs.msg import Float64MultiArray  
import math

def goal_set():    
    rate = rospy.Rate(100)   
    move = Float64MultiArray() 
    while not rospy.is_shutdown():      
        goal = rospy.get_param('goal')
        g_x, g_y, g_z, g_theta = goal[0], goal[1], goal[2], goal[3]  
        move.data = [g_x, g_y, g_z, g_theta]           
        pub.publish(move) 
        rate.sleep()    

   
if __name__ == '__main__':  

    rospy.init_node("goal_set")   
    rospy.set_param('goal', [0.3, 0.3, -0.3, math.radians(0)])         
    pub = rospy.Publisher('/goal_set', Float64MultiArray, queue_size=10)   
    goal_set()    
    rospy.spin()      