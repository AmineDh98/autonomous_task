#!/usr/bin/python

import numpy as np
import rospy
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64MultiArray  

def vel_move():   
    rate = rospy.Rate(100)   
    move = Float64MultiArray() 
    while not rospy.is_shutdown():  
        wl = rospy.get_param('left_wheel')   
        wr = rospy.get_param('right_wheel')   
        move.data = [wl, wr]         
        # rospy.loginfo(move)    
        pub.publish(move) 
        rate.sleep()   
   
if __name__ == '__main__': 

    rospy.init_node("velocity_command")  
    rospy.set_param('left_wheel', 3.0)                  
    rospy.set_param('right_wheel', 3.0)                                         
    # wl = rospy.get_param('left_wheel')       
    # wr = rospy.get_param('right_wheel') 
    pub = rospy.Publisher('/kobuki/commands/wheel_velocities', Float64MultiArray, queue_size=10) 
    vel_move()   
    rospy.spin()
 





# for publishing
    # def publish_odom(self, Xk, Pk, current_time, v, w): ## Xk, Pk from mean and rest is same 
    #     self.Xk = Xk
    #     self.Pk = Pk 
 
    #     q = quaternion_from_euler(0, 0, self.Xk[2,0]) 

    #     odom = Odometry()
    #     odom.header.stamp = current_time 
    #     odom.header.frame_id = "world"   
    #     odom.child_frame_id = "kobuki/base_footprint"

    #     odom.pose.pose.position.x =  self.Xk[0,0] 
    #     odom.pose.pose.position.y =  self.Xk[1,0]
    #     odom.pose.covariance = [self.Pk[0,0], self.Pk[0,1], 0.0, 0.0, 0.0, self.Pk[0,2],
    #                             self.Pk[1,0], self.Pk[1,1], 0.0, 0.0, 0.0, self.Pk[1,2],  
    #                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
    #                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
    #                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    #                             self.Pk[2,0], self.Pk[2,1], 0.0, 0.0, 0.0, self.Pk[2,2]]             

    #     # self.Pk[0,0],  self.Pk[1,1]   
    #     odom.pose.pose.orientation.x = q[0]
    #     odom.pose.pose.orientation.y = q[1]
    #     odom.pose.pose.orientation.z = q[2] 
    #     odom.pose.pose.orientation.w = q[3]    

        

    #     odom.twist.twist.linear.x = v 
    #     odom.twist.twist.angular.z = w
    #     return self.Xk, odom, q  

    # def observation(self, Pk, n, e): 
    #     self.n = n
    #     self.e = e 
    #     noise = 0.02   
    #     self.Pk = Pk
    #     Vkn = np.array([[noise],[noise]])  
    #     Hk = np.array([[1, 0, 0],    # 2,3 matrix   
    #                     [0, 1, 0]])     
    #     # print(self.n,self.e,self.d)  
    #     Vk = np.eye(2)   
    #     o_Xk = np.array([[self.n],[self.e], [0.0]])  # position in x and y with gps   
        
    #     Zk = np.dot(Hk,o_Xk)  # without noise  2,1
    #     Zkn = Zk + Vkn       # with noise    2,1 
    #     before_inv = np.dot(np.dot(Hk, self.Pk),Hk.T) + np.dot(np.dot(Vk, self.Rk),Vk.T)
    #     # print(before_inv) 
    #     inv = np.linalg.inv(before_inv)  
    #     # print(inv)  
    #     K_gain = np.dot(np.dot(self.Pk, Hk.T), inv) 
    #     print('pk',self.Pk)
    #     print('Zk',Zk)  
    #     print('Zkn',Zkn)   
    #     print('K_gain',K_gain)  
    #     print('Hk',Hk )  
    #     return Zk, Zkn, K_gain, Hk 
     
    # def update(self, Zk, Zkn, K_gain, Hk, Xk, Pk): 
    #     self.Xk = Xk 
    #     self.Pk = Pk
    #     I = np.eye(3) 
    #     self.Xk = self.Xk + K_gain @ (Zkn - Zk) 
    #     self.Pk = ((I - (K_gain @ Hk))@ self.Pk.T) 
    #     print('new_xk',self.Xk)  
    #     print('new_pk', self.Pk) 
    #     return self.Xk, self.Pk 

    





