#!/usr/bin/env python

# general libraries  
import numpy as np 
 
# ros libraries 
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import transforms3d.axangles as t_ax    


# function provides rotation along one axis as transformation matrix
def rot(axis, theta):  
    matrix = np.eye(4)  
    if axis == 'x':
        matrix[0:3, 0:3] = t_ax.axangle2mat([1, 0, 0], theta) 
    elif axis == 'z':
        matrix[0:3, 0:3] = t_ax.axangle2mat([0, 0, 1], theta)  
    else:
        raise ValueError("Invalid axis. Must be 'x', or 'z'.") 
    matrix
    return matrix # 4 by 4 matrix  

# function provides translation as transformation matrix
def transl(translation):  
    if len(translation) != 3:
        raise ValueError("Invalid translation vector. Must have three elements.")

    matrix = np.eye(4)
    matrix[:3, 3] = translation
 
    return matrix  # 4 by 4 matrix   

# function to evaluate the kinematics of the robot 
def kinematics(theta):  #theta is a list of four angles   

    T = [np.eye(4)] 
  
    # transformation of each link with the previous link  
    T1 = rot('z', theta[0, 0]) @ transl(np.array([0.0132, 0, 0])) @ rot('x', -np.pi/2) @ transl(np.array([0, 0.108, 0]))
    T2 = transl(np.array([-0.142*np.sin(theta[1,0]), 0.142*np.cos(theta[1, 0]), 0])) 
    print('T2', T2)
    T3 = transl(np.array([0.1588*np.cos(theta[2, 0]), 0.1588*np.sin(theta[2 ,0]), 0])) @ rot('x', np.pi/2) @ transl(np.array([0.056, 0, 0]))
    T4 = rot('z', theta[3, 0]) @ transl(np.array([0, 0, 0.0722]))  

    T_l = [T1, T2, T3, T4] 
      
    for i in range(len(T_l)):    
        t = T_l[i] 
        t = np.dot(T[-1], t)        
        T.append(t)  

    return T  # list of transformation of each link wih the base link 

# function to find the jacobians
def Jacobian(theta, link): # theta is a list of four angles and link is from 1 to 4  

    print('j', theta) 
    print('j', theta[0])  
     
    J = np.array([ 
                        [-np.sin(theta[0,0]) * (0.0692 - 0.142 * np.sin(theta[1,0]) + 0.1588 * np.cos(theta[2,0])) , -0.142 * np.cos(theta[0,0]) * np.cos(theta[1,0])  ,-0.1588 * np.sin(theta[2,0]) * np.cos(theta[0,0])  ,0],
                        [np.cos(theta[0,0]) * (0.0692 - 0.142 * np.sin(theta[1,0]) + 0.1588 * np.cos(theta[2,0])) , -0.142 * np.sin(theta[0,0]) * np.cos(theta[1,0])  ,-0.1588 * np.sin(theta[2,0]) * np.sin(theta[0,0])  ,0], 
                        [0, 0.142 * np.sin(theta[1,0])  ,-0.1588 * np.cos(theta[2,0])  ,0],    
                        [ 0    , 0     ,  0   , 0 ], 
                        [ 0    , 0     ,  0   , 0 ],
                        [1    , 0     ,   0   , 1 ]])  
    print(J)
    J = J[:, :link]   
    return J   


# function to publish the end-effeector pose
def pose_EE(transformation_matrix):    # transformation_matrix is a 4 by 4 matrix   
    t = transformation_matrix[:3,-1] 
    q = tf.quaternion_from_matrix(transformation_matrix)     

    p = PoseStamped()
    p.header.frame_id = "swiftpro/manipulator_base_link" 
    p.header.stamp = rospy.Time.now()

    p.pose.position.x = t[0]
    p.pose.position.y = t[1]
    p.pose.position.z = t[2] 

    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
 
    return p 


# function to publish the goal pose 
def goal_pose(trans, angle):
    quat = quaternion_from_euler(0, 0, angle)  

    g_pose = PoseStamped()
    g_pose.header.frame_id = "swiftpro/manipulator_base_link"
    g_pose.pose.position.x = trans[0]
    g_pose.pose.position.y = trans[1]
    g_pose.pose.position.z = trans[2] 

    g_pose.pose.orientation.x = quat[0] 
    g_pose.pose.orientation.y = quat[1]
    g_pose.pose.orientation.z = quat[2]
    g_pose.pose.orientation.w = quat[3] 

    return g_pose 


# function to find the weighted DLS (i.e. to find jacobian inverse) 
def W_DLS(A, damping): 
    '''
        Function computes the damped least-squares (DLS) solution to the matrix inverse problem.

        Arguments:
        A (Numpy array): matrix to be inverted
        damping (double): damping factor

        Returns:
        (Numpy array): inversion of the input matrix
    '''  
    w = np.diag([1, 1, 1, 1])  # more weight less movement of the link   
    w_i = np.linalg.inv(w)     


    A_damped = A @ w_i @ A.T + damping**2 * np.eye(A.shape[0])  
    A_damped_inv = np.linalg.inv(A_damped) 
    A_DLS = w_i @ A.T @ A_damped_inv    
    return A_DLS # Implement the formula to compute the DLS of matrix A. 