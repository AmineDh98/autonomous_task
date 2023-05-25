#!/usr/bin/env python

# general libraries 
import numpy as np 

# importing the functions 
from hands_on_func_define import pose_EE, kinematics, Jacobian, W_DLS    

class Manipulator:   
    def __init__(self, theta):
        self.theta = theta   
        self.revolute = [True, True , True , True]
        self.dof = len(self.revolute)
        
    '''
        Method that updates the state of the robot.

        Arguments:
        dq (Numpy array): a column vector of joint velocities
        dt (double): sampling time
    '''
    def update(self, dq, dt):

        self.theta += dq * dt 
        self.T = kinematics(self.theta)   
 
    '''
        Method that returns the end-effector Jacobian.
    '''
    def getEEJacobian(self): 
        return Jacobian(self.theta, 4) # 4 is a link   

    '''
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        self.T = kinematics(self.theta) 
        return self.T[-1]  

    '''
        Method that returns the position of a selected joint. 

        Argument:
        joint (integer): index of the joint 

        Returns:
        (double): position of the joint
    '''
    def getJointPos(self, joint): 
        return self.q[joint] 

    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof
    
    '''
        Method that returns transformation of a selected link. 
    '''
    def get_Se_LTransform(self, link):
        return self.T[link] 
 
'''
    Base class representing an abstract Task.
'''
class Task:
    '''
        Constructor.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)
    '''
    def __init__(self, name, desired):
        self.name = name        # task title
        self.sigma_d = desired  # desired sigma 
 
        
    '''
        Method updating the task variables (abstract).

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def update(self, robot):
        pass

    ''' 
        Method setting the desired sigma.

        Arguments:
        value(Numpy array): value of the desired sigma (goal)
    '''
    def setDesired(self, value):
        self.sigma_d = value

    '''
        Method returning the desired sigma.
    '''
    def getDesired(self):
        return self.sigma_d

    '''
        Method returning the task Jacobian.
    '''
    def getJacobian(self):
        return self.J

    '''
        Method returning the task error (tilde sigma).
    '''    
    def getError(self):
        return self.err 
    
 
'''
    Subclass of Task, representing the 2D position task. 
'''
## 2D position refers to the (x,y) coordinates of the end effector in a 2D plane

class Position3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link
                
    def update(self, robot):  
        self.J = robot.getEEJacobian()[0:3]          # Update task Jacobian
        X = robot.getEETransform()    
        self.err = (self.getDesired() - X[0:3,3].reshape(3,1)).reshape(3,1)       # Update task error 

'''
    Subclass of Task, representing the 2D orientation task.
'''

## 2D orientation refers to the angle between the end effector and the horizontal axis in the same 2D plane. It is usually measured in radians.

class Orientation3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link  


    def update(self, robot):
        
        self.J = (robot.getEEJacobian()[-1]).reshape(1,4)   # last row of the jacobian 
        Y = robot.getEETransform()
        orien = np.arctan2(Y[1,0], Y[0,0]) 
        self.err = self.getDesired() - orien  
        
'''
    Subclass of Task, representing the 2D configuration task. 
'''

## 2D configuration refers to the combination of the 2D position and 2D orientation of the end effector.

# class Configuration2D(Task):
#     def __init__(self, name, desired):
#         super().__init__(name, desired)  
#         # print(self.getDesired) 
#         #self.J = # Initialize with proper dimensions
#         #self.err = # Initialize with proper dimensions
#         self.J = np.array([[0.0, 0.0, 0.0],
#                            [0.0, 0.0, 0.0],
#                            [1.0, 1.0, 1.0]])
#         self.err = np.array([[0.0],
#                              [0.0], 
#                              [0.0]])
#         self.config = np.zeros((3,1))
        
#     def update(self, robot):
#         print(self.sigma_d)  
#         #self.J = # Update task Jacobian
#         #self.err = # Update task error
#         self.J[0:2,:] = robot.getEEJacobian()
#         Z = robot.getEETransform() 
#         eep = Z[0:2,3].reshape(2,1)
#         # print(eep)
#         orien = np.arctan2(Z[1,0], Z[0,0])
#         self.config[0,0] = eep[0]
#         self.config[1,0] = eep[1] 
#         self.config[2,0] = orien  
#         # print(self.getDesired()) 
#         # de = np.array([[0.5], [1.0], [0.0]])
#         # self.err = de - self.config  
#         self.err = self.getDesired() - self.config
#         # pass # to remove

# ''' 
#     Subclass of Task, representing the joint position task.
# '''

# ## Joint position refers to the angles between the links of the manipulator. 

# class JointPosition(Task): 
#     def __init__(self, name, desired): 
#         super().__init__(name, desired)
#         #self.J = # Initialize with proper dimensions
#         #self.err = # Initialize with proper dimensions
#         self.J = np.array([[1.0, 0.0, 0.0]]) 
#         self.err = np.zeros(1)

#     def update(self, robot):
#         #self.J = # Update task Jacobian  
#         #self.err = # Update task error 
#         self.J = np.array([[1.0, 0.0, 0.0]])  

#         self.err = self.getDesired() - robot.getJointPos(0)
  