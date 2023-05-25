#!/usr/bin/env python

# general libraries 
import numpy as np  
import math 
  
# importing the functions 
from mob_hands_on_func_define import *     

class Manipulator:   
    def __init__(self, theta): 
        self.theta = theta   
        self.revolute = [True, False, True, True , True , True]  
        self.dof = len(self.revolute)
        self.r = 0.0
        self.eta = np.zeros((3,1)) 
        self.T = np.zeros((4,4))  

    '''
        Method that updates the state of the robot. 

        Arguments:
        dq (Numpy array): a column vector of joint velocities
        dt (double): sampling time  
    '''
    
    def update(self, dq, dt, state):  # dq is a 6 by 1 vector, 1st is angular velocity and 2nd is linear velocity

        self.theta += (dq[2:, 0]).reshape(-1,1) * dt  
 
        # for moving along an arc

        # if abs(dq[0,0]) < 1e-6: 
        #     R = 1e6  # Set R to a very large value to simulate a straight line   
        # else: 
        #     R = dq[1,0] / dq[0,0] 
         
        # # R = dq[1,0] / dq[0,0]  # Calculate the radius of the circle 
        # delta_theta = dq[0,0]*dt   

        # delta_x = R * (math.sin(self.eta[2,0] + delta_theta) - math.sin(self.eta[2,0]))
        # delta_y = R * (-math.cos(self.eta[2,0] + delta_theta) + math.cos(self.eta[2,0]))   
        
        # self.eta[0,0] = self.eta[0,0] +  delta_x
        # self.eta[1,0] = self.eta[1,0] +  delta_y
        # self.eta[2,0] = self.eta[2,0] +  delta_theta    

        # self.eta[2,0] = self.eta[2,0] + dq[0,0]*dt    
        # self.eta[0,0] = self.eta[0,0] + dq[1,0]*dt*math.cos(self.eta[2,0])  
        # self.eta[1,0] = self.eta[1,0] + dq[1,0]*dt*math.sin(self.eta[2,0])  

        self.eta[2,0] = state[2] 
        self.eta[0,0] = state[0]
        self.eta[1,0] = state[1] 
        
        # Base kinematics  
        Tb = np.array([[math.cos(self.eta[2,0]), -math.sin(self.eta[2,0]), 0, self.eta[0,0]],
                       [math.sin(self.eta[2,0]), math.cos(self.eta[2,0]), 0, self.eta[1,0]], 
                       [0,0,1,0], 
                       [0,0,0,1]])   
  
        # self.robot_angle = math.atan2(self.eta[1,0], self.eta[0,0]) 
        self.T = kinematics_tb(self.theta, Tb)       
  
    '''
        Method that returns the end-effector Jacobian.
    ''' 
    def getEEJacobian(self): 
        return Jacobian(self.theta, self.eta[2,0], self.eta[0,0], 6) # 4 is a link        

    ''' 
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        # self.T = kinematics_tb(self.theta, Tb)
        return self.T[-1]   

    '''
        Method that returns the position of a selected joint. 

        Argument:
        joint (integer): index of the joint 

        Returns:
        (double): position of the joint
    '''
    def getJointPos(self, joint): 
        return self.theta[joint]     
    
    '''
        Method that returns base of the robot. 
    '''

    def getBasePose(self):
        return self.eta 
    
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
        self.mobi_base = None  
 
        
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
        Method returning the mobile base position.
    ''' 
    def get_mobi_base(self):
        return self.mobi_base 
    
        '''
        Method returning the mobile base position.
    ''' 
    def get_eep(self):
        return self.eep  
 
'''
    Subclass of Task, representing the 2D position task. 
'''
## 2D position refers to the (x,y) coordinates of the end effector in a 2D plane

class Position3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link
        self.J = np.zeros((3, self.link))

        self.err = np.zeros((3,1)) 
                
    def update(self, robot):  
        self.J = robot.getEEJacobian()[0:3]          # Update task Jacobian 

        X = (robot.getEETransform()[0:3, 3]).reshape(3,1)  
        # print('x', X)  
        # print('desired', self.getDesired()) 
    
        self.err = ((self.getDesired()).reshape(3,1) - X).reshape(3,1)       # Update task error     
 
'''
    Subclass of Task, representing the 2D orientation task.   
'''

## 2D orientation refers to the angle between the end effector and the horizontal axis in the same 2D plane. It is usually measured in radians.

class Orientation3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link   


    def update(self, robot):
        
        self.J = (robot.getEEJacobian()[-1]).reshape(1,self.link)   # last row of the jacobian  
        Y = robot.getEETransform()
        orien = np.arctan2(Y[1,0], Y[0,0]) 
        self.err = self.getDesired() - orien  
        
'''
    Subclass of Task, representing the 2D configuration task. 
''' 

## 2D configuration refers to the combination of the 2D position and 2D orientation of the end effector.

class Configuration3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link
        self.J = np.zeros((4, self.link)) 
        self.config = np.zeros((4,1))  

    def update(self, robot):
        # Jacobian 
        self.J[0:3,:] = robot.getEEJacobian()[0:3] 
        self.J[-1,:] = robot.getEEJacobian()[-1].reshape(1,4)  

        transf = robot.getEETransform() 
        eep = transf[0:3,3].reshape(3,1)
        orien = np.arctan2(transf[1,0], transf[0,0])   
        self.config = np.vstack([eep, orien])  
        self.err = self.getDesired() - self.config  
        # pass # to remove


## Joint position refers to the angles between the links of the manipulator.
 
class Jointlimits3D(Task):  
    def __init__(self, name, desired, activation, link): 
        super().__init__(name, desired)
        self.activation = activation 
        self.link = link   
        # self.J = np.zeros((1,5))   
        # self.J = np.zeros((1,5)) 
        # self.err = np.zeros((1))  

    # wraps the angle between -pi to +pi 
    def wrap_angle(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi

    def update(self, robot):    

        self.J = (robot.getLinkJacobian(self.link)[5,:]).reshape(1,5) 
        # print('jac',self.J)  
        # self.err = self.getDesired() - robot.getJointPos(0)   

        # self.distance = self.getDesired() - robot.getJointPos(self.link)  
          
        # orien = robot.getJointPos(self.link)  
        link_transform = robot.get_Se_LTransform(self.link)     
        orien = np.arctan2(link_transform[1,0], link_transform[0,0]) 
        print('angle', orien)        

        if self.a == 1 and orien > self.activation[2]: 
            self.a = 0   
            self.active = False  
            self.err = 0.0   
  
        if self.a == -1 and orien < self.activation[0]:   
            self.a = 0 
            self.active = False 
            self.err = 0.0   

        if self.a== 0 and orien > self.activation[1]:   
            self.a = -1 
            self.active = True 
            self.err = -1.0   

        if self.a == 0 and orien < self.activation[3]:   
            self.a = 1 
            self.active = True   
            self.err = 1.0   

# jointlimit3D class is not tested yet 
class JointLimit3D(Task):
    def __init__(self, name, thresh, min_angle, max_angle, link):
        super().__init__(name, thresh)
        self.min_angle = min_angle 
        self.max_angle = max_angle
        # Assigning the value to the maximal and minimal possible angle
        self.ralpha =  thresh[0]
        self.rdelta = thresh[1]
        # Assigning the value to delta and alpha
        self.link = link
        self.act = 0
        # Initializing activation indicator to 0 
        # Defining elements of this subclass 

    def update(self, robot):
        self.J = robot.Jacobian(self.link)[5, :].reshape(1, robot.getDOF())
        qi = np.arctan2(robot.Kinematics()[self.link + 1][1, 0], robot.Kinematics()[self.link + 1][1, 1])
        # Initializing the angle of the specified part of the manipulator
        if self.act == 0 and qi >= (self.max_angle - self.ralpha):
            self.act = -1
        elif self.act == 0 and qi <= (self.min_angle + self.ralpha):
            self.act = 1
        elif self.act == -1 and qi <= (self.max_angle - self.rdelta):
            self.act = 0
        elif self.act == 1 and qi >= (self.min_angle + self.rdelta):
            self.act = 0
        # Implementing the activation function
        self.err = self.act

  
