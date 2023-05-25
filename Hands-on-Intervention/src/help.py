#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import time
import numpy as np 
import math 
from scipy.spatial.transform import Rotation as R
import tf
from tf.transformations import quaternion_from_euler

class Manipulator():
    
    def __init__(self, theta, theta2, theta3, theta4):
        self.revolute = [True, True , True , True]
        self.dof = len(self.revolute)
        self.theta = theta
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.goal = np.zeros((1,4))
        self.q = np.zeros(self.dof).reshape(-1, 1)
        self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.JointState_callback)
        self.pose_EE_pub = rospy.Publisher('pose_EE', PoseStamped, queue_size=10)
        # Send velocity to the manipulator 
        self.joint_velocity= rospy.Publisher("/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)
        self.goal_check = rospy.Publisher('/goal_check', PoseStamped, queue_size=10) 
        
    def JointState_callback(self,data):
        if data.name == ['swiftpro/joint1', 'swiftpro/joint2', 'swiftpro/joint3', 'swiftpro/joint4']:
            self.theta, self.theta2, self.theta3, self.theta4 = data.position 
            self.q = [[self.theta], [self.theta2], [self.theta3], [self.theta4]]
            self.update(self.q)
            goals = [[0, [0.15, 0.15, -0.15], 1], [1, [math.radians(20)]]]
            #goals = [[2, [0.15, -0.15, -0.11, math.radians(-100)], 3]]
            #goals = [[0, [0.11, 0.11, -0.03], 3], [3, [math.radians(30)], 2]]
            #goals = [[4, [math.radians(45), math.radians(60)], 1], [0, [0.15, 0.15, -0.15], 3]]
            self.Priority(goals)
            self.pose_EE()

    def pose_EE(self):
        r = R.from_matrix(self.Kinematics()[-1][:3,:3])
        q = R.as_quat(r)

        p = PoseStamped()
        p.header.frame_id = "swiftpro/manipulator_base_link"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x=self.Kinematics()[-1][0,3]
        p.pose.position.y=self.Kinematics()[-1][1,3]
        p.pose.position.z=self.Kinematics()[-1][2,3]

        p.pose.orientation.x=q[0]
        p.pose.orientation.y=q[1]
        p.pose.orientation.z=q[2]
        p.pose.orientation.w=q[3]

        self.pose_EE_pub.publish(p)

    def Kinematics(self):
        self.T = [np.eye(4)]
        self.T4 = np.array([ 
                            [np.cos(self.theta), 0, np.sin(self.theta), 0.0132 * np.cos(self.theta)],
                            [np.sin(self.theta), 0, np.cos(self.theta), 0.0132 * np.sin(self.theta)],
                            [0, -1, 0, - 0.108],
                            [0, 0, 0, 1]])
        self.T.append(self.T4)
        self.T3 = np.array([ 
                            [np.cos(self.theta), 0, -np.sin(self.theta), (0.0132 - 0.142 * np.sin(self.theta2)) * np.cos(self.theta)],
                            [np.sin(self.theta), 0, np.cos(self.theta), (0.0132 - 0.142 * np.sin(self.theta2))* np.sin(self.theta)],
                            [0, -1, 0, - 0.142 * np.cos(self.theta2) - 0.108],
                            [0, 0, 0, 1]])
        self.T.append(self.T3)
        self.T2 = np.array([ 
                            [np.cos(self.theta), -np.sin(self.theta), 0, (0.0692 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)) * np.cos(self.theta)],
                            [np.sin(self.theta), np.cos(self.theta), 0, (0.0692 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)) * np.sin(self.theta)],
                            [0, 0, 1, -0.108 - 0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3)],
                            [0, 0, 0, 1]])
        self.T.append(self.T2)
        self.T1 = np.array([ 
                            [np.cos(self.theta + self.theta4),  -np.sin(self.theta + self.theta4), 0, (0.0692 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)) * np.cos(self.theta)],
                            [np.sin(self.theta + self.theta4), np.cos(self.theta + self.theta4), 0, (0.0692 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)) * np.sin(self.theta)],
                            [0 , 0, 1, -0.0358 -  0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3)],
                            [ 0    , 0     ,   0   , 1 ]])
        self.T.append(self.T1)
        return self.T
    
    def Jacobian(self, link):
        
        self.J = np.array([ 
                            [-np.sin(self.theta) * (0.0692 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3)) , -0.142 * np.cos(self.theta) * np.cos(self.theta2)  ,-0.1588 * np.sin(self.theta3) * np.cos(self.theta)  ,0],
                            [np.cos(self.theta) * (0.0692 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3)) , -0.142 * np.sin(self.theta) * np.cos(self.theta2)  ,-0.1588 * np.sin(self.theta3) * np.sin(self.theta)  ,0],
                            [0 , 0.142 * np.sin(self.theta2)  ,-0.1588 * np.cos(self.theta3)  ,0],
                            [ 0    , 0     ,   0   , 0 ],
                            [ 0    , 0     ,   0   , 0 ],
                            [1    , 0     ,   0   , 1 ]])
        
        if link == 2:
            self.J[:, -1] = 0
        elif link == 1:
            self.J[:, -2:] = 0
        elif link == 0:
            self.J[:, -3:] = 0
        return self.J
            
    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof
    
    def update(self, q):
        self.q = q
        self.T = self.Kinematics()[-1]
    
    def DLS(self, A, damping):
    
        x = A @ np.transpose(A)
        DLS = np.transpose(A) @ np.linalg.inv(x + damping**2 * np.identity(x.shape[0]))
        return  DLS
    
    def send_velocity(self, q):

        p = Float64MultiArray()
        p.data = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        self.joint_velocity.publish(p)

    def getJointPos(self, joint):
        return self.q[joint]                                    
    
    def Priority(self, goals):
     
        tasks = []
        for goal in goals:
            if goal[0] == 0:
                tasks.append(Position3D("End-effector position", np.array(goal[1]).reshape(3,1), goal[2]))
                # end-effector

                pose_stamped = PoseStamped()
                # Set the reference frame for the marker
                pose_stamped.header.frame_id = "swiftpro/manipulator_base_link"
                pose_stamped.pose.position.x = goal[1][0]
                pose_stamped.pose.position.y = goal[1][1]
                pose_stamped.pose.position.z = goal[1][2]
                self.goal_check.publish(pose_stamped)

            elif goal[0] == 1:
                tasks.append(Orientation3D("End-effector orientation", np.array(goal[1]), 3))

                r = R.from_matrix(self.Kinematics()[-1][:3,:3])
                q = R.as_quat(r)
                yaw = goal[1][0]  # Assuming the desired angle is in radians
                quaternion = quaternion_from_euler(0, 0, yaw)

                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "swiftpro/link8B"
                pose_stamped.pose.orientation.x = quaternion[0]
                pose_stamped.pose.orientation.y = quaternion[1]
                pose_stamped.pose.orientation.z = quaternion[2]
                pose_stamped.pose.orientation.w = quaternion[3]
                self.goal_check.publish(pose_stamped)

            elif goal[0] == 2:
                tasks.append(Configuration3D("Configuration", np.array(goal[1]).reshape(4, 1), goal[2]))

                x = goal[1][0]
                y = goal[1][1]
                z = goal[1][2]
                theta = goal[1][3]

                # Create a PoseStamped message for the position and orientation
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "swiftpro/manipulator_base_link"
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = z

                # Convert desired angle (theta) to quaternion
                quaternion = quaternion_from_euler(0, 0, theta)

                pose_stamped.pose.orientation.x = quaternion[0]
                pose_stamped.pose.orientation.y = quaternion[1]
                pose_stamped.pose.orientation.z = quaternion[2]
                pose_stamped.pose.orientation.w = quaternion[3]

                # Publish the pose
                self.goal_check.publish(pose_stamped)

            elif goal[0] == 3:
                tasks.append(JointPosition3D("Joint position", np.array(goal[1]), goal[2]))

                yaw = goal[1][0]  # Assuming the desired angle is in radians
                quaternion = quaternion_from_euler(0, 0, yaw)

                pose_stamped = PoseStamped()
                if goal[2] == 0:
                    pose_stamped.header.frame_id = "swiftpro/link6"
                elif goal[2] == 1:
                    pose_stamped.header.frame_id = "swiftpro/link3"
                elif goal[2] == 2:
                    pose_stamped.header.frame_id = "swiftpro/link8B"
                pose_stamped.pose.orientation.x = quaternion[0]
                pose_stamped.pose.orientation.y = quaternion[1]
                pose_stamped.pose.orientation.z = quaternion[2]
                pose_stamped.pose.orientation.w = quaternion[3]
                self.goal_check.publish(pose_stamped)

            elif goal[0] == 4:
                tasks.append(JointLimit3D("Joint position", np.array([0.04, 0.08]), np.array(goal[1][0]), np.array(goal[1][1]), goal[2]))

        dof = self.getDOF()
        P = np.eye(dof)
        dq = np.zeros((dof, 1))
        for i in range(len(tasks)):

            tasks[i].update(self)
            print("tasks:",tasks[i].getJacobian())
            print("P:",P)
            Jbar = tasks[i].getJacobian() @ P
            dq = dq + self.DLS(Jbar, 0.1) @ (tasks[i].getError() - (tasks[i].getJacobian() @ dq))
            P = P - np.linalg.pinv(Jbar) @ Jbar
        self.send_velocity(dq)
        self.update(dq)
        
class Task:
    
    def __init__(self, name, desired):
        
        self.name = name 
        self.sigma_d = desired 
        self.act = 0
    
    def update(self, robot):
        pass

    def isActive(self):
        return True 

    def setDesired(self, value):
        self.sigma_d = value

    def getDesired(self):
        return self.sigma_d

    def getJacobian(self):
        return self.J

    def getError(self):
        return self.err
    

class Position3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link 
               
    def update(self, robot):
        self.J = robot.Jacobian(self.link)[0:3, :]
        self.err = (self.getDesired() - robot.Kinematics()[self.link + 1][0:3, 3].reshape(3, 1)).reshape(3, 1)
        

class Orientation3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link 
          
    def update(self, robot):
        self.J = robot.Jacobian(3)[-1, :].reshape(1, 4)
        angle = np.arctan2(robot.Kinematics()[-1][1,0], robot.Kinematics()[-1][0,0])
        self.err = self.sigma_d - np.array([[angle]])
       
class Configuration3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link
        
    def update(self, robot):
        a = robot.Jacobian(self.link)[0:3, :]
        b = robot.Jacobian(3)[-1, :].reshape(1, 4)
        self.J = np.concatenate([a, b])
        s1 = robot.Kinematics()[self.link + 1][0:3, 3].reshape(3, 1)
        s2 = np.arctan2(robot.Kinematics()[self.link + 1][1,0], robot.Kinematics()[self.link + 1][0,0])
        sigma = np.vstack([s1, s2])
        self.err = self.sigma_d - sigma 

class JointPosition3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link
        
    def update(self, robot):
        self.J = np.zeros((1, robot.getDOF()))
        self.J[0, self.link] = 1
        sigma = robot.getJointPos(self.link)
        self.err = self.sigma_d - sigma

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
      
if __name__ == '__main__':
    try:
        rospy.init_node('kinematics_node', anonymous=True)
        a = Manipulator(0, 0, 0, 0)   
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
