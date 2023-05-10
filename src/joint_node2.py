#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
class kinematics:
    def __init__(self):

        self.theta = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta4 = 0
        self.T1 = np.eye(4)
        self.T2 = np.eye(4)
        self.T3 = np.eye(4)
        self.T4 = np.eye(4)
        self.T = np.eye(4)
        self.transformation_mat = np.eye(4)
        #self.pub = rospy.Publisher('/turtlebot/swiftpro/joint_velocity_controller/command', Float64MultiArray, queue_size=10)
        self.marker_EE_pub = rospy.Publisher('position_EE', Marker, queue_size=10)   
        self.pose_EE_pub = rospy.Publisher('pose_EE', PoseStamped, queue_size=10)   
        self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.callback)

    def rot(self, axis, theta):
        if axis == 'x':
            matrix = np.array([[1, 0, 0, 0],
                            [0, round(np.cos(theta), 2), -round(np.sin(theta), 2), 0],
                            [0, round(np.sin(theta), 2), round(np.cos(theta), 2), 0],
                            [0, 0, 0, 1]])
        elif axis == 'y':
            matrix = np.array([[round(np.cos(theta), 2), 0, round(np.sin(theta), 2), 0],
                            [0, 1, 0, 0],
                            [-round(np.sin(theta), 2), 0, round(np.cos(theta), 2), 0]
                            , [0, 0, 0, 1]])
        elif axis == 'z':
            matrix = np.array([[round(np.cos(theta), 2), -round(np.sin(theta), 2), 0, 0],
                            [round(np.sin(theta), 2), round(np.cos(theta), 2), 0, 0],
                            [0, 0, 1, 0], 
                            [0, 0, 0, 1]])
        else:
            raise ValueError("Invalid axis. Must be 'x', 'y', or 'z'.")
        matrix
        return matrix

    def translation_matrix(self, translation):
        if len(translation) != 3:
            raise ValueError("Invalid translation vector. Must have three elements.")

        matrix = np.eye(4)
        matrix[:3, 3] = translation

        return matrix

    def callback(self, data):
        names = ['swiftpro/joint1', 'swiftpro/joint2', 'swiftpro/joint3', 'swiftpro/joint4']
        if data.name == names:
            # print(data.position)

            self.theta, self.theta2, self.theta3, self.theta4 = data.position
            self.T1 = self.rot('z', self.theta)@self.translation_matrix(np.array([0.0132, 0, 0]))@self.rot('x', -np.pi/2)@self.translation_matrix(np.array([0, 0.108, 0]))
            self.T2 = self.translation_matrix(np.array([-0.142*np.sin(self.theta2), 0.142*np.cos(self.theta2), 0]))
            self.T3 = self.translation_matrix(np.array([0.1588*np.cos(self.theta3), 0.1588*np.sin(self.theta3), 0]))@self.rot('x', np.pi/2)@self.translation_matrix(np.array([0.056, 0, 0]))
            
            #self.T = self.rot('z', self.theta2)@self.translation_matrix(np.array([0, 0.142, 0]))@self.rot('z', -self.theta2)@self.rot('z', self.theta3)@self.translation_matrix(np.array([0.1588, 0, 0]))@self.rot('z', -self.theta3)@self.rot('x', np.pi/2)@self.translation_matrix(np.array([0.056, 0, 0]))
            
            self.T4 = self.rot('z', self.theta4)@self.translation_matrix(np.array([0, 0, 0.0722]))
            self.transformation_mat = self.T1@self.T2@self.T3@self.T4
            #self.transformation_mat = self.T1@self.T@self.T4
            position_EE = self.transformation_mat[:, -1]
            print("tranformation matrix:", position_EE)
            self.marker_EE(position_EE)
            self.pose_EE()

    def pose_EE(self):
        t = self.transformation_mat[:3,-1]
        r = R.from_matrix(self.transformation_mat[:3,:3])
        q = R.as_quat(r)

        p = PoseStamped()
        p.header.frame_id = "swiftpro/manipulator_base_link"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x=self.transformation_mat[0,3]
        p.pose.position.y=self.transformation_mat[1,3]
        p.pose.position.z=self.transformation_mat[2,3]

        p.pose.orientation.x=q[0]
        p.pose.orientation.y=q[1]
        p.pose.orientation.z=q[2]
        p.pose.orientation.w=q[3]

        self.pose_EE_pub.publish(p)

    def marker_EE(self, position_EE):
        marker = Marker()
        marker.header.frame_id = "swiftpro/manipulator_base_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = position_EE[0]
        marker.pose.position.y = position_EE[1]
        marker.pose.position.z = position_EE[2]
        
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.g = 0.5
        marker.color.r = 0.5
        marker.color.a = 0.9

        self.marker_EE_pub.publish(marker)



if __name__ == '__main__':
    try:
        rospy.init_node('kinematics_node', anonymous=True)
        K = kinematics()
        #K.kinematics_Publisher()
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


