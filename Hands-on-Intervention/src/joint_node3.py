#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray
import tf.transformations as tf
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class kinematics:
    def __init__(self):

        self.theta = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta4 = 0
        self.current_pose = np.zeros(1, 3)
        self.current_pose = self.current_pose[0]
        self.T1 = np.eye(4)
        self.T2 = np.eye(4)
        self.T3 = np.eye(4)
        self.T4 = np.eye(4)
        self.T5 = np.eye(4)
        self.T = np.eye(4)
        self.transformation_mat = np.eye(4)
        self.marker_EE_pub = rospy.Publisher('position_EE', Marker, queue_size=10)   
        self.pose_EE_pub = rospy.Publisher('pose_EE', PoseStamped, queue_size=10)   
        self.cmd_pub = rospy.Publisher('cmd_pub', Float64MultiArray, queue_size=10)
        self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.callback)
        self.odom_sub = rospy.Subscriber('turtlebot/odom', Odometry, self.odometry_callback)

    def odometry_callback(self, odom):

        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
        self.current_v = odom.twist.twist.linear.x
        self.current_w = odom.twist.twist.angular.z


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

            self.theta, self.theta2, self.theta3, self.theta4 = data.position
            self.T1 = self.rot('z', self.theta)@self.translation_matrix(np.array([0.0132, 0, 0]))@self.rot('x', -np.pi/2)@self.translation_matrix(np.array([0, 0.108, 0]))
            self.T2 = self.translation_matrix(np.array([-0.142*np.sin(self.theta2), 0.142*np.cos(self.theta2), 0]))
            self.T3 = self.translation_matrix(np.array([0.1588*np.cos(self.theta3), 0.1588*np.sin(self.theta3), 0]))@self.rot('x', np.pi/2)@self.translation_matrix(np.array([0.056, 0, 0]))
            #self.T2 = self.rot('z', self.theta2)@self.translation_matrix(np.array([0, 0.142, 0]))@self.rot('z', -np.pi/2)@self.rot('z', -self.theta2)
            #self.T3 = self.rot('z', self.theta3)@self.translation_matrix(np.array([0, 0.1588, 0]))@self.rot('z', -self.theta3)@self.translation_matrix(np.array([0, 0.0566, 0]))@self.rot('z', np.pi/2)@self.rot('x', np.pi/2)
            
            self.T4 = self.rot('z', self.theta4)@self.translation_matrix(np.array([0, 0, 0.0722]))
            self.transformation_mat = self.T1@self.T2@self.T3@self.T4
            position_EE = self.transformation_mat[:, -1]
            print("tranformation matrix:", position_EE)
            self.x = (0.0692 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)) * np.cos(self.theta)
            self.y = (0.0692 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)) * np.sin(self.theta)
            self.z = -0.0358 -  0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3)
            self.marker_EE(position_EE)
            self.pose_EE()

    def _send_commnd_(self, v, w):
        # cmd = Twist()
        # cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        # cmd.linear.y = 0
        # cmd.linear.z = 0
        # cmd.angular.x = 0
        # cmd.angular.y = 0
        # cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        # self.cmd_pub.publish(cmd)
        
        rate = rospy.Rate(100)   
        move = Float64MultiArray() 
         
        v_r = (2 * v + w * self.wheel_base_distance) / (2 * self.wheel_radius)
        v_l = (2 * v - w * self.wheel_base_distance) / (2 * self.wheel_radius)  
        move.data = [v_r, v_l]            
        self.cmd_pub.publish(move)

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
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    
    
    
    ########################################

#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray
import tf.transformations as tf
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def shutdown_hook():
    # Unregister the marker_EE_pub topic
    K.marker_EE_pub.unregister()

class kinematics:
    def __init__(self):

        self.theta = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta4 = 0
        self.current_pose = np.zeros((1, 3))
        self.current_pose = self.current_pose[0]
        self.T1 = np.eye(4)
        self.T2 = np.eye(4)
        self.T3 = np.eye(4)
        self.T4 = np.eye(4)
        self.T5 = np.eye(4)
        self.T = np.eye(4)
        self.wheel_radius = 0.035
        self.wheel_base_distance = 0.230
        self.x_robot = 0.0
        self.y_robot  = 0.0
        self.th_robot  = 0.0
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.left_wheel_received = False
        self.transformation_mat = np.eye(4)
        self.T_robot_manipulator = np.eye(4)
        self.marker_EE_pub = rospy.Publisher('position_EE', Marker, queue_size=10)   
        self.pose_EE_pub = rospy.Publisher('pose_EE', PoseStamped, queue_size=10)   
        self.cmd_pub = rospy.Publisher('cmd_pub', Float64MultiArray, queue_size=10)
        self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.callback)
        self.odom_sub = rospy.Subscriber('turtlebot/odom', Odometry, self.odom_callback)
    
    def __del__(self):
        self.marker_EE_pub.unregister()

    # def odom_callback(self, odom):

    #     _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
    #                                                           odom.pose.pose.orientation.y,
    #                                                           odom.pose.pose.orientation.z,
    #                                                           odom.pose.pose.orientation.w])
    #     self.current_p = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    #     self.current_v = odom.twist.twist.linear.x
    #     self.current_w = odom.twist.twist.angular.z


    def odom_callback(self, odom):
        orientation = [
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        ]
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)

        position = [
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z
        ]
        self.current_p = np.concatenate((position, [yaw]))



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

            self.theta, self.theta2, self.theta3, self.theta4 = data.position
            self.T1 = self.rot('z', self.theta)@self.translation_matrix(np.array([0.0132, 0, 0]))@self.rot('x', -np.pi/2)@self.translation_matrix(np.array([0, 0.108, 0]))
            self.T2 = self.translation_matrix(np.array([-0.142*np.sin(self.theta2), 0.142*np.cos(self.theta2), 0]))
            self.T3 = self.translation_matrix(np.array([0.1588*np.cos(self.theta3), 0.1588*np.sin(self.theta3), 0]))@self.rot('x', np.pi/2)@self.translation_matrix(np.array([0.056, 0, 0]))
            #self.T2 = self.rot('z', self.theta2)@self.translation_matrix(np.array([0, 0.142, 0]))@self.rot('z', -np.pi/2)@self.rot('z', -self.theta2)
            #self.T3 = self.rot('z', self.theta3)@self.translation_matrix(np.array([0, 0.1588, 0]))@self.rot('z', -self.theta3)@self.translation_matrix(np.array([0, 0.0566, 0]))@self.rot('z', np.pi/2)@self.rot('x', np.pi/2)
            self.T4 = self.rot('z', self.theta4)@self.translation_matrix(np.array([0, 0, 0.0722]))
            self.transformation_mat = self.T1@self.T2@self.T3@self.T4
            position_EE = self.transformation_mat[:, -1]
            print("tranformation matrix:", position_EE)
            self.x = (0.0692 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)) * np.cos(self.theta)
            self.y = (0.0692 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)) * np.sin(self.theta)
            self.z = -0.0358 -  0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3)
            self.marker_EE(position_EE)
            self.pose_EE()

            # # Length of the robot in x-y plane from the base of the manipulator to the end-effector
            # length = 0.0132 + math.sin(self.theta2) * 0.142 + math.cos(self.theta3) * 0.1588 + 0.0565 
            # # From fixed transformation between robot and arm x translation
            # angle = math.atan2(length, 0.051)
            # distance = np.sqrt(length**2 + 0.051**2)
            
            # #Linear velocity for the base 
            # vx_base = math.cos(self.current_pose[3])
            # vy_base = math.sin(self.current_pose[3])
            
            # # Angular velocity 
            # wx_base = math.sin(math.radians(90) + self.current_pose[3] + angle) * distance
            # wy_base = -math.cos(math.radians(90) + self.current_pose[3] + angle) * distance

            self.J = np.array(np.eye(6))
            

            self.T_robot_manipulator = np.array([
                             [math.cos(-1.571) , -math.sin(-1.571), 0, 0.051],
                                [math.sin(-1.571), math.cos(-1.571), 0, 0],
                                [0             ,  0               , 1 , -0.198],
                                [0             ,  0               , 0, 1]])
            self.T_base1 = np.array([
                                [math.cos(self.theta0) , -math.sin(self.theta0), 0, self.d],
                                [math.sin(self.theta0), math.cos(self.theta0), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
            self.T_base = np.array([
                                [math.cos(self.current_p[3]) , -math.sin(self.current_p[3]), 0, self.current_p[0]],
                                [math.sin(self.current_p[3]), math.cos(self.current_p[3]), 0, self.current_p[1]],
                                [0, 0, 1, self.current_p[2]],
                                [0, 0, 0, 1]])
            self.T =  self.T_base @ self.T_robot_manipulator @ self.transformation_mat   

            Final1 = np.array([[-d*math.sin(ctheta)*math.cos(θ₀) - d*math.sin(θ₀)*math.cos(ctheta) + (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)*(-sin(ctheta)*math.sin(θ₀) + cos(ctheta)*math.cos(θ₀))*math.cos(θ₁) + (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)*(-sin(ctheta)*math.cos(θ₀) -sin(θ₀)*math.cos(ctheta))*math.sin(θ₁) - 0.051*math.sin(ctheta)*math.cos(θ₀) - 0.051*math.sin(θ₀)*math.cos(ctheta)],
                                [-d*math.sin(ctheta)*math.sin(θ₀) + d*math.cos(ctheta)*math.cos(θ₀) + (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)*(-sin(ctheta)*math.sin(θ₀) + cos(ctheta)*math.cos(θ₀))*math.sin(θ₁) + (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)*(sin(ctheta)*math.cos(θ₀) + sin(θ₀)*math.cos(ctheta))*math.cos(θ₁) - 0.051*math.sin(ctheta)*math.sin(θ₀) + 0.051*math.cos(ctheta)*math.cos(θ₀)],
                                [0],
                                [0],
                                [0],
                                [1]])
            Final2 = np.array([[-sin(ctheta)*math.sin(θ₀) + cos(ctheta)*math.cos(θ₀) ],
                                [sin(ctheta)*math.cos(θ₀) + sin(θ₀)*math.cos(ctheta)],
                                [0],
                                [0],
                                [0],
                                [0]])
            Final3 = np.array([[(-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)*(-sin(ctheta)*math.sin(θ₀) cos(ctheta)*math.cos(θ₀))*math.cos(θ₁) - (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)*(sin(ctheta)*math.cos(θ₀) +sin(θ₀)*math.cos(ctheta))*math.sin(θ₁)],
                                [(-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)*(sin(ctheta)*math.cos(θ₀) + sin(θ₀)*math.cos(ctheta))*math.cos(θ₁) - (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)*(sin(ctheta)*math.sin(θ₀)- cos(ctheta)*math.cos(θ₀))*math.sin(θ₁) ],
                                [0.142*math.sin(θ₂)],
                                [0],
                                [0],
                                [1]])
            Final4 = np.array([[-0.142*(sin(ctheta)*math.sin(θ₀)  + cos(ctheta)*math.cos(θ₀))*math.sin(θ₁)*math.cos(θ₂) - 0.142*( sin(ctheta)*math.cos(θ₀) + sin(θ₀)*math.cos(ctheta))*math.cos(θ₁)*math.cos(θ₂)],
                                [-0.142*(sin(ctheta)*math.cos(θ₀) +sin(θ₀)*math.cos(ctheta))*math.sin(θ₁)*math.cos(θ₂) - 0.142*(sin(ctheta)*math.sin(θ₀) - cos(ctheta)*math.cos(θ₀))*math.cos(θ₁)*math.cos(θ₂)],
                                [0.142*math.sin(θ₂)],
                                [0],
                                [0],
                                [0]])
            Final5 = np.array([[-0.1588*(-sin(ctheta)*math.sin(θ₀) + cos(ctheta)*math.cos(θ₀))*math.sin(θ₁)*math.sin(θ₃) - 0.1588*(sin(ctheta)*math.cos(θ₀) + sin(θ₀)*math.cos(ctheta) )*math.sin(θ₃)*math.cos(θ₁)],
                                [-0.1588*(sin(ctheta)*math.cos(θ₀) + sin(θ₀)*math.cos(ctheta))*math.sin(θ₁)*math.sin(θ₃) - 0.1588*(sin(ctheta)*math.sin(θ₀) - cos(ctheta)*math.cos(θ₀))*math.sin(θ₃)*math.cos(θ₁) ],
                                [-0.1588*math.cos(θ₃)],
                                [0],
                                [0],
                                [0]])
            Final6 = np.array([[0],
                               [0],
                               [0]
                               [0],
                               [0],
                               [1]]) 
            self.J[:, 1] = Final1  
            self.J[:, 2] = Final2 
            self.J[:, 3] = Final3  
            self.J[:, 4] = Final4 
            self.J[:, 5] = Final5
            self.J[:, 6] = Final6                                      

    def _send_commnd_(self, v, w):
        # cmd = Twist()
        # cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        # cmd.linear.y = 0
        # cmd.linear.z = 0
        # cmd.angular.x = 0
        # cmd.angular.y = 0
        # cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        # self.cmd_pub.publish(cmd)
           
        move = Float64MultiArray() 
         
        v_r = (2 * v + w * self.wheel_base_distance) / (2 * self.wheel_radius)
        v_l = (2 * v - w * self.wheel_base_distance) / (2 * self.wheel_radius)  
        move.data = [v_r, v_l]            
        self.cmd_pub.publish(move)

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
        
        rospy.on_shutdown(shutdown_hook)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



################################################################



            J1 = np.array([[-dx*math.sin(ctheta) + (- math.sin(ctheta))*(-0.142*math.sin(theta[1, 0]) + 0.1588*math.cos(theta[2, 0]) + 0.0692)*math.sin(theta[0, 0]) + (math.cos(ctheta))*(-0.142*math.sin(theta[1, 0]) + 0.1588*math.cos(theta[2, 0]) + 0.0692)*math.cos(theta[0, 0]) - 0.051*math.sin(ctheta)],
                                [dx*math.cos(ctheta) + (math.cos(ctheta))*(-0.142*math.sin(theta[1, 0]) + 0.1588*math.cos(theta[2, 0]) + 0.0692)*math.sin(theta[0, 0]) + (math.sin(ctheta))*(-0.142*math.sin(theta[1, 0]) + 0.1588*math.cos(theta[2, 0]) + 0.0692)*math.cos(theta[0, 0]) + 0.051*math.cos(ctheta)],
                                [0],
                                [0],
                                [0],
                                [1]])
            J2 = np.array([[math.cos(ctheta) ],
                                [math.sin(ctheta)],
                                [0],
                                [0],
                                [0],
                                [0]])
            J3 = np.array([[(math.cos(ctheta))*(-0.142*math.sin(theta[1, 0]) + 0.1588*math.cos(theta[2, 0]) + 0.0692)*math.cos(theta[0, 0]) - (math.sin(ctheta))*(-0.142*math.sin(theta[1, 0]) + 0.1588*math.cos(theta[2, 0]) + 0.0692)*math.sin(theta[0, 0])],
                                [-(- math.cos(ctheta))*(-0.142*math.sin(theta[1, 0]) + 0.1588*math.cos(theta[2, 0]) + 0.0692)*math.sin(theta[0, 0]) + (math.sin(ctheta))*(-0.142*math.sin(theta[1, 0]) + 0.1588*math.cos(theta[2, 0]) + 0.0692)*math.cos(theta[0, 0]) ],
                                [0],
                                [0],
                                [0],
                                [1]])
            J4 = np.array([[-0.142*(math.cos(ctheta))*math.sin(theta[0, 0])*math.cos(theta[1, 0]) - 0.142*(math.sin(ctheta))*math.cos(theta[0, 0])*math.cos(theta[1, 0])],
                                [-0.142*(- math.cos(ctheta))*math.cos(theta[0, 0])*math.cos(theta[1, 0]) - 0.142*(math.sin(ctheta))*math.sin(theta[0, 0])*math.cos(theta[1, 0])],
                                [0.142*math.sin(theta[1, 0])],
                                [0],
                                [0],
                                [0]])
            J5 = np.array([[-0.1588*(math.cos(ctheta))*math.sin(theta[0, 0])*math.sin(theta[2, 0]) - 0.1588*(math.sin(ctheta))*math.sin(theta[2, 0])*math.cos(theta[0, 0])],
                                [-0.1588*(- math.cos(ctheta))*math.sin(theta[2, 0])*math.cos(theta[0, 0]) - 0.1588*(math.sin(ctheta))*math.sin(theta[0, 0])*math.sin(theta[2, 0])],
                                [-0.1588*math.cos(theta[2, 0])],
                                [0],
                                [0],
                                [0]])
            J6 = np.array([[0],
                               [0],
                               [0]
                               [0],
                               [0],
                               [1]])


            ###############################################################################
            
            
            
            
            
            q1
            (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)⋅(-sin(ar)*math.sin(ctheta) - cos(ar)*math.cos(ctheta))*math.cos(θ₁) + (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)⋅(- sin(ar)*math.cos(ctheta) - sin(ctheta)*math.cos(ar))*math.sin(θ₁) - 0.051*math.sin(ar)*math.cos(ctheta) - 0.051*math.sin(ctheta)*math.cos(ar)                              
            (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)⋅(-sin(ar)*math.sin(ctheta) - cos(ar)*math.cos(ctheta))*math.sin(θ₁) + (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)⋅(sin(ar)*math.cos(ctheta) + sin(ctheta)*math.cos(ar) - 0.051*math.sin(ar)*math.sin(ctheta) + 0.051*math.cos(ar)*math.cos(ctheta)
            0
            q2
            cos(ar)
            sin(ar)
            0
            q3
            (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)⋅(-sin(ar)*math.sin(ctheta) + cos(ar)*math.cos(ctheta))*math.cos(θ₁) - (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)⋅(sin(ar)*math.cos(ctheta) + sin(ctheta)*math.cos(ar))*math.sin(θ₁)
            (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)⋅(sin(ar)*math.cos(ctheta) + sin(ctheta)*math.cos(ar))*math.cos(θ₁) - (-0.142*math.sin(θ₂) + 0.1588*math.cos(θ₃) + 0.0692)⋅(sin(ar)*math.sin(ctheta) - cos(ar)*math.cos(ctheta))*math.sin(θ₁)
            0
            q4
            -0.142⋅(-sin(ar)*math.sin(ctheta) + cos(ar)*math.cos(ctheta))*math.sin(θ₁)*math.cos(θ₂) - 0.142⋅(sin(ar)*math.cos(ctheta) + sin(ctheta)*math.cos(ar))*math.cos(θ₁)*math.cos(θ₂)
            -0.142⋅(sin(ar)*math.cos(ctheta) + sin(ctheta)*math.cos(ar))*math.sin(θ₁)*math.cos(θ₂) - 0.142⋅(sin(ar)*math.sin(ctheta) - cos(ar)*math.cos(ctheta))*math.cos(θ₁)*math.cos(θ₂)
            0.142*math.sin(θ₂)
            q5
            -0.1588⋅(-sin(ar)*math.sin(ctheta) + cos(ar)*math.cos(ctheta))*math.sin(θ₁)*math.sin(θ₃) - 0.1588⋅(sin(ar)*math.cos(ctheta) + sin(ctheta)*math.cos(ar))*math.sin(θ₃)*math.cos(θ₁)
            -0.1588⋅(sin(ar)*math.cos(ctheta) + sin(ctheta)*math.cos(ar))*math.sin(θ₁)*math.sin(θ₃) - 0.1588⋅(sin(ar)*math.sin(ctheta) - cos(ar)*math.cos(ctheta))*math.sin(θ₃)*math.cos(θ₁)
            -0.1588*math.cos(θ₃)
            q6
            0
            0
            0
