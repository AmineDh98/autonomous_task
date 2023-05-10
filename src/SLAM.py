#!/usr/bin/python

import numpy as np
import rospy
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from plotjuggler.plotjuggler import PlotJuggler


class DifferentialDrive:
    def __init__(self) -> None:

        # robot constants
        self.wheel_radius = 0.035
        self.wheel_base_distance = 0.230

        # pose of the robot
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        #covarience
        self.sigma = 0.05
        #self.noise = np.array([[self.sigma**2, 0],[0, self.sigma**2]])

        self.noise = np.diag([self.sigma**2,self.sigma**2])

        #stete vector
        self.xk = np.array([0,
                            0,
                            0])
        self.Pk = np.zeros((3,3))

        self.M = [0.0] * 36
        
        
        # velocity and angular velocity of the robot
        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.left_wheel_received = False

        self.last_time = rospy.Time.now()
        
        #defining old position received from the sensor

        self.x_sensor = 0.0
        self.y_sensor = 0.0
        self.theta_sensor = 0.0
        self.old_time = None

       

       

        # joint state subscriber
        self.js_sub = rospy.Subscriber("kobuki/joint_states", JointState, self.joint_state_callback)

        # odom publisher
        self.odom_pub = rospy.Publisher("kobuki/odom", Odometry, queue_size=10)

        #tf init
        self.tf_br = TransformBroadcaster()






    def joint_state_callback(self, msg):

        if msg.name[0] == "kobuki/wheel_left_joint":
            self.left_wheel_velocity = msg.velocity[0]
            self.left_wheel_received = True
            return
        elif msg.name[0] == "kobuki/wheel_right_joint":
            self.right_wheel_velocity = msg.velocity[0]
            if (self.left_wheel_received):
                # Do calculations
                left_lin_vel = (self.left_wheel_velocity ) * self.wheel_radius 
                right_lin_vel = (self.right_wheel_velocity ) * self.wheel_radius

                v = (left_lin_vel + right_lin_vel) / 2.0
                w = (right_lin_vel - left_lin_vel) / self.wheel_base_distance
             
                #calculate dt
                current_time = rospy.Time.from_sec(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
                dt = (current_time - self.last_time).to_sec()
                self.last_time = current_time

                # integrate position

                self.x = self.x + np.cos(self.th) * v * dt 
                self.y = self.y + np.sin(self.th) * v * dt
                self.th = self.th + (w) * dt

                

                

                # calculating jacobians
       

                Ak = np.array([[1,0, - np.sin(self.th) * v * dt],[0,1,np.cos(self.th) * v * dt],[0,0,1]])
                Wk = np.array([[-0.5 * np.cos(self.th)*dt, -0.5 * np.cos(self.th)*dt],[0.5 * np.sin(self.th)*dt, 0.5 * np.sin(self.th)*dt],[dt/self.wheel_base_distance, -dt/self.wheel_base_distance]])
                
                P1 = np.dot(np.dot(Ak,self.Pk),np.transpose(Ak)) 
                P2 = np.dot(np.dot(Wk,self.noise),np.transpose(Wk))
                
                

                self.Pk = P1 + P2
                self.xk = np.array([self.x,
                                    self.y,
                                    self.th])
                
                q = quaternion_from_euler(0, 0, self.th)


                # Reset flag
                self.left_wheel_received = False

                # Publish odom

                odom = Odometry()
                odom.header.stamp = current_time
                odom.header.frame_id = "world_ned"
                odom.child_frame_id = "kobuki/base_footprint"

                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y

                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2]
                odom.pose.pose.orientation.w = q[3]

               


                

           
                
               # Publishing covariance
                odom.pose.covariance = [self.Pk[0][0], self.Pk[0][1], 0, 0, 0, self.Pk[0][2],
                                        self.Pk[1][0], self.Pk[1][1], 0, 0, 0, self.Pk[1][2],
                                        0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 
                                        self.Pk[2][0], self.Pk[2][1], 0, 0, 0, self.Pk[2][2]]
             

                

                odom.twist.twist.linear.x = v
                odom.twist.twist.angular.z = w

             
                self.odom_pub.publish(odom)

                self.tf_br.sendTransform((self.x, self.y, 0.0), q, rospy.Time.now(), odom.child_frame_id, odom.header.frame_id)

   

if __name__ == '__main__':

    rospy.init_node("differential_drive")

    robot = DifferentialDrive()
   

    rospy.spin()