#!/usr/bin/python

# working with one feature everything hardcored  

import numpy as np 
import math 
import rospy  
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState
from sensor_msgs.msg import NavSatFix 
from tf.broadcaster import TransformBroadcaster 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pymap3d import geodetic2ned
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float64MultiArray
from stonefish_ros.msg import BeaconInfo 
   
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped  
    
class DifferentialDrive:
    def __init__(self, sensor_noise, sensor_noise_angle):   

        self.tf_br = TransformBroadcaster()    
        # robot constants 
        self.wheel_radius = 0.035 # meters      
        self.wheel_base_distance = 0.23 # meters   
 
        # initial pose of the robot  
        self.Xk = np.zeros([4,1]) 
        # self.Xk[0][0] = 2.15  
        # self.Xk[1][0] = 1.4
        # self.Xk[2][0] = -0.2
        # self.Xk[3][0] = -math.pi/2
        
        # velocity and angular velocity of the robot
        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.left_wheel_received = False

        self.last_time = rospy.Time.now()

        self.id = 0 
        self.beacon_list = []  
        self.rz = 0.0 # robot state z component  

        # self.Xk = np.zeros((5,1))
        # self.Pk = np.zeros((5,5)) 

        self.Hk = np.zeros((3,4))   
        self.F1 = np.zeros((4,4))
        self.F1_I = np.diag((1,1,1))   
        # self.F1[3:,3:] = F1_I  
        self.F2 =  np.zeros((4,2)) 
        self.F2_o = np.zeros((3,3))   
        self.g1 = np.zeros((4,1)) 
        self.g2 = np.zeros((4,3))
        
        self.num = 0
        self.num_2 = 0

        self.map12 = {}  

        # covariance details
        self.Pk = np.array([[0.6, 0, 0, 0],    # 2,2 matrix  
                            [0, 0.2, 0, 0], 
                            [0, 0, 0.0, 0], 
                            [0, 0, 0, 0.01]])       # 3,3 matrix with diagonal 0.04     
        # self.Pk[0:3,0:3] = self.Pk 
        
        self.Qk = np.array([[0.2**2, 0],    # 2,2 matrix   
                             [0, 0.2**2]])      

        self.Rk =  np.array([[0.2**2, 0, 0],    # 2,2 matrix     
                             [0, 0.02**2, 0], 
                             [0, 0, 0.02**2]])     

        self.fv = None       
  
        # joint state subscriber 
        self.js_sub = rospy.Subscriber("/turtlebot/joint_states", JointState, self.joint_state_callback)
        # self.gps = rospy.Subscriber("/kobuki/stonefish_simulator/GPS_ground_truth", NavSatFix, self.gps)
        # self.beacon = rospy.Subscriber("/kobuki/sensors/usbl/beacon_info", BeaconInfo, self.beacon_fn)   
        self.aruco_pose = rospy.Subscriber("/aruco_position", Float64MultiArray, self.aruco_position)     
        # odom publisher  
        self.odom_pub = rospy.Publisher("kobuki/odom", Odometry, queue_size=10)  

        # self.path_pub = rospy.Publisher('gps_path', Path, queue_size=10)
        # self.marker_pub = rospy.Publisher('gps_marker', Marker, queue_size=10)
        self.marker_pub1 = rospy.Publisher('/gps/ellipsoid_marker', Marker, queue_size=10)  

        self.marker_pub = rospy.Publisher("/beacons_viz", MarkerArray, queue_size=10)
           

    def wrap_angle(self, ang):
        if isinstance(ang, np.ndarray):
            ang[ang > np.pi] -= 2 * np.pi
            return ang
        else: 
            return ang + (2.0 * math.pi * math.floor((math.pi - ang) / (2.0 * math.pi)))

    def aruco_position(self, beacon): 
        # print('id',beacon.beacon_id, 'range', beacon.range, 'angle', beacon.azimuth, 'elevation', beacon.elevation)  
        print('data received from camera',beacon.data)  

        # feature in the robot frame

        self.c_fx = beacon.data[2] # this is the observed value  (beacon.x) 
        self.c_fy = beacon.data[0]
        self.c_fz = beacon.data[1]   

        self.r_cx = 0.0   # this will be fixed  
        self.r_cy = 0.0
        self.r_cz = 0.0 
        self.r_c_yaw = 0.0 
        

        self.rm_r_c =   np.array([[self.r_cx + self.c_fx*np.cos(self.r_c_yaw) - self.c_fy*np.sin(self.r_c_yaw)],   # add in Xk
                            [self.r_cy + self.c_fx*np.sin(self.r_c_yaw) + self.c_fy*np.cos(self.r_c_yaw)],  
                            [self.r_cz + self.c_fz]])  
        
        self.x_b = self.rm_r_c[0,0]
        self.y_b = self.rm_r_c[1,0]  
        self.z_b = self.rm_r_c[2,0]   

        #--------------------------------------------------------------------------------------------------------------------------------


        self.id = beacon.data[3]    
        self.range =  math.sqrt(self.c_fx**2 + self.c_fy**2 + self.c_fz**2) 
        self.azimuth = math.atan2(self.c_fy, self.c_fx) 
        self.azimuth = self.wrap_angle(self.azimuth)  
        self.elevation = math.atan2(self.c_fz, math.sqrt(self.c_fx**2 + self.c_fy**2))       
        self.elevation = self.wrap_angle(self.elevation)    

        # print('range',self.range, 'azimuth', self.azimuth, 'elevation', self.elevation)  
        J_p2c = np.array([[np.cos(self.elevation)*np.cos(self.azimuth), -self.range*np.cos(self.elevation)*np.sin(self.azimuth), -self.range*np.sin(self.elevation)*np.cos(self.azimuth)],
                        [np.cos(self.elevation)*np.sin(self.azimuth), self.range*np.cos(self.elevation)*np.cos(self.azimuth), -self.range*np.sin(self.elevation)*np.sin(self.azimuth)], 
                        [np.sin(self.azimuth), 0 , self.range*np.cos(self.elevation)]])  

        Rk_c = J_p2c @ self.Rk @ J_p2c.T   # uncertainty in camera frame in cartesian coordinates

        J2 = np.array([[np.cos(self.r_c_yaw), -np.sin(self.r_c_yaw), 0], 
                        [np.sin(self.r_c_yaw), np.cos(self.r_c_yaw), 0],  
                        [0, 0 , 1]])  

        Rk_r = J2 @ Rk_c @ J2.T   # uncertainty in robot frame in cartesian coordinates 

        self.Xk, self.Pk  = self.update(self.Xk, self.Pk, self.x_b, self.y_b, self.z_b, Rk_r, self.id)    # take in Xk and Pk       
        self.quad(self.Xk, self.Pk, self.current_time, self.v, self.w)       
       
    def update(self, Xk, Pk, x_b, y_b, z_b, Rk_r, id): 
        self.Xk = Xk
        self.Pk = Pk
        self.x_b = x_b  
        self.y_b = y_b 
        self.z_b = z_b
        Rk_r = Rk_r
        self.id = id
        # print(self.id) 
        if self.id not in self.beacon_list:      
        
            # wrapping the angle   
            self.Xk[3,0] = self.wrap_angle(self.Xk[3,0])   

            self.beacon_list.append(self.id)  

            # increasing the matrix size
            self.Xk = np.pad(self.Xk, ((0, 3), (0, 0)))  # 5 by 1 
            self.Hk = np.pad(self.Hk, ((0, 0), (0, 3)))  
            self.g1 = np.pad(self.g1, ((0, 3), (0, 3)))  
            self.g2 = np.pad(self.g2, ((0, 3), (0, 0)))   


            self.F1 = np.pad(self.F1, ((0, 3), (0, 3)))
            self.F1[-3:,-3:] = self.F1_I  
            self.F2 = np.pad(self.F2, ((0, 3), (0, 0))) 
            

            # position of the feature in robot frame in cartesian coordinate 
            Zk = np.array([[self.x_b],[self.y_b], [self.z_b]])  
            self.fv = Zk 

            I4 = np.eye(4)
            I3 = np.eye(3)  
            O = np.zeros((3,2))
            Vk = np.eye(3)   


            th = self.Xk[3,0]
            # feature in the world frame 
            Zk_p = np.array([[self.Xk[0,0] + Zk[0,0]*np.cos(th) - Zk[1,0]*np.sin(th)],   # add in Xk
                            [self.Xk[1,0] + Zk[0,0]*np.sin(th) + Zk[1,0]*np.cos(th)], 
                            [self.Xk[2,0] + Zk[2,0]]])  
            
            self.Xk[-3:] = Zk_p  # fill only last three rows   
            #   -----------------------------------------------------------------
            J_1_p = np.array([[1, 0, 0, -Zk[0,0]*np.sin(th)-Zk[1,0]*np.cos(th)],
                            [0, 1, 0,  Zk[0,0]*np.cos(th)-Zk[1,0]*np.sin(th)],
                            [0, 0, 1,  0]]) 
            
            J_2_p = np.array([[np.cos(th), -np.sin(th), 0], 
                            [np.sin(th), np.cos(th), 0],
                            [0, 0, 1]]) 
        

            # accumulated uncertainty 
            if self.id == 1: 
                self.g1[0:4, 0:4] =  I4 
                self.g1[-3:,:4] = J_1_p  
            else:
                self.g1[-6:-3,-3:] = I3 
                self.g1[-3:,:4] = J_1_p  

            self.g2[:,:] = 0       # 5 by 2  
            self.g2[-3:,:] = J_2_p  # 5 by 2    

            self.Pk = self.g1 @ self.Pk @ self.g1.T + self.g2 @ Rk_r @ self.g2.T 
            self.g1[-2:,:3] = 0

            # increasing the map size to include features 
            self.map12[self.num_2] = 0.0 

            self.num_2 += 1  
 
        else:   
            
            print('ID',self.id) 
            idx = self.beacon_list.index(self.id) 
            f_i_1, f_i_2 = 4 + 3*idx, 7 + 3*idx  # index of the feature in the state vector (x,y,z) for feature 1, it is 4 to 7 

            self.Xk[3,0] = self.wrap_angle(self.Xk[3,0]) 
            # position of the feature in robot frame in cartesian coordinate  
            Zk1 = np.array([[self.x_b],[self.y_b], [self.z_b]])  

            I_1 = np.eye(self.Pk.shape[0]) 
            Vk = np.eye(3) 

            map = self.Xk[f_i_1: f_i_2,:] ###  # 3 by 1 map of feature 1 (matrix)   

            th = self.Xk[3,0] 
            
            h_x = np.array([[-self.Xk[0,0]*np.cos(th)-self.Xk[1,0]*np.sin(th)+map[0,0]*np.cos(th)+map[1,0]*np.sin(th)],
                         [self.Xk[0,0]*np.sin(th)-self.Xk[1,0]*np.cos(th)-map[0,0]*np.sin(th)+map[1,0]*np.cos(th)],
                         [-self.rz + map[2,0]]])   

            # innovation
            Ykn = (Zk1 - h_x) 
            print(' Ykn',  Ykn)   
            
            self.Hk[:,:] = 0 
            repeat = np.array([[np.cos(th), np.sin(th), 0],
                               [-np.sin(th), np.cos(th), 0],
                               [0, 0, 1]])  

            self.Hk[:,0:4] = np.array([[-np.cos(th), -np.sin(th), 0, self.Xk[0,0]*np.sin(th)-self.Xk[1,0]*np.cos(th)-map[0,0]*np.sin(th)+map[1,0]*np.cos(th)],    
                                        [np.sin(th), -np.cos(th), 0, self.Xk[0,0]*np.cos(th)+self.Xk[1,0]*np.sin(th)-map[0,0]*np.cos(th)-map[1,0]*np.sin(th)],
                                        [0, 0, -1, 0]])         
 
            self.Hk[:,f_i_1:f_i_2] = repeat   
   
            # ------------------------------------

        
            before_inv = (self.Hk @ (self.Pk @ self.Hk.T)) + (Vk @ (Rk_r @ Vk.T))  
            inv = np.linalg.pinv(before_inv)   
            K_gain = ((self.Pk @ self.Hk.T) @ inv) 


             
            self.Xk = self.Xk + K_gain @ (Ykn)     
            print('update',  self.Xk)         
            self.Pk = ((I_1 - (K_gain @ self.Hk))@ self.Pk @ (I_1 - (K_gain @ self.Hk)).T)   

            # visualizing the aruco as a point in the simulation
            for m in range(len(self.map12)):    
                 self.map12[m] = [self.Xk[3*m+4,0], self.Xk[3*m+5,0], self.Xk[3*m+6,0]]   
            self.modem_visualization(self.map12)   
               
       
        return self.Xk, self.Pk 
    
    def modem_visualization(self, map): # feature contain all the modem positions as a column vector   
         
        self.map = map
        # print(self.map)

        ma = MarkerArray()
        # z = -2.0

        for i in range(len(self.map)):   

            idx = list(self.map.keys())[i]
            print('map_index',self.map[idx])  
            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            marker.id = i*2 

            marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = self.map[idx][0]
            marker.pose.position.y = self.map[idx][1]
            marker.pose.position.z = self.map[idx][2] 
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1 

            marker.color.r = 1.0
            marker.color.a = 0.9

            ma.markers.append(marker)

            marker_text = Marker()
            marker_text.header.frame_id = "world"
            marker_text.type = marker_text.TEXT_VIEW_FACING
            marker_text.action = marker_text.ADD
            marker_text.id = i*2 + 1

            marker_text.header.stamp = rospy.Time.now()
            marker_text.pose.position.x = self.map[idx][0]
            marker_text.pose.position.y = self.map[idx][1]
            marker_text.pose.position.z = self.map[idx][2]  + 1.0  
            marker_text.pose.orientation.w = 1.0

            marker_text.scale.x = 0.1
            marker_text.scale.y = 0.1
            marker_text.scale.z = 0.3
        
            # marker_text.scale.z = 1.0

            marker_text.color.r = 1.0
            marker_text.color.a = 0.9

            marker_text.text = "Aruco "+str(idx) 

            ma.markers.append(marker_text)

            # uncertainty of
            marker1 = Marker() 
            marker1.header.frame_id = 'world' 
            marker1.header.stamp = rospy.Time.now()
            marker1.ns = 'feature_ellipsoid' 
            marker1.id = i*100+1
            marker1.type = Marker.SPHERE 
            marker1.action = Marker.ADD
            marker1.color = ColorRGBA(0.0, 1.0, 0.0, 1.0) 

            # Set marker position to GPS fix location
            marker1.pose.position.x = self.map[idx][0]
            marker1.pose.position.y = self.map[idx][1]  
            marker1.pose.position.z = self.map[idx][2]  
            marker1.pose.orientation.w = 1.0 

            # Set marker scale to represent measurement uncertainty
            marker1.scale.x = 2*np.sqrt(self.Pk[i*3+4, i*3+4])*np.sqrt(5.991)
            marker1.scale.y = 2*np.sqrt(self.Pk[i*3+5, i*3+5])*np.sqrt(5.991)     
            marker1.scale.z = 0.01  

            ma.markers.append(marker1) 
        self.marker_pub.publish(ma)              

    def prediction(self, left_wheel_velocity, right_wheel_velocity, msg):

        self.left_wheel_velocity = left_wheel_velocity 
        self.right_wheel_velocity = right_wheel_velocity

        left_lin_vel = self.left_wheel_velocity * self.wheel_radius
        right_lin_vel = self.right_wheel_velocity * self.wheel_radius

        self.v = (left_lin_vel + right_lin_vel) / 2.0      
        self.w = (left_lin_vel - right_lin_vel) / self.wheel_base_distance  
        
        #calculate dt 
        self.current_time = rospy.Time.from_sec(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
        dt = (self.current_time - self.last_time).to_sec()
        self.last_time = self.current_time  

        # -----------------------------------------------------------------------------------------------------
        # -----------------------------------------------------------------------------------------------------
        self.Xk[0,0] = self.Xk[0,0] + np.cos(self.Xk[3,0]) * self.v * dt 
        self.Xk[1,0] = self.Xk[1,0] + np.sin(self.Xk[3,0]) * self.v * dt  
        self.Xk[2,0] = self.rz 
        self.Xk[3,0] = self.Xk[3,0] + self.w * dt 

        self.Xk[3,0] = self.wrap_angle(self.Xk[3,0])  # wrapping the angle 
                    
        Ak = np.array([[1, 0, 0, -np.sin(self.Xk[3,0])*self.v*dt],  
                        [0, 1, 0, np.cos(self.Xk[3,0])*self.v*dt],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])     

        Bk = np.array([[np.cos(self.Xk[3,0])*dt*0.5*self.wheel_radius, np.cos(self.Xk[3,0])*dt*0.5*self.wheel_radius],    
                            [np.sin(self.Xk[3,0])*dt*0.5*self.wheel_radius, np.sin(self.Xk[3,0])*dt*0.5*self.wheel_radius],   
                            [0, 0],     
                            [(dt*self.wheel_radius)/self.wheel_base_distance, -(dt*self.wheel_radius)/self.wheel_base_distance]])     
        
        self.F1[0:4,0:4] = Ak
        self.F2[0:4,:] = Bk     
        self.Pk = self.F1 @ self.Pk @ self.F1.T + self.F2 @ self.Qk @ self.F2.T 
        return self.Xk, self.Pk     

    def quad(self, Xk, Pk, current_time, v, w): 
                self.Xk = Xk
                self.Pk = Pk 
                self.v = v 
                self.w = w
                self.current_time = current_time 
 
                q = quaternion_from_euler(0, 0, self.Xk[3,0])   

                odom = Odometry() 
                odom.header.stamp = self.current_time    
                odom.header.frame_id = "world"   
                odom.child_frame_id = "turtlebot/kobuki/base_footprint"

                odom.pose.pose.position.x =  self.Xk[0,0]  
                odom.pose.pose.position.y =  self.Xk[1,0]
                odom.pose.pose.position.z =  self.Xk[2,0] 
                odom.pose.covariance = [self.Pk[0,0], self.Pk[0,1], self.Pk[0,2], 0.0, 0.0, self.Pk[0,3],   
                                        self.Pk[1,0], self.Pk[1,1], self.Pk[1,2], 0.0, 0.0, self.Pk[1,3],     
                                        self.Pk[2,0], self.Pk[2,1], self.Pk[2,2], 0.0, 0.0, self.Pk[2,3],      
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        self.Pk[3,0], self.Pk[3,1], self.Pk[3,2], 0.0, 0.0, self.Pk[3,3]]     

                # odom.pose.covariance = [math.sqrt(abs(self.Pk[0,0])), math.sqrt(abs(self.Pk[0,1])), 0.0, 0.0, 0.0, math.sqrt(abs(self.Pk[0,2])),   
                #                         math.sqrt(abs(self.Pk[1,0])), math.sqrt(abs(self.Pk[1,1])), 0.0, 0.0, 0.0, math.sqrt(abs(self.Pk[1,2])),   
                #                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     
                #                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    
                #                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                #                         math.sqrt(abs(self.Pk[3,0])), math.sqrt(abs(self.Pk[2,1])), 0.0, 0.0, 0.0, math.sqrt(abs(self.Pk[2,2]))] 

                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1] 
                odom.pose.pose.orientation.z = q[2] 
                odom.pose.pose.orientation.w = q[3]    

                odom.twist.twist.linear.x = self.v  
                odom.twist.twist.angular.z = self.w
                # print('Xk', self.Xk)
                # print('self.Pk[2,2]', self.Pk[2,2])       
                
                self.odom_pub.publish(odom)   
                self.tf_br.sendTransform((self.Xk[0,0], self.Xk[1,0], self.Xk[2,0]), q, rospy.Time.now(), odom.child_frame_id, odom.header.frame_id)    
 
    def joint_state_callback(self, msg): 
 
        if msg.name[0] == "turtlebot/kobuki/wheel_left_joint":
            self.left_wheel_velocity = msg.velocity[0]
            self.left_wheel_received = True
            return 
        elif msg.name[0] == "turtlebot/kobuki/wheel_right_joint": 
            self.right_wheel_velocity = msg.velocity[0] 
            if (self.left_wheel_received): 
                # Reset flag
                self.left_wheel_received = False        
 
                self.Xk, self.Pk =  self.prediction(self.left_wheel_velocity, self.right_wheel_velocity, msg) 
                # print('pred', self.Xk)        
                # self.Xk, self.Pk  = self.update(self.Xk, self.Pk, self.x_b, self.y_b)        

                self.quad(self.Xk, self.Pk, self.current_time, self.v, self.w)          
                 

   
if __name__ == '__main__': 
 
    rospy.init_node("differential_drive")
    sensor_noise = 0.1
    sensor_noise_angle = 0.2  
    robot = DifferentialDrive(sensor_noise, sensor_noise_angle)   
    rospy.spin()  

    





