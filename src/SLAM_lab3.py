#!/usr/bin/python

import numpy as np
import rospy
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pymap3d import geodetic2ned
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from stonefish_ros.msg import BeaconInfo
from wrap_angle import *



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
        self.sigma = 0.5
        #self.noise = np.array([[self.sigma**2, 0],[0, self.sigma**2]])

        self.noise = np.diag([self.sigma**2,self.sigma**2])

        #stete vector
        self.xk = np.array([0,
                            0,
                            0])
        
        #self.Pk = np.zeros((3,3))
        self.Pk = np.eye(3) * 2

        #Define features
        self.features = []

        #Defining the current time
        self.current_time=0.0
        
        
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
        

        self.wait = True
        
        self.all_features_detected = False

        
         #Defining beacon positions
        self.beacon_pos = []
        
       
        self.beacon_altitude = -2.0


        #Defining usbl noise

        self.usbl_noise = np.diag([0.5, 0.2])

       

        # joint state subscriber
        self.js_sub = rospy.Subscriber("kobuki/joint_states", JointState, self.joint_state_callback)


        #ground truth subscriber

        self.truth_sub = rospy.Subscriber('/kobuki/stonefish_simulator/Sensor_odometry', Odometry, self.truth_callback)

        #Define a subscriber for the usbl

        self.usbl_sub = rospy.Subscriber('/kobuki/sensors/usbl/beacon_info', BeaconInfo, self.usbl_callback)

        # odom publisher
        self.odom_pub = rospy.Publisher("kobuki/odom", Odometry, queue_size=10)

        # path publisher

        self.path_pub = rospy.Publisher("kobuki/path", Path, queue_size = 10)
        self.path = Path()
        self.path.header.frame_id = "world_ned"

        #GPS path publisher

        #self.GPS_pub = rospy.Publisher("kobuki/GPS", Marker, queue_size = 10)
        self.path_gps = Marker()
        self.path_gps.header.frame_id = "world_ned"

        #ground truth publisher

        self.truth_pub = rospy.Publisher("kobuki/truth", Path, queue_size = 10)
        self.truth = Path()
        
        #beacon position publisher using odometry
        
        self.beacon2_pub = rospy.Publisher("kobuki/beacon2", Odometry, queue_size = 10)
        self.beacon3_pub = rospy.Publisher("kobuki/beacon3", Odometry, queue_size = 10)
        self.beacon4_pub = rospy.Publisher("kobuki/beacon4", Odometry, queue_size = 10)
        self.beacon5_pub = rospy.Publisher("kobuki/beacon5", Odometry, queue_size = 10)
        
        
        
        #beacon position publisher using MarkerArray
        
        self.beacon_pub = rospy.Publisher("kobuki/beacon", MarkerArray, queue_size = 10)
        
        
        #tf init
        self.tf_br = TransformBroadcaster()
        
        
    def beacon_publisher(self):
        
        """This function publishes the beacon positions as an ArrayMarker.

        """
        
        ma = MarkerArray()


        for i in range(len(self.beacon_pos)):
            beacon = self.beacon_pos[i]
            
            marker = Marker()
            marker.header.frame_id = "world_ned"
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            marker.id = i+2

            marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = beacon[0]
            marker.pose.position.y = beacon[1]
            marker.pose.position.z = beacon[2]
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            marker.color.r = 0.2
            marker.color.a = 0.9
            marker.color.b = 1.0

            ma.markers.append(marker)
            
            self.beacon_pub.publish(ma)

          
        
        
    def beacons_publisher(self,id):
    
        """This function publishes the beacon positions as an odometry with mean and covariances

        """
        
        ind = self.features.index(id)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_ned"
        odom.child_frame_id = "kobuki/base_footprint"
        

        odom.pose.pose.position.x = self.xk[2+2*ind+1]
        odom.pose.pose.position.y = self.xk[2+2*ind+2]
        odom.pose.pose.position.z = self.beacon_altitude
        
    
        # Publishing covariance
        odom.pose.covariance = [self.Pk[2+2*ind+1][2+2*ind+1], self.Pk[2+2*ind+1][2+2*ind+2], 0, 0, 0, 0,
                                self.Pk[2+2*ind+2][2+2*ind+1], self.Pk[2+2*ind+2][2+2*ind+2], 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 
                                0, 0, 0, 0, 0, 0, 
                                0, 0, 0, 0, 0, 0, 
                                0, 0, 0, 0, 0, 0]
        
        if id == 2:
            self.beacon2_pub.publish(odom)
        elif id == 3:
            self.beacon3_pub.publish(odom)
        elif id == 4:
            self.beacon4_pub.publish(odom)
        elif id == 5:
            self.beacon5_pub.publish(odom)
            
        #print(self.features)




    def Compounding(self, z, x):
        
        """""
        This Function gets the robot position and the sensor reading as input and return the feature position with respect to the world frame
        
        """""
        
        ranges = np.sqrt(z[0]**2 - self.beacon_altitude**2)
        F = np.array([x[0] + np.cos(x[2])*np.cos(z[1])*ranges - np.sin(x[2])*np.sin(z[1])*ranges,
                    x[1] + np.sin(x[2])*np.cos(z[1])*ranges + np.cos(x[2])*np.sin(z[1])*ranges])
        
        return F, ranges
    


    def get_jacobians(self, z, x, ranges):
        
        """""
        This function calculates the Jacobian matrices of the compounded feature position
        
        """""

        J1 = np.array([[1,0,-ranges*np.sin(z[1]+x[2])],
                       [0,1,ranges*np.cos(z[1]+x[2])]])
        
        J2 = np.array([[(z[0]*np.cos(z[1]+x[2]))/ranges, -ranges*np.sin(z[1]+x[2])],
                       [(z[0]*np.sin(z[1]+x[2]))/ranges, ranges*np.cos(z[1]+x[2])]])
        
        return J1, J2
        
    
    def Get_features(self, x, p, z, R):
        
        print("getting a new feature")
        

        F, ranges = self.Compounding(z,x)
        
        self.beacon_pos.append(np.array([F[0], F[1], -2.0]))

        x = np.append(x,(F[0],F[1]))
         
        
        J1, J2 = self.get_jacobians(z,x,ranges)
        
        G1 = np.eye(3+len(self.features)*2)
        
        
        if self.features != []:
            
            for t in self.features:
                
                J1 = np.hstack((J1, np.zeros((2,1))))
                J1 = np.hstack((J1, np.zeros((2,1))))
                
            G1 = np.vstack((G1,J1[0]))
            G1 = np.vstack((G1,J1[1]))
            
        else:
            G1 = np.vstack((G1,J1[0]))
            G1 = np.vstack((G1,J1[1]))
        
        
        G2 = np.zeros((3+len(self.features)*2,2))
        G2 = np.vstack((G2,J2[0]))
        G2 = np.vstack((G2,J2[1]))
        
        
        p = np.dot(np.dot(G1,p),G1.T) + np.dot(np.dot(G2,R),G2.T)
        

        return x, p
    
    def observation_model(self, x, beacon_pos,n):
        print("observation of beacon ", n+2 , " = " , beacon_pos)
        dx = beacon_pos[0] - x[0]
        dy = beacon_pos[1] - x[1]
        range = np.sqrt(dx**2 + dy**2 + beacon_pos[2]**2)
        
        azimuth = np.arctan2(dy, dx) - x[2]
        azimuth = wrap_angle(azimuth)
        
        
        print ("expected rage = ", range)
        print ("expected angle = ", azimuth)
        h = np.array([range, azimuth])
        H = np.array([[-dx/range, -dy/range, 0],
                    [dy/range**2, -dx/range**2, -1]])
        
        jac = np.zeros((2, 3+len(self.features)*2 ))
        jac[0][0] = H[0][0]
        jac[0][1] = H[0][1]
        jac[0][2] = H[0][2]
        jac[1][0] = H[1][0]
        jac[1][1] = H[1][1]
        jac[1][2] = H[1][2]
        
        jac[0][3+(n*2)]= np.cos(self.xk[2])
        jac[0][4+n*2] = -np.sin(self.xk[2])
        jac[1][4+(n*2)]= np.cos(self.xk[2])
        jac[1][3+n*2] = np.sin(self.xk[2])
        
        
        
        return h, jac
    
   
    

    def update(self, x, P, z, R, beacon_pos,n):
        print("updating")
        predicted_measurement, H = self.observation_model(x, beacon_pos,n)
        S = np.dot(np.dot(H, P), H.T) + R
        K = np.dot(np.dot(P, H.T), np.linalg.inv(S))
        innovation = z - predicted_measurement
        innovation[1] = np.arctan2(np.sin(innovation[1]), np.cos(innovation[1])) # Wrap angle to [-pi, pi)
        x = x + np.dot(K, innovation)
        L = np.eye(3 + len(self.features)*2) - np.dot(K, H)
        P = np.dot(np.dot(L, P),L.T)
       
        print("iteration")
        
        return x, P

       





    def joint_state_callback(self, msg):
        #print("predicting")
        if self.wait == True:

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
                    w = (-right_lin_vel + left_lin_vel) / self.wheel_base_distance
                
                    #calculate dt
                    self.current_time = rospy.Time.from_sec(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
                    dt = (self.current_time - self.last_time).to_sec()
                    self.last_time = self.current_time

                    # integrate position

                    self.x = self.x + np.cos(self.th) * v * dt 
                    self.y = self.y + np.sin(self.th) * v * dt
                    self.th = self.th + (w) * dt
                    
                    self.xk[0] = self.x
                    self.xk[1] = self.y
                    self.xk[2] = self.th

                    # calculating jacobians
        
                    Ak = np.array([[1,0, - np.sin(self.th) * v * dt],[0,1,np.cos(self.th) * v * dt],[0,0,1]])
                    Wk = np.array([[-0.5 * np.cos(self.th)*dt, -0.5 * np.cos(self.th)*dt],[0.5 * np.sin(self.th)*dt, 0.5 * np.sin(self.th)*dt],[dt/self.wheel_base_distance, -dt/self.wheel_base_distance]])
                    
                    F1 = np.eye(3+len(self.features)*2)
                    
                    F1[0][0] = Ak[0][0]
                    F1[0][1] = Ak[0][1]
                    F1[0][2] = Ak[0][2]
                    F1[1][0] = Ak[1][0]
                    F1[1][1] = Ak[1][1]
                    F1[1][2] = Ak[1][2]
                    F1[2][0] = Ak[2][0]
                    F1[2][1] = Ak[2][1]
                    F1[2][2] = Ak[2][2]
                    
                    
                    F2 = np.zeros((3 +len(self.features)*2, 2))

                    
                    F2[0][0] = Wk[0][0]
                    F2[0][1] = Wk[0][1]
                    
                    F2[1][0] = Wk[1][0]
                    F2[1][1] = Wk[1][1]
                
                    F2[2][0] = Wk[2][0]
                    F2[2][1] = Wk[2][1]
                    
                
                    
                    P1 = np.dot(np.dot(F1,self.Pk),F1.T) 
                    P2 = np.dot(np.dot(F2,self.noise),F2.T)
                
                    
                    
                    self.Pk = P1 + P2
                    
                    
                    # Reset flag
                    self.left_wheel_received = False

       

    def truth_callback(self, msg):
        
        
        """This function publishes the ground truth path of the robot
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        theta = msg.pose.pose.orientation.z

        self.truth.header.frame_id = "world_ned"
        self.truth.header.stamp = rospy.Time.now()
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world_ned"
       
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.truth.poses.append(pose)
        self.truth_pub.publish(self.truth)



        

        
    
    def usbl_callback(self, msg):
        
        
        if msg.beacon_id not in self.features:
            z = np.array([msg.range, wrap_angle(msg.azimuth)])
            self.wait = False
            self.xk, self.Pk =  self.Get_features(self.xk, self.Pk, z, self.usbl_noise)
            self.features.append(msg.beacon_id)
            self.beacon_publisher()
            self.wait = True
            if len(self.features) == 4:
                self.all_features_detected = True
            

        else:
            

            for i in range (len(self.features)):

                if msg.beacon_id ==  self.features[i] : # Only use measurements from beacon i
                    z = np.array([msg.range, wrap_angle(msg.azimuth)])
                    print ("Actual rage = ", msg.range)
                    print ("Actual angle = ", wrap_angle(msg.azimuth))
                    
                    
                    self.xk, self.Pk = self.update(self.xk, self.Pk, z, self.usbl_noise, self.beacon_pos[i],i)
                    
                    self.beacon_publisher()
                    self.beacons_publisher(msg.beacon_id)
                    
                if self.all_features_detected:
                    self.beacon_pos[i] = np.array([self.xk[2*i+3],self.xk[2*i+4], self.beacon_altitude])
                    self.beacon_publisher()
                    self.beacons_publisher(msg.beacon_id)
                    
        
                           

        q = quaternion_from_euler(0, 0, self.xk[2])

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_ned"
        odom.child_frame_id = "kobuki/base_footprint"

        odom.pose.pose.position.x = self.xk[0]
        odom.pose.pose.position.y = self.xk[1]

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
        

        self.odom_pub.publish(odom)

     

        self.belief_path()

        self.tf_br.sendTransform((self.xk[0], self.xk[1], 0.0), q, rospy.Time.now(), odom.child_frame_id, odom.header.frame_id) 

            

    def belief_path(self):
        
        """This function publishes the predicted path of the robot
        """
        self.path.header.stamp = rospy.Time.now()
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world_ned"
        pose.pose.position.x = self.xk[0]
        pose.pose.position.y = self.xk[1]
        pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.xk[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.poses.append(pose)
        self.path_pub.publish(self.path)


if __name__ == '__main__':

    rospy.init_node("differential_drive")

    robot = DifferentialDrive()
   
   

    rospy.spin()
