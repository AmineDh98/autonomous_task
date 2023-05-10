#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np 
import math 
import rospy  

from sensor_msgs.msg import NavSatFix, Image
from tf.broadcaster import TransformBroadcaster 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pymap3d import geodetic2ned
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float64MultiArray
from stonefish_ros.msg import BeaconInfo 
   
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped  

class Camera_Image: 
    def __init__(self):   
        self.image_pub = rospy.Publisher("/aruco_position", Float64MultiArray, queue_size=10)    
        self.i = 1 
        self.image_sub = rospy.Subscriber("/turtlebot/kobuki/sensors/realsense/color/image_color", Image, self.imageCallback)       
        # self.camera_param = rospy.Subscriber("/kobuki/sensors/realsense/color/image_color", Image, self.imageCallback)       
        
    def imageCallback(self, Image_msg): 
        self.bridge = CvBridge()  
        try:
            cv_image = self.bridge.imgmsg_to_cv2(Image_msg, "bgr8")  
        except CvBridgeError as e:
            print(e) 
        # cv2.namedWindow("topic_sub", cv2.WINDOW_NORMAL) 
        # cv2.imshow("topic_sub", cv_image)   

        # font = cv2.FONT_HERSHEY_SIMPLEX 
        # cv2.putText(cv_image,'Webcam Activated with ROS & OpenCV!',(10,350), font, 1,(255,255,255),2,cv2.LINE_AA)
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        # Set marker ID and length
        marker_id = 1
        marker_length = 0.2 

        # Load the ArUco dictionary
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

        # Load camera calibration parameters
        # if self.i == 1:    
        # fs = cv2.FileStorage("/home/syed/catkin_ws/src/turtlebot_simulation/src/calib_param_10.yml", cv2.FILE_STORAGE_READ)    
        # camera_matrix = fs.getNode("Camera Matrix").mat() 
        # # camera_matrix = camera_matrix.reshape(3,3) 
        # dist_coeffs = fs.getNode("Distortion Coefficients").mat() 
        # fs.release()  
        # print('camera_matrix',camera_matrix) 
        # print('dist_coeffs',dist_coeffs)     
        # self.i = 2 
        camera_matrix = np.array([[1396.8086675255468, 0.0, 960.0],
                                 [0.0, 1396.8086675255468, 540.0],   
                                 [0.0, 0.0, 1.0]]) 

        dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])   

        # Create a video capture object for the camera
        frame = cv_image 

        # Check if the camera is opened successfully
        # if not cap.isOpened():
        #     print("Camera cannot be opened")
        #     exit() 

        # Create a named window for displaying the video stream
        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL) 
        cv2.resizeWindow("Camera", 1200, 800)       

        # Continuously capture frames from the camera and process them
        
        # Capture a frame from the camera
        # ret, frame = cap.read()

        # Detect ArUco markers in the frame
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, dictionary)
        print('marker_ids', marker_ids)  
        # print('marker_ids', marker_corners)  
        # print('marker_ids', marker_corners[0])   
        # If at least one marker is detected, estimate its pose and draw an axis on it
        if marker_ids is not None and len(marker_ids) > 0:
            for i in range(len(marker_ids)):
                # print(marker_ids[i]) 
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners[i], marker_length, camera_matrix, dist_coeffs)   
                print('marker_ids',marker_ids[i],'tvecs',tvecs)    
                # print('tvecs',tvecs.shape) 
                print('rvecs',rvecs)    
                # print('rvecs',rvecs.shape)      
                rvecs = rvecs[0, :].reshape(1,3)    
                tvecs = tvecs[0,:].reshape(1,3)   
                cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids, (0, 255, 0)) 
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs, tvecs, 0.05)   

                # Display the X, Y, and Z coordinates of the marker on the frame

                x = tvecs[0][0]
                y = tvecs[0][1] 
                z = tvecs[0][2]      
                cv2.putText(frame, f"X: {x}", (10, 35), cv2.FONT_HERSHEY_DUPLEX, 0.1, 2, cv2.LINE_AA)
                cv2.putText(frame, f"Y: {y}", (10, 60), cv2.FONT_HERSHEY_DUPLEX, 0.1, 2, cv2.LINE_AA)
                cv2.putText(frame, f"Z: {z}", (10, 85), cv2.FONT_HERSHEY_DUPLEX, 0.1, 2, cv2.LINE_AA)   

                # Create a Float64MultiArray message to publish the point values 
                point_msg = Float64MultiArray()

                # Set the x, y, and z values of the point
                point_msg.data = [x, y, z, float(marker_ids[i])]    

                # Publish the point message
                self.image_pub.publish(point_msg)  


        # Display the frame on the screen
        cv2.imshow("Camera", frame) 
        cv2.waitKey(3)   

if __name__ == '__main__': 
 

    try:
        rospy.init_node("image_sub")  
        Camera_Image()   
        rospy.spin()  
    except rospy.ROSInterruptException:
        pass
