#!/usr/bin/python

# Class to convert GPS data to a marker for RVIZ

import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker

rospy.init_node('beacons2marker')

map = {2 : np.array([5.0, 5.0, -2.0]),   
     3 : np.array([-5.0, 5.0, -2.0]),    
    4 : np.array([5.0, -5.0, -2.0]), 
    5 : np.array([-5.0, -5.0, -2.0])}  
  

ma = MarkerArray()


for i in range(len(map)):

    idx = list(map.keys())[i]
    marker = Marker()
    marker.header.frame_id = "world_ned" 
    marker.type = marker.SPHERE
    marker.action = marker.ADD

    marker.id = i*2

    marker.header.stamp = rospy.Time.now()
    marker.pose.position.x = map[idx][0]
    marker.pose.position.y = map[idx][1]
    marker.pose.position.z = map[idx][2]
    marker.pose.orientation.w = 1.0
    
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3

    marker.color.r = 1.0
    marker.color.a = 0.9

    ma.markers.append(marker)

    marker_text = Marker()
    marker_text.header.frame_id = "world_ned"
    marker_text.type = marker_text.TEXT_VIEW_FACING
    marker_text.action = marker_text.ADD
    marker_text.id = i*2 + 1

    marker_text.header.stamp = rospy.Time.now()
    marker_text.pose.position.x = map[idx][0]
    marker_text.pose.position.y = map[idx][1]
    marker_text.pose.position.z = map[idx][2] + 1.0
    marker_text.pose.orientation.w = 1.0
  
    marker_text.scale.z = 1.0

    marker_text.color.r = 1.0
    marker_text.color.a = 0.9

    marker_text.text = "beacon "+str(idx)

    ma.markers.append(marker_text)

marker_pub = rospy.Publisher("/beacons_viz", MarkerArray, queue_size=10)


while not rospy.is_shutdown():
    for m in ma.markers: 
        m.header.stamp = rospy.Time.now()
    marker_pub.publish(ma)
    rospy.sleep(1)