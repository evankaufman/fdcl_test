#!/usr/bin/env python

# import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

topic = 'vismark'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)

rospy.init_node('register')

markerArray = MarkerArray()

count = 0
N_MARKER = 10
MARKERS_MAX = N_MARKER ** 3



r_min = -2.0
l_w = 4.0
cube_scale = l_w / N_MARKER * 0.8
idx = 0
for i in range(N_MARKER):
    for j in range(N_MARKER):
        for k in range(N_MARKER):
            marker = Marker()
            marker.header.frame_id = "/world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = cube_scale
            marker.scale.y = cube_scale
            marker.scale.z = cube_scale
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0

            marker.id = idx
            marker.pose.position.x = l_w/N_MARKER*i+r_min #math.cos(count / 50.0)
            marker.pose.position.y = l_w/N_MARKER*j+r_min #math.cos(count / 40.0)
            marker.pose.position.z = l_w/N_MARKER*k+r_min #math.cos(count / 30.0)
            markerArray.markers.append(marker)
            idx += 1



while not rospy.is_shutdown():

   # We add the new marker to the MarkerArray, removing the oldest
   # marker from it when necessary
   # if(count > MARKERS_MAX):
   #     markerArray.markers.pop(0)



   # Renumber the marker IDs
   # id = 0
   # for m in markerArray.markers:
   #     m.id = id
   #     id += 1

   # Publish the MarkerArray
   publisher.publish(markerArray)

   rospy.sleep(0.1)
