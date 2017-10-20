#!/usr/bin/env python

import rospy
# from tf import TransformListener
import tf
import tf2_ros
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import geometry_msgs.msg
# roslib.load_manifest('learning_tf')
# class myNode:
#     def __init__(self, *args):
#         self.tf = tf2_ros.TransformListener()
#         # rospy.Subscriber(... etc
#
#     def some_method(self):
#
#         if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
#             t = self.tf.getLatestCommonTime("/base_link", "/map")
#             position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
#             print position, quaternion
def handle_vicon_pose(msg):
    theta = msg.pose.pose.orientation.z
    msg = msg.pose.pose.position
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.get_time()
    t.header.frame_id = "base_pose_ground_truth"
    t.child_frame_id = "vicon/forge/forge"
    t.transform.translation.x = msg.x+1
    t.transform.translation.y = msg.y+1
    t.transform.translation.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    # rate = rospy.Rate(10.0)
    br.sendTransform(t)

    # br = tf.TransformBroadcaster()
    # br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, msg.pose.pose.orientation.z),
    #                  rospy.get_time(),"vicon/forge/forge","world")
if __name__ == '__main__':
    rospy.init_node('vicon_tf_broadcaster')
    # turtlename = rospy.get_param('~turtle')
    rospy.Subscriber("base_pose_ground_truth",Odometry,handle_vicon_pose)
    rospy.spin()
