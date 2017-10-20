#!/usr/bin/env python
import rospy
import tf2_ros
import tf
from tf import TransformListener
import tf2_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
from gazebo_msgs.msg import LinkStates
import message_filters

class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "odom"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "vicon"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)
def handle_vicon_pose(msg):
    for index, item in enumerate(msg.name):
        if item == 'quadrotor::base_link':
            # print index, item
            break


    pose = msg.pose[index]

    # msg = msg.pose.position
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/world"
    t.child_frame_id = "base_link"

    # listener = tf.TransformListener()
    # t = tf.getLatestCommonTime("/base_link","/world")

    # rate = rospy.Rate(100.0)

    t.transform.translation =  pose.position
    t.transform.rotation =  pose.orientation

    br.sendTransform(t)
    rate = rospy.Rate(1000)


    pub = rospy.Publisher("/vicon/forge/forge",geometry_msgs.msg.TransformStamped,queue_size=100)
    pub.publish(t)



if __name__ == '__main__':
    rospy.init_node('Gazebo_base')
    # tfb = FixedTFBroadcaster()

    rospy.Subscriber("gazebo/link_states",LinkStates,handle_vicon_pose)
    rospy.spin()
