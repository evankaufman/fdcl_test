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
class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            # rospy.sleep(0.1)

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
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w
    theta = np.arctan2(2*qx*qy+2*qz*qw, 1-2*qy*qy-2*qz*qz)
    # msg = msg.pose.position
    # br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "/world"
    t.child_frame_id = "vicon/forge/forge"

    # listener = tf.TransformListener()
    # t = tf.getLatestCommonTime("/base_link","/world")

    # rate = rospy.Rate(1000.0)

    # try:
    #     position, quartenion = listener.lookupTransform('/base_link','/world',rospy.Time(0))
    #     print position, quartenion
    #     # t.transform.translation  = position
    #     # t.transform.rotation = quartenion
    #
    #     t.transform.translation.x = position[0]
    #     t.transform.translation.y = position[1]
    #     t.transform.translation.z = position[2]
    #
    #     t.transform.rotation.x = quartenion[0]
    #     t.transform.rotation.y = quartenion[1]
    #     t.transform.rotation.z = quartenion[2]
    #     t.transform.rotation.w = quartenion[3]
    #
    #     # br.sendTransform(t)
    #     pub = rospy.Publisher("/vicon/forge/forge",geometry_msgs.msg.TransformStamped,queue_size=100)
    #     pub.publish(t)
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     print 'here'


    t.transform.translation.x =  msg.pose.position.x
    t.transform.translation.y =  msg.pose.position.y
    t.transform.translation.z =  msg.pose.position.z
    q = tf.transformations.quaternion_from_euler(0, 0, theta)
    t.transform.rotation.x =  q[0]
    t.transform.rotation.y =  q[1]
    t.transform.rotation.z =  q[2]
    t.transform.rotation.w =  q[3]

    # br.sendTransform(t)
    rate = rospy.Rate(1000)
    pub = rospy.Publisher("vicon/forge/forge",geometry_msgs.msg.TransformStamped,queue_size=100)
    pub.publish(t)



if __name__ == '__main__':
    rospy.init_node('ViconTF')
    # tfb = FixedTFBroadcaster()
    rospy.Subscriber("ground_truth_to_tf/pose",PoseStamped,handle_vicon_pose)
    rospy.spin()
