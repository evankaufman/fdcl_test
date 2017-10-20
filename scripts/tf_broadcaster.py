#!/usr/bin/env python
import rospy
import tf2_ros
import tf
import tf2_msgs.msg
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.01)

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
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    theta = np.arctan2(2*qx*qy+2*qz*qw, 1-2*qy*qy-2*qz*qz)
    msg = msg.pose.pose.position
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/ground_truth"
    t.child_frame_id = "/vicon/forge/forge"
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    # rate = rospy.Rate(10.0)
    br.sendTransform(t)
    rate = rospy.Rate(200)
    pub = rospy.Publisher("/vicon/forge/forge",geometry_msgs.msg.TransformStamped,queue_size=100)
    pub.publish(t)

if __name__ == '__main__':
    rospy.init_node('ViconTF')
    # tfb = FixedTFBroadcaster()
    rospy.Subscriber("/ground_truth/state",Odometry,handle_vicon_pose)
    rospy.spin()
