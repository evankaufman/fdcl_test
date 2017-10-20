#!/usr/bin/env python
import rospy
import tf2_ros
import tf
import tf2_msgs.msg
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def handle_vicon_pose(msg):

    theta = msg.pose.pose.orientation.z
    msg = msg.pose.pose.position
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "odom"
    t.transform.translation.x = msg.x + 23
    t.transform.translation.y = msg.y + 4
    t.transform.translation.z = 0.0
    q = tf.transformations.quaternion_from_euler(0., 0., theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    # rate = rospy.Rate(10.0)
    br.sendTransform(t)
    rate = rospy.Rate(10)
    pub = rospy.Publisher("world",geometry_msgs.msg.TransformStamped,queue_size=10)
    pub.publish(t)

if __name__ == '__main__':
    rospy.init_node('map_to_odom')
    # tfb = FixedTFBroadcaster()
    rospy.Subscriber("odom",Odometry,handle_vicon_pose)
    rospy.spin()
