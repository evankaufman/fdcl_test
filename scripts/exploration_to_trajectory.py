#!/usr/bin/env python
import rospy
from uav_geometric_controller.msg import states, trajectory
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from tf.transformations import quaternion_from_euler

desiredPoseTopic = 'desired_pose'
desiredTwistTopic = 'desired_twist'
desiredAccelTopic = 'desired_accel'
desiredTrajectory = 'Jetson/xc'
exploration_name = '/run_autonomous_exploration'

class exp_to_traj(object):

    def __init__(self):
        rospy.init_node('trajectory_echo', anonymous=True)
        rospy.Subscriber(desiredPoseTopic, PoseStamped, self.callback_pose)
        rospy.Subscriber(desiredTwistTopic, TwistStamped, self.callback_twist)
        rospy.Subscriber(desiredAccelTopic, AccelStamped, self.callback_accel)
        self.pub = rospy.Publisher(desiredTrajectory, trajectory, queue_size= 1)
        self.cmd = trajectory()
        self.cmd.b1 = [1, 0, 0]
        self.OmegaZ = 0
        self.getsPose = False
        self.getsVel = False
        self.getsAcc = False

    def callback_pose(self, msg):
        pos = msg.pose.position
        self.cmd.xc = [pos.x, pos.y, 1.0]
        # TODO: add b1
        # euler_from_quaternion(msg.pose.orientation)
        quat = msg.pose.orientation
        self.cmd.b1 = [1.0-2*quat.y*quat.y-2*quat.z*quat.z, 2*quat.x*quat.y+2*quat.z*quat.w, 0]
        self.cmd.b1dot = [-(2*quat.x*quat.y+2*quat.z*quat.w)*self.OmegaZ, (1.0-2*quat.y*quat.y-2*quat.z*quat.z)*self.OmegaZ, 0]
        self.getsPose = True
        pass

    def callback_twist(self, msg):
        pos = msg.twist.linear
        rot = msg.twist.angular
        self.OmegaZ = rot.z
        # self.cmd.xc_dot = [pos.x, pos.y, pos.z]
        self.cmd.xc_dot = [0, 0, 0]
        self.getsVel = True
        pass

    def callback_accel(self, msg):
        pos = msg.accel.linear
        # self.cmd.xc_2dot = [pos.x, pos.y, pos.z]
        self.cmd.xc_2dot = [0, 0, 0]
        self.getsAcc = True
        pass

    def publish(self):
        rate = rospy.Rate(100)
        if not rospy.has_param(exploration_name):
            rospy.set_param(exploration_name, False)

        while not rospy.is_shutdown():
            try:
                if rospy.get_param(exploration_name):
                    if self.getsPose and self.getsVel and self.getsAcc:
                        self.pub.publish(self.cmd)
                else:
                    self.getsPose = False
                    self.getsVel = False
                    self.getsAcc = False
                rate.sleep()
            except (rospy.exceptions):
                pass

if __name__ == '__main__':
    testobj = exp_to_traj()
    testobj.publish()
