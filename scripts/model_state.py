#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
import tf
from gazebo_msgs.msg import LinkStates
import message_filters
from tf.transformations import euler_from_quaternion

class move:
    def handle_link(self,link_msg):
        index = 0
        for index, item in enumerate(link_msg.name):
            if item == 'quadrotor::base_link':
                # print index, item
                break
        self.pose = link_msg.pose[index]

    def handle_goal_pose(self,goal_msg):
        self.goal_msg = goal_msg
        self.ok = True
    def step(self):
        # if all(self.pose.position==self.goal_msg.position):
        #     # self.ok = False
        #     self.idx = 1

        if self.idx < self.N:
            self.move_msg.pose.position.x = self.pose.position.x + (self.goal_msg.position.x - self.pose.position.x)/(self.N)*self.idx
            self.move_msg.pose.position.y = self.pose.position.y + (self.goal_msg.position.y - self.pose.position.y)/(self.N)*self.idx
            self.move_msg.pose.position.z = self.pose.position.z + (self.goal_msg.position.z - self.pose.position.z)/(self.N)*self.idx

            q = self.goal_msg.orientation
            theta = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
            # print 'theta: ', theta
            q = self.pose.orientation
            theta_cur = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
            # print 'theta+cur: ',theta_cur
            theta_n = theta[2]
            theta_cur_n = theta_cur[2]
            if theta[2]<0:
                theta_n = np.pi + theta[2]
            if theta_cur[2]<0:
                theta_cur_n = np.pi + theta_cur[2]
            theta_inc = theta_cur_n + (theta_n - theta_cur_n)*self.idx/self.N

            q = tf.transformations.quaternion_from_euler(0, 0, theta_inc)

            self.move_msg.pose.orientation.x = q[0]
            self.move_msg.pose.orientation.y = q[1]
            self.move_msg.pose.orientation.z = q[2]
            self.move_msg.pose.orientation.w = q[3]
            self.idx = self.idx + 1
        elif self.idx == self.N:
            self.move_msg.pose.position  =self.goal_msg.position
            self.move_msg.pose.orientation = self.goal_msg.orientation
            self.idx = 1
            print 'here!'
        # print "goal", self.goal_msg
    #
    def __init__(self):
        self.N = 32.
        self.ok = False
        self.pose = Pose()
        self.move_msg = ModelState()
        self.move_msg.twist = Twist()
        self.goal_msg = Pose()
        # self.move_msg.pose = Pose()
        self.pose = Pose()
        pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.idx = 1
        # self.move_msg = ModelState()
        self.move_msg.reference_frame = "world"
        self.move_msg.model_name = "quadrotor"
        # rate = rospy.Rate(1)
        # k = 0
        while not rospy.is_shutdown():
            rospy.Subscriber('/goal',Pose,self.handle_goal_pose)
            rospy.Subscriber('/gazebo/link_states',LinkStates,self.handle_link)
            # print 'goal::::', self.goal_msg
            if self.ok:
                self.step()
                pub.publish(self.move_msg)
                # print self.move_msg
                rospy.sleep(0.2)
                # print k
                # k = k +1


    # if __name__ == '__main__':
    #     try:
    #          setObject()
    #     except rospy.ROSInterruptException:
    #          pass
NAME = 'Gazebo_ros'
if __name__ == '__main__':
    # rostest.unitrun('stage_ros', NAME, TestStageRos, sys.argv)
    rospy.init_node('move_model')
    # ts = message_filters.TimeSynchronizer([link_sub,goal_sub],1)
    # ts.registerCallback(handle_goal_pose)
    #  tfb = FixedTFBroadcaster()
    # rospy.Subscriber("/goal",Pose,handle_goal_pose)
    try:
        move_it = move()
    except rospy.ROSInterruptException: pass
    # rospy.spin()
