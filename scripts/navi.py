#!/usr/bin/env python import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

goal_pos = Pose()

class map_navigation():

  def choose(self):

    choice='q'

    rospy.loginfo("|-------------------------------|")
    rospy.loginfo("|PRESSE A KEY:")
    rospy.loginfo("|'0': Cafe ")
    rospy.loginfo("|'1': Office 1 ")
    rospy.loginfo("|'2': Office 2 ")
    rospy.loginfo("|'3': Office 3 ")
    rospy.loginfo("|'q': Quit ")
    rospy.loginfo("|-------------------------------|")
    rospy.loginfo("|WHERE TO GO?")
    choice = input()
    return choice

  def __init__(self):


    # declare the coordinates of interest
    self.xCafe = 13.50
    self.yCafe = 20.20
    self.xOffice1 = 27.70
    self.yOffice1 = 12.50
    self.xOffice2 = 30.44
    self.yOffice2 = 12.50
    self.xOffice3 = 35.20
    self.yOffice3 = 13.50
    self.goalReached = False
    # initiliaze
    # rospy.init_node('map_navigation', anonymous=False)
    # choice = self.choose()
    choice = 0
    if (choice == 0):
      print goal_pos
      self.goalReached = self.moveToGoal(goal_pos.position.x,goal_pos.position.y)

    elif (choice == 1):

      self.goalReached = self.moveToGoal(self.xOffice1, self.yOffice1)

    elif (choice == 2):

      self.goalReached = self.moveToGoal(self.xOffice2, self.yOffice2)

    elif (choice == 3):

      self.goalReached = self.moveToGoal(self.xOffice3, self.yOffice3)

    if (choice!='q'):

      if (self.goalReached):
        rospy.loginfo("Congratulations!")
        #rospy.spin()
      else:
         rospy.loginfo("Hard Luck!")


    while choice != 'q':
      choice = self.choose()
      if (choice == 0):

        self.goalReached = self.moveToGoal(goal_pos.position.x,oal_pos.position.y)

      elif (choice == 1):

        self.goalReached = self.moveToGoal(self.xOffice1, self.yOffice1)

      elif (choice == 2):

        self.goalReached = self.moveToGoal(self.xOffice2, self.yOffice2)

      elif (choice == 3):

        self.goalReached = self.moveToGoal(self.xOffice3, self.yOffice3)

      if (choice!='q'):

        if (self.goalReached):
          rospy.loginfo("Congratulations!")
          #rospy.spin()
        else:
          rospy.loginfo("Hard Luck!")


  def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep()

def moveToGoal(xGoal,yGoal):

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")


    goal = MoveBaseGoal()

    #set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal*/

    goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True

    else:
            rospy.loginfo("The robot failed to reach the destination")
            return False
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.poses[4])
    goal_pos = data.poses[4]
    moveToGoal(goal_pos.position.x,goal_pos.position.y)
    # self.goal_rc = data.poses[1]

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("goto_points", PoseArray, callback)
    # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        listener()

        # map_navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
