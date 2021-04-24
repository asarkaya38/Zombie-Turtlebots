#!/usr/bin/env python  
import rospy
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

#this method will make the robot move to the goal location
def move_to_goal(xGoal,yGoal):

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

   print("Next Location:" + str(x_goal) + ", " +str(y_goal))
#    ac.wait_for_result(rospy.Duration(60))

#    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
#            rospy.loginfo("You have reached the destination")
#            return True

#    else:
#            rospy.loginfo("The robot failed to reach the destination")
#            return False

   

if __name__ == '__main__':
   rospy.init_node('nonzombie_navigation', anonymous=False)

   next_goal_topic = "/run_loc"
   next_goal = rospy.wait_for_message(next_goal_topic, Point)
   x_goal = next_goal.x
   y_goal = next_goal.y
#    x_goal = 2.02880191803
#    y_goal = -1.02200937271
   while not rospy.is_shutdown():
           move_to_goal(x_goal, y_goal)
           time.sleep(5)
   rospy.spin()
