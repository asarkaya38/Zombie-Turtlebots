#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist

robotName = sys.argv[1]

def talker():
   pub = rospy.Publisher('/' + robotName + '/cmd_vel/', Twist, queue_size=10)
   rospy.init_node('runner', anonymous=True)
   rate = rospy.Rate(10) # 1hz

   vel_msg = Twist()
   vel_msg.linear.x = 0.5
   vel_msg.angular.z = -0.3
   
   init_str = "Runner started @ %s" % rospy.get_time()
   rospy.loginfo(init_str)

   while not rospy.is_shutdown():
      pub.publish(vel_msg)
      rate.sleep()


if __name__ == '__main__':
   try:
      talker()
   except rospy.ROSInterruptException:
      pass
