#!/usr/bin/env python
import rospy
import sys
from math import atan2, sqrt
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry

robotSelfName = sys.argv[1]
robotRunnerName = sys.argv[2]

subscribeFreqHertz = 0.5

class follower:

    # Class Initializer
    def __init__(self):

        # Set up node variables
        self.vel_msg = Twist()
        self.current_theta = 0
        self.self_position = Point()
        self.runner_position = Point()

        # Set up publishers and subscribers
        self.vel_pub = rospy.Publisher('/' + robotSelfName + '/cmd_vel/', Twist, queue_size=10)

        # Init Node
        rospy.init_node('follower', anonymous=True)
        self.rate = rospy.Rate(subscribeFreqHertz)

    # Get self and runner positions
    def get_current_points(self):
        odom_data = rospy.wait_for_message('/' + robotSelfName + '/odom/', Odometry)
        self.self_position = odom_data.pose.pose.position
        self.self_position.x = round(self.self_position.x,4)
        self.self_position.y = round(self.self_position.y,4)
        quaternion = (
            odom_data.pose.pose.orientation.x,
            odom_data.pose.pose.orientation.y,
            odom_data.pose.pose.orientation.z,
            odom_data.pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        self.current_theta = round(euler[2],4)
        odom_data = rospy.wait_for_message('/' + robotRunnerName + '/odom/', Odometry)
        self.runner_position = odom_data.pose.pose.position
        self.runner_position.x = round(self.runner_position.x,4)
        self.runner_position.y = round(self.runner_position.y,4)

    # Move to goal point
    def decide_motion(self):
        if (self.euclidean_distance() >= 1):
            self.vel_msg.linear.x = self.linear_vel()
            self.vel_msg.angular.z = self.angular_vel()           
        else:
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0

        self.vel_pub.publish(self.vel_msg)
        

    # Euclidean distance between current pose and the goal.
    # http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
    def euclidean_distance(self):
        return sqrt(pow((self.runner_position.x - self.self_position.x), 2) + pow((self.runner_position.y - self.self_position.y), 2))

    # Calculate linear velocity to move towards goal
    # http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
    def linear_vel(self, constant=0.1):
        return constant * self.euclidean_distance()

    # Calculate difference in angle between goal and current pose
    # http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal   
    def steering_angle(self):
        return atan2(self.runner_position.y - self.self_position.y, self.runner_position.x - self.self_position.x)

    # Calculate angular velocity to move towards goal
    # http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
    def angular_vel(self, constant=1):
        return constant * (self.steering_angle() - self.current_theta)
    
    # Main Program
    def program(self):
        while not rospy.is_shutdown():
            self.get_current_points()
            self.decide_motion()
            self.rate.sleep()

if __name__ == '__main__':
   try:
      robot = follower()
      robot.program()

   except rospy.ROSInterruptException:
      pass
