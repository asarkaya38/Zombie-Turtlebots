import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry

robotSelfName = sys.argv[1]
robotFollowName = sys.argv[2]

subscribeFreqHertz = 1

class follower:

    # Class Initializer
    def __init__(self):
        # Set uo publishers and subscribers
        self.odom_sub = rospy.Subscriber('/' + robotFollowName + '/odom/', Odometry, self.get_odom)
        self.vel_pub = rospy.Publisher('/' + robotSelfName + '/cmd_vel/', Twist, queue_size=10)

        # Init Node
        rospy.init_node('follower', anonymous=True)

        # Set up internal variables
        self.position = Point()
        self.rate = rospy.Rate(subscribeFreqHertz)

    # Odometry Subscriber Callback
    def get_odom(self,data):
        self.position = data.pose.pose.position
    

    # Main Program
    def program(self):
        while not rospy.is_shutdown():
            init_str = "X: {0}  Y: {1}".format(self.position.x,self.position.y)
            rospy.loginfo(init_str)
            self.rate.sleep()

if __name__ == '__main__':
   try:
      robot = follower()
      robot.program()

   except rospy.ROSInterruptException:
      pass