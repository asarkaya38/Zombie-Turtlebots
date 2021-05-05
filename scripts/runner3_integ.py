#!/usr/bin/env python
import rospy
import sys
from math import atan2, sqrt
from std_msgs.msg import String
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

selfName = sys.argv[1]

# This should be made more flexible as a future addition
# This implementation hard codes for 3 zombies to prove
# the limited sensing algorithm
zombieNames = []
zombieNames.append(sys.argv[2])
zombieNames.append(sys.argv[3])
zombieNames.append(sys.argv[4])


safe_threshold = round(float(sys.argv[5]),4)
min_x = round(float(sys.argv[6]),4)
max_x = round(float(sys.argv[7]),4)
min_y = round(float(sys.argv[8]),4)
max_y = round(float(sys.argv[9]),4)

x_bounds = [min_x,max_x]
y_bounds = [min_y,max_y]

# Define Global Constants
mainProgramRate = 10
num_safe_points = 16
numZombies = 3

class runner:

    # Class Initializer
    def __init__(self):

        # Define Class Variables
        self.vel_msg = Twist()
        self.self_position = Point()
        self.current_theta = 0
        self.run_position = Point()
        self.safe_points = [[0]*2]*num_safe_points
        self.update_safe_zone()

        self.zombie_positions = []
        for i in range(numZombies):
            self.zombie_positions.append(Point())

        # Set up run point publisher
        self.run_point_pub = rospy.Publisher('/run_loc/', Point, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel/', Twist, queue_size=10)

        # Init Node
        rospy.init_node('runner', anonymous=True)

        self.rate = rospy.Rate(mainProgramRate)

    # Get self position
    def get_odom_self(self):
        odom_data = rospy.wait_for_message('/odom/', Odometry)
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
        self.update_safe_zone()
   
    # Get Zombie Positions
    def get_odom_zombies(self):
        for i in range(numZombies):
            odom_data = rospy.wait_for_message('/' + zombieNames[i] + '/odom/', Odometry)
            self.zombie_positions[i] = odom_data.pose.pose.position
            self.zombie_positions[i].x = round(self.zombie_positions[i].x,4)
            self.zombie_positions[i].y = round(self.zombie_positions[i].y,4)

    # Re-define safe zone based on runner current location
    # There's probably a more elegant way to do this
    def update_safe_zone(self):
        self.safe_points[0] = [self.self_position.x+(-safe_threshold/2),self.self_position.y+safe_threshold]
        self.safe_points[1] = [self.self_position.x+(-safe_threshold),self.self_position.y+safe_threshold]
        self.safe_points[2] = [self.self_position.x+(-safe_threshold),self.self_position.y+(safe_threshold/2)]
        self.safe_points[3] = [self.self_position.x+(-safe_threshold),self.self_position.y]
        self.safe_points[4] = [self.self_position.x+(-safe_threshold),self.self_position.y+(-safe_threshold/2)]
        self.safe_points[5] = [self.self_position.x+(-safe_threshold),self.self_position.y+(-safe_threshold)]
        self.safe_points[6] = [self.self_position.x+(-safe_threshold/2),self.self_position.y+(-safe_threshold)]
        self.safe_points[7] = [self.self_position.x,self.self_position.y+(-safe_threshold)]
        self.safe_points[8] = [self.self_position.x+(safe_threshold/2),self.self_position.y+(-safe_threshold)]
        self.safe_points[9] = [self.self_position.x+safe_threshold,self.self_position.y+(-safe_threshold)]
        self.safe_points[10] = [self.self_position.x+safe_threshold,self.self_position.y+(-safe_threshold/2)]
        self.safe_points[11] = [self.self_position.x+safe_threshold,self.self_position.y]
        self.safe_points[12] = [self.self_position.x+safe_threshold,self.self_position.y+(safe_threshold/2)]
        self.safe_points[13] = [self.self_position.x+safe_threshold,self.self_position.y+safe_threshold]
        self.safe_points[14] = [self.self_position.x+(safe_threshold/2),self.self_position.y+safe_threshold]
        self.safe_points[15] = [self.self_position.x,self.self_position.y+safe_threshold]
        self.max_safe_x = self.self_position.x+safe_threshold
        self.min_safe_x = self.self_position.x-safe_threshold
        self.max_safe_y = self.self_position.y+safe_threshold
        self.min_safe_y = self.self_position.y-safe_threshold

    # Find the next safe point to move to
    def find_safe_point(self):
        average_zombie_point = Point()
        numZombiesInSafeZone = 0

        # Find the average zombie point to run away from
        for i in range(numZombies):
            if self.zombie_in_safe_zone(self.zombie_positions[i]):
                numZombiesInSafeZone = numZombiesInSafeZone + 1
                average_zombie_point.x = average_zombie_point.x + self.zombie_positions[i].x
                average_zombie_point.y = average_zombie_point.y + self.zombie_positions[i].y

        # Find the farthest safe point from the average zombie point
        if (numZombiesInSafeZone > 0):
            average_zombie_point.x = average_zombie_point.x / numZombiesInSafeZone
            average_zombie_point.y = average_zombie_point.y / numZombiesInSafeZone

            max_euclidean = 0
            max_index = 0
            current_euclidean = 0   
            current_safe_point = Point()

            for i in range(num_safe_points):
                current_safe_point.x = self.safe_points[i][0]
                current_safe_point.y = self.safe_points[i][1]
                if self.bounds_check(current_safe_point,x_bounds,y_bounds):
                    current_euclidean = abs(self.euclidean_distance(current_safe_point,average_zombie_point))
                    if current_euclidean > max_euclidean:
                        max_euclidean = current_euclidean
                        max_index = i

            self.run_position.x = self.safe_points[max_index][0]
            self.run_position.y = self.safe_points[max_index][1]
            self.run_point_pub.publish(self.run_position)

    # Euclidean distance between current pose and the goal.
    # http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
    def euclidean_distance(self,a,b):
        return sqrt(pow((b.x - a.x), 2) + pow((b.y - a.y), 2))

    # Decide if the given zombie is in the current safe zone
    def zombie_in_safe_zone(self,zombie_point):
        if zombie_point.x < self.max_safe_x and zombie_point.x > self.min_safe_x and zombie_point.y < self.max_safe_y and zombie_point.y > self.min_safe_y:
            return True
        else:
            return False

    # Check to see if the current point is in the defined world bounds
    def bounds_check(self,point,x_bounds,y_bounds):
        if (point.x > x_bounds[0] and point.x < x_bounds[1] and point.y > y_bounds[0] and point.y < y_bounds[1]):
            return True
        else:
            return False

    # Main Program
    def program(self):
        while not rospy.is_shutdown():
            self.get_odom_self()
            self.get_odom_zombies()
            self.find_safe_point()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        robot = runner()
        robot.program()
    except rospy.ROSInterruptException:
        pass
