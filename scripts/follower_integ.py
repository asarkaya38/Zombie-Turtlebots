#!/usr/bin/env python
import rospy
import sys
import tf
import time
from statistics import mean
from math import atan2, sqrt
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


robotSelfName = sys.argv[1]
robotRunnerName = sys.argv[2]

subscribeFreqHertz = 30#0.5
global stop_distance
stop_distance = 0.35 #1.0


class follower:

    # Class Initializer
    def __init__(self):

        # Set up node variables
        self.vel_msg = Twist()
        self.current_theta = 0
        self.self_position = Point()
        self.runner_position = Point()
        self.turning_left_flag = 0

        # Set up publishers and subscribers
        self.velocity_publisher = rospy.Publisher('/' + robotSelfName + '/cmd_vel/', Twist, queue_size=10)
        self.base_scan_subscriber = rospy.Subscriber('/' + robotSelfName + '/scan/', LaserScan, self.base_scan_callback)

        # Init Node
        rospy.init_node('follower', anonymous=True)
        self.rate = rospy.Rate(subscribeFreqHertz)
        
    #Laser Scanner Subscriber Callback
    def base_scan_callback(self,data):
        #Get distance to nearest obstacle in each regime around robot
        right = min(data.ranges[249:290])
        front_right = min(data.ranges[289:340])
        front = min(data.ranges[0:21] + data.ranges[339:359])
        front_left = min(data.ranges[20:70])
        left = min(data.ranges[69:110])
        
        self.right = right
        self.front_right = front_right
        self.front = front 
        self.front_left = front_left
        self.left = left
        
        self.forward_obstacle_flag = 0
        self.forward_obstacle_flag_goal = 1 # 1 = the way to the goal is blocked
        self.forward_adjust_left_flag = 0
        self.forward_adjust_right_flag = 0
        self.can_turn_right_flag = 0
        self.all_clear_flag = 1
        if data.header.stamp.secs < 1:
            self.turning_left_flag = 0
            
        
        #Raise flags based on distance in regime combinations
        if front < stop_distance or front_right < 0.25*stop_distance or front_left < 0.25*stop_distance:
            self.forward_obstacle_flag = 1
        elif right < stop_distance and front > stop_distance and front_left > stop_distance:
            self.forward_adjust_left_flag = 1
        elif right > stop_distance and front > stop_distance and front_right > stop_distance*2: #this *2 stops it from turning right when it is just a little away from the wall and not at a true "turn right" spot
            self.can_turn_right_flag = 1
        elif right > stop_distance and front > stop_distance and front_right > stop_distance:
            self.forward_adjust_right_flag = 1          
        if self.turning_left_flag == 1:
            if (right < 1.1*stop_distance and front > 1.5*stop_distance) or  min(left,right,front_left,front_right,front) == right:
                self.turned_left_flag = 1
                self.turning_left_flag = 0
        if front > 2*stop_distance:
            self.forward_obstacle_flag_goal = 0 #the way toward the obstacle is clear
        if right > stop_distance and front_right > stop_distance and front > stop_distance and front_left > stop_distance and left > stop_distance:
            self.all_clear_flag = 1

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
        self.yaw = round(euler[2],4)
        odom_data = rospy.wait_for_message('/odom/', Odometry)
        self.runner_position = odom_data.pose.pose.position
        self.runner_position.x = round(self.runner_position.x,4)
        self.runner_position.y = round(self.runner_position.y,4)
        
        self.goal = [self.runner_position.x, self.runner_position.y]
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
#####################################################################################################
    #Movement Callbacks
    
    #Calculate andgle between direction robot currently faces and direction to goal
    def yaw_to_goal(self):
        self.yaw_goal = atan2(self.goal[1]-self.self_position.y, self.goal[0]-self.self_position.x)        
        
    #Rotate to point towards goal
    def point_towards_goal(self):
        self.point_towards_goal_tolerance = 0.2
        #print('pointing towards goal')
        self.yaw_to_goal()
        #First decide if robot will turn CW or CCW
        if (self.yaw > 0 and self.yaw_goal > 0) or (self.yaw < 0 and self.yaw_goal < 0):          
            if self.yaw_goal < self.yaw:
                #turn CW
                self.turn_clockwise_towards_goal()
            else:
                #turn CCW
                self.turn_counter_clockwise_towards_goal()
        else:
            if self.yaw_goal > (self.yaw-3.14159):
                #turn CW
                self.turn_clockwise_towards_goal()
            else:
                #turn CCW
                self.turn_counter_clockwise_towards_goal()    
                
                
    #Turn CW towards goals
    def turn_clockwise_towards_goal(self):
        self.yaw_to_goal()
        while abs(self.yaw - self.yaw_goal) > self.point_towards_goal_tolerance: 
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = -15*angular_velocity
            self.velocity_publisher.publish(self.vel_msg)
            self.get_current_points()
            self.yaw_to_goal()
            #print(self.yaw - self.yaw_goal)
    
    #Turn CCW towards goal
    def turn_counter_clockwise_towards_goal(self,yaw_goal):
        self.yaw_to_goal()
        while abs(self.yaw - self.yaw_goal) > self.point_towards_goal_tolerance:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 15*angular_velocity
            self.velocity_publisher.publish(self.vel_msg)
            self.get_current_points()
            self.yaw_to_goal() 
            #print(self.yaw - self.yaw_goal)                   
                

                 

#            #Check if path straight to goal is clear. If so, break out of obstacle state
 #           self.yaw_to_goal()
  #          if abs(self.yaw - self.yaw_goal) < 0.1 and self.forward_obstacle_flag_goal == 0:
   #             self.dealing_with_obstacle = 0
    #        if sqrt((self.goal[0]-self.self_position.x)**2 + (self.goal[1]-self.self_position.y)**2) < 1.75*stop_distance:
     #           self.dealing_with_obstacle = 0
      #      self.get_current_points()
       #     self.rate.sleep()
            
 
#########################################################################################################            
#########################################################################################################
    # Euclidean distance between current pose and the goal.
    # http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
    def euclidean_distance(self):
        return sqrt(pow((self.runner_position.x - self.self_position.x), 2) + pow((self.runner_position.y - self.self_position.y), 2))
    # Calculate linear velocity to move towards goal
    # http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
    def linear_vel(self, constant=0.05):
        return constant * self.euclidean_distance()

    # Calculate difference in angle between goal and current pose
    # http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal   
    def steering_angle(self):
        return atan2(self.runner_position.y - self.self_position.y, self.runner_position.x - self.self_position.x)

    # Calculate angular velocity to move towards goal
    # http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
    def angular_vel(self, constant=1.0):
        return constant * (self.yaw_goal - self.yaw) 

################################################################################################################################
################################################################################################################################               
    # Move to goal point
    def decide_motion(self):
        if (self.euclidean_distance() >= 1 and self.all_clear_flag == 1 and self.going_around_wall_flag == 0 and self.forward_obstacle_flag == 0):
            #print('heading for target')
            self.vel_msg.linear.x = self.linear_vel()
            self.vel_msg.angular.z = self.angular_vel()           
        elif self.euclidean_distance() < 0.7:
            #print('chase ended')
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            #print('Zombies win!')
            time.sleep(10000)
        else:
            #print('going around wall')
            self.vel_msg.linear.x, self.vel_msg.angular.z = self.go_around_wall()
        self.velocity_publisher.publish(self.vel_msg)
################################################################################################################################
################################################################################################################################        

    def go_around_wall(self):
    
        #Check if runner is to the left or right of the way im currently facing
        #if to the left, turn left
        #if to the right, turn right
        
        #first, just always go left
        
        self.going_around_wall_flag = 1
        #Move around obstacle
        if self.forward_obstacle_flag == 1:
            #print('turning left')
            self.turn_left()
        elif self.forward_adjust_left_flag == 1 or self.forward_adjust_left_flag == 1:
            #print('adjusting')
            self.vel_msg.linear.x = linear_velocity
            self.vel_msg.angular.z = .5*angular_velocity#*(stop_distance-self.right)
        elif self.can_turn_right_flag == 1:
            #print('turning right')
            self.vel_msg.linear.x = 0.35*linear_velocity
            self.vel_msg.angular.z = -1.1*angular_velocity
        elif self.forward_obstacle_flag == 0 and self.forward_adjust_left_flag == 0 and self.forward_adjust_right_flag == 0 and self.can_turn_right_flag == 0:
            #print('going straight')
            self.vel_msg.linear.x = linear_velocity
            self.vel_msg.angular.z = 0
        self.get_current_points()
        self.yaw_to_goal()
        if abs(self.yaw - self.yaw_goal) < 0.03 and self.forward_obstacle_flag_goal == 0:
            #print('no more wall')
            #print(self.yaw)
            #print(self.yaw_goal) 
            #print(self.yaw - self.yaw_goal)   
            self.dealing_with_obstacle = 0
            self.going_around_wall_flag = 0 
            self.vel_msg.linear.x = 0.1*linear_velocity
            self.vel_msg.angular.z = 0
        return self.vel_msg.linear.x, self.vel_msg.angular.z
        
        
        

    
    #Turn left when stopped in front of obstacle       
    def turn_left(self):
        self.turning_left_flag = 1   
        while self.turned_left_flag == 0:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = angular_velocity
            self.velocity_publisher.publish(self.vel_msg)            
        self.turned_left_flag = 0
        self.turning_left_flag = 0    
    
    
        
        #follow the wall. Since i turned left, runner will be to the right of front. 
        #if the runner crosses in front of my view (is either straight ahead or to the left of front), back to the start of the follow algorithm
    
    
    
    
    
        #return self.vel_msg.linear.x, self.vel_msg.angular.z; 
    
              
                
                
                
                
                
                
                
                



##########################################################################################################
##########################################################################################################





    
    # Main Program
    def program(self):
    
    
        #Initialize dealing_with_obstacle flag
        self.dealing_with_obstacle = 0
        global angular_velocity
        angular_velocity = .2 #0.4
        global linear_velocity
        linear_velocity = .125 #0.6
        
        #Initialize stop distance- how far away from the wall the robot stops before hitting it
        robot_size = 0.7
        global stop_distance
        stop_distance = 0.35#1.0*robot_size
        
        #Initialize flags to zero
        self.forward_obstacle_flag = 0
        self.forward_adjust_left_flag = 0
        self.forward_adjust_right_flag = 0
        self.can_turn_right_flag = 0
        self.turned_left_flag = 0
        self.turning_left_flag = 0
        self.going_around_wall_flag = 0
    
        time.sleep(1)
        
        self.get_current_points()
        
        #Point towards runner to start
        if self.dealing_with_obstacle == 0:
            self.point_towards_goal()
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)    
        #time.sleep(2000)
        #print(self.yaw)
        #print(self.yaw_goal)
        while not rospy.is_shutdown():
            

            self.get_current_points()
            self.decide_motion()
            #print('goal x')
            #print(self.goal[0])
            #print('goal y')
            #print(self.goal[1])
            #print('my x')
            #print(self.self_position.x)
            #print('my y')
            #print(self.self_position.y)
    #        print('yaw to goal')
     #       print(self.yaw_goal)
      #      print('my yaw')
       #     print(self.yaw)
            self.rate.sleep()

if __name__ == '__main__':
   try:
      robot = follower()
      robot.program()

   except rospy.ROSInterruptException:
      pass
