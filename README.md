# multi_robot
# Before installing Turtlebot3 make sure to run
$ sudo apt-get update
$ sudo apt-get upgrade

# Assuming ROS Noetic
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b noetic-devel
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b noetic-devel
$ cd ~/catkin_ws && catkin_make

# For TB3 simulation packages
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make

# Setting some env variables, open the .bashrc file
$ cd
$ gedit .bashrc

# Add the following at the end of the file and save it
$ export TURTLEBOT3_MODEL=waffle
