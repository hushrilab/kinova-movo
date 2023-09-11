# Kinova Movo

This fork of the Kinova Movo repo upgrades the code base to ROS Noetic.

# Dependencies
This is a list of known dependencies for installing the kinova-movo package. Some might be missing, feel free to complement:

Ubuntu packages:
- `sudo apt-get install net-tools`

ROS:

- Install ROS Noetic: `sudo apt-get install ros-noetic-desktop-full`
- Building packages: `sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`
- Dependencies: `sudo apt-get install git-core python3-wstool python3-vcstools python3-rosdep ros-noetic-control-msgs ros-noetic-xacro ros-noetic-tf2-ros ros-noetic-rviz ros-noetic-cv-bridge ros-noetic-actionlib ros-noetic-actionlib-msgs ros-noetic-dynamic-reconfigure ros-noetic-trajectory-msgs ros-noetic-rospy-message-converter ros-noetic-costmap-2d ros-noetic-move-base-msgs ros-noetic-base-local-planner ros-noetic-realsense2-description ros-noetic-ira-laser-tools ros-noetic-trac-ik-kinematics-plugins ros-noetic-moveit-planners* ros-noetic-ros-controllers ros-noetic-moveit-simple-controller-manager`

# Guidelines for using the robot
(NOTE: this part needs to be polished)

## Start the robot 
1. Check the e-stop first 
2. Switch on, turn on the computer 
3. Connect to WIFI/ Cable 

## Start the robot computer 
1. Get into the robot computer 
  a. ssh robohub@movo1 
  b. Password: - 
2. cd Documents/fydp/movo_ws/src/kinova-movo 
3. git status 
4. git checkout influenceexperiment
  a. Check consistency with branches 
5. git stash if new things come up 
  a. git stash pop in the end 
6. cd  ../ .. catkin build 
7. source devel/setup.bash 
8. roslaunch movo_bringup movo2.launch first 
   a. while movo2 is launching, launch movo1 in the meanwhile 
   b. roslaunch movo_bringup movo1.launch 
9. cd kinova_movo_ws 
10. source 
11. export ROS_MASTER_URI=http://movo2:11311/ 
  a. ifconfig 
  b. export ROS_IP=129.97.71.96 
12. roslaunch movo_7 … custom…_robot.launch 
13. move the arm if there is collision (red) 
14. rosrun movo_7… movo_piclplace.py 
15. Always look at the robot if there is potential collision 
16. htop 
17. Kill the process: sudo kill -9 #PID
  a. Fn+f3

## Shut down the robot
1. Bring the robot to initial position 
2. Kill movo1 first, and then movo2 
3. Cd src/kinova-movo 
  a. git status 
  b. git checkout master 
4. cd .. /.., catkin build 
5. sudo shutdown -h now (on both movo1 and movo2)

## Original MOVO README
MOVO repository for the Kinova mobile manipulator

Remote PC and sim does not need movo_network or movo_robot.

Setup Instructions: https://github.com/Kinovarobotics/kinova-movo/wiki/1.-Setup-Instructions
