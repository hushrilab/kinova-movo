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
1. Get into the robot computers in two different terminals: `ssh robohub@movo1` for movo1 and `ssh robohub@movo2` for movo2. 
2. If needed (e.g. you changed the code, or you want to make sure the code is on your desired status), perform the following:
   a. On both computers, `cd Documents/fydp/movo_ws/src/kinova-movo` to access the  
   b. `git status` 
   c. `git checkout your branch`: check consistency with branches (`influenceexperiment` for this branch) 
   d. `git stash` if new things come up to save temporarily the status and bring the branch back to a clean state, `git stash pop` in the end to resume them.
   e. `cd  ../ ..`
   f. `catkin build` 
   g. `source devel/setup.bash` 
3. On movo2: `roslaunch movo_bringup movo2.launch` first;
4. While movo2 is launching, launch movo1 in the meanwhile: `roslaunch movo_bringup movo1.launch` 
5. `cd kinova_movo_ws` 
6. `source `
14. `export ROS_MASTER_URI=http://movo2:11311/`
15. Check your own IP address with `ifconfig`, then `export ROS_IP=129.97.71.96` (replace with your own IP) 
16. `roslaunch movo_7 … custom…_robot.launch`
17. move the arm if there is collision (red) 
18. `rosrun movo_7… movo_pickplace.py` 
19. Always look at the robot if there is potential collision 
20. htop 
21. Kill the process: sudo kill -9 #PID (search the process with `Fn+F3` or `F3`)

## Shut down the robot
1. Bring the robot to initial position 
2. Kill movo1 first, and then movo2 
3. cd src/kinova-movo, check repository status `git status`, then `git checkout master` 
4. `cd .. /..`, `catkin build` 
5. `sudo shutdown -h now` (on both movo1 and movo2)

## Original MOVO README
MOVO repository for the Kinova mobile manipulator

Remote PC and sim does not need movo_network or movo_robot.

Setup Instructions: https://github.com/Kinovarobotics/kinova-movo/wiki/1.-Setup-Instructions
