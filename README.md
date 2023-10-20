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


# Installation

1. If you do not have one already, create a ros workspace following the instructions here: https://wiki.ros.org/catkin/Tutorials/create_a_workspace
2. Clone this repository into the src folder and change to the `influenceexperiment` branch with `git checkout influenceexperiment`
3. Build your workspace with catkin build or catkin_make (as you prefer, or as you were doing with your workspace)

# Guidelines for using the robot
(NOTE: this part needs to be polished)

## Start the robot computers 
1. Check the e-stop at the back of the robot first, it has to be disabled for the robot to start properly.
2. Switch on the main switch if not already (there is a small lever under the cover of the robot on the same side as the status and battery LEDs).
3. Turn on the robot computers with the switch on the back.
4. Connect your local machine to the same network as the Movo computers via the RoboHub wifi or a cabled ethernet connection inside the RoboHub.
5. In a terminal, `ping movo1` and wait until you receive responses, then do the same thing `ping movo2` for the second onboard computer, they both must be on to proceed.

## Start the robot
1. Get into the robot computers in two different terminals: `ssh robohub@movo1` for movo1 and `ssh robohub@movo2` for movo2. 
2. If needed (e.g. you changed the code, or you want to make sure the code is on your desired status), perform the following:
    1. On both computers, `cd Documents/fydp/movo_ws/src/kinova-movo` to access the  
    2. `git status` 
    3. `git checkout your branch`: check consistency with branches (`influenceexperiment` for this branch) 
    4. `git stash` if new things come up to save temporarily the status and bring the branch back to a clean state, `git stash pop` in the end to resume them.
    5. `cd  ../ ..`
    6. `catkin build` 
    7. `source devel/setup.bash` 
3. On movo2: `roslaunch movo_bringup movo2.launch` first;
4. While movo2 is launching, launch movo1 in the meanwhile: `roslaunch movo_bringup movo1.launch` 
5. Enter the workspace where you have the kinova-movo repository (this repository)`cd your_ws` 
6. Source the setup files on both movo1 and movo2 with `source devel/setup.bash`
7. On your local machine, open a terminal and export the ros master ip: `export ROS_MASTER_URI=http://movo2:11311/`
8. In the same terminal, check your own IP address with `ifconfig`, then export the ros ip `export ROS_IP=129.97.71.96` (replace with your own IP)
9. Now check that you are connected with `rostopic list`, you should see all the topics of Movo. If not, you need to double check your connection and settings.
10. NOTE: every time you open a new terminal or source again setup.bash that needs to be connected to the robot, you must perform steps 6 to 8 in the new terminal.

## Running the experiment
To launch moveit with RViz:
```
roslaunch movo_7dof_moveit_config custom_moveit_rviz_robot.launch
```
This will launch the moveit node on your local machine and RViz.

At this point, you should be able to see if there are any collisions with the arms and should be able to use moveit to plan motions on the robot. If there are collisions (red areas in the visualization), press the emergency button to make the arms passive, move the arms out of collision and de-press the emergency button.

To launch the pickplace experiment sequence, open another terminal (remember to source and set your ips), then do the following:
```
rosrun movo_7dof_moveit_config movo_pickplace.py
```
**Always look at the robot if there is potential collision!!**

The process sometimes needs to be killed because it cannot be killed with CTRL+C. In a terminal:
- `htop` (sudo apt-get install htop if you do not have it)
- search the process with `Fn+F3` or `F3`
- Kill the process: sudo kill -9 #PID 

## Shut down the robot
1. Bring the robot to the initial/home position 
2. Kill the ros processes on movo1 first, and then movo2 
3. Check the status of the repositories, if you made any modifications that need to be saved, make sure to commit and push.
4. If you stashed some modifications (see point 2 in `Starting the robot`), unstash them and build the repositories on both computers
5. On both movo1 and movo2: `sudo shutdown -h now`
6. Turn off the robot by pressing the power button

# Troubleshooting
- If you are unable to start the robot, check that the emergency button is disengaged. If it was engaged at the startup of the robot, disengage it and restart the entire system.
- If one or more arms are not responding (the motors do not start, unable to connect):
    - Check if the arm is properly connected on the back of the robot and that the switch is turned on (the red switch on the arm control boxes (squared boxes).
    - Check if the fuse of the arm is not blown. Take off the front cover, on the right side there are 4 fuses, the number 5s are for the arms. Take them out carefully and check that they are not blown (the wire inside must be intact).
      
## Original MOVO README
MOVO repository for the Kinova mobile manipulator

Remote PC and sim does not need movo_network or movo_robot.

Setup Instructions: https://github.com/Kinovarobotics/kinova-movo/wiki/1.-Setup-Instructions
