#!/usr/bin/env python

# ============================ IMPORTS =============================
import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import numpy as np
from std_msgs.msg import String, Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix,quaternion_from_matrix
import pick_and_place_constants_without_user as const
#import pick_and_place_constants_without_user_45 as const
from gazebo_msgs.srv import SpawnModel,SpawnModelRequest
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
# from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes, Grasp, GripperTranslation, PlaceLocation, CollisionObject
from movo_action_clients.gripper_action_client import GripperActionClient
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Transform
from math import ceil
from movo_gl import MovoGl
from publisher_ee_state import PublisherEeState
from collect_data_movo import CollectDataMovo
import threading
# ==================================================================

class MovoPickPlace(MovoGl):
	""" MovoPickPlace """

	# ============================ INITIALIZATION ============================
	def __init__(self):

		MovoGl.__init__(self, const)

		self.move_groups = []
		self.gripper_groups = []
		self.touch_links_groups = []
		self.eef_links = []

		for i in range(const.NB_GROUPS):

			group_name = const.GROUP_NAME[i]
			gripper_name = const.GRIPPER_NAME[i]
			move_group = moveit_commander.MoveGroupCommander(group_name)
			move_group.set_planning_time(const.PLANNING_TIME)
			move_group.set_num_planning_attempts(const.PLANNING_ATTEMPTS)
			if i == 0 :
				gripper_group = GripperActionClient('right')
			else :
				gripper_group = GripperActionClient('left')
			touch_links = self.robot.get_link_names(group=gripper_name)
			for item in self.robot.get_link_names(group=const.GRIPPER_GROUP[i]):
				if item not in touch_links:
					touch_links.append(item)
			eef_link = move_group.get_end_effector_link()

			self.move_groups.append(move_group)
			self.gripper_groups.append(gripper_group)
			self.touch_links_groups.append(touch_links)
			self.eef_links.append(eef_link)

		self.const = const
		self.move_initial_pose()
		self.joints_right_relax =  self.move_group_gl.get_current_joint_values()[10:]
		self.joints_left_relax = self.move_group_gl.get_current_joint_values()[1:8]

		self.pub_ee_state = PublisherEeState(self.move_groups[0], self.move_groups[1])
		self.movo_data_collecter = CollectDataMovo("pickplace")

		# Create one thread for publisher
		t1 = threading.Thread(name="ee", target=self.pub_ee_state.ee_state_talker, args=())
		t1.start()

		rospy.on_shutdown(self.clean_shutdown)

		print("Initialization done for Movo PICK & PLACE task.")
		print("")
	# ========================================================================


	# ============================= PICK PIPELINE ============================
	def pick(self, obj_to_grasp, id_group):

		if self.const.IS_45 == True:
			head_joint_cmd = [0.55,-0.25]
		else:
			head_joint_cmd = [0.8,-0.25]
			
		print("===> Starting Picking sequence sequence")

		move_group = self.move_groups[id_group]
		eef_link = self.eef_links[id_group]
		touch_links = self.touch_links_groups[id_group]
		self.pose_obj = self.scene.get_object_poses([obj_to_grasp]).get(obj_to_grasp)

		# DETERMINE GRIPPER ORIENTATION
		quat_obj = self.pose_obj.orientation #odom
		rot_obj = quaternion_matrix([quat_obj.x,quat_obj.y,quat_obj.z,quat_obj.w]) #odom
		orientation = np.dot(rot_obj,np.dot(self.R_constraints_wrist_r,self.R_base_wrist_r)) # wrist
		orientation = np.dot(orientation,np.linalg.inv(self.R_base_wrist_r)) #odom
		quat_goal = quaternion_from_matrix(orientation)

		# MOVE TO APPROACH GRASP POSE
		# Get object pose in base link frame + wrist orientation
		obj_pose_base = PoseStamped()
		obj_pose_base.header.frame_id = move_group.get_planning_frame()
		obj_pose_base.pose.position.x = self.pose_obj.position.x
		obj_pose_base.pose.position.y = self.pose_obj.position.y
		obj_pose_base.pose.position.z = self.pose_obj.position.z
		obj_pose_base.pose.orientation = Quaternion(quat_goal[0],quat_goal[1],quat_goal[2],quat_goal[3])
		dim_z = self.get_dim_obj(obj_to_grasp,2)
		if id_group == 0: #right grabs from bottom object
			obj_pose_base.pose.position.z = obj_pose_base.pose.position.z - dim_z/4
		elif id_group == 1: #left grabs from up
			obj_pose_base.pose.position.z = obj_pose_base.pose.position.z + dim_z/4
		# change ref frame from base to wrist
		obj_pose_wrist = np.dot(self.R_base_wrist_r,np.array([obj_pose_base.pose.position.x,obj_pose_base.pose.position.y,obj_pose_base.pose.position.z,1]))
		obj_pose_wrist = np.dot(obj_pose_wrist,rot_obj)
		#add in approach in wrist frame
		dim_y = self.get_dim_obj(obj_to_grasp,1)
		obj_pose_wrist[0] = obj_pose_wrist[0] - self.const.APPROACH_PICK_X[id_group] - self.const.MARGIN_X[id_group]
		obj_pose_wrist[1] = obj_pose_wrist[1] - self.const.APPROACH_PICK_Y[id_group] - self.const.MARGIN_Y[id_group]
		obj_pose_wrist[2] = obj_pose_wrist[2] +  self.const.APPROACH_PICK_Z[id_group] + self.const.MARGIN_Z[id_group] + dim_y/2
		obj_pose_wrist = np.dot(rot_obj,obj_pose_wrist)
		# change ref frame from wrist to base
		obj_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), obj_pose_wrist)
		# define goal pose in base frame
		obj_pose_base.pose.position.x = obj_pose_base_v[0]
		obj_pose_base.pose.position.y = obj_pose_base_v[1]
		obj_pose_base.pose.position.z = obj_pose_base_v[2]

		arm_joint_cmd = self.get_joints_from_pose(id_group,move_group,obj_pose_base,"grasp_approach_pose")
		current_joints = self.move_group_gl.get_current_joint_values()
		joint_cmd = current_joints
		joint_cmd[10:] = arm_joint_cmd
		joint_cmd[8:10] = head_joint_cmd

		print(" Grasp approach pose")
		input()

		self.execute_joints_cmd(self.move_group_gl,joint_cmd,"grasp_approach_head_turn")

		# OPEN GRIPPER
		self.open_gripper(id_group)

		# MOVE TO GRASP POSE
		grasp_approach_pose= obj_pose_base
		grasp_pose_base = grasp_approach_pose
		# change ref frame from base to wrist
		grasp_pose_wrist = np.dot(self.R_base_wrist_r,np.array([grasp_pose_base.pose.position.x,grasp_pose_base.pose.position.y,grasp_pose_base.pose.position.z,1]))
		grasp_pose_wrist = np.dot(grasp_pose_wrist,rot_obj)
		grasp_pose_wrist[0] = grasp_pose_wrist[0] + self.const.APPROACH_PICK_X[id_group]
		grasp_pose_wrist[1] = grasp_pose_wrist[1] + self.const.APPROACH_PICK_Y[id_group]
		grasp_pose_wrist[2] = grasp_pose_wrist[2] -  self.const.APPROACH_PICK_Z[id_group] - self.const.DIST
		grasp_pose_wrist = np.dot(rot_obj,grasp_pose_wrist)
		# change ref frame from wrist to base
		grasp_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), grasp_pose_wrist)
		# define goal pose in base frame
		grasp_pose_base.pose.position.x = grasp_pose_base_v[0]
		grasp_pose_base.pose.position.y = grasp_pose_base_v[1]
		grasp_pose_base.pose.position.z = grasp_pose_base_v[2]

		print(" Grasp pose")
		input()
		success = self.move_attempt(id_group,move_group, grasp_pose_base, "grasp_pose")

		print(" Close gripper")
		input()
		# CLOSE GRIPPER
		self.close_gripper(id_group)

		# ATTACH OBJECT
		self.scene.attach_box(eef_link, obj_to_grasp, touch_links=touch_links)
		rospy.sleep(1)

		# MOVE TO RETREAT POSE
		pick_retreat_pose_base = grasp_pose_base
		# change ref frame from base to wrist
		pick_retreat_pose_wrist = np.dot(self.R_base_wrist_r,np.array([pick_retreat_pose_base.pose.position.x,pick_retreat_pose_base.pose.position.y,pick_retreat_pose_base.pose.position.z,1]))
		pick_retreat_pose_wrist = np.dot(pick_retreat_pose_wrist,rot_obj)
		pick_retreat_pose_wrist[0] = pick_retreat_pose_wrist[0] - self.const.RETREAT_X[id_group]
		pick_retreat_pose_wrist[1] = pick_retreat_pose_wrist[1] - self.const.RETREAT_Y[id_group]
		pick_retreat_pose_wrist[2] = pick_retreat_pose_wrist[2] +  self.const.RETREAT_Z[id_group]
		pick_retreat_pose_wrist = np.dot(rot_obj,pick_retreat_pose_wrist)
		# change ref frame from wrist to base
		pick_retreat_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), pick_retreat_pose_wrist)
		# define goal pose in base frame
		pick_retreat_pose_base.pose.position.x = pick_retreat_pose_base_v[0]
		pick_retreat_pose_base.pose.position.y = pick_retreat_pose_base_v[1]
		pick_retreat_pose_base.pose.position.z = pick_retreat_pose_base_v[2]

		print ("Retreat pose")
		input()
		success = self.move_attempt(id_group,move_group, pick_retreat_pose_base, "pick_retreat_pose")


		print ("Pick pipeline done.")
		print ("")

		return pick_retreat_pose_base
	# ========================================================================


	# ============================ ATTEMPT TO MOVE ===========================
	def move_attempt(self, id_group, group, pose, name):

		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 20:
			print ("Move attempt - try number: %s" % nb_try)
			plan = group.plan()
			is_ok = self.check_plan(id_group,group,plan,name)
			nb_try = nb_try + 1
		if is_ok:
			print ("Executing plan found to move to %s." % name)
			group.execute(plan[1])
			group.stop()
			group.clear_pose_targets()
			print ("Position %s reached." % name)
			print ("")
			return True
		else :
			print ("[ERROR] No plan found to move to %s." % name)
			print ("")
			sys.exit(1)
	# ========================================================================


	# ======================== GET POSE JOINTS' VALUES =======================
	def get_plan_from_pose(self, id_group, group, pose, name):

		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 20:
			print ("Move attempt - try number: %s" % nb_try)
			plan = group.plan()
			is_ok = self.check_plan(id_group,group,plan,name)
			nb_try = nb_try + 1
		if is_ok:
			return plan
		else:
			print ("[ERROR] No plan found to move to %s." % name)
			print ("")
			sys.exit(1)
	# ========================================================================


	# ======================== EXECUTE PLAN COMMAND =======================
	def execute_plan_cmd(self, move_group, plan, name):

		print ("Executing plan found to %s." % name)
		move_group.execute(plan[1])
		move_group.stop()
		move_group.clear_pose_targets()
		print ("Position reached.")
		print ("")

	# ========================================================================


	# ========================= MOVE TO SWITCH POINT =========================
	def move_switch_point(self, obj_to_grasp):

		head_joint_cmd = [0.0,-0.3]
		dim_y = self.get_dim_obj(obj_to_grasp,1)

		# FIRST GROUP MOVES TO SWITCH POINT
		p = PoseStamped()
		p.header.frame_id = self.move_groups[0].get_planning_frame()
		p.pose.position.x = self.const.SWITCH_POSITION.get("x") - self.const.MARGIN_X[0]
		p.pose.position.y = self.const.SWITCH_POSITION.get("y") - self.const.MARGIN_Y[0]
		p.pose.position.z = self.const.SWITCH_POSITION.get("z") - self.const.MARGIN_Z[0] - dim_y/2
		quaternion = quaternion_from_euler(self.const.EULER_SWITCH[0][0],self.const.EULER_SWITCH[0][1],self.const.EULER_SWITCH[0][2])
		p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
		plan_right = self.get_plan_from_pose(0,self.move_groups[0],p,"move_right_arm_to_switch_point")

		# SECOND GROUP MOVES TO SWITCH POINT AND GRABS
		self.open_gripper(1)
		p.pose.position.x = p.pose.position.x + self.const.OFFSET_X
		p.pose.position.y = p.pose.position.y + self.const.APPROACH_PICK_Y[1] - 2 * self.const.MARGIN_Y[1]
		p.pose.position.z = p.pose.position.z + self.const.APPROACH_PICK_Z[1]
		quaternion = quaternion_from_euler(self.const.EULER_SWITCH[1][0],self.const.EULER_SWITCH[1][1],self.const.EULER_SWITCH[1][2])
		p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
		plan_left = self.get_plan_from_pose(1,self.move_groups[1],p,"approach_left_arm_near_switch_point")

		plan_both_arms = self.combine_plans(plan_right,plan_left)
		self.move_group_head.set_joint_value_target(head_joint_cmd)
		plan_head = self.move_group_head.plan()
		plan_both_arms_head = self.combine_plans(plan_both_arms,plan_head)

		print ("Switch point action")
		input()
		self.execute_plan_cmd(self.move_group_gl,plan_both_arms_head,"right_switch_left_approach_head")

		print ("Left to switch point")
		input()
		p.pose.position.y = p.pose.position.y - self.const.APPROACH_PICK_Y[1]
		success = self.move_attempt(1,self.move_groups[1], p, "switch_point")

		print ("Close left gripper")
		input()
		self.close_gripper(1)
		self.scene.remove_attached_object(self.move_groups[0].get_end_effector_link(), name=obj_to_grasp)
		self.scene.remove_attached_object(self.eef_links[0], name=obj_to_grasp)
		self.scene.attach_box(self.eef_links[1], obj_to_grasp, touch_links=self.touch_links_groups[1])
		rospy.sleep(1)
		# FIRST GROUP UNGRASPS
		print ("Open right gripper")
		input()
		self.open_gripper(0)

		#MOVING AWAY
		p.pose.position.y = p.pose.position.y + self.const.RETREAT_Y[1] + self.const.MARGIN_Y[1]/2
		plan_left = self.get_plan_from_pose(1,self.move_groups[1],p,"move_away_left_arm")
		self.move_groups[0].set_joint_value_target(self.joints_right_relax)

		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 20:
			print ("Move attempt - try number: %s" % nb_try)
			plan_right = self.move_groups[0].plan()
			is_ok = self.check_plan(0,self.move_groups[0],plan_right,"relax_right")
			nb_try = nb_try + 1

		plan_both_arms = self.combine_plans(plan_right,plan_left)
		print ("Move away left - relax right")
		input()
		self.execute_plan_cmd(self.move_group_gl,plan_both_arms_head,"right_switch_left_approach_head")
		#self.execute_plan_cmd(self.move_l_group,plan_both_arms,"left_away_right_relax")

		return p.pose
	# ========================================================================


	# ============================= PLACE PIPELINE ===========================
	def place(self, obj_to_grasp, last_pose, id_group):

		if self.const.IS_45 == True:
			head_joint_cmd = [-0.55,-0.25]
		else:
			head_joint_cmd = [-0.8,-0.25]

		move_group = self.move_groups[id_group]
		eef_link = self.eef_links[id_group]
		touch_links = self.touch_links_groups[id_group]

		# DETERMINE GRIPPER ORIENTATION
		quat_obj = self.pose_obj.orientation #base
		rot_obj = quaternion_matrix([quat_obj.x,quat_obj.y,quat_obj.z,quat_obj.w]) #base
		rot_obj = np.linalg.inv(rot_obj)
		orientation = np.dot(rot_obj,np.dot(self.R_constraints_wrist_l,self.R_base_wrist_l)) # wrist
		orientation = np.dot(orientation,np.linalg.inv(self.R_base_wrist_l)) #base
		quat_goal = quaternion_from_matrix(orientation)

		# MOVE TO APPROACH PLACE POSE
		dim_y = self.get_dim_obj(obj_to_grasp,1)
		place_approach_pose_base = self.pose_obj
		place_approach_pose_base.position.x = self.pose_obj.position.x + self.const.OBJ_POS_GRASP.get(obj_to_grasp).get("translation")[0]
		place_approach_pose_base.position.y = self.pose_obj.position.y + self.const.OBJ_POS_GRASP.get(obj_to_grasp).get("translation")[1]
		place_approach_pose_base.position.z = self.pose_obj.position.z + self.const.OBJ_POS_GRASP.get(obj_to_grasp).get("translation")[2]
		place_approach_pose_base.orientation = Quaternion(quat_goal[0],quat_goal[1],quat_goal[2],quat_goal[3])
		# change ref frame from base to wrist
		place_approach_pose_wrist = np.dot(self.R_base_wrist_l,np.array([place_approach_pose_base.position.x,place_approach_pose_base.position.y,place_approach_pose_base.position.z,1]))
		place_approach_pose_wrist = np.dot(place_approach_pose_wrist,rot_obj)
		#add in approach in wrist frame
		place_approach_pose_wrist[0] = place_approach_pose_wrist[0] + self.const.APPROACH_PLACE_X[id_group] + self.const.MARGIN_X[id_group]
		place_approach_pose_wrist[1] = place_approach_pose_wrist[1] - self.const.APPROACH_PLACE_Y[id_group] - self.const.MARGIN_Y[id_group] - (ceil(self.const.APPROACH_PICK_Y[id_group] * 10 ** 2) / 10 ** 2) #-  dim_y/2  #- self.const.DIST
		if quat_obj.w != 1.0:
			place_approach_pose_wrist[1] = place_approach_pose_wrist[1] - 0.01
		place_approach_pose_wrist[2] = place_approach_pose_wrist[2] + self.const.APPROACH_PLACE_Z[id_group] + self.const.MARGIN_Z[id_group] +  dim_y/2 #+ (ceil(self.const.APPROACH_PICK_Z[id_group]/2 * 10 ** 2) / 10 ** 2)
		place_approach_pose_wrist = np.dot(rot_obj,place_approach_pose_wrist)
		# change ref frame from wrist to base
		place_approach_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_l), place_approach_pose_wrist)
		# define goal pose in base frame
		place_approach_pose_base.position.x = place_approach_pose_base_v[0]
		place_approach_pose_base.position.y = place_approach_pose_base_v[1]
		place_approach_pose_base.position.z = place_approach_pose_base_v[2]

		plan_left = self.get_plan_from_pose(id_group,move_group,place_approach_pose_base,"place_approach_pose")
		self.move_group_head.set_joint_value_target(head_joint_cmd)
		plan_head = self.move_group_head.plan()
		plan_arm_head = self.combine_plans(plan_left,plan_head)

		print ("Place approach pose")
		input()
		self.execute_plan_cmd(self.move_group_gl,plan_arm_head,"place_approach_head")

		# MOVE TO PLACE POSE
		place_pose_base = place_approach_pose_base
		# change ref frame from base to wrist
		place_pose_wrist = np.dot(self.R_base_wrist_l,np.array([place_approach_pose_base.position.x,place_approach_pose_base.position.y,place_approach_pose_base.position.z,1]))
		place_pose_wrist = np.dot(place_pose_wrist,rot_obj)
		place_pose_wrist[0] = place_pose_wrist[0] - self.const.APPROACH_PLACE_X[id_group]
		place_pose_wrist[1] = place_pose_wrist[1] + self.const.APPROACH_PLACE_Y[id_group]
		place_pose_wrist[2] = place_pose_wrist[2] - self.const.APPROACH_PLACE_Z[id_group]
		place_pose_wrist = np.dot(rot_obj,place_pose_wrist)
		# change ref frame from wrist to base
		place_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_l), place_pose_wrist)
		# define goal pose in base frame
		place_pose_base.position.x = place_pose_base_v[0]
		place_pose_base.position.y = place_pose_base_v[1]
		place_pose_base.position.z = place_pose_base_v[2]
		print ("Place pose")
		input()
		success = self.move_attempt(id_group,move_group, place_pose_base, "place_pose")

		# OPEN GRIPPER
		print ("Open gripper")
		input()
		self.open_gripper(id_group)

		# DETACH OBJECT
		self.scene.remove_attached_object(eef_link, name=obj_to_grasp)
		rospy.sleep(1)
		# RETURN TO RETREAT POSE
		place_retreat_pose_base = place_pose_base
		# change ref frame from base to wrist
		place_retreat_pose_wrist = np.dot(self.R_base_wrist_l,np.array([place_retreat_pose_base.position.x,place_retreat_pose_base.position.y,place_retreat_pose_base.position.z,1]))
		place_retreat_pose_wrist = np.dot(place_retreat_pose_wrist, rot_obj)
		place_retreat_pose_wrist[0] = place_retreat_pose_wrist[0] - self.const.RETREAT_PL_X[id_group]
		place_retreat_pose_wrist[1] = place_retreat_pose_wrist[1] - self.const.RETREAT_PL_Y[id_group]
		place_retreat_pose_wrist[2] = place_retreat_pose_wrist[2] - self.const.RETREAT_PL_Z[id_group]
		place_retreat_pose_wrist = np.dot(rot_obj,place_retreat_pose_wrist)
		# change ref frame from wrist to base
		place_retreat_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_l), place_retreat_pose_wrist)
		# define goal pose in base frame
		place_retreat_pose_base.position.x = place_retreat_pose_base_v[0]
		place_retreat_pose_base.position.y = place_retreat_pose_base_v[1]
		place_retreat_pose_base.position.z = place_retreat_pose_base_v[2]

		head_joint_cmd = [0.0,0.0]
		plan_left = self.get_plan_from_pose(id_group,move_group,place_retreat_pose_base,"place_retreat_pose")
		self.move_group_head.set_joint_value_target(head_joint_cmd)
		plan_head = self.move_group_head.plan()
		plan_arm_head = self.combine_plans(plan_left,plan_head)
		print ("Place retreat pose")
		input()
		self.execute_plan_cmd(self.move_group_gl,plan_arm_head,"place_retreat_head")

		print ("Place pipeline done.")
		print ("")
	# ========================================================================

	# ======================== GET POSE JOINTS' VALUES =======================
	def get_joints_from_pose(self,id_group, group, pose, name):

		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 10:
			print ("Move attempt - try number: %s" % nb_try)
			plan = group.plan()
			is_ok = self.check_plan(id_group,group,plan,name)
			nb_try = nb_try + 1

		if is_ok:
			nb_pts = len(plan[1].joint_trajectory.points)
			last_pt = plan[1].joint_trajectory.points[nb_pts-1]
			joints = last_pt.positions
			return joints
		else:
			print ("[ERROR] No plan found to move to %s." % name)
			print ("")
			sys.exit(1)

	# ========================================================================


	# ========================================================================
	def combine_plans(self,p1,p2):
		nb_pts1 = len(p1[1].joint_trajectory.points)
		nb_pts2 = len(p2[1].joint_trajectory.points)
		Nmin = min(nb_pts1,nb_pts2)
		if Nmin == nb_pts1:
			print ("min right")
			plan = p1
			plan_l = p2
			Nmax = nb_pts2
		else:
			print ("min left")
			plan = p2
			plan_l = p1
			Nmax =  nb_pts1
		plan_l[1].joint_trajectory.joint_names.extend(plan[1].joint_trajectory.joint_names)

		for i in range(Nmax):
			if i < Nmin:
				plan_l[1].joint_trajectory.points[i].positions = plan_l[1].joint_trajectory.points[i].positions + plan[1].joint_trajectory.points[i].positions
				plan_l[1].joint_trajectory.points[i].velocities = plan_l[1].joint_trajectory.points[i].velocities + plan[1].joint_trajectory.points[i].velocities
				plan_l[1].joint_trajectory.points[i].accelerations = plan_l[1].joint_trajectory.points[i].accelerations + plan[1].joint_trajectory.points[i].accelerations
			else:
				plan_l[1].joint_trajectory.points[i].positions = plan_l[1].joint_trajectory.points[i].positions + plan[1].joint_trajectory.points[Nmin-1].positions
				plan_l[1].joint_trajectory.points[i].velocities = plan_l[1].joint_trajectory.points[i].velocities + plan[1].joint_trajectory.points[Nmin-1].velocities
				plan_l[1].joint_trajectory.points[i].accelerations = plan_l[1].joint_trajectory.points[i].accelerations + plan[1].joint_trajectory.points[Nmin-1].accelerations

		return plan_l
	# ========================================================================


	# =========================== CHECK PLAN IS OK ===========================
	def check_plan(self, id_group, group, plan, name):
		""" checks if plan found and if joint values ok"""
		nb_pts = len(plan[1].joint_trajectory.points)
		if nb_pts == 0 : # No plan found
			return False

		print ("Checking plan...")
		for i in range(nb_pts):
			print ("Point %s:" % i)
			pt = plan[1].joint_trajectory.points[i]
			joints = pt.positions
			sh_l = joints[0] #shoulder lift
			sh_p = joints[1] #shoulder pan
			elb = joints[3] #elbow
			print ("sh_l : %s" % sh_l)
			print ("sh_p : %s" % sh_p)
			print ("elb : %s" % elb)

			if self.const.IS_45 == False:
				if id_group == 0: #right arm
					if sh_l < -1.2 or sh_l > 0.7:
						print ("Plan pb for sh_l r")
						return False
					if sh_p < -2.0 or sh_p > -0.9:
						print ("Plan pb for sh_p r")
						return False
					if elb <0.8 or elb > 2.3:
						print ("Plan pb for elb r")
						return False
				elif id_group == 1: #left arm
					if sh_l < -1.6 or sh_l > 1.2:
						print ("Plan pb for sh_l l")
						return False
					if sh_p < 0.2 or sh_p > 1.9:
						print ("Plan pb for sh_p l")
						return False
					if elb < -2.6 or elb > -1.3:
						print ("Plan pb for elb l")
						return False
			else:
				if id_group == 0: #right arm
					if sh_l < -1.7 or sh_l > 0.7:
						print ("Plan pb for sh_l r")
						return False
					if sh_p < -2.0 or sh_p > 0.13:
						print ("Plan pb for sh_p r")
						return False
					if elb < 1.2 or elb > 2.3:
						print ("Plan pb for elb r")
						return False
				elif id_group == 1: #left arm
					if sh_l < -1.6 or sh_l > 1.2:
						print ("Plan pb for sh_l l")
						return False
					if sh_p < 0.2 or sh_p > 1.7:
						print ("Plan pb for sh_p l")
						return False
					if elb < -2.6 or elb > -1.3:
						print ("Plan pb for elb l")
						return False

		print ("Plan checked!")
		return True

	# ========================================================================


	# ============================= PICK AND PLACE ===========================
	def pick_and_place_all(self):

		self.load_scene()
		rospy.sleep(1)
		
		print("===> Starting Pick and Place sequence")

		# Create one thread for data collecter
		# t2 = threading.Thread(name="movo_data", target=self.movo_data_collecter.activate_thread, args=())
		# t2.start()

		for obj_to_grasp in self.obj_to_grasp:
			last_pose = self.pick(obj_to_grasp, 0)
			last_pose = self.move_switch_point(obj_to_grasp)
			self.place(obj_to_grasp, last_pose, 1)
			self.execute_joints_cmd(self.move_groups[1],self.joints_left_relax,"relax_left")

		self.move_initial_pose()

		# self.movo_data_collecter.clean_shutdown()
		self.pub_ee_state.clean_shutdown()

	# ========================================================================

	def clean_shutdown(self):
		self.movo_data_collecter.clean_shutdown()
		self.pub_ee_state.clean_shutdown()

def main():
	try:
		print (" ============================================================================= ")
		print (" ============================ PICK AND PLACE TEST ============================ ")
		print (" ============================================================================= ")
		print (" ------------------------------------------ ")
		print (" ==> Press 'Enter' to initialize ")
		print (" ------------------------------------------ ")
		input()
		PP_test = MovoPickPlace()

		print (" ------------------------------------------------------ ")
		print (" ==> Press 'Enter' to launch pick and place ")
		print (" ------------------------------------------------------ ")
		input()
		PP_test.pick_and_place_all()

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
	    return

if __name__ == '__main__':
  main()
