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
#import pick_and_place_constants_without_user as const
import pick_and_place_constants_without_user_45 as const
from gazebo_msgs.srv import SpawnModel,SpawnModelRequest
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from movo_action_clients.gripper_action_client import GripperActionClient
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Transform
from math import ceil
# ==================================================================

class OptiBothHandsPickAndPlace(object):
	""" OptiBothHandsPickAndPlace """

	# ============================ INITIALIZATION ============================
	def __init__(self):

		super(OptiBothHandsPickAndPlace, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('opti_both_hands_pick_and_place', anonymous=True)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()

		group_name_gl = "upper_body"
		self.move_group_gl = moveit_commander.MoveGroupCommander(group_name_gl)
		self.move_group_gl.set_planning_time(const.PLANNING_TIME)
		self.move_group_gl.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

		l_group_name = "both_arms"
		self.move_l_group = moveit_commander.MoveGroupCommander(l_group_name)
		self.move_l_group.set_planning_time(const.PLANNING_TIME)
		self.move_l_group.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

		group_name_head = "head"
		self.move_group_head = moveit_commander.MoveGroupCommander(group_name_head)
		self.move_group_head.set_planning_time(const.PLANNING_TIME)
		self.move_group_head.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

		# Rotation matrix between right wrist init and base odom
		self.R_base_wrist_r = np.array([[1.0,0.0,0.0,0.0],
										[0.0,0.0,-1.0,0.0],
										[0.0,1.0,0.0,0.0],
										[0.0,0.0,0.0,1.0]])
		self.R_constraints_wrist_r = np.array([[0.0,-1.0,0.0,0.0],
											[-1.0,0.0,0.0,0.0],
											[0.0,0.0,-1.0,0.0],
											[0.0,0.0,0.0,1.0]])
		# Rotation matrix between left wrist init and base odom
		self.R_base_wrist_l = np.array([[1.0,0.0,0.0,0.0],
										[0.0,0.0,-1.0,0.0],
										[0.0,1.0,0.0,0.0],
										[0.0,0.0,0.0,1.0]])
		self.R_constraints_wrist_l = np.array([[0.0,-1.0,0.0,0.0],
											[1.0,0.0,0.0,0.0],
											[0.0,0.0,1.0,0.0],
											[0.0,0.0,0.0,1.0]])

		self.robot = robot
		self.scene = scene
		self.obj_to_grasp = const.OBJ_NAMES_GRASP
		self.obj_scene = const.OBJ_NAMES_SCENE

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
			touch_links = robot.get_link_names(group=gripper_name)
			for item in robot.get_link_names(group=const.GRIPPER_GROUP[i]):
				if item not in touch_links:
					touch_links.append(item)
			eef_link = move_group.get_end_effector_link()

			self.move_groups.append(move_group)
			self.gripper_groups.append(gripper_group)
			self.touch_links_groups.append(touch_links)
			self.eef_links.append(eef_link)

		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
	                                                   moveit_msgs.msg.DisplayTrajectory,
	                                                   queue_size=20)

		self.move_initial_pose()
		self.joints_right_relax =  self.move_group_gl.get_current_joint_values()[10:]
		self.joints_left_relax = self.move_group_gl.get_current_joint_values()[1:8]


		print "Initialization done."
		print ""
	# ========================================================================


	# ============================= RESET SCENE ==============================
	def reset_scene(self):

		# OBJECTS TO GRASP
		for obj_name in self.obj_to_grasp:
			for i in range(const.NB_GROUPS):
				self.scene.remove_attached_object(self.eef_links[i], name=obj_name)
			self.scene.remove_world_object(obj_name)
		# OBJECTS OF THE ENVIRONMENT
		for obj_name_scene in self.obj_scene:
				self.scene.remove_world_object(obj_name_scene)
	# ========================================================================


	# ============================== LOAD SCENE ==============================
	def load_scene(self):

		#self.reset_scene()
		rospy.sleep(2)

		# OBJECTS OF THE ENVIRONMENT
		for obj_name_scene in self.obj_scene:
			p = PoseStamped()
			p.header.frame_id = self.robot.get_planning_frame()
			p.pose.position.x = const.OBJ_POS_SCENE.get(obj_name_scene).get("x")
			p.pose.position.y = const.OBJ_POS_SCENE.get(obj_name_scene).get("y")
			p.pose.position.z = const.OBJ_POS_SCENE.get(obj_name_scene).get("z")
			quaternion = quaternion_from_euler(const.OBJ_POS_SCENE.get(obj_name_scene).get("roll"),const.OBJ_POS_SCENE.get(obj_name_scene).get("pitch"),const.OBJ_POS_SCENE.get(obj_name_scene).get("yaw"))
			p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

			obj_type = const.OBJ_POS_SCENE.get(obj_name_scene).get("type")
			if obj_type == "BOX":
				self.scene.add_box(obj_name_scene, p, const.OBJ_POS_SCENE.get(obj_name_scene).get("dim"))
			elif obj_type == "CYLINDER":
				self.scene.add_cylinder(obj_name_scene, p, const.OBJ_POS_SCENE.get(obj_name_scene).get("h"), const.OBJ_POS_SCENE.get(obj_name_scene).get("r"))
			elif obj_type == "PLANE":
				self.scene.add_plane(obj_name_scene, p, const.OBJ_POS_SCENE.get(obj_name_scene).get("n"), const.OBJ_POS_SCENE.get(obj_name_scene).get("offset"))
			elif obj_type == "SPHERE":
				self.scene.add_sphere(obj_name_scene, p, const.OBJ_POS_SCENE.get(obj_name_scene).get("r"))
			else:
				print "[ERROR] Type of object %s to add not supported." % obj_type

		# OBJECTS TO GRASP
		for obj_name in self.obj_to_grasp:
			p.header.frame_id = self.robot.get_planning_frame()
			p.pose.position.x = const.OBJ_POS_GRASP.get(obj_name).get("x")
			p.pose.position.y = const.OBJ_POS_GRASP.get(obj_name).get("y")
			p.pose.position.z = const.OBJ_POS_GRASP.get(obj_name).get("z")
			quaternion = quaternion_from_euler(const.OBJ_POS_GRASP.get(obj_name).get("roll"),const.OBJ_POS_GRASP.get(obj_name).get("pitch"),const.OBJ_POS_GRASP.get(obj_name).get("yaw"))
			p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
			obj_type = const.OBJ_POS_GRASP.get(obj_name).get("type")
			if obj_type == "BOX":
				self.scene.add_box(obj_name, p, const.OBJ_POS_GRASP.get(obj_name).get("dim"))
			elif obj_type == "CYLINDER":
				self.scene.add_cylinder(obj_name, p, const.OBJ_POS_GRASP.get(obj_name).get("h"), const.OBJ_POS_GRASP.get(obj_name).get("r"))
			elif obj_type == "PLANE":
				self.scene.add_plane(obj_name, p, const.OBJ_POS_GRASP.get(obj_name).get("n"), const.OBJ_POS_GRASP.get(obj_name).get("offset"))
			elif obj_type == "SPHERE":
				self.scene.add_sphere(obj_name, p, const.OBJ_POS_GRASP.get(obj_name).get("r"))
			else:
				print "[ERROR] Type of object %s to add not supported." % obj_type
		print "Scene set up done."
		print ""
	# ========================================================================


	# ============================= MOVE INITIAL =============================
	def move_initial_pose(self):
		self.move_group_gl.set_named_target("home_grasp");
		plan = self.move_group_gl.plan()
		if len(plan.joint_trajectory.points) == 0 :
			print("[ERROR] No plan found to move to initial pose.")
			sys.exit(1)
		else :
			print("Executing plan found to reach initial pose...")
			self.move_group_gl.execute(plan)
			self.move_group_gl.stop()
			self.move_group_gl.clear_pose_targets()
			self.open_gripper(0)
			self.open_gripper(1)
			print "Initial pose reached."
			print ""
	# ========================================================================


	# ============================= OPEN GRIPPER =============================
	def open_gripper(self, id_group):

		gripper_group = self.gripper_groups[id_group]
		print("Executing plan found to open the gripper...")
		gripper_group.command(const.JOINT_OPEN_GRIPPER)
		print "Gripper opened."
		print ""
	# ========================================================================


	# ============================ CLOSE GRIPPER =============================
	def close_gripper(self, id_group):

		gripper_group = self.gripper_groups[id_group]
		print("Executing plan found to close the gripper...")
		gripper_group.command(const.JOINT_CLOSE_GRIPPER)
		print "Gripper closed."
		print ""
	# ========================================================================

	# ============================= PICK PIPELINE ============================
	def pick(self, obj_to_grasp, id_group):

		if const.IS_45 == True:
			head_joint_cmd = [0.55,-0.25]
		else:
			head_joint_cmd = [0.8,-0.25]

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
		obj_pose_wrist[0] = obj_pose_wrist[0] - const.APPROACH_PICK_X[id_group] - const.MARGIN_X[id_group]
		obj_pose_wrist[1] = obj_pose_wrist[1] - const.APPROACH_PICK_Y[id_group] - const.MARGIN_Y[id_group]
		obj_pose_wrist[2] = obj_pose_wrist[2] +  const.APPROACH_PICK_Z[id_group] + const.MARGIN_Z[id_group] + dim_y/2
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
		self.execute_joints_cmd(self.move_group_gl,joint_cmd,"grasp_approach_head_turn")

		# OPEN GRIPPER
		self.open_gripper(id_group)

		# MOVE TO GRASP POSE
		grasp_approach_pose= obj_pose_base
		grasp_pose_base = grasp_approach_pose
		# change ref frame from base to wrist
		grasp_pose_wrist = np.dot(self.R_base_wrist_r,np.array([grasp_pose_base.pose.position.x,grasp_pose_base.pose.position.y,grasp_pose_base.pose.position.z,1]))
		grasp_pose_wrist = np.dot(grasp_pose_wrist,rot_obj)
		grasp_pose_wrist[0] = grasp_pose_wrist[0] + const.APPROACH_PICK_X[id_group]
		grasp_pose_wrist[1] = grasp_pose_wrist[1] + const.APPROACH_PICK_Y[id_group]
		grasp_pose_wrist[2] = grasp_pose_wrist[2] -  const.APPROACH_PICK_Z[id_group] - const.DIST
		grasp_pose_wrist = np.dot(rot_obj,grasp_pose_wrist)
		# change ref frame from wrist to base
		grasp_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), grasp_pose_wrist)
		# define goal pose in base frame
		grasp_pose_base.pose.position.x = grasp_pose_base_v[0]
		grasp_pose_base.pose.position.y = grasp_pose_base_v[1]
		grasp_pose_base.pose.position.z = grasp_pose_base_v[2]
		success = self.move_attempt(id_group,move_group, grasp_pose_base, "grasp_pose")

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
		pick_retreat_pose_wrist[0] = pick_retreat_pose_wrist[0] - const.RETREAT_X[id_group]
		pick_retreat_pose_wrist[1] = pick_retreat_pose_wrist[1] - const.RETREAT_Y[id_group]
		pick_retreat_pose_wrist[2] = pick_retreat_pose_wrist[2] +  const.RETREAT_Z[id_group]
		pick_retreat_pose_wrist = np.dot(rot_obj,pick_retreat_pose_wrist)
		# change ref frame from wrist to base
		pick_retreat_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), pick_retreat_pose_wrist)
		# define goal pose in base frame
		pick_retreat_pose_base.pose.position.x = pick_retreat_pose_base_v[0]
		pick_retreat_pose_base.pose.position.y = pick_retreat_pose_base_v[1]
		pick_retreat_pose_base.pose.position.z = pick_retreat_pose_base_v[2]
		success = self.move_attempt(id_group,move_group, pick_retreat_pose_base, "pick_retreat_pose")


		print "Pick pipeline done."
		print ""

		return pick_retreat_pose_base
	# ========================================================================

	# ============================ ATTEMPT TO MOVE ===========================
	def move_attempt(self, id_group, group, pose, name):

		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 20:
			print "Move attempt - try number: %s" % nb_try
			plan = group.plan()
			is_ok = self.check_plan(id_group,group,plan,name)
			nb_try = nb_try + 1
		if is_ok:
			print "Executing plan found to move to %s." % name
			group.execute(plan)
			group.stop()
			group.clear_pose_targets()
			print "Position %s reached." % name
			print ""
			return True
		else :
			print "[ERROR] No plan found to move to %s." % name
			print ""
			sys.exit(1)
	# ========================================================================


	# ============================= GET DIM OBJ ==============================
	def get_dim_obj(self, obj_name, axis):

		obj_type = const.OBJ_POS_GRASP.get(obj_name).get("type")

		if obj_type == "BOX":
			dim = const.OBJ_POS_GRASP.get(obj_name).get("dim")[axis]
		elif obj_type == "CYLINDER":
			if axis == 2:
				dim = const.OBJ_POS_GRASP.get(obj_name).get("h")
			else:
				dim = const.OBJ_POS_GRASP.get(obj_name).get("r") * 2
		elif obj_type == "SPHERE":
			dim = const.OBJ_POS_GRASP.get(obj_name).get("r") * 2
		else:
			print "[ERROR] Type of object %s to grasp not supported." % obj_type

		return dim
	# ========================================================================

	# ======================== GET POSE JOINTS' VALUES =======================
	def get_plan_from_pose(self, id_group, group, pose, name):

		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 20:
			print "Move attempt - try number: %s" % nb_try
			plan = group.plan()
			is_ok = self.check_plan(id_group,group,plan,name)
			nb_try = nb_try + 1
		if is_ok:
			return plan
		else:
			print "[ERROR] No plan found to move to %s." % name
			print ""
			sys.exit(1)
	# ========================================================================


	# ======================== EXECUTE JOINT COMMAND =======================
	def execute_joints_cmd(self, move_group, joint_cmd, name):

		move_group.set_joint_value_target(joint_cmd)
		plan = move_group.plan()
		if len(plan.joint_trajectory.points) == 0:
				print "[ERROR] No plan found to: %s." % name
				sys.exit(1)
		else:
			print "Executing plan found to %s." % name
			move_group.execute(plan)
			move_group.stop()
			move_group.clear_pose_targets()
			print "Position reached."
			print ""

	# ========================================================================

	# ======================== EXECUTE PLAN COMMAND =======================
	def execute_plan_cmd(self, move_group, plan, name):

		print "Executing plan found to %s." % name
		move_group.execute(plan)
		move_group.stop()
		move_group.clear_pose_targets()
		print "Position reached."
		print ""

	# ========================================================================


	# ========================= MOVE TO SWITCH POINT =========================
	def move_switch_point(self, obj_to_grasp):

		head_joint_cmd = [0.0,-0.3]
		dim_y = self.get_dim_obj(obj_to_grasp,1)

		# FIRST GROUP MOVES TO SWITCH POINT
		p = PoseStamped()
		p.header.frame_id = self.move_groups[0].get_planning_frame()
		p.pose.position.x = const.SWITCH_POSITION.get("x") - const.MARGIN_X[0]
		p.pose.position.y = const.SWITCH_POSITION.get("y") - const.MARGIN_Y[0]
		p.pose.position.z = const.SWITCH_POSITION.get("z") - const.MARGIN_Z[0] - dim_y/2
		quaternion = quaternion_from_euler(const.EULER_SWITCH[0][0],const.EULER_SWITCH[0][1],const.EULER_SWITCH[0][2])
		p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
		plan_right = self.get_plan_from_pose(0,self.move_groups[0],p,"move_right_arm_to_switch_point")

		# SECOND GROUP MOVES TO SWITCH POINT AND GRABS
		self.open_gripper(1)
		p.pose.position.x = p.pose.position.x + const.OFFSET_X
		p.pose.position.y = p.pose.position.y + const.APPROACH_PICK_Y[1] - 2 * const.MARGIN_Y[1]
		p.pose.position.z = p.pose.position.z + const.APPROACH_PICK_Z[1]
		quaternion = quaternion_from_euler(const.EULER_SWITCH[1][0],const.EULER_SWITCH[1][1],const.EULER_SWITCH[1][2])
		p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
		plan_left = self.get_plan_from_pose(1,self.move_groups[1],p,"approach_left_arm_near_switch_point")

		plan_both_arms = self.combine_plans(plan_right,plan_left)
		self.move_group_head.set_joint_value_target(head_joint_cmd)
		plan_head = self.move_group_head.plan()
		plan_both_arms_head = self.combine_plans(plan_both_arms,plan_head)
		self.execute_plan_cmd(self.move_group_gl,plan_both_arms_head,"right_switch_left_approach_head")

		p.pose.position.y = p.pose.position.y - const.APPROACH_PICK_Y[1]
		success = self.move_attempt(1,self.move_groups[1], p, "switch_point")

		self.close_gripper(1)
		self.scene.remove_attached_object(self.move_groups[0].get_end_effector_link(), name=obj_to_grasp)
		self.scene.remove_attached_object(self.eef_links[0], name=obj_to_grasp)
		self.scene.attach_box(self.eef_links[1], obj_to_grasp, touch_links=self.touch_links_groups[1])
		rospy.sleep(1)
		# FIRST GROUP UNGRASPS
		self.open_gripper(0)

		#MOVING AWAY
		p.pose.position.y = p.pose.position.y + const.RETREAT_Y[1] + const.MARGIN_Y[1]/2
		plan_left = self.get_plan_from_pose(1,self.move_groups[1],p,"move_away_left_arm")
		self.move_groups[0].set_joint_value_target(self.joints_right_relax)

		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 20:
			print "Move attempt - try number: %s" % nb_try
			plan_right = self.move_groups[0].plan()
			is_ok = self.check_plan(0,self.move_groups[0],plan_right,"relax_right")
			nb_try = nb_try + 1

		plan_both_arms = self.combine_plans(plan_right,plan_left)
		self.execute_plan_cmd(self.move_l_group,plan_both_arms,"left_away_right_relax")

		return p.pose
	# ========================================================================


	# ============================= PLACE PIPELINE ===========================
	def place(self, obj_to_grasp, last_pose, id_group):

		if const.IS_45 == True:
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
		place_approach_pose_base.position.x = self.pose_obj.position.x + const.OBJ_POS_GRASP.get(obj_to_grasp).get("translation")[0]
		place_approach_pose_base.position.y = self.pose_obj.position.y + const.OBJ_POS_GRASP.get(obj_to_grasp).get("translation")[1]
		place_approach_pose_base.position.z = self.pose_obj.position.z + const.OBJ_POS_GRASP.get(obj_to_grasp).get("translation")[2]
		place_approach_pose_base.orientation = Quaternion(quat_goal[0],quat_goal[1],quat_goal[2],quat_goal[3])
		# change ref frame from base to wrist
		place_approach_pose_wrist = np.dot(self.R_base_wrist_l,np.array([place_approach_pose_base.position.x,place_approach_pose_base.position.y,place_approach_pose_base.position.z,1]))
		place_approach_pose_wrist = np.dot(place_approach_pose_wrist,rot_obj)
		#add in approach in wrist frame
		place_approach_pose_wrist[0] = place_approach_pose_wrist[0] + const.APPROACH_PLACE_X[id_group] + const.MARGIN_X[id_group]
		place_approach_pose_wrist[1] = place_approach_pose_wrist[1] - const.APPROACH_PLACE_Y[id_group] - const.MARGIN_Y[id_group] - (ceil(const.APPROACH_PICK_Y[id_group] * 10 ** 2) / 10 ** 2) #-  dim_y/2  #- const.DIST
		if quat_obj.w != 1.0:
			place_approach_pose_wrist[1] = place_approach_pose_wrist[1] - 0.01
		place_approach_pose_wrist[2] = place_approach_pose_wrist[2] + const.APPROACH_PLACE_Z[id_group] + const.MARGIN_Z[id_group] +  dim_y/2 #+ (ceil(const.APPROACH_PICK_Z[id_group]/2 * 10 ** 2) / 10 ** 2)
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
		self.execute_plan_cmd(self.move_group_gl,plan_arm_head,"place_approach_head")

		# MOVE TO PLACE POSE
		place_pose_base = place_approach_pose_base
		# change ref frame from base to wrist
		place_pose_wrist = np.dot(self.R_base_wrist_l,np.array([place_approach_pose_base.position.x,place_approach_pose_base.position.y,place_approach_pose_base.position.z,1]))
		place_pose_wrist = np.dot(place_pose_wrist,rot_obj)
		place_pose_wrist[0] = place_pose_wrist[0] - const.APPROACH_PLACE_X[id_group]
		place_pose_wrist[1] = place_pose_wrist[1] + const.APPROACH_PLACE_Y[id_group]
		place_pose_wrist[2] = place_pose_wrist[2] - const.APPROACH_PLACE_Z[id_group]
		place_pose_wrist = np.dot(rot_obj,place_pose_wrist)
		# change ref frame from wrist to base
		place_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_l), place_pose_wrist)
		# define goal pose in base frame
		place_pose_base.position.x = place_pose_base_v[0]
		place_pose_base.position.y = place_pose_base_v[1]
		place_pose_base.position.z = place_pose_base_v[2]
		success = self.move_attempt(id_group,move_group, place_pose_base, "place_pose")

		# OPEN GRIPPER
		self.open_gripper(id_group)

		# DETACH OBJECT
		self.scene.remove_attached_object(eef_link, name=obj_to_grasp)
		rospy.sleep(1)
		# RETURN TO RETREAT POSE
		place_retreat_pose_base = place_pose_base
		# change ref frame from base to wrist
		place_retreat_pose_wrist = np.dot(self.R_base_wrist_l,np.array([place_retreat_pose_base.position.x,place_retreat_pose_base.position.y,place_retreat_pose_base.position.z,1]))
		place_retreat_pose_wrist = np.dot(place_retreat_pose_wrist, rot_obj)
		place_retreat_pose_wrist[0] = place_retreat_pose_wrist[0] - const.RETREAT_PL_X[id_group]
		place_retreat_pose_wrist[1] = place_retreat_pose_wrist[1] - const.RETREAT_PL_Y[id_group]
		place_retreat_pose_wrist[2] = place_retreat_pose_wrist[2] - const.RETREAT_PL_Z[id_group]
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
		self.execute_plan_cmd(self.move_group_gl,plan_arm_head,"place_retreat_head")

		print "Place pipeline done."
		print ""
	# ========================================================================

	# ======================== GET POSE JOINTS' VALUES =======================
	def get_joints_from_pose(self,id_group, group, pose, name):

		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 10:
			print "Move attempt - try number: %s" % nb_try
			plan = group.plan()
			is_ok = self.check_plan(id_group,group,plan,name)
			nb_try = nb_try + 1

		if is_ok:
			nb_pts = len(plan.joint_trajectory.points)
			last_pt = plan.joint_trajectory.points[nb_pts-1]
			joints = last_pt.positions
			return joints
		else:
			print "[ERROR] No plan found to move to %s." % name
			print ""
			sys.exit(1)

	# ========================================================================


	# ============================= MOVE HEAD ===============================
	def move_head(self,joint_values):
		self.move_group_head.set_joint_value_target(joint_values)
		plan = self.move_group_head.plan()
		if len(plan.joint_trajectory.points) == 0 :
			print "[ERROR] No plan found to move head."
			print ""
			sys.exit(1)
		else :
			print "Executing plan found to move head."
			self.move_group_head.execute(plan)
			self.move_group_head.stop()
			self.move_group_head.clear_pose_targets()
			print "Position reached."
			print ""
	# ========================================================================

	# ========================================================================
	def combine_plans(self,p1,p2):
		nb_pts1 = len(p1.joint_trajectory.points)
		nb_pts2 = len(p2.joint_trajectory.points)
		Nmin = min(nb_pts1,nb_pts2)
		if Nmin == nb_pts1:
			print "min right"
			plan = p1
			plan_l = p2
			Nmax = nb_pts2
		else:
			print "min left"
			plan = p2
			plan_l = p1
			Nmax =  nb_pts1
		plan_l.joint_trajectory.joint_names.extend(plan.joint_trajectory.joint_names)

		for i in range(Nmax):
			if i < Nmin:
				plan_l.joint_trajectory.points[i].positions = plan_l.joint_trajectory.points[i].positions + plan.joint_trajectory.points[i].positions
				plan_l.joint_trajectory.points[i].velocities = plan_l.joint_trajectory.points[i].velocities + plan.joint_trajectory.points[i].velocities
				plan_l.joint_trajectory.points[i].accelerations = plan_l.joint_trajectory.points[i].accelerations + plan.joint_trajectory.points[i].accelerations
			else:
				plan_l.joint_trajectory.points[i].positions = plan_l.joint_trajectory.points[i].positions + plan.joint_trajectory.points[Nmin-1].positions
				plan_l.joint_trajectory.points[i].velocities = plan_l.joint_trajectory.points[i].velocities + plan.joint_trajectory.points[Nmin-1].velocities
				plan_l.joint_trajectory.points[i].accelerations = plan_l.joint_trajectory.points[i].accelerations + plan.joint_trajectory.points[Nmin-1].accelerations

		return plan_l
	# ========================================================================


	# =========================== CHECK PLAN IS OK ===========================
	def check_plan(self, id_group, group, plan, name):
		""" checks if plan found and if joint values ok"""
		nb_pts = len(plan.joint_trajectory.points)
		if nb_pts == 0 : # No plan found
			return False

		print "Checking plan..."
		for i in range(nb_pts):
			print "Point %s:" % i
			pt = plan.joint_trajectory.points[i]
			joints = pt.positions
			sh_l = joints[0] #shoulder lift
			sh_p = joints[1] #shoulder pan
			elb = joints[3] #elbow
			print "sh_l : %s" % sh_l
			print "sh_p : %s" % sh_p
			print "elb : %s" % elb

			if const.IS_45 == False:
				if id_group == 0: #right arm
					if sh_l < -1.2 or sh_l > 0.7:
						print "Plan pb for sh_l r"
						return False
					if sh_p < -2.0 or sh_p > -0.9:
						print "Plan pb for sh_p r"
						return False
					if elb < 1.2 or elb > 2.3:
						print "Plan pb for elb r"
						return False
				elif id_group == 1: #left arm
					if sh_l < -1.6 or sh_l > 1.2:
						print "Plan pb for sh_l l"
						return False
					if sh_p < 0.2 or sh_p > 1.9:
						print "Plan pb for sh_p l"
						return False
					if elb < -2.6 or elb > -1.3:
						print "Plan pb for elb l"
						return False
			else:
				if id_group == 0: #right arm
					if sh_l < -1.7 or sh_l > 0.7:
						print "Plan pb for sh_l r"
						return False
					if sh_p < -2.0 or sh_p > 0.13:
						print "Plan pb for sh_p r"
						return False
					if elb < 1.2 or elb > 2.3:
						print "Plan pb for elb r"
						return False
				elif id_group == 1: #left arm
					if sh_l < -1.6 or sh_l > 1.2:
						print "Plan pb for sh_l l"
						return False
					if sh_p < 0.2 or sh_p > 1.7:
						print "Plan pb for sh_p l"
						return False
					if elb < -2.6 or elb > -1.3:
						print "Plan pb for elb l"
						return False

		print "Plan checked!"
		return True

	# ========================================================================


	# ============================= PICK AND PLACE ===========================
	def pick_and_place_all(self):

		self.load_scene()
		rospy.sleep(1)

		for obj_to_grasp in self.obj_to_grasp:
			last_pose = self.pick(obj_to_grasp, 0)
			last_pose = self.move_switch_point(obj_to_grasp)
			self.place(obj_to_grasp, last_pose, 1)
			self.execute_joints_cmd(self.move_groups[1],self.joints_left_relax,"relax_left")

		self.move_initial_pose()

	# ========================================================================



def main():
	try:
		print " ============================================================================= "
		print " ============================ PICK AND PLACE TEST ============================ "
		print " ============================================================================= "

		print " ------------------------------------------ "
		print " ==> Press 'Enter' to initialize "
		print " ------------------------------------------ "
		raw_input()
		PP_test = OptiBothHandsPickAndPlace()

		print " ------------------------------------------------------ "
		print " ==> Press 'Enter' to launch pick and place "
		print " ------------------------------------------------------ "
		raw_input()
		PP_test.pick_and_place_all()

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
	    return

if __name__ == '__main__':
  main()
