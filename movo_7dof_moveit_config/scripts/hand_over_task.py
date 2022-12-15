#!/usr/bin/env python

# ============================ IMPORTS ============================
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String, Duration
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix, quaternion_from_matrix
import pick_and_place_constants_with_user as const
#import pick_and_place_constants_with_user_45 as const
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from movo_action_clients.gripper_action_client import GripperActionClient
from math import ceil
# ==================================================================

class HandOverTask(object):
	""" HandOverTask """

	# ============================ INITIALIZATION ============================
	def __init__(self):

		super(HandOverTask, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('hand_over_task', anonymous=True)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()

		group_name_gl = "upper_body"
		self.move_group_gl = moveit_commander.MoveGroupCommander(group_name_gl)
		self.move_group_gl.set_planning_time(const.PLANNING_TIME)
		self.move_group_gl.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

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

		group_name = const.GROUP_NAME
		gripper_name = const.GRIPPER_NAME
		move_group = moveit_commander.MoveGroupCommander(group_name)
		move_group.set_planning_time(const.PLANNING_TIME)
		move_group.set_num_planning_attempts(const.PLANNING_ATTEMPTS)
		gripper_group = GripperActionClient('right')
		touch_links = robot.get_link_names(group=gripper_name)
		for item in robot.get_link_names(group=const.GRIPPER_GROUP):
			if item not in touch_links:
				touch_links.append(item)
		eef_link = move_group.get_end_effector_link()

		self.move_group = move_group
		self.gripper_group = gripper_group
		self.touch_links = touch_links
		self.eef_link = eef_link

		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
	                                                   moveit_msgs.msg.DisplayTrajectory,
	                                                   queue_size=20)

		self.move_initial_pose()
		self.joints_right_relax =  self.move_group_gl.get_current_joint_values()[10:]
		self.joints_left_relax = self.move_group_gl.get_current_joint_values()[1:8]

		self.change = False
		self.nb = 120
		self.l_forces = []
		self.force_mean = [0,0,0]
		self.force_sum = [0,0,0]
		self.force_mem = [0,0,0]
		self.baseline = 0.0

		#self._srv = rospy.ServiceProxy('/compute_fk', moveit_msgs.srv.GetPositionFK)
		#self._srv.wait_for_service()

		print "Initialization done."
		print ""
	# ========================================================================



	# ========================================================================
	# TODO change, not same sensor as TALOS
	def callback_wrist_sensor(self,data):

		force = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]
		if len(self.l_forces) == self.nb: # list full
			self.l_forces.pop(0) # remove first element
		self.l_forces.append(force)

		# mean value
		for i in range(len(self.l_forces)):
			for j in range(len(self.force_mean)):
				self.force_mean[j] = self.force_mean[j] + self.l_forces[i][j]
		for k in range(len(self.force_mean)):
			self.force_mean[k] = self.force_mean[k]/len(self.l_forces)
		#rospy.loginfo("Mean force vector:\n%s",self.force_mean)
		if len(self.l_forces) < self.nb:
			self.baseline = self.baseline + self.force_mean[2]
		else:
			rospy.loginfo("Mean force vector:\n%s",self.force_mean)
			print "Baseline: %s" % (self.baseline/self.nb)
		if self.force_mem != [0,0,0] and len(self.l_forces) == self.nb and abs(self.force_mem[2] - self.baseline/self.nb) > 5.0: #TO CHANGE
			print "User has grasped the object : %s" % self.force_mean[2]
			self.change = True
		else:
			self.force_mem = self.force_mean
	# ========================================================================


	# ========================================================================
	# TODO change, not same sensor as TALOS
	def listener(self):
		self.sub = rospy.Subscriber("/right_wrist_ft", WrenchStamped, self.callback_wrist_sensor)
	# ========================================================================


	# ============================= RESET SCENE ==============================
	def reset_scene(self):

		# OBJECTS TO GRASP
		for obj_name in self.obj_to_grasp:
			self.scene.remove_attached_object(self.eef_link, name=obj_name)
			self.scene.remove_world_object(obj_name)
		# OBJECTS OF THE ENVIRONMENT
		for obj_name_scene in self.obj_scene:
				self.scene.remove_world_object(obj_name_scene)
	# ========================================================================


	# ============================== LOAD SCENE ==============================
	def load_scene(self):

		self.reset_scene()
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


	# ============================= OPEN GRIPPER =============================
	def open_gripper(self):

		gripper_group = self.gripper_group
		print("Executing plan found to open the gripper...")
		gripper_group.command(const.JOINT_OPEN_GRIPPER)
		print "Gripper opened."
		print ""
	# ========================================================================


	# ============================ CLOSE GRIPPER =============================
	def close_gripper(self):

		gripper_group = self.gripper_group
		print("Executing plan found to close the gripper...")
		gripper_group.command(const.JOINT_CLOSE_GRIPPER)
		print "Gripper closed."
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
			self.open_gripper()
			print "Initial pose reached."
			print ""
	# ========================================================================


	# ============================= PICK PIPELINE ============================
	def pick(self, obj_to_grasp):

		if const.IS_45 == True:
			head_joint_cmd = [0.55,-0.25]
		else:
			head_joint_cmd = [0.8,-0.25]

		move_group = self.move_group
		eef_link = self.eef_link
		touch_links = self.touch_links
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
		obj_pose_base.pose.position.z = obj_pose_base.pose.position.z - dim_z/4

		# change ref frame from base to wrist
		obj_pose_wrist = np.dot(self.R_base_wrist_r,np.array([obj_pose_base.pose.position.x,obj_pose_base.pose.position.y,obj_pose_base.pose.position.z,1]))
		obj_pose_wrist = np.dot(obj_pose_wrist,rot_obj)
		#add in approach in wrist frame
		dim_y = self.get_dim_obj(obj_to_grasp,1)
		obj_pose_wrist[0] = obj_pose_wrist[0] - const.APPROACH_PICK_X - const.MARGIN_X
		obj_pose_wrist[1] = obj_pose_wrist[1] - const.APPROACH_PICK_Y - const.MARGIN_Y
		obj_pose_wrist[2] = obj_pose_wrist[2] +  const.APPROACH_PICK_Z + const.MARGIN_Z + dim_y/2
		obj_pose_wrist = np.dot(rot_obj,obj_pose_wrist)
		# change ref frame from wrist to base
		obj_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), obj_pose_wrist)
		# define goal pose in base frame
		obj_pose_base.pose.position.x = obj_pose_base_v[0]
		obj_pose_base.pose.position.y = obj_pose_base_v[1]
		obj_pose_base.pose.position.z = obj_pose_base_v[2]

		#success = self.move_attempt(move_group, obj_pose_base, "grasp_approach_pose")
		arm_joint_cmd = self.get_joints_from_pose(move_group,obj_pose_base,"grasp_approach_pose")
		current_joints = self.move_group_gl.get_current_joint_values()
		joint_cmd = current_joints
		joint_cmd[10:] = arm_joint_cmd
		joint_cmd[8:10] = head_joint_cmd
		self.execute_joints_cmd(self.move_group_gl,joint_cmd,"grasp_approach_head_turn")

		# OPEN GRIPPER
		self.open_gripper()

		# MOVE TO GRASP POSE
		grasp_approach_pose= obj_pose_base
		grasp_pose_base = grasp_approach_pose
		# change ref frame from base to wrist
		grasp_pose_wrist = np.dot(self.R_base_wrist_r,np.array([grasp_pose_base.pose.position.x,grasp_pose_base.pose.position.y,grasp_pose_base.pose.position.z,1]))
		grasp_pose_wrist = np.dot(grasp_pose_wrist,rot_obj)
		grasp_pose_wrist[0] = grasp_pose_wrist[0] + const.APPROACH_PICK_X
		grasp_pose_wrist[1] = grasp_pose_wrist[1] + const.APPROACH_PICK_Y
		grasp_pose_wrist[2] = grasp_pose_wrist[2] -  const.APPROACH_PICK_Z - const.DIST
		grasp_pose_wrist = np.dot(rot_obj,grasp_pose_wrist)
		# change ref frame from wrist to base
		grasp_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), grasp_pose_wrist)
		# define goal pose in base frame
		grasp_pose_base.pose.position.x = grasp_pose_base_v[0]
		grasp_pose_base.pose.position.y = grasp_pose_base_v[1]
		grasp_pose_base.pose.position.z = grasp_pose_base_v[2]
		success = self.move_attempt(move_group, grasp_pose_base, "grasp_pose")

		# CLOSE GRIPPER
		self.close_gripper()

		# ATTACH OBJECT
		self.scene.attach_box(eef_link, obj_to_grasp, touch_links=touch_links)
		rospy.sleep(1)
		# MOVE TO RETREAT POSE
		pick_retreat_pose_base = grasp_pose_base
		# change ref frame from base to wrist
		pick_retreat_pose_wrist = np.dot(self.R_base_wrist_r,np.array([pick_retreat_pose_base.pose.position.x,pick_retreat_pose_base.pose.position.y,pick_retreat_pose_base.pose.position.z,1]))
		pick_retreat_pose_wrist = np.dot(pick_retreat_pose_wrist,rot_obj)
		pick_retreat_pose_wrist[0] = pick_retreat_pose_wrist[0] - const.RETREAT_X
		pick_retreat_pose_wrist[1] = pick_retreat_pose_wrist[1] - const.RETREAT_Y
		pick_retreat_pose_wrist[2] = pick_retreat_pose_wrist[2] +  const.RETREAT_Z
		pick_retreat_pose_wrist = np.dot(rot_obj,pick_retreat_pose_wrist)
		# change ref frame from wrist to base
		pick_retreat_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), pick_retreat_pose_wrist)
		# define goal pose in base frame
		pick_retreat_pose_base.pose.position.x = pick_retreat_pose_base_v[0]
		pick_retreat_pose_base.pose.position.y = pick_retreat_pose_base_v[1]
		pick_retreat_pose_base.pose.position.z = pick_retreat_pose_base_v[2]
		success = self.move_attempt(move_group, pick_retreat_pose_base, "pick_retreat_pose")

		print "Pick pipeline done."
		print ""

		return pick_retreat_pose_base
	# ========================================================================

	# ============================ ATTEMPT TO MOVE ===========================
	def move_attempt(self, group, pose, name):

		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 20:
			print "Move attempt - try number: %s" % nb_try
			plan = group.plan()
			is_ok = self.check_plan(group,plan,name)
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


	# =========================== CHECK PLAN IS OK ===========================
	def check_plan(self, group, plan, name):
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

			if sh_l < -1.5 or sh_l > -0.5:
				print "Plan pb for sh_l r"
				return False
			if sh_p < -2.0 or sh_p > -0.3:
				print "Plan pb for sh_p r"
				return False
			if elb < 0.6 or elb > 2.3:
				print "Plan pb for elb r"
				return False

		print "Plan checked!"
		return True

	# ======================================================================


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

	def idle(self):
		#function to wait
		return ""


	# ========================== FORWARD KINEMATICS ==========================
	def fk(self, group, joint_values):

		fk_request = moveit_msgs.srv.GetPositionFKRequest()
		fk_request.fk_link_names = [group.get_end_effector_link()]
		fk_request.robot_state.joint_state.name = group.get_active_joints()
		fk_request.robot_state.joint_state.position = joint_values
		fk_request.header.frame_id = self.robot.get_planning_frame()
		fk_result = self._srv.call(fk_request)
		return fk_result.pose_stamped[0].pose
	# ========================================================================


	# ========================= MOVE TO SWITCH POINT =========================
	def move_handover_point(self, obj_to_grasp):

		head_joint_cmd = [0.0,-0.1]

		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = const.SWITCH_POSITION.get("x") #- const.DIST
		p.pose.position.y = const.SWITCH_POSITION.get("y")
		p.pose.position.z = const.SWITCH_POSITION.get("z")
		quaternion = quaternion_from_euler(const.EULER_SWITCH[0],const.EULER_SWITCH[1],const.EULER_SWITCH[2])
		p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

		# MOVES TO SWITCH POINT
		joints_right = self.get_joints_from_pose(self.move_group, p,"move_right_to_switch_point")

		arm_joint_cmd = joints_right
		current_joints = self.move_group_gl.get_current_joint_values()
		joint_cmd = current_joints
		joint_cmd[10:] = arm_joint_cmd
		joint_cmd[8:10] = head_joint_cmd
		self.execute_joints_cmd(self.move_group_gl,joint_cmd,"move_right_switch_head_turn")

		# UNCOMMENT FOR LISTENNER INPUT - APPLY FORCES AND ALL
		# self.change = False
		# self.baseline = 0.0
		# self.l_forces = []
		# self.listener()
		# #UNGRASP OBJECT WHEN USER HAS IT
		# while self.change == False:
		# 	self.idle()

		# self.sub.unregister()
		self.open_gripper()
		self.scene.remove_attached_object(self.move_group.get_end_effector_link(), name=obj_to_grasp)
		self.scene.remove_attached_object(self.eef_link, name=obj_to_grasp)
		rospy.sleep(2)
		print "User has taken object"
		self.scene.remove_world_object(obj_to_grasp)

		#MOVING AWAY
		p.pose.position.x = p.pose.position.x + const.APPROACH_PLACE_X
		joints_right = self.get_joints_from_pose(self.move_group, p,"move_away_from_switch_point")
		self.execute_joints_cmd(self.move_group, joints_right, "move_right_away")

		#RELAX ARM
		self.execute_joints_cmd(self.move_group, self.joints_right_relax, "relax_right")
		return p.pose
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


	# ======================== GET POSE JOINTS' VALUES =======================
	def get_joints_from_pose(self, group, pose, name):

		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 10:
			print "Move attempt - try number: %s" % nb_try
			plan = group.plan()
			is_ok = self.check_plan(group,plan,name)
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


	# ============================= PICK AND PLACE ===========================
	def pick_and_place_all(self):

		self.load_scene()
		rospy.sleep(1)

		for obj_to_grasp in self.obj_to_grasp:

		 	last_pose = self.pick(obj_to_grasp)
		 	last_pose = self.move_handover_point(obj_to_grasp)

		# BACK TO INITIAL POSE
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
		PP_test = HandOverTask()

		print " ------------------------------------------ "
		print " ==> Press 'Enter' to launch pick and place "
		print " ------------------------------------------ "
		raw_input()
		PP_test.pick_and_place_all()


	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
	    return

if __name__ == '__main__':
  main()
