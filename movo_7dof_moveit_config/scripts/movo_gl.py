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
from gazebo_msgs.srv import SpawnModel,SpawnModelRequest
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
# from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes, Grasp, GripperTranslation, PlaceLocation, CollisionObject
from movo_action_clients.gripper_action_client import GripperActionClient
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Transform
from math import ceil
# ==================================================================

class MovoGl(object):
	""" MovoGl """

	# ============================ INITIALIZATION ============================
	def __init__(self, const):

		super(MovoGl, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('movo_tasks', anonymous=True)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()

		group_name_gl = "upper_body"
		self.move_group_gl = moveit_commander.MoveGroupCommander(group_name_gl)
		self.move_group_gl.set_planning_time(const.PLANNING_TIME)
		self.move_group_gl.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

		#l_group_name = "both_arms"
		#self.move_l_group = moveit_commander.MoveGroupCommander(l_group_name)
		#self.move_l_group.set_planning_time(const.PLANNING_TIME)
		#self.move_l_group.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

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


		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
	                                                   moveit_msgs.msg.DisplayTrajectory,
	                                                   queue_size=20)

	# ========================================================================


	# ============================= RESET SCENE ==============================
	def reset_scene(self):

		# OBJECTS TO GRASP
		for obj_name in self.obj_to_grasp:
			for i in range(self.const.NB_GROUPS):
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
			p.pose.position.x = self.const.OBJ_POS_SCENE.get(obj_name_scene).get("x")
			p.pose.position.y = self.const.OBJ_POS_SCENE.get(obj_name_scene).get("y")
			p.pose.position.z = self.const.OBJ_POS_SCENE.get(obj_name_scene).get("z")
			quaternion = quaternion_from_euler(self.const.OBJ_POS_SCENE.get(obj_name_scene).get("roll"),self.const.OBJ_POS_SCENE.get(obj_name_scene).get("pitch"),self.const.OBJ_POS_SCENE.get(obj_name_scene).get("yaw"))
			p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

			obj_type = self.const.OBJ_POS_SCENE.get(obj_name_scene).get("type")
			if obj_type == "BOX":
				self.scene.add_box(obj_name_scene, p, self.const.OBJ_POS_SCENE.get(obj_name_scene).get("dim"))
			elif obj_type == "CYLINDER":
				self.scene.add_cylinder(obj_name_scene, p, self.const.OBJ_POS_SCENE.get(obj_name_scene).get("h"), self.const.OBJ_POS_SCENE.get(obj_name_scene).get("r"))
			elif obj_type == "PLANE":
				self.scene.add_plane(obj_name_scene, p, self.const.OBJ_POS_SCENE.get(obj_name_scene).get("n"), self.const.OBJ_POS_SCENE.get(obj_name_scene).get("offset"))
			elif obj_type == "SPHERE":
				self.scene.add_sphere(obj_name_scene, p, self.const.OBJ_POS_SCENE.get(obj_name_scene).get("r"))
			else:
				print ("[ERROR] Type of object %s to add not supported." % obj_type)

		# OBJECTS TO GRASP
		for obj_name in self.obj_to_grasp:
			p.header.frame_id = self.robot.get_planning_frame()
			p.pose.position.x = self.const.OBJ_POS_GRASP.get(obj_name).get("x")
			p.pose.position.y = self.const.OBJ_POS_GRASP.get(obj_name).get("y")
			p.pose.position.z = self.const.OBJ_POS_GRASP.get(obj_name).get("z")
			quaternion = quaternion_from_euler(self.const.OBJ_POS_GRASP.get(obj_name).get("roll"),self.const.OBJ_POS_GRASP.get(obj_name).get("pitch"),self.const.OBJ_POS_GRASP.get(obj_name).get("yaw"))
			p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
			obj_type = self.const.OBJ_POS_GRASP.get(obj_name).get("type")
			if obj_type == "BOX":
				self.scene.add_box(obj_name, p, self.const.OBJ_POS_GRASP.get(obj_name).get("dim"))
			elif obj_type == "CYLINDER":
				self.scene.add_cylinder(obj_name, p, self.const.OBJ_POS_GRASP.get(obj_name).get("h"), self.const.OBJ_POS_GRASP.get(obj_name).get("r"))
			elif obj_type == "PLANE":
				self.scene.add_plane(obj_name, p, self.const.OBJ_POS_GRASP.get(obj_name).get("n"), self.const.OBJ_POS_GRASP.get(obj_name).get("offset"))
			elif obj_type == "SPHERE":
				self.scene.add_sphere(obj_name, p, self.const.OBJ_POS_GRASP.get(obj_name).get("r"))
			else:
				print ("[ERROR] Type of object %s to add not supported." % obj_type)
		print ("Scene set up done.")
		print ("")
	# ========================================================================


	# ============================= MOVE INITIAL =============================
	def move_initial_pose(self):
		self.move_group_gl.set_named_target("home_grasp");
		plan = self.move_group_gl.plan()
		if len(plan[1].joint_trajectory.points) == 0 :
			print("[ERROR] No plan found to move to initial pose.")
			sys.exit(1)
		else :
			print("Executing plan found to reach initial pose...")
			self.move_group_gl.execute(plan[1])
			self.move_group_gl.stop()
			self.move_group_gl.clear_pose_targets()
			self.open_gripper(0)
			self.open_gripper(1)
			print ("Initial pose reached.")
			print ("")
	# ========================================================================


	# ============================= OPEN GRIPPER =============================
	def open_gripper(self, id_group):

		gripper_group = self.gripper_groups[id_group]
		print("Executing plan found to open the gripper...")
		gripper_group.command(self.const.JOINT_OPEN_GRIPPER)
		print ("Gripper opened.")
		print ("")
	# ========================================================================


	# ============================ CLOSE GRIPPER =============================
	def close_gripper(self, id_group):

		gripper_group = self.gripper_groups[id_group]
		print("Executing plan found to close the gripper...")
		gripper_group.command(self.const.JOINT_CLOSE_GRIPPER)
		print ("Gripper closed.")
		print ("")
	# ========================================================================


	# ============================= GET DIM OBJ ==============================
	def get_dim_obj(self, obj_name, axis):

		obj_type = self.const.OBJ_POS_GRASP.get(obj_name).get("type")

		if obj_type == "BOX":
			dim = self.const.OBJ_POS_GRASP.get(obj_name).get("dim")[axis]
		elif obj_type == "CYLINDER":
			if axis == 2:
				dim = self.const.OBJ_POS_GRASP.get(obj_name).get("h")
			else:
				dim = self.const.OBJ_POS_GRASP.get(obj_name).get("r") * 2
		elif obj_type == "SPHERE":
			dim = self.const.OBJ_POS_GRASP.get(obj_name).get("r") * 2
		else:
			print ("[ERROR] Type of object %s to grasp not supported." % obj_type)

		return dim
	# ========================================================================


	# ======================== EXECUTE JOINT COMMAND =======================
	def execute_joints_cmd(self, move_group, joint_cmd, name):

		move_group.set_joint_value_target(joint_cmd)
		plan = move_group.plan()
		if len(plan[1].joint_trajectory.points) == 0:
				print ("[ERROR] No plan found to: %s." % name)
				sys.exit(1)
		else:
			print ("Executing plan found to %s." % name)
			move_group.execute(plan[1])
			move_group.stop()
			move_group.clear_pose_targets()
			print ("Position reached.")
			print ("")
	# ========================================================================


	# ============================= MOVE HEAD ===============================
	def move_head(self,joint_values):
		self.move_group_head.set_joint_value_target(joint_values)
		plan = self.move_group_head.plan()
		if len(plan[1].joint_trajectory.points) == 0 :
			print ("[ERROR] No plan found to move head.")
			print ("")
			sys.exit(1)
		else :
			print ("Executing plan found to move head.")
			self.move_group_head.execute(plan[1])
			self.move_group_head.stop()
			self.move_group_head.clear_pose_targets()
			print ("Position reached.")
			print ("")
	# ========================================================================
