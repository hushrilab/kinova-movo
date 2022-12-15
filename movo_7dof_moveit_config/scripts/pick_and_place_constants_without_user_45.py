#!/usr/bin/env python
NB_GROUPS = 2

# ===== GENERAL - ROBOT AND PLANNING PARAMETERS =====
GROUP_NAME = ["right_arm","left_arm"]
GRIPPER_NAME = ["right_gripper","left_gripper"]
GRIPPER_GROUP = ["right_gripper","left_gripper"]
DIST = 0.04 # estimated distance between tool and wrist link
JOINT_CLOSE_GRIPPER = 0.4
JOINT_OPEN_GRIPPER = 0.0
PLANNING_TIME = 10
PLANNING_ATTEMPTS = 50
SWITCH_POSITION = {
	"x" : 0.7,
	"y" : 0,
	"z" : 0.9,
	"roll" : 0.0,
	"pitch" : 0.0,
	"yaw" : 0.0
}
IS_45 = True

# ===== SCENE ELEMENTS =====
OBJ_NAMES_SCENE = ["shelf1","shelf2","shelf3","shelf4"]
OBJ_POS_SCENE = {
	"shelf1" : {
		"type" : "BOX",
		"x" : 0.75,
		"y" : -0.7,
		"z" : 0.4,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.5,
		"dim" : (0.65, 0.6, 0.8)
	},
	"shelf2" : {
		"type" : "BOX",
		"x" : 0.75,
		"y" : 0.7,
		"z" : 0.4,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : -0.5,
		"dim" : (0.65, 0.6, 0.8)
	},
	"shelf3" : {
		"type" : "BOX",
		"x" : 1.80,
		"y" : -0.7,
		"z" : 0.4,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : -0.5,
		"dim" : (0.65, 0.6, 0.8)
	},
	"shelf4" : {
		"type" : "BOX",
		"x" : 1.80,
		"y" : 0.7,
		"z" : 0.4,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.5,
		"dim" : (0.65, 0.6, 0.8)
	}
}


# ===== ITEMS TO GRASP =====
OBJ_NAMES_GRASP = ["object1","object2","object3","object4"]
OBJ_POS_GRASP = {
	"object1" : {
		"type" : "BOX",
		"x" : 0.86,
		"y" : -0.39,
		"z" : 1.0,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.5,
		"dim" : (0.055, 0.055, 0.4),
		"translation" : (0.12, 0.85, 0.0)
	},
	"object2" : {
		"type" : "BOX",
		"x" : 0.72,
		"y" : -0.48,
		"z" : 1.0,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.5,
		"dim" : (0.05, 0.05, 0.4),
		"translation" : (0.12, 1.02, 0.0)
	},
	"object3" : {
		"type" : "BOX",
		"x" : 0.91,
		"y" : -0.49,
		"z" : 1.0,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.5,
		"dim" : (0.05, 0.05, 0.4),
		"translation" : (0.0, 0.85, 0.0)
	},
	"object4" : {
		"type" : "BOX",
		"x" : 0.77,
		"y" : -0.58,
		"z" : 1.0,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.5,
		"dim" : (0.05, 0.05, 0.4),
		"translation" : (0.0, 1.03, 0.0)
	}
}


# APPROACH DISTANCES - PICKING // INITIAL OBJECT POSE
APPROACH_PICK_X = [0.0, 0.0]
APPROACH_PICK_Y = [0.0, 0.1]
APPROACH_PICK_Z = [0.08, 0.2]

# RETREAT DISTANCES - PRE PLACING // INITIAL OBJECT POSE
RETREAT_X = [0.0, 0.0]
RETREAT_Y = [0.05, 0.12]
RETREAT_Z = [0.15, 0.16]

RETREAT_PL_X = [0.0, 0.0]
RETREAT_PL_Y = [0.18, 0.0]
RETREAT_PL_Z = [0.05, 0.15]

# APPROACH DISTANCES - PLACING // INITIAL OBJECT POSE
APPROACH_PLACE_X = [0.0, 0.0]
APPROACH_PLACE_Y = [0.0, 0.08]
APPROACH_PLACE_Z = [0.05, -0.08]

#GRIPPER ORIENTATION FOR SWITCH POINT
EULER_SWITCH = [[-3.1415,0,1.5708],[0,0,-1.5708]]

#MARGINS
MARGIN_X = [0.0, 0.0]
MARGIN_Y = [0.0, 0.005]
MARGIN_Z = [0.005, 0.005]

OFFSET_X = 0.0
