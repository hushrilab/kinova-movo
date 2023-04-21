#!/usr/bin/env python
NB_GROUPS = 2

# ===== GENERAL - ROBOT AND PLANNING PARAMETERS =====
GROUP_NAME = ["right_arm","left_arm"]
GRIPPER_NAME = ["right_gripper","left_gripper"]
GRIPPER_GROUP = ["right_gripper","left_gripper"]
DIST = 0.04 # estimated distance between tool and wrist link
JOINT_CLOSE_GRIPPER = 0.0
JOINT_OPEN_GRIPPER = 50.0
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
IS_45 =  False

# ===== SCENE ELEMENTS =====
OBJ_NAMES_SCENE = ["shelf1","shelf2","shelf3","shelf4"]
OBJ_POS_SCENE = {
	"shelf1" : {
		"type" : "BOX",
		"x" : 0.65,
		"y" : -0.9,
		"z" : 0.435,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (1.21, 0.6, 0.87)
	},
	"shelf2" : {
		"type" : "BOX",
		"x" : 0.65,
		"y" : 0.9,
		"z" : 0.435,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (1.21, 0.6, 0.87)
	},
	"shelf3" : {
		"type" : "BOX",
		"x" : 1.85,
		"y" : -0.9,
		"z" : 0.435,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.65, 0.6, 0.87)
	},
	"shelf4" : {
		"type" : "BOX",
		"x" : 1.85,
		"y" : 0.9,
		"z" : 0.435,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.65, 0.6, 0.87)
	}
}


# ===== ITEMS TO GRASP =====
OBJ_NAMES_GRASP = ["object1","object2","object3"]#,"object4"]
OBJ_POS_GRASP = {
	"object1" : {
		"type" : "BOX",
		"x" : 0.65,
		"y" : -0.7,
		"z" : 1.07,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.05, 0.05, 0.395),
		"translation" : (0.0, 1.4, 0.0)
	},
	"object2" : {
		"type" : "BOX",
		"x" : 0.50,
		"y" : -0.7,
		"z" : 1.07,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.05, 0.05, 0.395),
		"translation" : (0.0, 1.4, 0.0)
	},
	"object3" : {
		"type" : "BOX",
		"x" : 0.35,
		"y" : -0.7,
		"z" : 1.07,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.05, 0.05, 0.395),
		"translation" : (0.0, 1.4, 0.0)
	},
	"object4" : {
		"type" : "BOX",
		"x" : 0.55,
		"y" : -0.65,
		"z" : 1.0,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.05, 0.05, 0.4),
		"translation" : (0.0, 1.6, 0.0)
	}
}

# APPROACH DISTANCES - PICKING // INITIAL OBJECT POSE
APPROACH_PICK_X = [0.0, 0.0]
APPROACH_PICK_Y = [0.0, 0.1]
APPROACH_PICK_Z = [0.08, 0.15]

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
