#!/usr/bin/env python

import sys
import os
import csv
import matplotlib.pyplot as plt
from math import ceil, floor
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

sampling_rate = 50.33  # Hz when using collect_data_talos.py


def get_data_ee(file_ee):
	file = open(file_ee)
	csvreader = csv.reader(file)
	ee_position = []
	ee_orientation = []
	for row in csvreader:
		position = [float(row[0]), float(row[1]), float(row[2])]
		orientation = [float(row[3]), float(row[4]), float(row[5]), float(row[6])]
		ee_position.append(position)
		ee_orientation.append(orientation)
	return ee_position, ee_orientation

def get_data_joint(file_joint, nb_joint):
	file = open(file_joint)
	csvreader = csv.reader(file)
	joint_data = []
	for row in csvreader:
		joint_data.append(float(row[nb_joint]))
	return joint_data

def plot_ee_3d(data_ee_pos, ee_name):

	fig = plt.figure()
	ax = fig.add_subplot(111,projection='3d')
	xs = []
	ys = []
	zs = []
	for pos in data_ee_pos:
		xs.append(pos[0])
		ys.append(pos[1])
		zs.append(pos[2])
	col = np.arange(len(data_ee_pos))
	ax.scatter(xs, ys, zs, cmap="cool",c=col, marker='o')
	ax.set_xlabel("X axis")
	ax.set_ylabel("Y axis")
	ax.set_zlabel("Z axis")
	plt.title("Evolution of " + ee_name + " in 3D //odom_link")
	plt.show()

def plot_ee_time(data_ee_pos, ee_name):
	samples = len(data_ee_pos)
	time_ms = samples * 1000 /sampling_rate
	xlist = [a * ceil(time_ms)/samples for a in range(samples)]
	plt.figure(1)
	plt.plot([x / 1000 for x in xlist], data_ee_pos)
	plt.title("Position evolution of " + ee_name)
	plt.xlabel("Time (s)")
	plt.ylabel("Position //odom_link (m)")
	plt.legend(["x","y","z"])
	plt.show()


def plot_joint_time(data_joint_pos, data_joint_vel, data_joint_eff, nb_joint):
	samples = len(data_joint_pos)
	time_ms = samples * 1000 /sampling_rate
	xlist = [a * ceil(time_ms)/samples for a in range(samples)]
	plt.figure(1)
	plt.plot([x / 1000 for x in xlist], data_joint_pos)
	plt.title("Joint " +  str(nb_joint) + " position evolution")
	plt.xlabel("Time (s)")
	plt.ylabel("Joint position (rad)")
	plt.show()
	plt.figure(2)
	plt.plot([x / 1000 for x in xlist], data_joint_vel)
	plt.title("Joint " +  str(nb_joint) + " velocity evolution")
	plt.xlabel("Time (s)")
	plt.ylabel("Joint velocity")
	plt.show()
	plt.figure(3)
	plt.plot([x / 1000 for x in xlist], data_joint_eff)
	plt.title("Joint " +  str(nb_joint) + " effort evolution")
	plt.xlabel("Time (s)")
	plt.ylabel("Joint effort")
	plt.show()



def main(action_name):

	folder_name = os.getcwd() + "/data/movo_" + action_name
	file_ee_r = folder_name + "_pose_r.csv"
	file_ee_l = folder_name + "_pose_l.csv"
	file_jointpos = folder_name + "_jointpos.csv"
	file_jointvel = folder_name + "_jointvel.csv"
	file_jointeff = folder_name + "_jointeff.csv"
	nb_joint = 5 # ex 30 = torso turn

	ee_pos_r, ee_or_r = get_data_ee(file_ee_r)
	ee_pos_l, ee_or_l = get_data_ee(file_ee_l)
	joint_pos = get_data_joint(file_jointpos, nb_joint)
	joint_vel = get_data_joint(file_jointvel, nb_joint)
	joint_eff = get_data_joint(file_jointeff, nb_joint)

	plot_ee_time(ee_pos_r, "ee_right")
	plot_ee_time(ee_pos_l, "ee_left")
	plot_joint_time(joint_pos, joint_vel, joint_eff, nb_joint)
	plot_ee_3d(ee_pos_r, "ee_right")
	plot_ee_3d(ee_pos_l, "ee_left")


if __name__ == "__main__":

	if len(sys.argv)!=2:
		print "Missing argument, precise pickplace or handover action."
		print ""
	else:
		action_name = sys.argv[1]
		if action_name != "handover" and action_name != "pickplace":
			print "Action name not recognized, the action should be pickplace or handover."
		main(action_name)
