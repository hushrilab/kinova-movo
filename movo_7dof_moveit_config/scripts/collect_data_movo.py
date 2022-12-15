#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
import os

class CollectDataMovo(object):

    def __init__(self, action_name):

        super(CollectDataMovo,self).__init__()

        self._ee_pose_r = Pose()
        self._ee_pose_l = Pose()
        self._joint_pos = [0] * 19
        self._joint_vel = [0] * 19
        self._joint_eff = [0] * 19

        # ros subscribers
        rospy.Subscriber('/ee_state_r', PoseStamped, self.callback_ee_state_r)
        rospy.Subscriber('/ee_state_l', PoseStamped, self.callback_ee_state_l)
        rospy.Subscriber('/joint_states', JointState, self.callback_joint_state)

        folder_name = os.getcwd() + "/data/movo_" + action_name
        self._filepose_r = open(folder_name+"_pose_r.csv","w+")
        self._filepose_l = open(folder_name+"_pose_l.csv","w+")
        self._filejointpos = open(folder_name+"_jointpos.csv","w+")
        self._filejointvel = open(folder_name+"_jointvel.csv","w+")
        self._filejointeff = open(folder_name+"_jointeff.csv","w+")

        self._rate = 50.33
        self.finished = False

        #print("Running. Ctrl-c to quit")

    def callback_ee_state_r(self,data):
        self._ee_pose_r = data.pose

    def callback_ee_state_l(self,data):
        self._ee_pose_l = data.pose

    def callback_joint_state(self,data):
        self._joint_pos = data.position
        self._joint_vel = data.velocity
        self._joint_eff = data.effort

    def activate_thread(self):
        # set control rate
        control_rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown() and not self.finished:
            self._dump_data()
            control_rate.sleep()

    def _dump_data(self):
        pose_r = [0] * 7
        pose_r[0] = self._ee_pose_r.position.x
        pose_r[1] = self._ee_pose_r.position.y
        pose_r[2] = self._ee_pose_r.position.z
        pose_r[3] = self._ee_pose_r.orientation.x
        pose_r[4] = self._ee_pose_r.orientation.y
        pose_r[5] = self._ee_pose_r.orientation.z
        pose_r[6] = self._ee_pose_r.orientation.w
        self.write_data(self._filepose_r, pose_r)

        pose_l = [0] * 7
        pose_l[0] = self._ee_pose_l.position.x
        pose_l[1] = self._ee_pose_l.position.y
        pose_l[2] = self._ee_pose_l.position.z
        pose_l[3] = self._ee_pose_l.orientation.x
        pose_l[4] = self._ee_pose_l.orientation.y
        pose_l[5] = self._ee_pose_l.orientation.z
        pose_l[6] = self._ee_pose_l.orientation.w
        self.write_data(self._filepose_l, pose_l)

        self.write_data(self._filejointpos, self._joint_pos)
        self.write_data(self._filejointvel, self._joint_vel)
        self.write_data(self._filejointeff, self._joint_eff)

    def write_data(self, fileobj, data):
        for datum in data:
            fileobj.write("%f, " % datum)
        fileobj.write("\n")

    def clean_shutdown(self):
        self.finished = True
        self._filepose_r.close()
        self._filepose_l.close()
        self._filejointpos.close()
        self._filejointvel.close()
        self._filejointeff.close()

def main():
    # Starting node connection to ROS
    print("Initializing node... ")
    rospy.init_node("movodatadumper")
    dumper = CollectDatMovo()
    rospy.on_shutdown(dumper.clean_shutdown)
    dumper.activate_thread()

if __name__ == "__main__":
    main()
