#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class PublisherEeState(object):

    def __init__(self,move_group_r, move_group_l):
        super(PublisherEeState,self).__init__()
        self.move_group_r = move_group_r
        self.move_group_l = move_group_l
        self.finished = False

    def ee_state_talker(self):
        pub_r = rospy.Publisher('ee_state_r', PoseStamped, queue_size=10)
        pub_l = rospy.Publisher('ee_state_l', PoseStamped, queue_size=10)
        rate = rospy.Rate(50.33) # 10hz
        while not rospy.is_shutdown() and not self.finished:
            pos_ee_r = self.move_group_r.get_current_pose()
            pos_ee_l = self.move_group_l.get_current_pose()
            pub_r.publish(pos_ee_r)
            pub_l.publish(pos_ee_l)
            rate.sleep()

    def clean_shutdown(self):
        self.finished = True
