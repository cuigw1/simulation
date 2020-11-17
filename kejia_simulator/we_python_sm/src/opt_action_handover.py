#!/usr/bin/env python
# -*- coding: utf-8 -*-

from option import Option
import world
import rospy
from robot import robot as device
import conf
from std_msgs.msg import String


class Opt_action_handover(Option):

    def __init__(self):
        labels = ['init', 'move_arm', 'speak', 'wait', 'open_paw', 'armbk', 'finish']
        Option.__init__(self, labels, 'finish')
        self.obj = None
        self.person = None
        self.hand = None
        self.goal_angles = [0, 45, 90, -45, 0]
        self.robot_pub = rospy.Publisher("/hri/robot_response", String, queue_size=50)

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if len(argv) >= 3:
                self.obj = argv[0]
                self.person = argv[1]
                self.hand = argv[2]
                self.goto(self.SPEAK)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.MOVE_ARM:
            if self.subopt_reach_target():
                self.goto(self.WAIT)
            else:
                self.flow_to('rob_move_arm', self.goal_angles)

        elif self.cur_state == self.SPEAK:
            s = 'here you are, please careful'
            rospy.loginfo(s)
            msg = String()
            msg.data = s
            self.robot_pub.publish(s)
            self.goto(self.MOVE_ARM)

        elif self.cur_state == self.WAIT:
            if self.s_count == 50:
                self.goto(self.OPEN_PAW)

        elif self.cur_state == self.OPEN_PAW:
            if self.s_count == 1:
                device.open_paw()
            if self.s_count == 10 * device.release_time:
                self.goto(self.ARMBK)

        elif self.cur_state == self.ARMBK:
            device.left_armbk()
            self.goto(self.FINISH)

        elif self.cur_state == self.FINISH:
            pass
