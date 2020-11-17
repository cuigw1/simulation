#!/usr/bin/env python
# -*- coding: utf-8 -*-

from option import Option
import world
import rospy
import util
from robot import robot as device
import conf


class Opt_action_putdown(Option):

    def __init__(self):
        labels = ['init', 'findpos', 'prepose', 'adjustpose', 'slamgo', 'putdown', 'finish']
        Option.__init__(self, labels, 'finish')
        self.room = None
        self.furniture = None
        self.object = None
        self.goal = None
        self.candinates = None
        self.max_dist = 0.4

    def findpos(self, p):
        for c in self.candinates:
            dist = util.geom_dist2(p, c[1])
            rospy.loginfo('dist: %.3f' % dist)
            if dist < self.max_dist:
                self.candinates.remove(c)
                return c[2]
        return None

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv:
                print(argv, world.rob_pose)
                self.object = {'name': argv[0]}
                self.room = world.topography.get_room(world.rob_pose)
                self.candinates, self.furniture = world.topography.get_candinates_of_furniture(
                    self.room['name'], argv[1])
                self.gripper = argv[2]
                self.goto(self.FINDPOS)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.FINDPOS:
            pos = self.findpos(world.rob_pose)
            if pos:
                height = 0.75
                if 'height' in self.furniture:
                    height = self.furniture['height']
                self.goal = [(pos[0]+pos[2])*0.5, (pos[1]+pos[3])*0.5, height + 0.1,
                             0.07, 0.07, 0.1]  # forniture height + object height
                self.goto(self.PUTDOWN)
            elif self.candinates:
                self.goal = self.candinates[0][1][0:3]
                self.goto(self.ADJUSTPOSE)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.ADJUSTPOSE:
            if self.subopt_reach_target():
                self.goto(self.FINDPOS)
            else:
                self.flow_to('slam_move', self.goal)

        elif self.cur_state == self.PUTDOWN:
            if self.subopt_reach_target():
                device.move_ev(device.normal_ev)
                self.goto(self.FINISH)
            else:
                self.flow_to('blind_put', self.goal)

        elif self.cur_state == self.FINISH:
            pass
