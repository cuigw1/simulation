#!/usr/bin/env python
# -*- coding: utf-8 -*-

from option import Option
from opt_action_scan import Opt_action_scan
import world
import rospy
import util
import conf
from robot import robot as device


class Opt_action_pickup(Option):

    def __init__(self):
        labels = ['init', 'findobj', 'prepose', 'adjustpose', 'pickup', 'check', 'finish']
        Option.__init__(self, labels, 'finish')
        self.room = None
        self.furniture = None
        self.object = None
        self.goal = None
        self.max_dist = 0.4
        self.max_dist2edge = 0.4
        self.max_try_times = 2
        self.try_times = 0

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv:
                self.object = {'name': argv[0], 'type': argv[-1]}
                self.room = world.topography.get_room(world.rob_pose)
                self.candinates, self.furniture = world.topography.get_candinates_of_furniture(
                    self.room['name'], argv[1])
                self.gripper = argv[2]
                self.try_times = 0
                self.goto(self.FINDOBJ)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.FINDOBJ:
            if self.subopt_reach_target():
                objs = Opt_action_scan.result[:]
                found = False
                if objs:
                    for obj in objs:
                        if obj['name'] == self.object['name']:
                            self.object = obj
                            found = True
                            rospy.loginfo('found %s', str(self.object))
                if not found:
                    rospy.logerr('can not find %s from %s:%s' %
                                 (self.object['name'], self.room['name'], self.furniture['name']))
                    world.action_execution_result['status'] = 'fail'
                    if self.try_times > 0:
                        world.action_execution_result['detail'] = 'found a %s on the %s in the %s, but can not grasp it' % (
                            self.object['type'], argv[1], self.room['name'])
                    self.goto(self.FINISH)
                else:
                    self.goto(self.PREPOSE)
            else:
                self.flow_to('action_scan', [self.object['name'], self.furniture['name'], self.object['type']])

        elif self.cur_state == self.PREPOSE:
            if self.s_count == 1:
                candinates = world.topography.get_candinate_of_furniture(
                    self.object['pose'][0:3], self.room['name'], self.furniture['name'], d=0.3)
                self.goal = None
                for p, d in zip(candinates[0], candinates[2]):
                    if d < self.max_dist2edge:
                        self.goal = p[1][0:3]
                        break
            if self.goal:
                dist = util.geom_dist2(self.goal, world.rob_pose)
                rospy.loginfo('dist: %.3f' % dist)
                if dist > self.max_dist:
                    self.goto(self.ADJUSTPOSE)
                else:
                    self.goto(self.PICKUP)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.ADJUSTPOSE:
            if self.subopt_reach_target():
                self.goto(self.PICKUP)
            else:
                self.flow_to('slam_move', self.goal)

        elif self.cur_state == self.PICKUP:
            if self.s_count == 1:
                if self.object['type'] in conf.model_size:
                    size = conf.model_size[self.object['type']]
                    self.object['pose'][2] += max(size[2]/2.0, 0.03)
                    self.open_width = max(size[1] - 0.01, 0.02)
                else:
                    self.object['pose'][2] += 0.06
                    self.open_width = 0.07
                rospy.logerr(self.open_width)
                rospy.logerr(world.paw)
            if self.subopt_reach_target():
                rospy.logerr(world.paw)
                self.goto(self.CHECK)
            else:
                self.flow_to('blind_fetch', [self.object['pose'][0],
                                             self.object['pose'][1],
                                             self.object['pose'][2],
                                             self.open_width])
        elif self.cur_state == self.CHECK:
            self.try_times += 1
            paw_value = world.paw[0] + world.paw[1]
            if self.open_width + 0.003 < paw_value:
                rospy.loginfo('pickup success %.3f %.3f' % (paw_value, self.open_width))
                self.goto(self.FINISH)
            else:
                rospy.logwarn('pickup failed %.3f %.3f' % (paw_value, self.open_width))
                device.left_armbk()
                if self.try_times == self.max_try_times:
                    world.action_execution_result = {'status': 'fail',
                                                     'detail': 'found a %s on the %s in the %s, but can not grasp it'
                                                     % (self.object['type'], argv[1], self.room['name'])}
                    self.goto(self.FINISH)
                else:
                    self.goto(self.FINDOBJ)

        elif self.cur_state == self.FINISH:
            pass
