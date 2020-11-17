#!/usr/bin/env python
# -*- coding: utf-8 -*-

from option import Option
import world
import rospy
from opt_func_gotopose import Opt_func_slam_move
import tf
import math
import util


class Opt_action_moveto(Option):

    def __init__(self):
        labels = ['init', 'find_pos', 'go', 'check', 'finish']
        Option.__init__(self, labels, 'finish')
        self.furniture = None
        self.room = None
        self.candicates = None
        self.curr = -1

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv:
                self.room = argv[1]
                self.furniture = argv[0]
                self.candicates = None
                self.goto(self.FIND_POS)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.FIND_POS:
            candicates = world.topography.get_candinates_of_furniture(self.room, self.furniture)
            if candicates:
                self.candicates, self.furniture = candicates
                self.curr = 0
                self.goto(self.GO)
            else:
                rospy.logerr('no candicates of %s-%s found' % (self.room, self.furniture))

        elif self.cur_state == self.GO:
            if self.s_count == 1:
                if self.curr < len(self.candicates):
                    self.goal = self.candicates[self.curr][1]
                else:
                    if Opt_func_slam_move.result == False:
                        world.action_execution_result['status'] = 'fail'
                    self.goto(self.FINISH)
            else:
                if self.subopt_reach_target():
                    if Opt_func_slam_move.result == False:
                        world.action_execution_result['status'] = 'fail'
                    self.goto(self.FINISH)
                else:
                    self.flow_to('slam_move', self.goal)

        elif self.cur_state == self.FINISH:
            pass


class Opt_action_movetohuman(Option):

    def __init__(self):
        labels = ['init', 'find_pos', 'go', 'check', 'finish']
        Option.__init__(self, labels, 'finish')
        self.human = None
        self.room = None

    def findperson(self, name, rect):
        world.shared_resource_lock.acquire()
        persons = world.objects
        world.shared_resource_lock.release()
        res = []
        for k, v in persons.items():
            if world.topography.in_rect(v['pose'][0:3], rect):
                if k == name:
                    person = {'name': k, 'type': v['type'], 'pose': v['pose'], 'size': v['size']}
                    return person
        return None

    def findpose(self, pose):
        dist = 1.2
        r = tf.transformations.euler_from_quaternion(pose[3:7])
        x = pose[0] + dist * math.cos(r[2])
        y = pose[1] + dist * math.sin(r[2])
        alpha = math.atan2(pose[1] - y, pose[0] - x)
        res = [x, y, alpha]
        return res

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv:
                self.room = argv[1]
                self.human = argv[0]
                rospy.loginfo('human %s, room %s' % (self.human, self.room))
                self.goto(self.FIND_POS)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.FIND_POS:
            room = world.topography.get_room_by_name(self.room)
            if room:
                person = self.findperson(self.human, room['rect'])
                if person:
                    self.goal = self.findpose(person['pose'])
                    self.goto(self.GO)
                else:
                    world.action_execution_result['status'] = 'fail'
                    self.goto(self.FINISH)
            else:
                world.action_execution_result['status'] = 'fail'
                self.goto(self.FINISH)

        elif self.cur_state == self.GO:
            if self.subopt_reach_target():
                if Opt_func_slam_move.result == False:
                    world.action_execution_result['status'] = 'fail'
                self.goto(self.FINISH)
            else:
                self.flow_to('slam_move', self.goal)

        elif self.cur_state == self.FINISH:
            pass


class Opt_action_movein(Option):

    def __init__(self):
        labels = ['init', 'find_pos', 'goa', 'gob', 'check', 'finish']
        Option.__init__(self, labels, 'finish')
        self.door = None
        self.rooma = None
        self.roomb = None
        self.candicates = []
        self.dist = 0.5

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv:
                self.roomb = argv[0]
                self.rooma = argv[1]
                self.door = argv[2]
                self.candicates = []
                self.goto(self.FIND_POS)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.FIND_POS:
            self.candicates = world.topography.get_candinate_of_door(self.rooma, self.roomb, self.door, d=self.dist/2.0)
            if self.candicates:
                self.goto(self.GOA)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.GOA:
            if self.s_count == 1:
                self.goal = self.candicates[0]
                rospy.loginfo('a %s:%s [%.3f %.3f %.3f]' %
                              (self.rooma, self.door, self.goal[0], self.goal[1], self.goal[2]))
            else:
                if self.subopt_reach_target():
                    self.goto(self.GOB)
                else:
                    self.flow_to('slam_move', self.goal)

        elif self.cur_state == self.GOB:
            if self.s_count == 1:
                self.goal = util.pose_to_self(self.candicates[1], world.rob_pose)
                rospy.loginfo('b %s:%s [%.3f %.3f %.3f]' %
                              (self.rooma, self.door, self.goal[0], self.goal[1], self.goal[2]))
            else:
                if self.subopt_reach_target():
                    self.goto(self.FINISH)
                else:
                    self.flow_to("rob_go_dist", [self.dist, 0.5])

        elif self.cur_state == self.FINISH:
            pass
