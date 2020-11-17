#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from option import Option
import world
from robot import robot as device
import util
import rospy


class Opt_func_go_dist(Option):
    """ rob go dist """

    def __init__(self):
        labels = ['init', 'godest', 'stop', 'finish']
        Option.__init__(self, labels, 'finish')
        self.target_dist = 0.
        self.odom_start_pose = None
        self.scale = 1
        self.odom_scale_x = 1.0
        self.direction = 1

    def transfer(self, argv=None):
        """argv: the distance to go """

        if self.cur_state == self.INIT:
            self.odom_start_pose = world.odom_pose
            if type(argv) == float:
                self.target_dist = math.fabs(argv) * self.odom_scale_x
                self.sign = util.sign(argv)
                self.scale = 1
            else:
                self.target_dist = math.fabs(argv[0]) * self.odom_scale_x
                self.sign = util.sign(argv[0])
                self.scale = math.fabs(argv[1])
            self.goto(self.GODEST)

        elif self.cur_state == self.GODEST:
            diff = util.geom_dist2(world.odom_pose, self.odom_start_pose)
            diff = self.target_dist - diff
            if math.fabs(diff) < 0.01:
                self.goto(self.STOP)
            else:
                vel = self.sign * util.sign(diff) * \
                    math.sqrt(math.fabs(diff)) * self.scale
                s = 1.0
                vel *= s
                device.rob_go(vel, 0, 0)

        elif self.cur_state == self.STOP:
            device.rob_stop()
            self.goto(self.FINISH)


class Opt_func_turn_dist(Option):
    """ rob go dist """

    def __init__(self):
        labels = ['init', 'turndest', 'stop', 'finish']
        Option.__init__(self, labels, 'finish')
        self.target_r = 0.0
        self.speed_scale = 1.0
        self.target_error = 0.05

    def transfer(self, argv=None):
        """argv: the distance to turn """
        if self.cur_state == self.INIT:
            self.goto(self.TURNDEST)
            if type(argv) == float:
                self.target_r = util.normalize(world.odom_pose[2] + argv)
                self.speed_scale = 1.0
                self.target_error = 0.05
            elif type(argv) == list:
                rospy.loginfo(argv)
                if len(argv) == 3:
                    self.target_r = util.normalize(
                        world.odom_pose[2] + argv[0])
                    self.target_error = argv[2]
                    self.speed_scale = argv[1]
                elif len(argv) == 2:
                    self.target_r = util.normalize(
                        world.odom_pose[2] + argv[0])
                    self.target_error = 0.05
                    self.speed_scale = argv[1]
                elif len(argv) == 1:
                    self.target_error = 0.05
                    self.target_r = util.normalize(
                        world.odom_pose[2] + argv[0])
                    self.speed_scale = 1.0
                else:
                    print "wrong arg length"
                    self.goto(self.FINISH)
            else:
                print "wrong arg type"
                self.goto(self.FINISH)

        elif self.cur_state == self.TURNDEST:
            if abs(util.normalize(world.odom_pose[2] - self.target_r)) <= self.target_error:
                self.goto(self.STOP)
            else:
                diff = util.normalize(self.target_r - world.odom_pose[2])
                vel = util.sign(diff) * math.sqrt(math.fabs(diff / math.pi))
                vel = vel * 0.7 * self.speed_scale
                if abs(vel) < 0.01 and vel != 0:
                    vel = 0.01 * vel / abs(vel)
                print(abs(util.normalize(world.odom_pose[2] - self.target_r)), vel)
                device.rob_turn(vel)

        elif self.cur_state == self.STOP:
            device.rob_stop()
            self.goto(self.FINISH)

        elif self.cur_state == self.FINISH:
            pass


class Opt_func_move_y(Option):
    error_dist = 0.05
    error_r = 0.1

    def __init__(self):
        labels = ['init', 'turn', 'ahead', 'turnback', 'goback', 'finish']
        Option.__init__(self, labels, 'finish')

        self.target_pose = 0

    def transfer(self, argv):
        if self.cur_state == self.INIT:
            self.target_pose = argv
            self.goto(self.TURN)

        elif self.cur_state == self.TURN:
            if self.subopt_reach_target():
                self.goto(self.AHEAD)
            else:
                self.flow_to(
                    "rob_turn_dist", [-math.pi / 6.0 * self.target_pose / abs(self.target_pose), 1.5])

        elif self.cur_state == self.AHEAD:
            if self.subopt_reach_target():
                self.goto(self.TURNBACK)
            else:
                self.flow_to("rob_go_dist", [-abs(self.target_pose*2), 4])

        elif self.cur_state == self.TURNBACK:
            if self.subopt_reach_target():
                self.goto(self.GOBACK)
            else:
                self.flow_to("rob_turn_dist", [
                             math.pi / 6.0 * self.target_pose / abs(self.target_pose), 1.5])

        elif self.cur_state == self.GOBACK:
            if self.subopt_reach_target():
                self.goto(self.FINISH)
            else:
                self.flow_to("rob_go_dist", [
                             abs(self.target_pose * math.sqrt(3.0)), 1.5])
        elif self.cur_state == self.FINISH:
            pass


class Opt_func_blind_move(Option):
    move_dist = 0

    def __init__(self):
        labels = ['init', 'turn', 'move', 'finish']
        Option.__init__(self, labels, 'finish')

    def transfer(self, argv):
        if self.cur_state == self.INIT:
            self.goal = argv['goal']
            self.dist = argv['dist']
            self.goto(self.TURN)
        elif self.cur_state == self.TURN:
            if self.s_count == 1:
                self.turn_dist = math.atan2(self.goal[1], self.goal[0])
                rospy.loginfo('turn dist: %f' % self.turn_dist)
            if math.fabs(self.turn_dist) > 1.0 / 360:
                if self.subopt_reach_target():
                    self.goto(self.MOVE)
                else:
                    self.flow_to("rob_turn_dist", [self.turn_dist, 0.8])
            else:
                self.goto(self.MOVE)
        elif self.cur_state == self.MOVE:
            if self.s_count == 1:
                self.go_dist = util.geom_dist(self.goal[0], self.goal[1]) - self.dist
                Opt_func_blind_move.move_dist = self.go_dist
                rospy.loginfo('go dist: %f' % self.go_dist)
            if math.fabs(self.go_dist) > 0.01:
                if self.subopt_reach_target():
                    self.goto(self.FINISH)
                else:
                    self.flow_to("rob_go_dist", [self.go_dist, 0.5])
            else:
                self.goto(self.FINISH)


class Opt_func_move_ev(Option):
    def __init__(self):
        labels = ['init', 'move', 'check', 'finish']
        Option.__init__(self, labels, 'finish')

    def transfer(self, argv):
        if self.cur_state == self.INIT:
            self.ev = argv
            if self.ev != None and self.ev >= device.ev_range[0] and self.ev <= device.ev_range[1]:
                self.goto(self.MOVE)
            else:
                self.goto(self.FINISH)
        elif self.cur_state == self.MOVE:
            device.move_ev(self.ev)
            self.goto(self.CHECK)
        elif self.cur_state == self.CHECK:
            if self.s_count < 100:
                if math.fabs(world.ev - self.ev) < 0.005:
                    self.goto(self.FINISH)
            else:
                self.goto(self.FINISH)
        elif self.cur_state == self.FINISH:
            pass


class Opt_func_slam_move(Option):
    result = False

    def __init__(self):
        labels = ['init', 'turn', 'navigate', 'check', 'finish']
        Option.__init__(self, labels, 'finish')
        self.goal = None
        self.turn_dist = 0
        self.min_turn_dist = 120.0 / 180 * math.pi

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            self.goal = argv
            if self.goal == None:
                rospy.logwarn('no goal !!!')
            else:
                rospy.loginfo(argv)
                theta = util.geom_theta(self.goal, world.rob_pose)
                self.turn_dist = theta - world.rob_pose[2]
                if self.turn_dist > math.pi:
                    self.turn_dist -= 2 * math.pi
                elif self.turn_dist > math.pi:
                    self.turn_dist += 2 * math.pi
                if abs(self.turn_dist) > self.min_turn_dist:
                    self.goto(self.TURN)
                else:
                    self.goto(self.NAVIGATE)

        elif self.cur_state == self.TURN:
            if self.subopt_reach_target():
                self.goto(self.NAVIGATE)
            else:
                self.flow_to("rob_turn_dist", [self.turn_dist, 1.2])

        elif self.cur_state == self.NAVIGATE:
            world.navigate_result = None
            device.slam_move(self.goal)
            self.goto(self.CHECK)
        elif self.cur_state == self.CHECK:
            if world.navigate_result:
                if world.navigate_result.status.status == 4:
                    rospy.logwarn(world.navigate_result.status.text)
                    Opt_func_slam_move.result = False
                else:
                    rospy.loginfo(world.navigate_result.status.text)
                    Opt_func_slam_move.result = True
                self.goto(self.FINISH)
        elif self.cur_state == self.FINISH:
            pass
