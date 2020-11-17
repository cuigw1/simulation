#!/usr/bin/env python
# -*- coding: utf-8 -*-

from option import Option
from arm_ctl import Arm_control
from opt_func_gotopose import Opt_func_blind_move
import rospy
import world
import util
from robot import robot as device


class Opt_func_fetch(Option):
    def __init__(self):
        labels = ['init', 'blind_move', 'init_ev', 'calc', 'open_paw', 'move_arm', 'pre_arm', 'forward', 'lift_up', 'PRE_ARM2', 'back', 'hold',
                  'adj_ev', 'close_paw', 'open_paw', 'wait', 'finish']
        Option.__init__(self, labels, 'finish')
        self.arm_angles = None
        self.best_dist = 0.90
        self.obj_info = None
        self.push_dist = 0.2
        self.wait = [1, None]
        self.liftup_height = 0.1
        self.back_dist = 0.0
        self.grasp_time = device.grasp_time
        self.pre_arm_angles = device.pre_arm_angles

    def find_avail_ev(self):
        step = 0.02
        ev = world.ev
        p = util.pose_to_self(self.obj_info[0:3], world.rob_pose)
        rospy.loginfo('ev: %.3f, objinbase:[%.3f %.3f %.3f], arm: [%.3f %.3f %.3f]' % (
            ev, p[0], p[1], p[2], device.base2arm[0], device.base2arm[1], device.base2arm[2] + ev - device.init_ev))
        pos = util.base_to_arm(p)
        pos[0] -= self.push_dist
        rospy.loginfo('ev: %.3f, objinarm:[%.3f %.3f %.3f], arm: [%.3f %.3f %.3f]' % (
            ev, pos[0], pos[1], pos[2], device.base2arm[0], device.base2arm[1], device.base2arm[2] + ev - device.init_ev))
        while ev >= device.ev_range[0] - 0.01 and ev <= device.ev_range[1] + 0.01:
            angles = Arm_control.calc_angles(pos)
            if angles:
                rospy.loginfo('angles: [%.3f %.3f %.3f %.3f 0]' % (angles[0], angles[1], angles[2], angles[3]))
                return [ev, angles]
            else:
                rospy.logwarn('ev: %.3f, objinarm:[%.3f %.3f %.3f], arm: [%.3f %.3f %.3f]' % (
                    ev, pos[0], pos[1], pos[2], device.base2arm[0], device.base2arm[1], device.base2arm[2] + ev - device.init_ev))
            if pos[2] > -step and pos[2] < step:
                break
            if pos[2] > 0:
                pos[2] -= step
                ev += step
            else:
                pos[2] += step
                ev -= step
        return None

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv:
                self.obj_info = argv[0:3]
                self.gripper_open_width = argv[3]
                self.goto(self.BLIND_MOVE)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.BLIND_MOVE:
            if self.s_count == 1:
                self.local_obj_pos = util.pose_to_self(self.obj_info[0:3], world.rob_pose)
            if self.subopt_reach_target():
                self.back_dist = Opt_func_blind_move.move_dist
                rospy.loginfo('blind move dist %.3f' % self.back_dist)
                self.goto(self.INIT_EV)
            else:
                self.flow_to('rob_blind_move', {'goal': self.local_obj_pos, 'dist': self.best_dist})

        elif self.cur_state == self.INIT_EV:
            if self.s_count == 1:
                h = util.base_to_arm(self.obj_info[0:3])[2]
                self.tar_ev = world.ev + device.arm_parts[1] + h
                if self.tar_ev > device.ev_range[1]:
                    self.tar_ev = device.ev_range[1]
                elif self.tar_ev < device.ev_range[0]:
                    self.tar_ev = device.ev_range[0]
            if self.subopt_reach_target():
                self.wait = [15, self.CALC]
                self.goto(self.WAIT)
            else:
                self.flow_to('rob_move_ev', self.tar_ev)

        elif self.cur_state == self.CALC:
            ret = self.find_avail_ev()
            if ret:
                self.tar_ev, self.arm_angles = ret
                self.goto(self.OPEN_PAW)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.ADJ_EV:
            if self.subopt_reach_target():
                self.goto(self.PRE_ARM)
            else:
                self.flow_to('rob_move_ev', self.tar_ev)

        elif self.cur_state == self.OPEN_PAW:
            if self.s_count == 1:
                device.open_paw()
            if self.s_count == 30:
                self.goto(self.ADJ_EV)

        elif self.cur_state == self.PRE_ARM:
            if self.subopt_reach_target():
                self.goto(self.MOVE_ARM)
            else:
                self.flow_to('rob_pre_fetch', self.pre_arm_angles)
        elif self.cur_state == self.MOVE_ARM:
            if self.subopt_reach_target():
                self.goto(self.FORWARD)
            else:
                self.flow_to('rob_move_arm', self.arm_angles + [0])
        elif self.cur_state == self.FORWARD:
            if self.subopt_reach_target():
                self.goto(self.CLOSE_PAW)
            else:
                self.flow_to('rob_go_dist', [self.push_dist, 0.8])
        elif self.cur_state == self.CLOSE_PAW:
            if self.s_count == 1:
                device.close_paw(self.gripper_open_width)
            if self.s_count == self.grasp_time * 10:
                self.goto(self.LIFT_UP)
        elif self.cur_state == self.LIFT_UP:
            if self.s_count == 1:
                self.tar_ev = min(self.liftup_height + world.ev, 0)
            else:
                if self.subopt_reach_target():
                    self.goto(self.BACK)
                else:
                    self.flow_to('rob_move_ev', self.tar_ev)
        elif self.cur_state == self.PRE_ARM2:
            if self.subopt_reach_target():
                self.goto(self.HOLD)
            else:
                self.flow_to('rob_move_arm', self.pre_arm_angles)
        elif self.cur_state == self.BACK:
            if self.s_count == 1:
                self.back_dist += (self.push_dist + 0.1)
            if self.subopt_reach_target():
                device.move_ev(device.normal_ev)
                self.goto(self.HOLD)
            else:
                self.flow_to('rob_go_dist', [-self.back_dist, 0.5])
        elif self.cur_state == self.HOLD:
            if self.subopt_reach_target():
                self.goto(self.FINISH)
            else:
                self.flow_to('rob_hold')
        elif self.cur_state == self.WAIT:
            if self.s_count > self.wait[0]:
                tar = self.wait[1]
                if tar == self.CALC:
                    self.goto(self.CALC)
                else:
                    self.goto(self.FINISH)
            else:
                pos = util.pose_to_self(self.obj_info[0:3], world.rob_pose)
                rospy.loginfo('objinbase: [%.3f, %.3f, %.3f]' % (pos[0], pos[1], pos[2]))
        elif self.cur_state == self.FINISH:
            pass


class Opt_func_hold(Option):
    def __init__(self):
        labels = ['init', 'pre_armbk', 'hold', 'finish']
        Option.__init__(self, labels, 'finish')
        self.pre_arm_angles = device.pre_arm_angles
        self.pre_armbk_angles = [-90, 0, 90, 0, 0]
        self.armbk_angles = [-90, -60, 60, 90, 0]

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            self.goto(self.HOLD)

        elif self.cur_state == self.PRE_ARMBK:
            if self.subopt_reach_target():
                self.goto(self.HOLD)
            else:
                self.flow_to('rob_move_arm', self.pre_armbk_angles)

        elif self.cur_state == self.HOLD:
            if self.subopt_reach_target():
                self.goto(self.FINISH)
            else:
                self.flow_to('rob_move_arm', self.armbk_angles)
        elif self.cur_state == self.FINISH:
            pass


class Opt_func_prefetch(Option):
    def __init__(self):
        labels = ['init', 'pre_armbk', 'pre_fetch', 'finish']
        Option.__init__(self, labels, 'finish')
        self.goal_angles = None
        self.pre_prefetch_angles = [0, 0, 150, 0, 0]

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv:
                self.goal_angles = argv
            else:
                self.goal_angles = None
            self.goto(self.PRE_ARMBK)

        elif self.cur_state == self.PRE_ARMBK:
            if self.subopt_reach_target():
                self.goto(self.PRE_FETCH)
            else:
                self.flow_to('rob_simple_move_arm', self.pre_prefetch_angles)

        elif self.cur_state == self.PRE_FETCH:
            if self.subopt_reach_target():
                self.goto(self.FINISH)
            else:
                self.flow_to('rob_simple_move_arm', self.goal_angles)
        elif self.cur_state == self.FINISH:
            pass


class Opt_func_liftup(Option):
    def __init__(self):
        labels = ['init', 'lift_up', 'adj_ev', 'finish']
        Option.__init__(self, labels, 'finish')
        self.arm_angles = None
        self.tar_pos = None

    def find_avail_ev(self):
        step = 0.02
        ev = world.ev
        pos = self.tar_pos
        rospy.loginfo('ev: %.3f, objinarm:[%.3f %.3f %.3f], arm: [%.3f %.3f %.3f]' % (
            ev, pos[0], pos[1], pos[2], device.base2arm[0], device.base2arm[1], device.base2arm[2] + ev - device.init_ev))
        while ev >= device.ev_range[0] - 0.01 and ev <= device.ev_range[1] + 0.01:
            angles = Arm_control.calc_angles(pos)
            if angles:
                rospy.loginfo('angles: [%.3f %.3f %.3f %.3f 0]' % (angles[0], angles[1], angles[2], angles[3]))
                return [ev, angles]
            else:
                rospy.logwarn('ev: %.3f, objinarm:[%.3f %.3f %.3f], arm: [%.3f %.3f %.3f]' % (
                    ev, pos[0], pos[1], pos[2], device.base2arm[0], device.base2arm[1], device.base2arm[2] + ev - device.init_ev))
            if pos[2] > -step and pos[2] < step:
                break
            if pos[2] > 0:
                pos[2] -= step
                ev += step
            else:
                pos[2] += step
                ev -= step
        return None

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv and len(argv) == 3:
                self.tar_pos = argv
                self.goto(self.ADJ_EV)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.ADJ_EV:
            if self.s_count == 1:
                ret = self.find_avail_ev()
                if ret:
                    self.tar_ev, self.arm_angles = ret
                else:
                    rospy.logwarn('can not find solution')
                    self.goto(self.FINISH)
            else:
                if self.subopt_reach_target():
                    self.goto(self.LIFT_UP)
                else:
                    self.flow_to('rob_move_ev', self.tar_ev)

        elif self.cur_state == self.LIFT_UP:
            if self.subopt_reach_target():
                self.goto(self.FINISH)
            else:
                self.flow_to('rob_move_arm', self.arm_angles + [0])

        elif self.cur_state == self.FINISH:
            pass
