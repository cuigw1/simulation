#!/usr/bin/env python
# -*- coding: utf-8 -*-

from option import Option
from arm_ctl import Arm_control
import rospy
import world
import util
from robot import robot as device


class Opt_func_put(Option):
    def __init__(self):
        labels = ['init', 'blind_move', 'calc', 'init_ev', 'open_paw', 'move_arm', 'pre_arm', 'back', 'armbk',
                  'adj_ev', 'open_paw', 'wait', 'finish']
        Option.__init__(self, labels, 'finish')
        self.arm_angles = None
        self.best_dist = 0.65
        self.obj_info = None
        self.wait = [1, None]
        self.put_height = 0.05
        self.back_dist = 0.2
        self.release_time = device.release_time
        self.pre_arm_angles = device.pre_arm_angles

    def find_avail_ev(self):
        step = 0.02
        ev = world.ev
        p = util.pose_to_self(self.obj_info[0:3], world.rob_pose)
        rospy.loginfo('ev: %.3f, objinbase:[%.3f %.3f %.3f]' % (ev, p[0], p[1], p[2]))
        pos = util.base_to_arm(p)
        pos[2] += self.put_height
        rospy.loginfo('ev: %.3f, objinbase:[%.3f %.3f %.3f]' % (ev, p[0], p[1], p[2]))
        while ev >= device.ev_range[0] - 0.01 and ev <= device.ev_range[1] + 0.01:
            angles = Arm_control.calc_angles(pos)
            if angles:
                rospy.loginfo('angles: [%.3f %.3f %.3f %.3f 0]' % (angles[0], angles[1], angles[2], angles[3]))
                return [ev, angles]
            else:
                rospy.logwarn('ev: %.3f, objinarm:[%.3f %.3f %.3f]' % (ev, pos[0], pos[1], pos[2]))
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
            if argv and len(argv) >= 6:
                self.obj_info = argv
                self.goto(self.BLIND_MOVE)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.BLIND_MOVE:
            if self.s_count == 1:
                self.local_obj_pos = util.pose_to_self(self.obj_info[0:3], world.rob_pose)
            if self.subopt_reach_target():
                self.goto(self.INIT_EV)
            else:
                self.flow_to('rob_blind_move', {'goal': self.local_obj_pos, 'dist': self.best_dist})

        elif self.cur_state == self.INIT_EV:
            if self.s_count == 1:
                h = util.base_to_arm(self.obj_info[0:3])[2]
                self.tar_ev = world.ev + device.arm_parts[1] + h + self.obj_info[5]
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
                self.goto(self.ADJ_EV)
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
            if self.s_count == 10 * self.release_time:
                self.goto(self.BACK)

        elif self.cur_state == self.PRE_ARM:
            if self.subopt_reach_target():
                self.goto(self.MOVE_ARM)
            else:
                self.flow_to('rob_pre_put', self.pre_arm_angles)
        elif self.cur_state == self.MOVE_ARM:
            if self.subopt_reach_target():
                self.goto(self.OPEN_PAW)
            else:
                self.flow_to('rob_move_arm', self.arm_angles + [0])
        elif self.cur_state == self.BACK:
            if self.subopt_reach_target():
                self.goto(self.ARMBK)
            else:
                self.flow_to('rob_go_dist', [-self.back_dist, 0.5])
        elif self.cur_state == self.ARMBK:
            device.left_armbk()
            self.goto(self.FINISH)
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


class Opt_func_preput(Option):
    def __init__(self):
        labels = ['init', 'pre_arm', 'pre_arm2', 'pre_fetch', 'finish']
        Option.__init__(self, labels, 'finish')
        self.goal_angles = None
        self.pre_preput_angles = [-90, 0, 140, -50, 0]
        self.pre_preput_angles2 = [0, 0, 140, -50, 0]

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv:
                self.goal_angles = argv
                self.goto(self.PRE_ARM)
            else:
                self.goto(self.FINISH)

        elif self.cur_state == self.PRE_ARM:
            if self.subopt_reach_target():
                self.goto(self.PRE_ARM2)
            else:
                self.flow_to('rob_move_arm', self.pre_preput_angles)

        elif self.cur_state == self.PRE_ARM2:
            if self.subopt_reach_target():
                self.goto(self.PRE_FETCH)
            else:
                self.flow_to('rob_simple_move_arm', self.pre_preput_angles2)

        elif self.cur_state == self.PRE_FETCH:
            if self.subopt_reach_target():
                self.goto(self.FINISH)
            else:
                self.flow_to('rob_move_arm', self.goal_angles)
        elif self.cur_state == self.FINISH:
            pass
