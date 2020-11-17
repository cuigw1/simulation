#!/usr/bin/env python
# -*- coding:utf-8 -*-
import conf
import math
import rospy
from robot import robot as device
import world
from option import Option


L0, L1, L2, L3 = device.arm_parts[0:4]


class Arm_control():

    def __init__(self):
        pass

    @staticmethod
    def set_arm_speed(target_sy, target_el, target_wy):
        pass

    @staticmethod
    def reset_arm_speed():
        pass

    @staticmethod
    def calc_angles(pose):
        """ inverse kinematics """
        PI = math.pi
        D2R = PI / 180
        R2D = 180 / PI
        [SZ, SY, EL, WY] = range(0, 4)
        armj_limit = [[-120 * D2R, 120 * D2R],
                      [-30 * D2R, 150 * D2R],
                      [-150 * D2R, 150 * D2R],
                      [-150 * D2R, 90 * D2R]]

        [x, y, z] = pose

        def within(jid, jval):
            return jid >= 0 and jid < 4 and \
                jval >= armj_limit[jid][0] and \
                jval <= armj_limit[jid][1]

        if x <= 0:
            return None

        shoulder_z = math.atan(y/x)
        lxy = math.sqrt(x*x + y*y)
        Lod2 = x*x + y*y + z*z + L3*L3 - 2*L3*lxy
        if math.sqrt(Lod2) > (L1+L2):
            return None  # triangle's attiibute

        tmp = (L1*L1 + L2*L2 - Lod2)/(2*L1*L2)
        if tmp > 1:
            tmp = 1
        elif tmp < -1:
            tmp = -1
        elbow_y = PI - math.acos(tmp)

        if Lod2 == 0.:
            return None
        tmp = (L1*L1 + Lod2 - L2*L2)/(2*L1*math.sqrt(Lod2))
        if tmp > 1:
            tmp = 1
        elif tmp < -1:
            tmp = -1
        d_cod = math.acos(tmp)
        d_dod = 0.0
        if lxy == L3:
            d_dod = PI / 2
        else:
            d_dod = math.atan(math.fabs(z) / math.fabs(lxy - L3))

        shoulder_y = 0.0
        wrist_y = 0.0

        #plan4  (shoulder_y < 0)
        if lxy < L3:
            elbow_y = math.fabs(elbow_y)
            shoulder_y = -PI/2 - d_cod + d_dod
            wrist_y = PI + d_cod - d_dod - elbow_y
            if (not within(SZ, shoulder_z)) or \
                (not within(SY, shoulder_y)) or \
                (not within(EL, elbow_y)) or \
                    (not within(WY, wrist_y)):
                return None
            else:
                   #print "plan4-1:"
                   #print (shoulder_z*R2D, shoulder_y*R2D, elbow_y*R2D, wrist_y*R2D)
                return [shoulder_z*R2D, shoulder_y*R2D, elbow_y*R2D, wrist_y*R2D]

        #plan1  (elbow_y>0)
        if z > 0:
            shoulder_y = PI/2 - d_cod + d_dod
            wrist_y = d_cod - d_dod - elbow_y
        else:
            shoulder_y = PI/2 - d_cod - d_dod
            wrist_y = d_cod + d_dod - elbow_y

        if (not within(SZ, shoulder_z)) or \
           (not within(SY, shoulder_y)) or \
           (not within(EL, elbow_y)) or \
           (not within(WY, wrist_y)):  # plan2 (elbow_y < 0)
            if z >= 0:
                return None
            elbow_y = -math.fabs(elbow_y)  # -
            shoulder_y = PI/2 + d_cod - d_dod
            wrist_y = -d_cod + d_dod - math.fabs(elbow_y)
        else:
            #print "plan1:"
            #print (shoulder_z*R2D, shoulder_y*R2D, elbow_y*R2D, wrist_y*R2D)
            if shoulder_y > 0:
                return [shoulder_z*R2D, shoulder_y*R2D, elbow_y*R2D, wrist_y*R2D]

        if (not within(SZ, shoulder_z)) or \
           (not within(SY, shoulder_y)) or \
           (not within(EL, elbow_y)) or \
           (not within(WY, wrist_y)):
            #plan3  (elbow_y > 0)
            elbow_y = math.fabs(elbow_y)  # +
            shoulder_y = PI/2 - d_cod - d_dod
            wrist_y = d_cod + d_dod - elbow_y
        else:
                #print "plan2:"
                #print (shoulder_z*R2D, shoulder_y*R2D, elbow_y*R2D, wrist_y*R2D)
            if shoulder_y > 0:
                return [shoulder_z*R2D, shoulder_y*R2D, elbow_y*R2D, wrist_y*R2D]

        if (not within(SZ, shoulder_z)) or \
           (not within(SY, shoulder_y)) or \
           (not within(EL, elbow_y)) or \
           (not within(WY, wrist_y)):
            #plan4  (elbow_y > 0)
            elbow_y = math.fabs(elbow_y)
            shoulder_y = -PI/2 - d_cod + d_dod
            wrist_y = PI + d_cod - d_dod - elbow_y
        else:
                #print "plan3:"
                #print (shoulder_z*R2D, shoulder_y*R2D, elbow_y*R2D, wrist_y*R2D)
            return [shoulder_z*R2D, shoulder_y*R2D, elbow_y*R2D, wrist_y*R2D]

        if (not within(SZ, shoulder_z)) or \
           (not within(SY, shoulder_y)) or \
           (not within(EL, elbow_y)) or \
           (not within(WY, wrist_y)):
            return None
        else:
                #print "plan4:"
                #print (shoulder_z*R2D, shoulder_y*R2D, elbow_y*R2D, wrist_y*R2D)
            return [shoulder_z*R2D, shoulder_y*R2D, elbow_y*R2D, wrist_y*R2D]

    @staticmethod
    def getReachDist(h):
        if h < -(L1+L2) or h > L2:
            return None

        min_dist = 0
        max_dist = L1 + L2 + L3
        min_dist_final = 0.
        max_dist_final = 0.
        for i in range(0, int(max_dist / 0.01)):
            if Arm_control.calc_angles([min_dist, 0, h]) == None:
                min_dist = min_dist + 0.01
            else:
                min_dist_final = min_dist
                break

        for i in range(0, int(max_dist / 0.01)):
            if Arm_control.calc_angles([max_dist, 0, h]) == None:
                max_dist = max_dist - 0.01
            else:
                max_dist_final = max_dist

        if min_dist_final != 0 and max_dist_final != 0:
            return [min_dist_final, max_dist_final]
        return None


class Opt_func_move_arm(Option):
    def __init__(self):
        labels = ['init', 'step', 'check', 'final_step', 'finish']
        Option.__init__(self, labels, 'finish')
        self.step_angles = [0, 0, 0, 0, 0]
        self.goal_angles = [0, 0, 0, 0, 0]
        self.step = 10

    def reach(self, a, b):
        for i, j in zip(a, b):
            if math.fabs(i - j) > 0.5:
                return False
        return True

    def part_reach(self, a, b):
        for i, j in zip(a[0:3], b[0:3]):
            if math.fabs(i - j) > 0.5:
                return False
        return True

    def step_goal(self):
        angels = world.arm[:]
        for i, v in enumerate(angels):
            diff = self.goal_angles[i] - v
            ratio = 1.0 if diff >= 0 else -1.0
            angels[i] += min(self.step, math.fabs(diff)) * ratio
        angels[3] = 90.0 - angels[1] - angels[2]
        return angels

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv == None or len(argv) < 5:
                rospy.logwarn('wrong parameters')
                self.goto(self.FINISH)
            else:
                self.goal_angles = argv[0:5]
                self.goto(self.CHECK)

        elif self.cur_state == self.STEP:
            if self.s_count % 10 == 1:
                self.step_angles = self.step_goal()
                device.move_arm(self.step_angles)
            if self.reach(self.step_angles, world.arm):
                self.goto(self.CHECK)

        elif self.cur_state == self.CHECK:
            if self.reach(self.goal_angles, world.arm):
                self.goto(self.FINISH)
            elif self.part_reach(self.goal_angles, world.arm):
                self.goto(self.FINAL_STEP)
            else:
                self.goto(self.STEP)

        elif self.cur_state == self.FINAL_STEP:
            if self.s_count % 10 == 1:
                device.move_arm(self.goal_angles)
            if self.reach(self.goal_angles, world.arm):
                self.goto(self.FINISH)

        elif self.cur_state == self.FINISH:
            pass


class Opt_func_simple_move_arm(Option):
    def __init__(self):
        labels = ['init', 'move', 'finish']
        Option.__init__(self, labels, 'finish')
        self.goal_angles = [0, 0, 0, 0, 0]

    def reach(self, a, b):
        for i, j in zip(a, b):
            if math.fabs(i - j) > 0.5:
                return False
        return True

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if argv == None or len(argv) < 5:
                rospy.logwarn('wrong parameters')
                self.goto(self.FINISH)
            else:
                self.goal_angles = argv[0:5]
                self.goto(self.MOVE)

        elif self.cur_state == self.MOVE:
            if self.s_count % 10 == 1:
                device.move_arm(self.goal_angles)
            if self.reach(self.goal_angles, world.arm):
                self.goto(self.FINISH)

        elif self.cur_state == self.FINISH:
            pass
