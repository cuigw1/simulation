#!/usr/bin/env python
# coding=utf-8
import os
import re
import yaml
import rospy
import json
from solver import Solver
from plan import Plan
from Queue import Queue
from common_msgs.msg import CmdMsg
from std_msgs.msg import String


class Monitor:
    def __init__(self):
        # command
        self.cmds = Queue()
        rospy.Subscriber('/task_planner/cmd', String, self.cmd_callback)
        # plan
        self.plan = None
        # action
        self.action_pub = rospy.Publisher('/executor/action', String, queue_size=50)

    def cmd_callback(self, msg):
        self.cmds.put(msg)

    def updateAfterAction(self, plan):
        if plan.curr_step < plan.nsteps:
            rospy.loginfo('executed step-%d %s' % (plan.curr_step, plan.steps[plan.curr_step]))
            plan.curr_step += 1

    def deal_cmd(self, msg):
        cmd = json.loads(msg.data)
        if cmd['type'] == 'executor':
            if cmd['status'] != 'fail':
                self.updateAfterAction(self.plan)
                if self.plan.finished():
                    rospy.loginfo('task finished')
                    self.plan = None
                else:
                    self.dispatch(self.plan)

    def dispatch(self, plan):
        if not plan.finished():
            rospy.loginfo('dispatch step-%d %s', plan.curr_step, plan.steps[plan.curr_step])
            msg = String()
            action = {}
            action['action'] = plan.steps[plan.curr_step]
            action['obj'] = plan.objinfo
            msg.data = json.dumps(action)
            self.action_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(10)  # 10hz
        if self.plan != None:
            self.dispatch(self.plan)
        while self.plan != None:
            if not self.cmds.empty():
                cmd = self.cmds.get()
                self.deal_cmd(cmd)
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('task_planner', anonymous=True)
        dirname, _ = os.path.split(os.path.abspath(__file__))
        solver = Solver({'solver': '../res/solver/clingo', 'time_limit': 30, 'quiet': True, 'max_steps': 20})
        solver.solver = os.path.realpath(dirname + '/' + solver.solver)
        inst = ['../res/domain/robot/kejia.lp', '../res/domain/inst/init.lp', '../res/domain/inst/goal.lp']
        inst = [os.path.realpath(dirname + '/' + f) for f in inst]
        example = Monitor()
        example.plan = Plan(solver.execute(inst))
        example.run()
    except rospy.ROSInterruptException:
        pass
