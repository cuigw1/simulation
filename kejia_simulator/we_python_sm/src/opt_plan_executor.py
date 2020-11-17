#!/usr/bin/env python
# -*- coding: utf-8 -*-

from Queue import Queue
from option import Option
from std_msgs.msg import String
import conf
import rospy
import re
from robot import robot as device
import json
import world


class Opt_executor(Option):
    def __init__(self):
        labels = ['init', 'wait_action', 'executing', 'result', 'finish']
        Option.__init__(self, labels, 'finish')
        self.actions = Queue()
        self.action_sub = rospy.Subscriber(conf.topic_executor_action, String, self.action_callback)
        self.feedback_pub = rospy.Publisher(conf.topic_executor_result, String, queue_size=10)
        self.action_mapping = {'putdown': 'action_putdown',
                               'pickup': 'action_pickup',
                               'receive': None,
                               'give': 'action_handover',
                               'findPerson': None,
                               'findObj': 'action_scan',
                               'moveIn': 'action_movein',
                               'moveTo': 'action_moveto',
                               'moveToHuman': 'action_movetohuman',
                               'open': None,
                               'close': None,
                               'accompany': None}

    def action_callback(self, msg):
        data = json.loads(msg.data)
        l = list(filter(None, re.split(r'[(,)]', data['action'])))
        rospy.loginfo('executor action: %s' % msg.data)
        if l[0] in self.action_mapping:
            if l[0] == 'moveTo' and l[1] in conf.humans:
                l[0] = 'moveToHuman'
            action = {'action': self.action_mapping[l[0]], 'params': l[1:]}
            if 'obj' in data:
                action['params'].append(data['obj'])
            self.actions.put(action)
        else:
            rospy.logwarn('unknown action %s' % msg.data)

    def action_feedback(self, result):
        msg = String()
        msg.data = json.dumps(result)
        self.feedback_pub.publish(msg)

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if self.s_count == 20:
                device.left_armbk()
                device.move_ev(device.normal_ev)
                self.goto(self.WAIT_ACTION)

        elif self.cur_state == self.WAIT_ACTION:
            if not self.actions.empty():
                self.action = self.actions.get()
                if self.action['action']:
                    self.goto(self.EXECUTING)
                else:
                    rospy.logerr('action %s has not been implemented yet' % self.action['action'])

        elif self.cur_state == self.EXECUTING:
            if self.s_count == 1:
                world.action_execution_result = {'status': 'success', 'detail': None}
            if self.subopt_reach_target():
                device.neck_pan(0)
                device.neck_tilt(0)
                self.goto(self.RESULT)
            else:
                self.flow_to(self.action['action'], self.action['params'])

        elif self.cur_state == self.RESULT:
            if world.action_execution_result['status'] == 'success':
                rospy.loginfo('action execution result: %s', world.action_execution_result['status'])
                self.action_feedback({'type': 'executor', 'status': 'success'})
            else:
                rospy.logwarn('action execution result: %s', world.action_execution_result['status'])
                self.action_feedback({'type': 'executor', 'status': 'fail',
                                      'detail': world.action_execution_result['detail']})
            self.goto(self.WAIT_ACTION)

        elif self.cur_state == self.FINISH:
            pass
