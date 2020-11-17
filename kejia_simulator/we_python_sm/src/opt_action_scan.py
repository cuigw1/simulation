#!/usr/bin/env python
# -*- coding: utf-8 -*-

from option import Option
from std_msgs.msg import String
import json
import world
import rospy
import util
import conf
from robot import robot as device


class Opt_action_scan(Option):
    result = []

    def __init__(self):
        labels = ['init', 'scan', 'findobj', 'publish', 'finish']
        Option.__init__(self, labels, 'finish')
        self.room = None
        self.furniture = None
        self.object = None
        self.objects = {}
        self.object_pub = rospy.Publisher(conf.topic_vision_pub, String, queue_size=10)

    def findobj(self, name, rect):
        world.shared_resource_lock.acquire()
        objs = world.objects
        world.shared_resource_lock.release()
        res = []
        for k, v in objs.items():
            if world.topography.in_rect(v['pose'][0:3], rect):
                obj = {'name': k, 'type': v['type'], 'pose': v['pose'], 'size': v['size']}
                # types = self.object['type'].copy()
                # if not k in types:
                #     for n, t in types.items():
                #         if t == obj['type']:
                #             obj['name'] = n
                #             types.pop(n)
                res.append(obj)
        rospy.logwarn(res)
        return res

    def transfer(self, argv=None):
        if self.cur_state == self.INIT:
            if self.s_count == 10:
                if argv:
                    self.object = {'name': argv[0], 'type': argv[-1]}
                    self.room = world.topography.get_room(world.rob_pose)
                    self.furniture = {'name': argv[1],
                                      'rect': world.topography.get_furniture(argv[1], self.room['name'])}
                    self.goto(self.SCAN)
                else:
                    self.goto(self.FINISH)

        elif self.cur_state == self.SCAN:
            if self.s_count == 1:
                device.neck_tilt(-30)
                device.neck_pan(0)
            elif self.s_count == 15:
                self.goto(self.FINDOBJ)

        elif self.cur_state == self.FINDOBJ:
            self.objects['room'] = self.room['name']
            self.objects['location'] = self.furniture['name']
            self.objects['object'] = self.findobj(self.object['name'], self.furniture['rect'])
            if self.objects['object']:
                found = False
                for obj in self.objects['object']:
                    if obj['name'] == self.object['name']:
                        Opt_action_scan.result.append(obj)
                        found = True
                    rospy.loginfo('found %s' % obj['name'])
                if not found:
                    world.action_execution_result['status'] = 'fail'
            else:
                rospy.logwarn('found nothing')
                world.action_execution_result['status'] = 'fail'
            self.goto(self.PUBLISH)

        elif self.cur_state == self.PUBLISH:
            if self.s_count == 1:
                msg = String()
                msg.data = json.dumps({'type': 'scanlocation', 'result': self.objects})
                self.object_pub.publish(msg)
            if self.s_count == 10:
                self.goto(self.FINISH)

        elif self.cur_state == self.FINISH:
            pass
