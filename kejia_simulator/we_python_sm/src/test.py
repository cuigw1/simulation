#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import conf
import json
from std_msgs.msg import String
import time


class Test():
    def __init__(self):
        self.actions = [
            {'action': 'moveTo(jamie,study)'},
            # {'action': 'moveIn(kitchen,living_room,living_room_kitchen)'},
            # {'action': 'moveTo(kitchen_table,kitchen)'},
            # {'action': 'moveTo(cupboard,kitchen)'},
            # {'action': 'pickup(coke4,cupboard,gripper', 'obj': {'coke4': 'coke_can'}},
            # {'action': 'moveIn(living_room,kitchen,living_room_kitchen)'},
            # {'action': 'moveTo(living_table,living_room)'},
            # {'action': 'putdown(coke4,living_table,gripper)', 'obj': {'coke4': 'coke_can'}},
            # {'action': 'pickup(coke4,living_table,gripper', 'obj': {'coke4': 'coke_can'}},
            # {'action': 'moveIn(kitchen,living_room,living_room_kitchen)'},
            # {'action': 'moveTo(cupboard,kitchen)'},
            # {'action': 'putdown(coke4,cupboard,gripper)', 'obj': {'coke4': 'coke_can'}},
            # {'action': 'moveIn(living_room,kitchen,living_room_kitchen)'},
            # {'action': 'moveTo(living_table,living_room)'}
        ]
        self.feedback_sub = rospy.Subscriber(conf.topic_executor_result, String, self.callback)
        self.action_pub = rospy.Publisher('/executor/action', String, queue_size=10)
        self.index = 0
        self.allowed_send = True

    def callback(self, msg):
        self.allowed_send = True

    def run(self):
        time.sleep(1.0)
        while True:
            if self.allowed_send:
                msg = String()
                msg.data = json.dumps(self.actions[self.index])
                print(self.actions[self.index])
                self.action_pub.publish(msg)
                self.allowed_send = False
                self.index += 1
                if self.index == len(self.actions):
                    self.index = 0
            time.sleep(1.0)


if __name__ == '__main__':
    try:
        rospy.init_node('test000', anonymous=True)
        test = Test()
        test.run()
        rospy.spin()
    except Exception as e:
        print(e)
