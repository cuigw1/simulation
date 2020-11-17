#!/usr/bin/python
# -*- coding: utf-8 -*-
import os

# log configure
do_log = True
logger = None
show_opt = True


# text to action
topic_executor_action = '/executor/action'
topic_executor_result = '/task_planner/cmd'
topic_vision_pub = '/task_planner/cmd'

# model type
model_types = ['bed', 'closet', 'desk', 'tv', 'sofa', 'teapoy', 'trash', 'table', 'plant',
               'cupboard2', 'worktable', 'microwave', 'refrigerator', 'washmachine', 'bookshelf', 'book',
               'beer', 'coke', 'bottle', 'sticky_notes', 'cup', 'bowl', 'snacks', 'soda_can', 'kejia', 'biscuits', 'human']
model_size = {'beer': [0.07, 0.07, 0.15], 'bottle': [0.07, 0.07, 0.16],
              'cup': [0.08, 0.08, 0.10], 'bowl': [0.12, 0.12, 0.04], 'coke': [0.08, 0.08, 0.12]}
models = []
humans = ['jamie']

# odom
topic_odom = '/odom'

# slam
topic_slam_goal = 'move_base/goal'
topic_slam_result = 'move_base/result'

# frame transform
fm_tf_timeout = 0.2
