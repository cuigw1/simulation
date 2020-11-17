#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import rospy


class Object:
    def __init__(self, name=None, type=None, pose=None, stamp=0):
        self.name = name
        self.type = type
        self.pose = pose
        self.stamp = stamp


shared_resource_lock = threading.Lock()

sync = -1

# joints
ev = 0.0
arm = [0, 0, 0, 0, 0]
paw = [0, 0]
pan = 0
tilt = 0

# robot pose
rob_is_stop = True
odom_pose = [0.0, 0.0, 0.0]
move_speed = [0.0, 0.0, 0.0]
odom_pose_time = 0.0
rob_pose = [0, 0, 0]

# objects
objects = {}

# slam
navigate_result = None

# topography
topography = None

# action
action_execution_result = {}
