#!/usr/bin/env python
# -*- coding: utf-8 -*-

import conf
import rospy
import world
import math
import tf
from robot import robot as device
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
from PyKDL import Rotation
from topo import Topography


def odom_callback(msg):
    odom_pose_x = msg.pose.pose.position.x
    odom_pose_y = msg.pose.pose.position.y
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    r = Rotation.Quaternion(qx, qy, qz, qw)
    (roll, pitch, odom_pose_r) = r.GetRPY()
    world.shared_resource_lock.acquire()
    world.odom_pose = [odom_pose_x, odom_pose_y, odom_pose_r]
    world.move_speed = [msg.twist.twist.linear.x,
                        msg.twist.twist.linear.y, msg.twist.twist.angular.z]
    if world.move_speed[0] < 0.01 and world.move_speed[1] < 0.01 and world.move_speed[2] < 0.01:
        world.rob_is_stop = True
    else:
        world.rob_is_stop = False
    world.odom_pose_time = msg.header.stamp.to_sec()
    world.shared_resource_lock.release()


def slam_result_callback(msg):
    world.navigate_result = msg


def register_subscriber():
    # robot hw info
    device.register_subscriber()
    # odom
    rospy.Subscriber(conf.topic_odom, Odometry, odom_callback, None, None, 65536, True)
    # slam result
    rospy.Subscriber(conf.topic_slam_result, MoveBaseActionResult, slam_result_callback, None, None, 10, True)
    # tf listener
    device.tf_listener = tf.TransformListener()
    # topography
    world.topography = Topography()
