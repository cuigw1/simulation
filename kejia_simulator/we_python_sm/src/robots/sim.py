#!/usr/bin/env pythons
# -*- coding: utf-8 -*-

import math
import rospy
import world
import conf
import tf
import numpy as np
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates


# name
robot = 'kejia'

# base model
base_model = [0.25, 0.25, -0.25, -0.25]

# elevator
controller_ev = 'ev_position_controller'
pub_ev_cmd = rospy.Publisher('/' + robot + '/' + controller_ev + '/command',
                             Float64, queue_size=50)
ev_range = [-0.7, 0.0]
init_ev = 0.0
normal_ev = -0.15

# pan and tilt
controller_tilt = 'tilt_position_controller'
controller_pan = 'pan_position_controller'
pub_pan_cmd = rospy.Publisher('/' + robot + '/' + controller_pan + '/command',
                              Float64, queue_size=50)
pub_tilt_cmd = rospy.Publisher('/' + robot + '/' + controller_tilt + '/command',
                               Float64, queue_size=50)

# arm
controller_arm = ['sz_position_controller',
                  'sy1_position_controller',
                  'sy2_position_controller',
                  'sy3_position_controller',
                  'wr_position_controller']
arm_parts = [0.145, 0.21, 0.21, 0.255]  # to finger
pub_arm0_cmd = rospy.Publisher('/' + robot + '/' + controller_arm[0] + '/command',
                               Float64, queue_size=50)
pub_arm1_cmd = rospy.Publisher('/' + robot + '/' + controller_arm[1] + '/command',
                               Float64, queue_size=50)
pub_arm2_cmd = rospy.Publisher('/' + robot + '/' + controller_arm[2] + '/command',
                               Float64, queue_size=50)
pub_arm3_cmd = rospy.Publisher('/' + robot + '/' + controller_arm[3] + '/command',
                               Float64, queue_size=50)
pub_arm4_cmd = rospy.Publisher('/' + robot + '/' + controller_arm[4] + '/command',
                               Float64, queue_size=50)
pre_arm_angles = [0, 30, 90, -30, 0]
base2arm = [0.215, 0.000, 1.325-0.145]
# gripper
controller_fingers = ['lf_position_controller',
                      'rf_position_controller']
pub_lf_cmd = rospy.Publisher('/' + robot + '/' + controller_fingers[0] + '/command',
                             Float64, queue_size=50)
pub_rf_cmd = rospy.Publisher('/' + robot + '/' + controller_fingers[1] + '/command',
                             Float64, queue_size=50)
finger_len = 0.1
max_paw_value = 0.085 * 2
init_paw_value = 0.01
grasp_time = 10
release_time = 5

# joint state topic
topic_joint_states = '/' + robot + '/joint_states'

# base move
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, tcp_nodelay=True, queue_size=100)

# slam move
slam_move_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

# text to speech
pub_tts = rospy.Publisher('/' + robot + '/tts', String, queue_size=50)

# tf listener
tf_listener = None


# tts
def say(text):
    msg = String()
    msg.data = text
    pub_tts.publish(msg)


# move ev
def move_ev(height):
    if height > ev_range[1]:
        height = ev_range[1] - 0.001
    elif height < ev_range[0]:
        height = ev_range[0] + 0.001
    msg = Float64()
    msg.data = height
    pub_ev_cmd.publish(msg)


# move neck
def neck_tilt(val):
    msg = Float64()
    msg.data = val / 180.0 * math.pi
    pub_tilt_cmd.publish(msg)


def neck_pan(val):
    msg = Float64()
    msg.data = val / 180.0 * math.pi
    pub_pan_cmd.publish(msg)


# move arm
def left_armbk():
    move_arm([90, 0, 150, 30, 0])


def right_armbk():
    move_arm([-90, 0, 150, 30, 0])


def move_arm(angles):
    rospy.loginfo('move arm [%.3f %.3f %.3f %.3f %.3f]'
                  % (angles[0], angles[1], angles[2], angles[3], angles[4]))
    move_lr(angles[0])
    move_p1(angles[1])
    move_p2(angles[2])
    move_p3(angles[3])
    move_wz(angles[4])


def move_lr(angle):
    D2R = math.pi / 180
    msg = Float64()
    msg.data = angle * D2R
    pub_arm0_cmd.publish(msg)


def move_p1(angle):
    D2R = math.pi / 180
    msg = Float64()
    msg.data = angle * D2R
    pub_arm1_cmd.publish(msg)


def move_p2(angle):
    D2R = math.pi / 180
    msg = Float64()
    msg.data = angle * D2R
    pub_arm2_cmd.publish(msg)


def move_p3(angle):
    D2R = math.pi / 180
    msg = Float64()
    msg.data = angle * D2R
    pub_arm3_cmd.publish(msg)


def move_wz(angle):
    D2R = math.pi / 180
    msg = Float64()
    msg.data = angle * D2R
    pub_arm4_cmd.publish(msg)


# slam
def slam_move(pose):
    goal = MoveBaseActionGoal()
    goal.goal.target_pose.header.frame_id = 'map'
    goal.goal.target_pose.pose.position.x = pose[0]
    goal.goal.target_pose.pose.position.y = pose[1]
    goal.goal.target_pose.pose.position.z = 0.0
    goal.goal.target_pose.pose.orientation.x = 0
    goal.goal.target_pose.pose.orientation.y = 0
    goal.goal.target_pose.pose.orientation.z = math.sin(pose[2]/2)
    goal.goal.target_pose.pose.orientation.w = math.cos(pose[2]/2)
    slam_move_pub.publish(goal)


# gripper
def close_paw(w=0):
    msg = Float64()
    msg.data = w / 2.0
    pub_lf_cmd.publish(msg)
    pub_rf_cmd.publish(msg)


def open_paw(w=max_paw_value):
    msg = Float64()
    msg.data = w / 2.0
    pub_lf_cmd.publish(msg)
    pub_rf_cmd.publish(msg)


# base move
def rob_go(x, y, r):
    cmd = Twist()
    cmd.linear.x = x
    cmd.linear.y = y
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = r
    cmd_vel_pub.publish(cmd)


def rob_stop():
    rob_go(0, 0, 0)


def rob_turn(r):
    rob_go(0, 0, r)


# callback - joints
def gazebo_joint_states(msg):
    # names = msg.name
    R2D = 180 / math.pi
    poses = msg.position
    world.shared_resource_lock.acquire()
    world.arm = np.asarray(poses[0:5]) * R2D
    world.paw = poses[5:7]
    world.ev = poses[7]
    world.pan = poses[8] * R2D
    world.tilt = poses[9] * R2D
    world.shared_resource_lock.release()


# model state topic - objects
topic_model_states = '/gazebo/model_states'


def get_model_type(name, types):
    for t in types:
        if t in name:
            return t
    return None


# callback - objects
def gazebo_model_states(msg):
    models = {}
    rob_pose = None
    for name, pose in zip(msg.name, msg.pose):
        p = pose.position
        q = pose.orientation
        mt = get_model_type(name, conf.model_types)
        size = [0.1, 0.1, 0.15]
        if mt in conf.model_size:
            size = conf.model_size[mt]
        models[name] = {'type': mt, 'pose': [p.x, p.y, p.z, q.x, q.y, q.z, q.w], 'size': size}
    if robot in models:
        p = models[robot]['pose']
        r = tf.transformations.euler_from_quaternion(p[3:7])
        rob_pose = [p[0], p[1], r[2]]
    world.shared_resource_lock.acquire()
    if models:
        world.objects = models
    if rob_pose:
        world.rob_pose = rob_pose
    world.shared_resource_lock.release()


def register_subscriber():
    # joint states
    rospy.Subscriber(topic_joint_states, JointState, gazebo_joint_states)
    # model states
    rospy.Subscriber(topic_model_states, ModelStates, gazebo_model_states)
