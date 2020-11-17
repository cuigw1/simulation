#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" utils used in programming"""
import math
import rospy
import tf
import conf
import world
import numpy as np
from tf import transformations
from geometry_msgs.msg import PointStamped
from robot import robot as device


# dist of point p3 to line (p1, p2)
def dist_point_line(p1, p2, p3):
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    return np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)


def in_rect(p, r):
    if p[0] > r[0] and p[0] < r[2] and p[1] > r[1] and p[1] < r[3]:
        return True
    return False


def in_polygon(point, points):
    from shapely.geometry import Polygon, Point
    poly = Polygon(points)
    return poly.contains(Point(point))


def to_rad(d):
    return d / 180.0 * math.pi


def to_degree(r):
    return r * 180.0 / math.pi


def normalize(r):
    if r > -math.pi and r < math.pi:
        return r
    while abs(r) > math.pi:
        if r > 0:
            r -= math.pi * 2
        elif r < 0:
            r += math.pi * 2
    return r


def diff_rad(a, b):
    na = normalize(a)
    nb = normalize(b)
    c = nb - na
    if c > math.pi:
        c -= math.pi * 2
    elif c < -math.pi:
        c += math.pi * 2
    return c


def pose_in_self(s, p):
    mc = math.cos(-s[2])
    ms = math.sin(-s[2])
    x = p[0] - s[0]
    y = p[1] - s[1]
    r = normalize(math.atan2(y, x) - s[2])
    return [x * mc - y * ms, x * ms + y * mc, r]


def geom_dist(dx, dy):
    return math.sqrt(dx * dx + dy * dy)


def geom_dist2(v1, v2):
    dx = v1[0] - v2[0]
    dy = v1[1] - v2[1]
    return math.sqrt(dx * dx + dy * dy)


def geom_dist3(v1, v2):
    dx = v1[0] - v2[0]
    dy = v1[1] - v2[1]
    dz = v1[2] - v2[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def geom_theta(v1, v2):
    dx = v1[0] - v2[0]
    dy = v1[1] - v2[1]
    return math.atan2(dy, dx)


def sign(a):
    if a > 0:
        return 1
    elif a < 0:
        return -1
    else:
        return 0


# obj: object pose in self [x,y,z]
# robot: robot pose [x,y,r]
# return: pose in world [x,y,z]
def pose_to_self2(obj, robot):
    diff_x = obj[0] - robot[0]
    diff_y = obj[1] - robot[1]
    dist = geom_dist(diff_x, diff_y)
    angle = normalize(math.atan2(diff_y, diff_x))
    diff_angle = normalize(angle - robot[2])
    x = math.cos(diff_angle) * dist
    y = math.sin(diff_angle) * dist
    pose = [x, y]
    return pose


# obj: object pose in world [x,y,z]
# robot: robot pose [x,y,r]
# return: pose in self [x,y,z]
def pose_to_self(obj, robot):
    diff_x = obj[0] - robot[0]
    diff_y = obj[1] - robot[1]
    dist = geom_dist(diff_x, diff_y)
    angle = normalize(math.atan2(diff_y, diff_x))
    diff_angle = normalize(angle - robot[2])
    x = math.cos(diff_angle) * dist
    y = math.sin(diff_angle) * dist
    pose = [x, y, obj[2]]
    return pose


def pose_to_world2(obj, robot):
    dist = geom_dist(obj[0], obj[1])
    angle = normalize(math.atan2(obj[1], obj[0]))
    diff_angle = normalize(angle + robot[2])
    x = robot[0] + dist * math.cos(diff_angle)
    y = robot[1] + dist * math.sin(diff_angle)
    pose = [x, y]
    return pose


def world_to_pose(pose):
    now = rospy.Time.now()
    pt = PointStamped()
    pt.header.seq = rospy.Time.now()
    pt.header.frame_id = '/map'
    pt.point.x = pose[0]
    pt.point.y = pose[1]
    pt.point.z = pose[2]
    try:
        device.tf_listener.waitForTransform('/base_link', '/map', now, rospy.Duration(conf.fm_tf_timeout))
        transed = device.tf_listener.transformPoint('/base_link', pt)
    except tf.Exception as expt:
        rospy.logwarn("Exception of tf(@update2baselink): {0}".format(expt))
    return [transed.point.x, transed.point.y, transed.point.z]


def pose_to_world(obj, robot):
    dist = geom_dist(obj[0], obj[1])
    angle = normalize(math.atan2(obj[1], obj[0]))
    diff_angle = normalize(angle + robot[2])
    x = robot[0] + dist * math.cos(diff_angle)
    y = robot[1] + dist * math.sin(diff_angle)
    pose = [x, y, obj[2]]
    return pose


def base_to_arm(pose):
    ret = pose[:]
    for i, v in enumerate(device.base2arm):
        ret[i] -= v
    ret[2] += (device.init_ev - world.ev)
    return ret
