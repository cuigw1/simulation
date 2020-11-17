#!/usr/bin/python
# -*- coding: utf-8 -*-
import conf
import os
import re
import math
import numpy as np
from robot import robot as device

# topography
# topo_file = os.path.join(os.environ['HOME'], 'catkin_ws/src/kejia/kejia_simulator/kejia_mapping/map/gpsr.txt')


class Topography:
    def __init__(self, argv=None):
        dirname, _ = os.path.split(os.path.abspath(__file__))
        self.topo_file = os.path.realpath(dirname + '/../../kejia_mapping/map/gpsr.txt')
        self.rooms = {}
        self.furnitures = {}
        self.doors = {}
        self.parse()

    def dis(self, p, rects, ds):
        for j, r in enumerate(rects):
            # y
            if p[0] > r[0] and p[0] < r[2]:
                if p[1] > r[3]:
                    ds[3] = min(abs(p[1]-r[3]), ds[3])
                elif p[1] < r[1]:
                    ds[1] = min(abs(r[1]-p[1]), ds[1])
                elif j == 0 and self.in_rect(p, r):
                    ds[1] = min(abs(r[3]-p[1]), ds[1])
                    ds[3] = min(abs(p[1]-r[1]), ds[3])
                else:
                    ds[1] = 0
                    ds[3] = 0
            # x
            if p[1] > r[1] and p[1] < r[3]:
                if p[0] > r[2]:
                    ds[2] = min(abs(p[0]-r[2]), ds[2])
                elif p[0] < r[0]:
                    ds[0] = min(abs(r[0]-p[0]), ds[0])
                elif j == 0 and self.in_rect(p, r):
                    ds[0] = min(abs(r[2]-p[0]), ds[0])
                    ds[2] = min(abs(p[0]-r[0]), ds[2])
                else:
                    ds[0] = 0
                    ds[2] = 0
            # print('-- %.2f %.2f, %.2f %.2f %.2f %.2f, %.2f %.2f %.2f %.2f' %
            #       (p[0], p[1], r[0], r[1], r[2], r[3], ds[0], ds[1], ds[2], ds[3]))

    def distances(self, pp, rects, d, md, model):
        for i in np.linspace(d, min(md), 10):
            p = pp[:]
            p[0] = pp[0] + i * pp[3]
            p[1] = pp[1] + i * pp[4]
            if self.in_rect(p, rects[0]):
                ds = [1000.0, 1000.0, 1000.0, 1000.0]
                self.dis([p[0]+model[0], p[1] + model[1]], rects, ds)
                self.dis([p[0]+model[0], p[1] + model[3]], rects, ds)
                self.dis([p[0]+model[2], p[1] + model[1]], rects, ds)
                self.dis([p[0]+model[2], p[1] + model[3]], rects, ds)
                if self.accept(ds, md):
                    print('ds:', ds)
                    return [ds, p]
        return None

    def accept(self, d, m):
        for i, j in zip(d, m):
            if i < j:
                return False
        return True

    def get_candinate_of_furniture(self, p, room, furniture, d=0.3):
        candinates, fi = self.get_candinates_of_furniture(room, furniture)
        rect = fi['rect']
        dists = []
        for c in candinates:
            if c[1][3] == -1:
                dists.append(p[0] - rect[0])
            elif c[1][3] == 1:
                dists.append(rect[2] - p[0])
            elif c[1][4] == -1:
                dists.append(p[1] - rect[1])
            elif c[1][4] == 1:
                dists.append(rect[3] - p[1])
        return [candinates, fi, dists]

    def get_candinates_of_rect(self, fr, rects, d=0.7, md=[0.1, 0.1, 0.1, 0.1], d2s=0.4):
        candinates = []
        cf = [(fr[0]+fr[2])*0.5, (fr[1]+fr[3])*0.5]
        lx = fr[2] - fr[0]
        ly = fr[3] - fr[1]
        print('cf', cf, lx, ly)
        p = [cf[0]-lx/2.0, cf[1], 0, -1, 0, ly]
        ds = self.distances(p, rects, d, md, device.base_model)
        if ds:
            ds.append([fr[0], fr[1], min(fr[2], fr[0]+d2s), fr[3]])
            candinates.append(ds)
        p = [cf[0]+lx/2.0, cf[1], math.pi, +1, 0, ly]
        ds = self.distances(p, rects, d, [md[2], md[3], md[0], md[1]], [
                            -device.base_model[0], -device.base_model[1], -device.base_model[2], -device.base_model[3]])
        if ds:
            ds.append([max(fr[0], fr[2]-d2s), fr[1], fr[2], fr[3]])
            candinates.append(ds)
        p = [cf[0], cf[1]-ly/2.0, math.pi/2, 0, -1, lx]
        ds = self.distances(p, rects, d, [md[1], md[2], md[3], md[0]], [
                            -device.base_model[1], device.base_model[0], -device.base_model[3], device.base_model[2]])
        if ds:
            ds.append([fr[0], fr[1], fr[2], min(fr[3], fr[1]+d2s)])
            candinates.append(ds)
        p = [cf[0], cf[1]+ly/2.0, -math.pi/2, 0, +1, lx]
        ds = self.distances(p, rects, d, [md[3], md[0], md[1], md[2]], [
                            device.base_model[1], -device.base_model[0], device.base_model[3], -device.base_model[2]])
        if ds:
            ds.append([fr[0], max(fr[1], fr[3]-d2s), fr[2], fr[3]])
            candinates.append(ds)
        candinates = sorted(candinates, key=lambda y: y[1][-1], reverse=True)
        for c in candinates:
            print('dist[%.3f %.3f %.3f %.3f] pose[%.3f %.3f %.3f] length[%.3f] %s[%.3f %.3f %.3f %.3f]' % (
                c[0][0], c[0][1], c[0][2], c[0][3], c[1][0], c[1][1], c[1][2], c[1][-1], 'furniture', fr[0], fr[1], fr[2], fr[3]))
        return candinates

    def get_candinates_of_furniture(self, room, furniture, d=0.7, md=[0.3, 0.3, 0.3, 0.3]):
        print(room, furniture)
        if room not in self.rooms:
            print('unknown room %s' % room)
            return None
        if furniture not in self.furnitures:
            print('unknown furniture %s' % furniture)
            return None
        if self.furnitures[furniture]['room'] != room:
            print('furniture(%s) not in room(%s)' % (furniture, room))
            return None
        rects = [self.rooms[room]['rect']]
        print('room rect', rects)
        fi = self.furnitures[furniture]
        for k, v in self.furnitures.items():
            if v['room'] == room:
                rects.append(v['rect'])
        print('get_candinates_of_furniture', rects)
        candinates = self.get_candinates_of_rect(fi['rect'], rects, d, md)
        fi['name'] = furniture
        return [candinates, fi]

    def get_candinate_of_door(self, rooma, roomb, door, d=0.2):
        poses = []
        dr = self.doors[door]['rect']
        r1 = self.rooms[rooma]['rect']
        r2 = self.rooms[roomb]['rect']
        if dr and r1 and r2:
            cd = [(dr[0]+dr[2])*0.5, (dr[1]+dr[3])*0.5]
            lx = math.fabs(dr[2] - dr[0])
            ly = math.fabs(dr[3] - dr[1])
            if ly > lx:
                if r1[0] > r2[0]:
                    poses.append([cd[0]+d, cd[1], -math.pi])
                    poses.append([cd[0]-d, cd[1], -math.pi])
                else:
                    poses.append([cd[0]-d, cd[1], 0])
                    poses.append([cd[0]+d, cd[1], 0])
            else:
                if r1[1] > r2[1]:
                    poses.append([cd[0], cd[1]+d, -math.pi/2])
                    poses.append([cd[0], cd[1]-d, -math.pi/2])
                else:
                    poses.append([cd[0], cd[1]-d, math.pi/2])
                    poses.append([cd[0], cd[1]+d, math.pi/2])
        return poses

    def in_rect(self, p, room):
        if p[0] > room[0] and p[0] < room[2] and p[1] > room[1] and p[1] < room[3]:
            return True
        return False

    def get_room(self, p):
        for k, v in self.rooms.items():
            print(k, v)
            if self.in_rect(p, v['rect']):
                return {'name': k, 'rect': v['rect']}
        return None

    def get_room_by_name(self, name):
        if name in self.rooms:
            return self.rooms[name]
        return None

    def get_furniture(self, furniture, room):
        if room not in self.rooms:
            print('unknown room %s' % room)
            return None
        if furniture not in self.furnitures:
            print('unknown furniture %s' % furniture)
            return None
        if self.furnitures[furniture]['room'] != room:
            print('furniture(%s) not in room(%s)' % (furniture, room))
            return None
        return self.furnitures[furniture]['rect']

    def door_of_room(self, door, room):
        p = door[0:2]
        if p[0] > room[0] and p[0] < room[2] and p[1] > room[1] and p[1] < room[3]:
            return p
        p = door[2:4]
        if p[0] > room[0] and p[0] < room[2] and p[1] > room[1] and p[1] < room[3]:
            return p
        return False, None

    def parse(self):
        with open(self.topo_file, 'r') as f:
            for line in f.readlines():
                ll = line.split()
                head, x1, y1, x2, y2 = ll[0:5]
                rect = [float(x1), float(y1), float(x2), float(y2)]
                l = re.split(r'[.:,]', head)
                if l[0] == 'room':
                    self.rooms[l[1]] = {'rect': rect}
                elif l[0] == 'furniture':
                    self.furnitures[l[1]] = {'rect': rect}
                    if len(ll) > 5:
                        self.furnitures[l[1]]['height'] = float(ll[5])
                elif l[0] == 'door':
                    self.doors[l[1]] = {'rect': rect, 'rooms': l[2:4]}
                else:
                    print('unknown type: %s' % l[0])
        for k, v in self.furnitures.items():
            x1, y1, x2, y2 = v['rect']
            cx = (x1 + x2) * 0.5
            cy = (y1 + y2) * 0.5
            for r, rv in self.rooms.items():
                if self.in_rect([cx, cy], rv['rect']):
                    self.furnitures[k]['room'] = r


# topo = Topography()
# print(topo.rooms)
