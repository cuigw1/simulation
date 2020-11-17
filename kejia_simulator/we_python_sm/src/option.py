#!/usr/bin/env pythons
# -*- coding: utf-8 -*-

import sys
import logging
import world
import conf
from service import Service


class Option:
    """The Class Option for state machine management
    """
    ################################################################
    ################Static Variable??################
    root = ""
    depth = 0
    path = []
    path_last = []
    opt_flow_last = ""
    optmap = {}
    ################################################################

    def __init__(self, labels=None, target=None):
        # current state, always equals the current state's index
        self.pre_state = None
        self.cur_state = 0
        # state num, which is an index to the states
        self.s_num = 0
        # state and option start timestamp
        self.s_start = 0
        self.o_start = 0
        # state cycle, which is 0 from entering a state and increasing by 1 each cycle.
        self.s_count = 0
        # state time, which is 0 from entering a state and increasing by time unit each cycle.
        self.s_time = 0
        # option time, which is 0 from entering the option and increasing by time unit each cycle.
        self.o_time = 0
        # a flag indicate whether the state is active.
        self.active = False
        self.identifier = ''
        self.states = {}  # dict for state change
        self.target = {}
        self.result = None
        if labels != None:
            for label in labels:
                self.states[self.s_num] = label
                setattr(self, label.upper(), self.s_num)
                if label == target:
                    self.target[self.s_num] = 1
                self.s_num += 1

    def disactive(self):
        self.active = False

    def enactive(self):
        self.active = True

    def get_result(self):
        return self.result

    def subopt_result(self):
        if self.s_time == 0:
            return None
        else:
            try:
                opt = Option.optmap[Option.path_last[Option.depth + 1]]
            except IndexError:
                return None
            else:
                return opt.get_result()

    def current_state(self):
        try:
            return [self.states[self.cur_state], self.s_count]
        except KeyError:
            return []

    def subopt_current_state(self):
        if self.s_time == 0:
            return []
        else:
            try:
                opt = Option.optmap[Option.path_last[Option.depth + 1]]
            except IndexError:
                return []
            else:
                return opt.current_state()

    def reach_target(self):
        try:
            return self.target[self.cur_state]
        except KeyError:
            return 0

    def subopt_reach_target(self):
        if self.s_time == 0:
            return False
        else:
            try:
                opt = Option.optmap[Option.path_last[Option.depth + 1]]
            except IndexError:
                return False
            else:
                return opt.reach_target() == 1

    def flow_to(self, name, argv=None):
        Option.depth += 1
        try:
            opt = Option.optmap[name]
        except KeyError:
            print "option <{0}> does not exist".format(name)
            sys.exit()
        else:
            opt.run(argv)

    def goto(self, state):
        self.s_start = world.sync
        self.s_time = 0
        self.s_count = 0
        self.pre_state = self.cur_state
        self.cur_state = state

    def run(self, argv=None):
        Option.path.append(self.identifier)
        if self.active:
            self.o_time = world.sync - self.o_start
            self.s_time = world.sync - self.s_start
            self.s_count += 1
        else:
            self.cur_state = 0
            self.s_count = 1  # calling times of current state
            self.o_start = world.sync  # start time of option
            self.s_start = world.sync  # start time of state
            self.o_time = 0  # number of cycles in option(may be dis-continous)
            self.s_time = 0  # number of cycles in state(may be dis-continuous)
            self.active = True
        self.transfer(argv)

    def __str__(self):
        return "->{0}({1}):{2}({3})".format(self.identifier,
                                            self.o_time,
                                            self.states[self.cur_state],
                                            self.s_count)

    def transfer(self, argv=None):
        pass

    @staticmethod
    def register(opt, name):
        opt.identifier = name
        Option.optmap[name] = opt

    @staticmethod
    def set_root(name):
        Option.root = name

    @staticmethod
    def execute(argv=None):
        Option.depth = 0
        Option.path = []
        opt = Option.optmap[Option.root]
        opt.run(argv)
        Option.opt_flow_last = ""
        for i in range(0, len(Option.path_last)):
            name = Option.path_last[i]
            if conf.show_opt:
                Option.opt_flow_last += str(Option.optmap[name])
            if i >= len(Option.path) or name != Option.path[i]:
                Option.optmap[name].disactive()
        Option.path_last = [path for path in Option.path]
        if conf.show_opt:
            sys.stdout.write("\r"+Option.opt_flow_last.ljust(65)[-65:])
            sys.stdout.flush()
        if conf.do_log:
            conf.logger.info(Option.opt_flow_last)

     ####################################################################################
     ################################  for service   ####################################

    def init_srv(self, srv_name, srv_type, srv_request):
        Service.init_srv(srv_name, srv_type, srv_request)

    def is_srv_finished(self):
        return Service.is_srv_finished()

    def get_srv_response(self):
        return Service.get_srv_response()
