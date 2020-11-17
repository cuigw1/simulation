#!/usr/bin/env python
# -*- coding: utf-8 -*-
import conf
from option import Option

# options
from arm_ctl import Opt_func_move_arm as rob_move_arm, \
    Opt_func_simple_move_arm as rob_simple_move_arm
from opt_func_fetch import Opt_func_fetch as fetch_obj, \
    Opt_func_liftup as arm_liftup, \
    Opt_func_prefetch as rob_pre_fetch, \
    Opt_func_hold as rob_hold
from opt_func_put import Opt_func_put as put_obj, \
    Opt_func_preput as rob_pre_put
from opt_func_gotopose import Opt_func_go_dist as rob_go_dist, \
    Opt_func_turn_dist as rob_turn_dist, \
    Opt_func_blind_move as rob_blind_move, \
    Opt_func_move_ev as rob_move_ev, \
    Opt_func_slam_move as slam_move
from opt_action_move import Opt_action_moveto as action_moveto, \
    Opt_action_movein as action_movein, \
    Opt_action_movetohuman as action_movetohuman
from opt_action_pickup import Opt_action_pickup as action_pickup
from opt_action_putdown import Opt_action_putdown as action_putdown
from opt_action_scan import Opt_action_scan as action_scan
from opt_action_handover import Opt_action_handover as action_handover
from opt_plan_executor import Opt_executor as action_executor


def register_options():
    Option.register(fetch_obj(), "blind_fetch")
    Option.register(put_obj(), "blind_put")
    Option.register(rob_go_dist(), "rob_go_dist")
    Option.register(rob_turn_dist(), "rob_turn_dist")
    Option.register(rob_blind_move(), "rob_blind_move")
    Option.register(rob_move_arm(), "rob_move_arm")
    Option.register(rob_simple_move_arm(), "rob_simple_move_arm")
    Option.register(rob_move_ev(), "rob_move_ev")
    Option.register(arm_liftup(), "rob_arm_liftup")
    Option.register(rob_hold(), "rob_hold")
    Option.register(rob_pre_fetch(), "rob_pre_fetch")
    Option.register(rob_pre_put(), "rob_pre_put")
    Option.register(slam_move(), "slam_move")
    Option.register(action_moveto(), 'action_moveto')
    Option.register(action_movein(), 'action_movein')
    Option.register(action_movetohuman(), 'action_movetohuman')
    Option.register(action_pickup(), 'action_pickup')
    Option.register(action_putdown(), 'action_putdown')
    Option.register(action_scan(), 'action_scan')
    Option.register(action_handover(), 'action_handover')
    Option.register(action_executor(), 'executor')
