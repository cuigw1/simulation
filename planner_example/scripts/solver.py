# coding=utf-8
import subprocess
import json
import os
from plan import Plan
import rospy


class Solver:
    def __init__(self, config):
        self.solver = config['solver']
        self.time_limit = config['time_limit']
        self.quiet = config['quiet']
        self.max_steps = config['max_steps']
        rospy.loginfo('solver: %s\n\tmax_step: %d\n\ttime_limit: %d\n\tquiet: %d' %
                      (self.solver, self.max_steps, self.time_limit, self.quiet))

    def execute(self, files, nstep=None, time=None, quiet=None, seed=None):
        if nstep == None:
            nstep = self.max_steps
        if time == None:
            time = self.time_limit
        if quiet == None:
            quiet = self.quiet
        cmd = '%s --outf=2 --time-limit=%d --quiet=%d -c n=%d ' % (
            self.solver, time, quiet, nstep)
        if seed != None:
            cmd += '--rand-freq=1 --seed=%d ' % seed
        cmd += ' '.join(files)
        rospy.loginfo(cmd)
        process = subprocess.Popen(cmd, shell=True,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        # wait for the process to terminate
        out, err = process.communicate()
        res = json.loads(out)
        result_num = res['Models']['Number']
        if result_num == 0:
            rospy.logwarn(err)
        # errcode = process.returncode
        res = json.loads(out)
        return res
