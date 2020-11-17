# coding=utf-8
import json
import rospy


class Plan:
    def __init__(self, text=None):
        self.Optimal = False
        self.result_num = 0
        self.cost = 0.0
        self.steps = []
        self.nsteps = 0
        self.holds = []
        self.goals = []
        self.objinfo = None
        self.curr_step = 0
        if text:
            self.parse(text)

    def parse(self, text):
        self.result_num = text['Models']['Number']
        if self.result_num == 0:
            rospy.logwarn('No solution !')
            return
        if 'Optimal' in text['Models'].keys():
            self.Optimal = text['Models']['Optimal']
            if not self.Optimal:
                rospy.logwarn('No Optimal Solution!')
        res = text['Call'][-1]['Witnesses'][-1]
        if 'Costs' in res.keys():
            self.cost = res['Costs'][0]
        self.nsteps = ''.join(res['Value']).count('occurs')
        self.steps = [None] * self.nsteps
        for _ in range(self.nsteps + 1):
            self.holds.append([])
        for a in res['Value']:
            if a[0:6] == 'occurs':
                p, t = a[7:-1].rsplit(',', 1)
                istep = int(t) - 1
                self.steps[istep] = p
            elif a[0:5] == 'holds':
                p, t = a[6:-1].rsplit(',', 1)
                istep = int(t)
                if istep <= self.nsteps:
                    self.holds[istep].append(p)
            elif a[0:6] == '-holds':
                p, t = a[7:-1].rsplit(',', 1)
                istep = int(t)
                if istep <= self.nsteps:
                    self.holds[istep].append('~'+p)
            elif a[0:4] == 'goal':
                self.goals.append(a[5:-1])
        rospy.loginfo('Found a plan %d steps' % len(self.steps))
        for i, step in enumerate(self.steps):
            print('  step %03d - %s' % (i, step))

    def merge(self, plan):
        if plan.nsteps > 0:
            self.holds += plan.holds if self.nsteps == 0 else plan.holds[1:]
            self.nsteps += plan.nsteps
            self.steps += plan.steps
            self.cost += plan.cost
            self.goals += plan.goals
            rospy.loginfo('%d + %d = %d' % (self.nsteps - plan.nsteps, plan.nsteps, self.nsteps))

    def finished(self):
        return self.curr_step >= self.nsteps
