#!/usr/bin/env python
# -*- coding: utf-8
import os
from service import Service
from info import register_subscriber
from robot import robot as device
import time
import threading
from options import register_options
from option import Option
import conf
import world
import sys
import rospy
import logging
import roslib
roslib.load_manifest('we_python_sm')


class DNGetMsg:
    def __init__(self):
        if conf.do_log:
            curr_path = sys.argv[0]
            curr_path = curr_path[0:curr_path.rfind('/')]
            curr_path = curr_path[0:curr_path.rfind('/')]
            log_path = curr_path + '/python.log'
            conf.logger = logging.getLogger()
            handler = logging.FileHandler(log_path, 'w')
            conf.logger.addHandler(handler)
            conf.logger.setLevel(logging.NOTSET)

            '''logging.basicConfig(filename=log_path, filemode='w',
                                level=logging.DEBUG)
            logging.debug('[')'''

            register_subscriber()


class StateMachine(threading.Thread):
    def __init__(self, root=None, argv=None):
        threading.Thread.__init__(self)
        self.stop_run = False
        self.rate = rospy.Rate(10.0)
        self.argv = argv
        world.sync = 0
        self.make_complete()
        if root != None:
            Option.set_root(root)
        register_options()
        time.sleep(0.5)

    def make_complete(self):
        if os.environ['SHELL'].find('zsh') >= 0:
            file_object = open(os.environ['HOME']+'/python_cmd', 'w')
            for key in Option.optmap:
                file_object.write(key+'\n')

    def on_exit(self, msg):
        rospy.signal_shutdown(msg)

    def run(self):
        try:
            while not self.stop_run:
                Option.execute(self.argv)
                world.sync += 1
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.logwarn('!! Ctrl + C entered !!')
            self.on_exit('Shutting down by user')
        except:
            rospy.logwarn('Unexpected error occurred!')
            device.say('Oh yeah, python crashed!')
            time.sleep(0.5)
            import traceback
            traceback.print_exc()
            self.on_exit('Unexpected error occurs!')

    def stop(self):
        self.stop_run = True


def start(msg=None):
    # rospy.init_node('we_python_sm', None, False, 2, False, False, True)
    rospy.init_node('we_python_sm', log_level=rospy.DEBUG, anonymous=True)
    brain = DNGetMsg()
    if msg == None or len(msg) > 3 or len(msg) == 1:
        stateMachineThread = StateMachine()
    elif len(msg) == 2:
        # msg[1] is name of subroutine
        stateMachineThread = StateMachine(msg[1])
    else:
        # msg[2] is the string argv of the subroutine specified by msg[1]
        stateMachineThread = StateMachine(msg[1], eval(msg[2]))

    stateMachineThread.start()

    serviceThread = Service()
    serviceThread.start()

    rospy.spin()

    serviceThread.stop()
    stateMachineThread.stop()

    serviceThread.join()
    stateMachineThread.join()


if __name__ == '__main__':
    start(sys.argv)
