#!/usr/bin/env python
# -*- coding: utf-8 -*-
# $Id: service.py 42 2012-03-14 03:44:55Z MinCheng $
import logging
import threading
import rospy


class Service(threading.Thread):
    stop_run = False
    new_srv = False
    srv_finished = False
    srv_name = ''
    srv_type = None
    srv_request = None
    srv_response = None
    proxy = None
    rate = None

    def __init__(self, root=None):
        threading.Thread.__init__(self)
        Service.rate = rospy.Rate(10)

    @staticmethod
    def run():
        while not rospy.is_shutdown() and not Service.stop_run:
            if not Service.new_srv:
                Service.rate.sleep()
            else:
                Service.execute()

    @staticmethod
    def execute():
        try:
            rospy.wait_for_service(Service.srv_name, 2.0)
            Service.proxy = rospy.ServiceProxy(
                Service.srv_name, Service.srv_type)
        except rospy.ROSException as ex:
            print str(ex)
            return

        try:
            Service.srv_response = Service.proxy(Service.srv_request)
        except rospy.ServiceException as ex:
            print "Exception when calling service '{0}': {1}".format(
                Service.srv_name, str(ex))
        Service.new_srv = False
        Service.srv_finished = True
        Service.srv_name = ''
        Service.srv_request = None

    @staticmethod
    def stop():
        Service.stop_run = True

    @staticmethod
    def init_srv(srv_name, srv_type, srv_request):
        if srv_name == Service.srv_name and srv_request == Service.srv_request and Service.new_srv == True:
            return
        else:
            Service.srv_name = srv_name
            Service.srv_type = srv_type
            Service.srv_request = srv_request
            Service.srv_response = None
            Service.srv_finished = False
            Service.new_srv = True

    @staticmethod
    def is_srv_finished():
        if Service.srv_finished:
            Service.srv_finished = False
            return True
        else:
            return False

    @staticmethod
    def get_srv_response():
        return Service.srv_response
