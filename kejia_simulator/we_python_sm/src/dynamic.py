#!/usr/bin/env python
# coding=utf-8

import re
import os


def add_model(model, name, pos):
    os.system('rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH`/%s/model.sdf -sdf -model %s -x %f -y %f -z %f'
              % (model, name, pos[0], pos[1], pos[2]))


def remove_model(name):
    os.system('rosservice call /gazebo/delete_model \"model_name: \'%s\'\"' % name)


# step = 3
# if step == 1:
#     remove_model('coke1')
#     remove_model('coke2')
#     remove_model('cup1')
#     add_model('coke_can', 'coke3', [-3.600, 1.600, 0.707])
#     add_model('coke_can', 'coke4', [-3.574, 1.844, 0.707])
# if step == 2:
#     add_model('beer', 'beer2', [-3.6, 1.2, 0.71])
#     remove_model('coke4')
# if step == 3:
#     remove_model('coke3')
#     remove_model('beer2')

# bring a coke from the kitchen for me
# get a cup from the living table for me
# change
# get a coke for me
# change
# bring a coke for me, then bring a beer for me
# change
# get a bowl for me
