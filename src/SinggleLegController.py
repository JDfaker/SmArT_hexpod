#!/usr/bin/env python3
import math
import numpy as np

import rospy
import sys
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from Leg import Leg

class LegController:
    def __init__(self, leg):
        rospy.init_node('leg_controller', anonymous=True)
        self.leg = leg
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')

        # publish the joint
        self.c1_pub = rospy.Publisher('/phantomx/j_c1_' + self.name + '_position_controller/command', Float64,
                queue_size=10)
        self.thigh_pub = rospy.Publisher('/phantomx/j_thigh_' + self.name + '_position_controller/command', Float64,
                queue_size=10)
        self.tibia_pub = rospy.Publisher('/phantomx/j_tibia_' + self.name + '_position_controller/command', Float64,
                queue_size=10)



    def move_to(self, target_position):
        pass

if __name__ == '__main__':
    rm_LC = LegController(leg=Leg('rm'))
