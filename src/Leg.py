#!/usr/bin/env python3
import math
import numpy as np

import rospy
import sys
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

class Leg():
    def __init__(self, name):
        self.name = name

        self.c1_state = None
        self.thigh_state = None
        self.tibia_state = None
        # get the information from each joint
        self.c1_state_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
                                                 JointControllerState, self.joint1_state_callback)
        self.thigh_state_sub = rospy.Subscriber('/phantomx/j_thigh_' + self.name + '_position_controller/state',
                                                 JointControllerState, self.joint2_state_callback)
        self.tibia_state_sub = rospy.Subscriber('/phantomx/j_thigh_' + self.name + '_position_controller/state',
                                                 JointControllerState, self.joint3_state_callback)

    def joint1_state_callback(self, data):
        self.c1_state = data.process_value
    def joint2_state_callback(self, data):
        self.thigh_state = data.process_value
    def joint3_state_callback(self, data):
        self.tibia_state = data.process_value

    def get_joints_state(self):
        return np.array([self.c1_state, self.thigh_state, self.tibia_state])
    def get_leg_name(self):
        return self.name

    def forward_kinematic(self):
        l1 = 0.054
        l2 = 0.066
        l3 = 0.16
        theta1 = self.c1_state
        theta2 = self.thigh_state
        theta3 = self.tibia_state
        return np.array([math.cos(theta1) * (l1 + l2*math.cos(theta2) + l3*math.cos(theta2+theta3)),
                         math.sin(theta1) * (l1 + l2*math.cos(theta2) + l3*math.cos(theta2+theta3)),
                         l2*math.sin(theta2) + l3*math.sin(theta2+theta3)])

    def inverse_kinematic(self, target):
        l1 = 0.054
        l2 = 0.066
        l3 = 0.16
        x = target[0]
        y = target[1]
        z = target[2]

        theta1 = math.atan2(y, x)
        theta2 = None
        args = [l3, (x - l1 * math.cos(theta1)) / (math.cos(theta1) * z), -l2 - l3]
        tan = np.roots(args)
        theta2_1 = math.atan(tan[0])
        theta2_2 = math.atan(tan[1])
        if theta2_1 >= 0:
            theta2 = theta2_1
        else:
            theta2 = theta2_2
        sin23 = (z - l2 * math.sin(theta2)) / l3
        theta3 = math.asin(sin23) - theta2
        return np.array([theta1, theta2, theta3])

if __name__ = "__main__":
    leg = Leg('rm')
    print(leg.get_joints_state())