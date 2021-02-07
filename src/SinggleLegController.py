#!/usr/bin/env python3
import math
import numpy as np

import rospy
import sys
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

class LegController:
    def __init__(self, name):
        rospy.init_node('leg_controller', anonymous=True)
        self.name = name
        self.joints_state = np.array([0, 0, 0])
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')

        # get the information from each joint
        self.c1_state_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
                                                 JointControllerState, self.joint1_state_callback)
        self.thigh_state_sub = rospy.Subscriber('/phantomx/j_thigh_' + self.name + '_position_controller/state',
                                                 JointControllerState, self.joint2_state_callback)
        self.tibia_state_sub = rospy.Subscriber('/phantomx/j_thigh_' + self.name + '_position_controller/state',
                                                 JointControllerState, self.joint3_state_callback)

        # publish the joint
        self.c1_pub = rospy.Publisher('/phantomx/j_c1_' + self.name + '_position_controller/command', Float64,
                queue_size=10)
        self.thigh_pub = rospy.Publisher('/phantomx/j_thigh_' + self.name + '_position_controller/command', Float64,
                queue_size=10)
        self.tibia_pub = rospy.Publisher('/phantomx/j_tibia_' + self.name + '_position_controller/command', Float64,
                queue_size=10)


    def joint1_state_callback(self, data):
        self.joints_state[0] = data.process_value
    def joint2_state_callback(self, data):
        self.joints_state[1] = data.process_value
    def joint3_state_callback(self, data):
        self.joints_state[2] = data.process_value


    def swing_trajetory(self):
        pass

    def forward_kinematic(self):
        pass

    def inverse_kinematic(self, point):
        pass

    def move_to(self, target_position):
        # target : ny.array([x y z])
        dt = np.array([rospy.get_time()]) - self.time_previous_step
        position_error = target_position - self.forward_kinematic()
        desired_joints_state = self.inverse_kinematic(target_position)
        current_joints_state = self.joints_state
        error = desired_joints_state - current_joints_state
        error_derivative = error/dt

if __name__ == '__main__':
    node = rospy.init_node('LegController', anonymous=True)
    LC = LegController('rm')
    print('started')
    rospy.spin()
