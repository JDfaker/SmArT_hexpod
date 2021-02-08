#!/usr/bin/env python3
import math
import numpy as np


def forward_kinematic(theta):
    l1 = 0.054
    l2 = 0.066
    l3 = 0.16
    theta1 = theta[0]
    theta2 = theta[1]
    theta3 = theta[2]
    return np.array([math.cos(theta1) * (l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)),
                     math.sin(theta1) * (l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)),
                     l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)])

def inverse_kinematic(target):
    l1 = 0.054
    l2 = 0.066
    l3 = 0.16
    x = target[0]
    y = target[1]
    z = target[2]

    theta1 = math.atan2(y,x)
    theta2 = np.arccos((-l3**2 + l2**2 + x**2 + y**2 + z**2)/(2*l2*math.sqrt(x**2 + y**2 + z**2))) + math.atan(z/math.sqrt(x**2 + y**2))
    theta3 = -np.arccos((x**2 + y**2 + z**2 - l2**2 - l3**2)/2*l2*l3)

    return np.array([theta1, theta2, theta3])

a = np.array([-1.57073831, 1.57073831, 1.57073831])
print(forward_kinematic(a))
print(inverse_kinematic(forward_kinematic(a)))
print(forward_kinematic(inverse_kinematic(forward_kinematic(a))))