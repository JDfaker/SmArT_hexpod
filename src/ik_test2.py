#!/usr/bin/env python3
import math
import numpy as np

def forward_kinematic(theta):
    l1 = 1
    l2 = 2
    l3 = 3
    theta1 = theta[0]
    theta2 = theta[1]
    theta3 = theta[2]
    return np.array([math.cos(theta1) * (l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)),
                     math.sin(theta1) * (l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)),
                     l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)])

def inverse_kinematic(target):
    l1 = 1
    l2 = 2
    l3 = 3
    x = target[0]
    y = target[1]
    z = target[2]

    theta1 = math.atan2(y, x)
    theta2 = None
    args = [l3, (x-l1*math.cos(theta1))/(math.cos(theta1)*z), -l2-l3]
    tan = np.roots(args)
    theta2_1 = math.atan(tan[0])
    theta2_2 = math.atan(tan[1])
    if theta2_1 >=0:
        theta2 = theta2_1
    else:
        theta2 = theta2_2
    sin23 = (z-l2*math.sin(theta2))/l3
    theta3 = math.asin(sin23)-theta2
    return np.array([theta1, theta2, theta3])

a = np.array([math.radians(45), math.radians(45), math.radians(-30)])
print(a)
print(forward_kinematic(a))
print(inverse_kinematic(forward_kinematic(a)))