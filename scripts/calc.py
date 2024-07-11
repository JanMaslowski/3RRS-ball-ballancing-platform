
# This file contains functions necessary for inverse kinematics calculations

import numpy as np

# Rotation matrix around Z-axis
def rotz(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

# Function to calculate the angles for the motors
# Takes the values: x_d, y_d, and h
# x_d: rotation angle around the x-axis (degrees), positive value indicates clockwise rotation
# y_d: rotation angle around the y-axis (degrees), positive value indicates clockwise rotation
# h: height in meters

def calculate_motor_angles(x_d, y_d, h):

    x_r = (2 * np.pi / 360) * x_d
    y_r = (2 * np.pi / 360) * y_d
    z_r = np.arctan(-(np.sin(x_r) * np.sin(y_r)) / (np.cos(x_r) * np.cos(y_r)))

    R = np.array([
        [np.cos(y_r) * np.cos(z_r), -np.cos(y_r) * np.sin(z_r), np.sin(y_r)],
        [np.sin(x_r) * np.sin(y_r) * np.cos(z_r) + np.cos(x_r) * np.sin(z_r),
         np.cos(x_r) * np.cos(z_r) - np.sin(x_r) * np.sin(y_r) * np.sin(z_r),
         -np.sin(x_r) * np.cos(y_r)],
        [np.sin(x_r) * np.sin(z_r) - np.cos(x_r) * np.sin(y_r) * np.cos(z_r),
         np.sin(x_r) * np.cos(z_r) + np.cos(x_r) * np.sin(y_r) * np.sin(z_r),
         np.cos(x_r) * np.cos(y_r)]
    ])
    R_s = R.copy()

    # Constants necessary for calculations
    b = 0.075  # Distance from the center of the platform to the spherical joint [m]
    l1 = 0.09  # Length of the first link [m]
    l2 = 0.18  # Length of the second link [m]
    p = 0.15  # Distance from the center of the manipulator to the joint
    alpha = [2 * np.pi / 3, 4 * np.pi / 3, 0]  # Angles between the joints at the base
    
    pv = np.zeros((3, 3))
    points = np.zeros((3, 3))
    basepoints = np.zeros((3, 3))

    for i in range(3):
        pv = np.dot(R_s, rotz(alpha[i]))
        O0O7j_vector2 = np.array([[0], [0], [h]]) + np.dot(pv, np.array([[p], [0], [0]]))
        points[i, :] = O0O7j_vector2.flatten()
        base = np.array([[0], [0], [0]]) + (np.eye(3).dot(rotz(np.degrees(alpha[i]))).dot(np.array([[p], [0], [0]])))
        basepoints[i, :] = base.flatten()
    
    A = np.zeros(3)
    B = np.zeros(3)
    C = np.zeros(3)
    thetavec = np.zeros(3)

    for k in range(3):
        A[k] = 2 * l1 * np.cos(alpha[k]) * (-points[k, 0] + b * np.cos(alpha[k]))
        B[k] = 2 * l1 * points[k, 2] * np.cos(alpha[k])**2
        C[k] = points[k, 0]**2 - 2 * b * points[k, 0] * np.cos(alpha[k]) + np.cos(alpha[k])**2 * (b**2 + l1**2 - l2**2 + points[k, 2]**2)
        theta_calc = 2 * np.arctan2(-B[k] + np.sqrt(A[k]**2 + B[k]**2 - C[k]**2), C[k] - A[k]) / (2 * np.pi) * 360
        thetavec[k] = theta_calc
        
    return thetavec  # Vector of rotation angles in joints 1, 2, 3
