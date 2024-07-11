# This file contains the controller

from scipy.interpolate import interp1d
import math

PID_i = 0

def PID(error, prev_error, time, flag):
    global PID_i
    # PID parameters
    kp = 0.0112
    kd = 0.0125
    ki = 0.000002 * 2
    
    # Limits
    max_ang = 11
    min_ang = -11
    max_val = 5
    min_val = -5
    
    # PID calculations
    PID_p = kp * error
    PID_d = kd * ((error - prev_error) / time)
    PID_i += ki * (error + prev_error)
    
    if flag == 0: 
        PID_d = 0
        PID_p = 0
    
    PID_total = PID_p + PID_i + PID_d
    PID_total = max(min(PID_total, max_val), min_val)  # Clamping PID_total within limits
    
    if error < 180:
        recalc_pid = interp1d([min_val, max_val], [min_ang, max_ang])
        return recalc_pid(PID_total)
    else:
        return 0

def check_distance_flag(x, y, window_size, threshold):
    # List storing the last distance values from the center
    distances = []

    # Calculate the distance from the center (square root of x^2 + y^2)
    distance = math.sqrt(x**2 + y**2)

    # Add the calculated distance to the list
    distances.append(distance)

    # If the list is larger than the window size, remove the oldest value
    if len(distances) > window_size:
        distances.pop(0)

    # Check the average distance from the last measurements
    for dist in distances:
        if dist > threshold + 2:
            return False
    
    # If the average distance is less than the threshold, return the flag
    return True  # Here you can return any flag indicating the condition is met
