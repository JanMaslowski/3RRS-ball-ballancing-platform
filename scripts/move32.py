# This file contains functions necessary for platform movement

import RPi.GPIO as GPIO
from time import sleep
import threading
from calc import calculate_motor_angles
import queue
from concurrent.futures import ThreadPoolExecutor
import concurrent.futures
from scipy.interpolate import interp1d
from math import sin, cos, radians

# Motion queue
motion_queue = queue.Queue()

############## SETUP PINS 
################################################################
# Pin setup
motor_pins = (7, 8, 14, 15, 23, 24)
resolution_pins = (17, 27, 22)
sensor_pins = (16, 20, 21)
resolution_modes = {
    "1": (0, 0, 0),
    "1/2": (1, 0, 0),
    "1/4": (0, 1, 0),
    "1/8": (1, 1, 0),
    "1/16": (0, 0, 1),
    "1/32": (1, 1, 1)
}

# Motor parameters
# Sensor pin, direction pin, step pin
motor_parameters = {"M1": (16, 15, 14), "M2": (20, 24, 23), "M3": (21, 8, 7)}
# Current angles of the motors
motor_angles = {"M1": 0, "M2": 0, "M3": 0}

# Pin mode setup
GPIO.setmode(GPIO.BCM)

# Sensors
GPIO.setup(sensor_pins, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Motors 
GPIO.setup(motor_pins, GPIO.OUT)
GPIO.setup(resolution_pins, GPIO.OUT)
GPIO.output(resolution_pins, resolution_modes["1/32"])
######################################################################
################### END PIN SETUP

# Function to return the status of a limit switch sensor
# Takes argument:
#   sensor_pin - the sensor pin to check
# Returns:
#   1 if the sensor is pressed, else 0
def sensor_status(sensor_pin):
    while True:
        status = GPIO.input(sensor_pin)
        sleep(0.005)
        return 1 if status == GPIO.HIGH else 0

# Function to return a motor to its zero position until the sensor signal is received
# Takes argument:
#   motor - the motor to move
def get_ground_position(motor):
    GPIO.output(motor_parameters[motor][1], 1)
    while sensor_status(motor_parameters[motor][0]) == 0:
        GPIO.output(motor_parameters[motor][2], GPIO.HIGH)
        sleep(0.005)
        GPIO.output(motor_parameters[motor][2], GPIO.LOW)

# Multi-threaded function to return all motors to their zero position
# Takes no arguments
def multi_get_ground_position():
    motors = ["M1", "M2", "M3"]
    threads = []
    for motor in motors:
        thread = threading.Thread(target=get_ground_position, args=(motor,))
        threads.append(thread)
        thread.start()
    for thread in threads:
        thread.join()
    for motor in motors:
        thread = threading.Thread(target=move_motor, args=(motor, 180, 0.001, 0))
        threads.append(thread)
        thread.start()
    for thread in threads:
        thread.join()
    for motor in motors:
        motor_angles[motor] = 0
    print("Motor angles:", motor_angles)

# Basic function to move motors
# Takes arguments:
#   motor - the motor to move
#   steps - the number of steps to take
#   freq - the frequency (sleep time) with a minimum value of 0.0002
#   direction - direction of movement (0 - up, 1 - down)
# Returns the angle to which the motor has moved relative to the base in degrees
def move_motor(motor, steps, freq, direction):
    GPIO.output(motor_parameters[motor][1], direction)
    for x in range(steps):
        GPIO.output(motor_parameters[motor][2], GPIO.HIGH)
        sleep(freq)
        GPIO.output(motor_parameters[motor][2], GPIO.LOW)
        sleep(freq)
    ang = -1 if direction == 1 else (1 if direction == 0 else None)
    motor_angles[motor] += 0.05625 * steps * ang

# Function to prepare values for movement
# Takes arguments:
#   xrot - rotation angle with respect to the x-axis
#   yrot - rotation angle with respect to the y-axis
#   h - height at which the platform center should be
# Returns a vector of values (number of steps in angles, direction of steps) for each motor
def prepare_motion(xrot, yrot, h):
    M1_angle, M2_angle, M3_angle = calculate_motor_angles(xrot, yrot, h)
    M1_move_diff = motor_angles["M1"] + M1_angle
    M1_dir = 0 if M1_move_diff < 0 else 1
    M2_move_diff = motor_angles["M2"] + M2_angle
    M2_dir = 0 if M2_move_diff < 0 else 1
    M3_move_diff = motor_angles["M3"] + M3_angle
    M3_dir = 0 if M3_move_diff < 0 else 1
    return (abs(M1_move_diff), M1_dir, abs(M2_move_diff), M2_dir, abs(M3_move_diff), M3_dir)

def slow_the_speed(steps):
    freq = 0.000165
    if steps < 10:
        k = interp1d([0, 10], [1.5, 1])
        factor = k(steps)
    else:
        factor = 1
    return freq * factor

# Thread pool function to move motors smoothly
def threads_pool_move(rot_x, rot_y, h):
    with ThreadPoolExecutor(max_workers=3) as executor:
        futures = []
        motion = prepare_motion(rot_x, rot_y, h)
        args_list = [
            ("M1", int(motion[0] * 17.778), slow_the_speed(int(motion[0] * 17.778)), motion[1]),
            ("M2", int(motion[2] * 17.778), slow_the_speed(int(motion[2] * 17.778)), motion[3]),
            ("M3", int(motion[4] * 17.778), slow_the_speed(int(motion[4] * 17.778)), motion[5])
        ]
        for args in args_list:
            future = executor.submit(move_motor_smooth, *args)
            futures.append(future)
        concurrent.futures.wait(futures)
        futures.clear()        

def move_motor_smooth(motor, steps, freq, direction):
    GPIO.output(motor_parameters[motor][1], direction)
    for x in range(steps):
        if x / steps < 0.20:
            m = interp1d([0, steps * 0.20], [1.6, 1])
            GPIO.output(motor_parameters[motor][2], GPIO.HIGH)
            sleep(m(x) * freq)
            GPIO.output(motor_parameters[motor][2], GPIO.LOW)
            sleep(m(x) * freq)
        elif x / steps > 0.8:
            n = interp1d([0.8 * steps, steps], [1, 1.6])
            GPIO.output(motor_parameters[motor][2], GPIO.HIGH)
            sleep(n(x) * freq)
            GPIO.output(motor_parameters[motor][2], GPIO.LOW)
            sleep(n(x) * freq)
        else:
            GPIO.output(motor_parameters[motor][2], GPIO.HIGH)
            sleep(freq)
            GPIO.output(motor_parameters[motor][2], GPIO.LOW)
            sleep(freq)
    ang = -1 if direction == 1 else (1 if direction == 0 else None)
    motor_angles[motor] += 0.05625 * steps * ang
