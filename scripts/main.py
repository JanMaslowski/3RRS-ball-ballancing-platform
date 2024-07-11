import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import threading
from picamera2 import Picamera2
import queue
from concurrent.futures import ThreadPoolExecutor
from scipy.interpolate import interp1d
import math

# Import custom modules
from calc import calculate_motor_angles
from camera import correct_pic, detect_circles, detect_orange_ball, correct_center
from move32 import threads_pool_move, multi_get_ground_position
from pid import PID, check_distance_flag

# Camera calibration parameters
DIM = (1280, 972)
K = np.array([[328.57988968885235, 0.0, 640.7563153198729], [0.0, 328.97666546524516, 487.1287892854078], [0.0, 0.0, 1.0]])
D = np.array([[-0.029109933568421023], [0.03052429657029856], [-0.02629029569947379], [0.006725228609111244]])

# Initialize undistort rectify map
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280, 972)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Perform initial platform alignment
multi_get_ground_position()
initial_picture = picam2.capture_array()
corrected_initial_pic = correct_pic(initial_picture, map1, map2)
_, const_plat_center_x, const_plat_center_y = detect_circles(corrected_initial_pic)

# PID and motion control parameters
previous_error_x = 0
previous_error_y = 0
iteration_time = 0.006
pid_x = 0
pid_y = 0
current_x_rot = 0
current_y_rot = 0
flagx = 0
flagy = 0
trajectory = []

# Main control loop
def main(show_trajectory_window=True):
    global previous_error_x, previous_error_y, iteration_time, pid_x, pid_y, current_x_rot, current_y_rot, flagx, flagy

    while True:
        start_time = time.time()

        # Capture and correct image
        image = picam2.capture_array()
        corrected_img = correct_pic(image, map1, map2)

        # Detect orange ball
        corrected_img, ball_center_x, ball_center_y = detect_orange_ball(corrected_img, trajectory, 50)

        # Correct platform center
        platform_center = correct_center(const_plat_center_x, const_plat_center_y, current_x_rot, current_y_rot)

        # Calculate offsets
        offset_x = ball_center_x - platform_center[0]
        offset_y = ball_center_y - platform_center[1]
        print("Offsets from center:", offset_x, offset_y)

        # PID control
        if abs(offset_x) < 180 and abs(offset_y) < 180:
            pid_x = PID(offset_x, previous_error_x, iteration_time, flagx)
            pid_y = PID(offset_y, previous_error_y, iteration_time, flagy)
            flagx, flagy = 1, 1
        else:
            flagx, flagy = 0, 0

        # Move motors
        threads_pool_move(pid_y, -pid_x, 0.2)

        # Update iteration time and errors
        iteration_time = time.time() - start_time
        previous_error_x = offset_x
        previous_error_y = offset_y

        # Display trajectory if enabled
        if show_trajectory_window:
            cv2.imshow('Trajectory', corrected_img)

        if cv2.waitKey(1) == ord('q'):
            break

        print("Iteration time:", iteration_time)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Platform control script with optional trajectory window display.")
    parser.add_argument("--show-trajectory", action="store_true", help="Enable to display the trajectory window.")
    args = parser.parse_args()

    main(show_trajectory_window=args.show_trajectory)
