# This file contains functions necessary for image processing

import cv2
import numpy as np
import math

def measure_trajectory_length(trajectory):
    length = 0.0
    for i in range(1, len(trajectory)):
        # Calculate distance between consecutive points
        x1, y1 = trajectory[i - 1]
        x2, y2 = trajectory[i]
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        length += distance
    return length

# Function to find the position of the orange ball and its center
# Takes arguments:
#       image - the image on which to search for the ball
# Returns:
#       Image with the ball marked
#       Ball center position in x
#       Ball center position in y
def detect_orange_ball(image, trajectory, max_frames):
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Define the range for the orange color in the HSV space
    lower_orange = np.array([0, 120, 120])
    upper_orange = np.array([25, 255, 255])
    # Create a mask for the orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    # Smooth the mask using morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)
    # Find the contours of the ball
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw a rectangle around the ball and calculate the ball's center
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:  # Minimum ball area size for detection
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Calculate the center of the detected ball
            ball_center_x = x + (w // 2)
            ball_center_y = y + (h // 2)
            
            trajectory.append((ball_center_x, ball_center_y))
            if len(trajectory) > max_frames:
                trajectory.pop(0)
            
            for i in range(1, len(trajectory)):
                cv2.line(image, trajectory[i - 1], trajectory[i], (255, 0, 0), 3)
                
            trajectory_length = measure_trajectory_length(trajectory)
            
            # Draw points at the center of the ball
            cv2.circle(image, (ball_center_x, ball_center_y), 5, (255, 0, 0), -1)
            
            # Return the image with the ball marked and the ball center coordinates
            return image, ball_center_x, ball_center_y
    
    # If no ball is detected, return None
    return image, 0, 0

# Function to find the center of the platform (tries to compensate for the offset relative to the center using tangent)
# It is computationally heavy due to the method of determining the platform center (averaging many centers)
# Takes arguments:
#       image - the image on which to search for the center
#       rotx - the rotation angle x in which the platform is currently located
#       roty - the rotation angle y in which the platform is currently located
# Returns:
#       Image with the center marked
#       Center position in x
#       Center position in y
def detect_circles(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 3, param1=100, param2=30, minRadius=180, maxRadius=300)
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        # Calculate the average position of the centers
        centers = circles[0, :, :2]
        avg_center = np.mean(centers, axis=0)
        # Calculate the distance of each center from the average
        distances = np.sqrt(np.sum((centers - avg_center) ** 2, axis=1))
        # Remove outlier centers (e.g., with large deviation)
        threshold = np.mean(distances) + 1.5 * np.std(distances)
        filtered_centers = centers[distances < threshold]
        # Recalculate the average position after filtering outliers
        avg_center = np.mean(filtered_centers, axis=0)
        avg_center = (int(avg_center[0]), int(avg_center[1]))
        
        # Draw the representative center
        cv2.circle(image, avg_center, 5, (0, 0, 255), -1)
        
        return image, avg_center[0], avg_center[1]
    return None, 0, 0

# Function to prepare the image by cropping and reducing distortions
# Takes arguments:
#       image - the image to prepare
# Returns:
#       Image ready for circle and ball detection
def correct_pic(image, map1, map2):
    height, width, _ = image.shape
    x = 316  # Example value x - 1/4 of the image width from the left
    y = 162  # Example value y - 1/4 of the image height from the top
    crop_width = 648  # Crop width +26
    crop_height = 648  # +18
    undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cropped_img = undistorted_img[y:y + crop_height, x:x + crop_width]
    return cropped_img

# Function to adjust the center by adding the tangent of the tilt
# Takes arguments:
#       center_x - platform center x-coordinate
#       center_y - platform center y-coordinate
#       rot_x - rotation angle around the x-axis
#       rot_y - rotation angle around the y-axis
# Returns:
#       Adjusted platform center coordinates x and y
def correct_center(center_x, center_y, rot_x, rot_y):
    l1 = 50
    radian_rotx = math.radians(rot_x)
    radian_roty = math.radians(rot_y)
    ang_x = -1 if rot_x < 0 else 1  # y-axis
    ang_y = -1 if rot_y > 0 else 1  # x-axis
    avg_center = (int(center_x + math.tan(abs(radian_roty)) * l1 * ang_y), int(center_y + math.tan(abs(radian_rotx)) * l1 * ang_x))
    return avg_center
