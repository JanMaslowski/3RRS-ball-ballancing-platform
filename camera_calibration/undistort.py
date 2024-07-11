import cv2
import numpy as np
import os
import glob
import sys
from PIL import Image

DIM = (2592, 1944)
K = np.array([[656.6413532777106, 0.0, 1304.5722080378341], [0.0, 656.0173417138636, 978.8320078126184], [0.0, 0.0, 1.0]])
D = np.array([[-0.029661844663170657], [0.08098845638809368], [-0.174968678040767], [0.114136862115802]])

def undistort(img_path):
    img = cv2.imread(img_path)
    h, w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    desktop_width, desktop_height = 1920, 1080  # Change these values to your desktop resolution
    undistorted_resized = cv2.resize(undistorted_img, (2592, 1944))
    
    # Display the resized undistorted image
    cv2.imshow("Undistorted and Resized", undistorted_resized)
    save_path = "ADD YOUR PATH HERE"
    undistorted_pil = Image.fromarray(cv2.cvtColor(undistorted_resized, cv2.COLOR_BGR2RGB))
    undistorted_pil.save(save_path)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)
