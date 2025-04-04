
import cv2 as cv
import numpy as np
import time


def detectBallWitContours(frame, width, height):
    # Webots camera image is in BGRA format, convert it to a NumPy array
    image_array = np.frombuffer(frame, np.uint8).reshape((height, width, 4))

    # Convert from BGRA (Webots) to BGR (OpenCV)
    bgr_frame = cv.cvtColor(image_array, cv.COLOR_BGRA2BGR)

    # Convert BGR to HSV for color detection
    hsv = cv.cvtColor(bgr_frame, cv.COLOR_BGR2HSV)

    # Define the red color mask
    mask1 = cv.inRange(hsv, (0, 100, 0), (10, 255, 255))
    mask2 = cv.inRange(hsv, (175, 100, 0), (180, 255, 255))
    mask = cv.bitwise_or(mask1, mask2)

    # Erode to reduce noise
    mask = cv.erode(mask, (3, 3), iterations=3)

    # Blur the mask
    blur = cv.bilateralFilter(mask, 7, 75, 75)

    # Find contours
    contours, _ = cv.findContours(blur.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return None, None  # No contours found

    # Get the largest contour and its center
    c = max(contours, key=cv.contourArea)
    (x, y), radius = cv.minEnclosingCircle(c)
    #X->
    #X direction from right to left
    #Y\/
    #Y direction from top to bottom
    return (int(x), int(y)), int(radius)
 