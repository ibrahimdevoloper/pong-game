"""main_controller controller."""

from controller import Robot
from controller import PositionSensor
from controller import Keyboard
from controller import Camera
import cv2 as cv
import numpy as np
import time

# functions
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
    
    return (int(x), int(y)), int(radius)
    
def findPos(ballPos):
    return 0.00027*ballPos + 0.0092

# create the Robot instance
robot = Robot()

# get the time step of the current world
timestep = int(robot.getBasicTimeStep())
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# get the motor instance
bottomBat = robot.getDevice('bottom_bat_motor')
bottomPosition = 0.11
bottomBat.setPosition(bottomPosition)

# get the motor instance
topBat = robot.getDevice('top_bat_motor')
topPosition = 0.11
topBat.setPosition(topPosition)

# get the position sensor instance
bottomSol = robot.getDevice('bottom_rack')
bottomSolPosition = 0.01
bottomSol.setPosition(bottomSolPosition)
posHigh = 0.2
poslow = 0.02

# Get the camera instance
camera = robot.getDevice('camera')
camera.enable(timestep)  # Enable the camera with the simulation time step

# Get the camera resolution
width = camera.getWidth()
height = camera.getHeight()

# Main loop:
while robot.step(timestep) != -1:
    # Read the position sensor
    key = keyboard.getKey()
    if key == keyboard.RIGHT:  # Move motor up
        print("Right key pressed")
        bottomPosition += 0.01
        if bottomPosition <= posHigh:
            bottomBat.setPosition(bottomPosition)  # Move motor in positive direction
        else:
            bottomPosition = posHigh
    elif key == keyboard.LEFT:  # Move motor down
        print("Left key pressed")
        bottomPosition -= 0.01
        if bottomPosition >= poslow:
            bottomBat.setPosition(bottomPosition)  # Move motor in positive direction
        else:
            bottomPosition = poslow  # Move motor in negative direction
    elif key == keyboard.UP:
        print("Up key pressed")
        bottomSol.setPosition(0.02)
        robot.step(30 * timestep)
        bottomSol.setPosition(0.01)

    # Get the image from the Webots camera
    image = camera.getImage()

    # Detect the ball using contours
    position, radius = detectBallWitContours(image, width, height)

    if position is not None:
        print(f"Ball detected at {position} with radius {radius}")
        newPos=findPos(position[1])
        topBat.setPosition(newPos)
        

    # Print the sensor value or process sensor data further
    # Process sensor data and send actuator commands

# Enter here exit cleanup code
