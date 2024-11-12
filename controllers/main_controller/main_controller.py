"""main_controller controller."""

from controller import Robot
from controller import PositionSensor
from controller import Keyboard
from controller import Camera
import cv2 as cv
import time

# functions
def detectBallWitContours(frame):

    gray = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    mask1 = cv.inRange(gray, (0, 100, 0), (10, 255, 255))
    mask2 = cv.inRange(gray, (175, 100, 0), (180, 255, 255))
    mask = cv.bitwise_or(mask1, mask2)
    mask = cv.erode(mask, (3,3), iterations=3)
    # mask = cv.dilate(mask, (3,3), iterations=2)
	
  
    blur = cv.bilateralFilter(mask, 7,75,75)
    contours, _ = cv.findContours(blur.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # _, contours, __ = cv.findContours(blur.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # Create empty centre array to store centroid center of mass
    center =   int(0), int(0)
    # Get the largest contour and its center 
    c = max(contours, key=cv.contourArea)
    (x, y), radius = cv.minEnclosingCircle(c)
    
    return (x, y), radius




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
# get the position sensor instance
bottomSol = robot.getDevice('bottom_rack')
bottomSolPosition = 0.01
bottomSol.setPosition(bottomSolPosition)
posHigh = 0.2
poslow =0.02

# Get the camera instance
camera = robot.getDevice('camera')
camera.enable(timestep)  # Enable the camera with the simulation time step


# Main loop:
while robot.step(timestep) != -1:
    # Read the position sensor
    key = keyboard.getKey()
    if key == keyboard.RIGHT:  # Move motor up
        print("Right key pressed")
        bottomPosition+=0.01
        if bottomPosition <= posHigh:
            bottomBat.setPosition(bottomPosition)  # Move motor in positive direction
        else :
            bottomPosition = posHigh
    elif key == keyboard.LEFT:  # Move motor down
        print("Left key pressed")
        bottomPosition-=0.01
        if bottomPosition >= poslow:
            bottomBat.setPosition(bottomPosition)  # Move motor in positive direction
        else :
            bottomPosition = poslow  # Move motor in negative direction
    elif key == keyboard.UP:
        print("Up key pressed")
        bottomSol.setPosition(0.02)
        robot.step(30*timestep) 
        bottomSol.setPosition(0.01)
    image = camera.getImage()
    detectBallWitContours(image)
     
    # Print the sensor value

    # Process sensor data and send actuator commands
    # newValue = bottomValue + 1.0  # Just an example; this adds 1 to the position
    # bottomBat.setPosition(0.19)

# Enter here exit cleanup code
