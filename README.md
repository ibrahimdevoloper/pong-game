# pong_game

### Setting up a robot simulation (Basic)
https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html

### Command to Launch Webots
'''
colcon build
source install/local_setup.bash
ros2 launch my_package robot_launch.py
'''

### Command to delete build files
'''
rm -rf build/ install/ log/
colcon build
'''