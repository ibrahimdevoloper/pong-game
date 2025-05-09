# pong_game

### Python Packages
```
pip install keyboard pynput
```

### Setting up a robot simulation (Basic)
https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html

### Setting up a robot simulation (Advanced)
https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Advanced.html

### Speed Estimation & Vehicle Tracking
https://www.youtube.com/watch?v=uWP6UjDeZvY

### Command to Launch Webots
```
colcon build
source install/local_setup.bash
ros2 launch pong_game robot_launch.py
```

### Command to delete build files
```
rm -rf build/ install/ log/
colcon build
```

### Important note
copy the  from  `fribe/grammars/simple/parser.grammar` and `fribe/grammars/simple/tokenizer.grammar` to `build/pong_game/build/lib/fribe/grammars/simple/parser.grammar` and `build/pong_game/build/lib/fribe/grammars/simple/tokenizer.grammar`
