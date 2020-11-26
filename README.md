Simulation for kejia with an simple task planning using ros and gazebo.

# Installation
Platform
```
Ubuntu 18.04, ROS melodic, Gazebo 9.0
```

ROS controllers
```
sudo apt install ros-melodic-effort-controllers
```

Get the code
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/cuigw1/simulation.git
cd ..
catkin_make
```


# Running a demo with the kejia, using Gazebo simulator
Start the gazebo and load the world:
```
roslaunch kejia_gazebo kejia_gpsr.launch
```

Start navigation:
```
roslaunch kejia_nav nav_dwa.launch
```

Start the action interface:
```
rosrun we_python_sm dndec.py executor
```

Run the demo:
```
rosrun planner_example planner.py
```

# Edit map
Generate map file
```
cd kejia_simulator/kejia_mapping/map
./pgm2map pgmfile mapfile
example: ./pgm2map gpsr gpsr.map
```

Edit map
```
add names to WEMapEditor.ini
./we_map_editor
```
load a .map file, than can edit the map, it will generate a txt file finally.