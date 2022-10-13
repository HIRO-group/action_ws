# Interactive perception for a robot manipulator


## Installation
### Build
```
git clone --recurse-submodules git@github.com:nataliya-dev/action_ws.git
cd action_ws
./install-ws-ubuntu.sh
```

### Source the Catkin Workspace
Setup your environment -- you can do this every time you work with this particular source install of the code, or you can add this to your shell.
```
source ~/action_ws/devel/setup.bash # or .zsh, depending on your shell
```

## Getting started
```
roslaunch franka_gazebo panda.launch rviz:=true
roslaunch pick_and_place main.launch
```

If you are developing the pick_and_place package, you may notice that building can take time. To build the package and ignore dependencies use:
```
catkin build pick_and_place --no-deps
```

otherwise:
```
catkin build
```

## Submodule overview
| library              | origin            | upstream  |
| :---                 |   :---            | :--- |
| libfranka            | nataliya-dev      | frankaemika |
| franka_ros           | nataliya-dev      | frankaemika |
| omplapp              | nataliya-dev      | ompl |
| ompl                 | nataliya-dev      | ompl|
| boost_sml            | PickNikRobotics   | n/a |
| moveit_tutorials     | ros-planning      | n/a |
| panda_moveit_config  | ros-planning      | n/a |
| moveit               | nataliya-dev      | ros-planning |


## TODO
- Step through the repulsed states via keyboard input
- Add repulsive vectors to every joint on visualizer for repulsed states
- Add a function to reduce the vector based on distance from obstacle
- Add a way to reduce the vector based on manipulability and direction vectors (if the repulsive vector does not align with manipulability the reduce it or remove it)
- Visualize trajectory via "original" planner (VFRRT or RRT) vs our contact planner. Publish them to different topics.

