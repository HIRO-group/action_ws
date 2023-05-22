# Interactive perception for a robot manipulator

## Installation

It is recommended that you uninstall any ros moveit packages on your system. Better yet, uninstall ros and install it from scratch.

### Build

```
git clone --recurse-submodules git@github.com:nataliya-dev/action_ws.git
cd action_ws
./install-ws-ubuntu.sh -i
./install-ws-ubuntu.sh -b
```

### Source the Catkin Workspace

Setup your environment -- you can do this every time you work with this particular source install of the code, or you can add this to your shell.

```
source ~/action_ws/devel/setup.bash # or .zsh, depending on your shell
```

## Getting started

```
roslaunch tacbot panda.launch
roslaunch tacbot generate_contact_plan.launch
```

If you are developing the tacbot package, you may notice that building can take time. To build the package and ignore other packages use:

```
catkin build --start-with tacbot
```

otherwise:

```
catkin build
```

## Submodule overview

| library             | origin          | upstream     |
| :------------------ | :-------------- | :----------- |
| libfranka           | nataliya-dev    | frankaemika  |
| franka_ros          | nataliya-dev    | frankaemika  |
| omplapp             | nataliya-dev    | ompl         |
| ompl                | nataliya-dev    | ompl         |
| boost_sml           | PickNikRobotics | n/a          |
| moveit_tutorials    | ros-planning    | n/a          |
| panda_moveit_config | ros-planning    | n/a          |
| moveit              | nataliya-dev    | ros-planning |
| ruckig              | pantor          | n/a          |

## TODO

- Add a setting to allow or disallow cached queries for collision here: moveit_planners/ompl/ompl_interface/src/detail/state_validity_checker.cpp
- The above can be done with a setting in the parent class of validity checking and then changing that setting through the SimpleSetup which lives in the context (model based planning context)
- Extract rawTrajectoryPtr and add it to the visualization
-
