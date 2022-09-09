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
| moveit               | ros-planning      | n/a |


## TODO
- If an upstream branch exists, pull from upstream branches and merge with the origin master branch.
- Check that the moveit library is using the action_ws ompl library.
- Build according to ros version, preferrably noetic.
