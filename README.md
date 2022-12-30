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
roslaunch pick_and_place panda.launch
roslaunch pick_and_place generate_contact_plan.launch
```

If you are developing the pick_and_place package, you may notice that building can take time. To build the package and ignore other packages use:
```
catkin build --start-with pick_and_place
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
| ruckig               | pantor            | n/a |


## TODO
- The simplest version of this planner is: evaluate a shortest distance from an obstacle to the a robot, if the distance is low then the cost is high. At every point in the plan, the cost is evaluated. If the cost is too high, the state is rejected. TRRT then tries to plan in the area of low costs rather than high costs, until the "temperature" is high enough.
- Currently, rather than cost of a robot state, we evaluate the cost to more from state A to state B. If that motion cost is away from an obstacle, even though it's still in proximity, the the state is likely to be accepted. Does this add much to the planning problem? Needs to be investigated.
- Currently, the cost is evaluated by running every point on the robot against every points on the cloud, evluating and averaging vectors, then computing an inverse jacobian to find a delta_q which will move the robot away from the obstacle in a specific direction. This motion is not taken into account at this stage. What is taken into account is how much the current state motion aligns with the repulsive motion. If that alignement is low then the cost is high, otherwise, the cost is low. Does this add much to the planning problem? How does it compare to the simplest solution? Need to investigate.
- The repulsion also needs to take into account manipulability. If the manipulability at a certain repulsive joint is low then that joint motion should not count much in the delta_q, thereby not add a lot to the cost.
- Manipulability and other aspects needs to be compared in the context of planning speed and success.
- Once the controller states that the joint foces are too high at a certain state, how do we take this into account wrt the vector field. The vector field is a function, which is calculated at every state. How do we incorporate a concept of memory into this? tbd, when we start to tie in the controller.
- bug: the path simplifier does not listen to the setting that we are setting.
