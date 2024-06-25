# MoveIt Task Constructor Framework

The Task Constructor framework provides a flexible and transparent way to define and plan actions that consist of multiple interdependent subtasks.
It draws on the planning capabilities of [MoveIt](https://moveit.ros.org/) to solve individual subproblems in black-box *planning stages*.
A common interface, based on MoveIt's PlanningScene is used to pass solution hypotheses between stages.
The framework enables the hierarchical organization of basic stages using *containers*, allowing for sequential as well as parallel compositions.

## Build Python Interface
Only when build type set to "install" will the build python interface be available.
```bash
cd <MoveItWorkspace>
catkin config --install
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8
source /PATH_TO_ROS_WORKSPACE/install/setup.zsh
```

## Dual-arm Motion Planning
```bash
cd <MoveItWorkspace>
# launch env in rviz
roslaunch moveit_task_constructor_demo dual_demo.launch
# add clip fixtures
python src/panda_dual_arms/fixture_scene.py -c /home/tp2/Documents/mios-wiring/clip_info_20240429_transform.pickle
# option 1: cpp interface
rosrun moveit_task_constructor_demo dual_pick
# option 2: python interface
python src/moveit_task_constructor/demo/scripts/dual_pick.py

```
