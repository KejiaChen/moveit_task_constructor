# MoveIt Task Constructor Framework

The Task Constructor framework provides a flexible and transparent way to define and plan actions that consist of multiple interdependent subtasks.
It draws on the planning capabilities of [MoveIt](https://moveit.ros.org/) to solve individual subproblems in black-box *planning stages*.
A common interface, based on MoveIt's PlanningScene is used to pass solution hypotheses between stages.
The framework enables the hierarchical organization of basic stages using *containers*, allowing for sequential as well as parallel compositions.

## Grasp Planning with Dual-arm Setup
```
cd MoveItWorkspace
roslaunch moveit_task_constructor_demo dual_demo.launch
python src/panda_dual_arms/fixture_scene.py -c /home/tp2/Documents/mios-wiring/clip_info_20240429_transform.pickle
rosrun moveit_task_constructor_demo dual_pick
```