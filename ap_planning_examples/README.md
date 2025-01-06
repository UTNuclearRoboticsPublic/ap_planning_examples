# AP Planning Examples
This is an examples package for planning with Affordance Primitives. See [here](https://github.com/UTNuclearRoboticsPublic/ap_planning) for the main repo. 

# Installing and Running Examples
Make sure you follow the install instructions in the [ap_planning repo](https://github.com/UTNuclearRoboticsPublic/ap_planning).

To install the demonstrations:
```sh
git clone https://github.com/TAMS-Group/bio_ik.git
git clone https://github.com/UTNuclearRoboticsPublic/ap_planning_examples.git
rosdep install --from-paths . --ignore-src -y
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

After you source the workspace, you should be able to run a number of demonstrations:
```sh
# This one starts a simulation Franka Emika Panda manipulator with single screw axis motions
roslaunch ap_planning_examples panda_demo.launch

# This one is the same arm, with multiple chained screw constraints
roslaunch ap_planning_examples chained_screws.launch

# Same arm, but with multiple independent screw axes
roslaunch ap_planning_examples unchained_screws.launch
```

This repository also contains a tool for visualizing the screw-constraint function working, debugging future improvements to it, etc. The following launch file will open RViz and let you drag around an "initial guess" coordinate frame. Every few seconds, the constraint function will be called to solve the closest pose to the initial guess that satisfies the constraints
```sh
roslaunch ap_planning_examples multiple_constraint_tester.launch
```

## Versioning
Note that this project is under constant development. The following versions are the latest ones that are known to work with these examples:
| Repository | Version |
| ---------- | ------- |
| [affordance_primitives](https://github.com/UTNuclearRobotics/affordance_primitives) | [v1.0.0](https://github.com/UTNuclearRobotics/affordance_primitives/releases/tag/v1.0.0)
| [ap_planning](https://github.com/UTNuclearRoboticsPublic/ap_planning) | [v1.0.0](https://github.com/UTNuclearRoboticsPublic/ap_planning/releases/tag/v1.0.0)
| [ap_planning_examples](https://github.com/UTNuclearRoboticsPublic/ap_planning_examples) | [v1.0.0](https://github.com/UTNuclearRoboticsPublic/ap_planning_examples/releases/tag/v1.0.0)
