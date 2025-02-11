To run the planners for Spot:
```bash
roslaunch vbats spota_planning.launch
```

Notable robot dependencies:
1. `spota_moveit_config` and `spot_description` packages from the `nrg_spot_manipulation_moveit` repos' `cca-sps-dss` branch linked [here](https://github.com/UTNuclearRobotics/nrg_spot_manipulation_moveit.git)
2. `ap_planning` libraries from the `noetic` branch [here](https://github.com/UTNuclearRoboticsPublic/ap_planning)
3. `affordance_primitives` libraries from the `noetic` branch [here](https://github.com/UTNuclearRobotics/affordance_primitives.git)
4. `bio_ik` from the `master` branch [here](https://github.com/TAMS-Group/bio_ik.git)

Alternatively, Dockerfile with unchanging setup including dependencies (3) and (4) available in the `ap_planning_examples` directory [here](https://github.com/CJans121/dockerfiles.git). To utilize the container, have (1), (2) and this repo in a folder and mount onto the container ROS workspace. 


