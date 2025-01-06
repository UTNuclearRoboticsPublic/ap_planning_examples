# Affordance Primitives
VaultBot-specific AT code, partnered with the robot-agnostic [affordance_primitives](https://github.com/UTNuclearRobotics/affordance_primitives) repository.

To set up a workspace just for testing Vbats, grab the repos from `vbats_repos.yaml` and verify you have the correct `vbats`/`affordance_primitives` versions that you want.

## Force Parser
Use to figure out how much force/torque the admittance controller feels during an AP move

Usage:
  - `rosbag record /compliance_wrench_delta/vaultbot/robot_manager/robots/vaultbot/right_wrench_to_joint_vel_pub/wrench /vaultbot/robot_manager/robots/vaultbot/right_jog_arm_server/delta_jog_cmds /vaultbot/robot_manager/robots/vaultbot/right_wrench_to_joint_vel_pub/set_applied_wrench`
  - `rosrun vbats force_parser _old:=false >> <data>.txt`
  - `rosbag play <the_bag_file>.bag`
  - Import `<data>.txt` to a spreadsheet
