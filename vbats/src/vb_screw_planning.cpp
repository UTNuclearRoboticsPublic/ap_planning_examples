#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <affordance_primitive_msgs/AffordancePrimitiveAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <ap_planning/ap_planning.hpp>

/*
 * LOVELY HARDCODING
 */
const std::string PLANNING_GROUP = "right_ur5";
const std::string PLANNING_FRAME = "base_link";
const std::string GRIPPER_ACTION =
    "/vaultbot/robot_manager/robots/vaultbot/geh6000il_action_server/"
    "geh6000il_action_server";
const std::string SCREW_EXEC_ACTION =
    "/vaultbot/robot_manager/robots/vaultbot/affordance_primitive_executor";
const std::string TAG_FRAME_NAME = "anchor_valve_qr_code";
const std::string EE_NAME = "r_temoto_end_effector";

/*
 * SCREW STUFF
 */
geometry_msgs::Vector3 getXAxis(const geometry_msgs::Quaternion& quat_msg) {
  geometry_msgs::Vector3 output_msg;

  // Extract from msg
  Eigen::Quaterniond quat;
  tf2::fromMsg(quat_msg, quat);
  quat.normalize();

  // Convert to rotation matrix and get the X axis
  Eigen::Matrix3d rot_mat = quat.toRotationMatrix();
  auto axis = rot_mat.col(0);

  // Back to msg type
  tf2::toMsg(axis, output_msg);
  return output_msg;
}

ap_planning::APPlanningRequest getScrewPath() {
  ap_planning::APPlanningRequest output;

  output.ee_frame_name = EE_NAME;
  output.planning_time = 10.0;

  // Hardcoded grasp pose...
  output.start_pose.header.frame_id = TAG_FRAME_NAME;
  output.start_pose.pose.position.x = 0.428;
  output.start_pose.pose.position.y = -0.506;
  output.start_pose.pose.position.z = 0.326;
  output.start_pose.pose.orientation.x = 0;
  output.start_pose.pose.orientation.y = 1;
  output.start_pose.pose.orientation.z = 0;
  output.start_pose.pose.orientation.w = 0;

  // Hardcoded path as well...
  ap_planning::ScrewSegment seg_approach;
  seg_approach.start_theta = -0.15;
  seg_approach.end_theta = 0.0;
  seg_approach.screw_msg.header.frame_id = TAG_FRAME_NAME;
  seg_approach.screw_msg.is_pure_translation = true;
  seg_approach.screw_msg.origin = output.start_pose.pose.position;
  seg_approach.screw_msg.axis = getXAxis(output.start_pose.pose.orientation);

  ap_planning::ScrewSegment seg_turn;
  seg_turn.start_theta = 0.0;
  seg_turn.end_theta = 1 * M_PI / 3;
  seg_turn.screw_msg.header.frame_id = TAG_FRAME_NAME;
  seg_turn.screw_msg.is_pure_translation = false;
  seg_turn.screw_msg.origin.x = 0.499;
  seg_turn.screw_msg.origin.y = -0.339;
  seg_turn.screw_msg.origin.z = 0.346;
  seg_turn.screw_msg.axis.x = 1;
  seg_turn.screw_msg.axis.y = 0;
  seg_turn.screw_msg.axis.z = 0;

  ap_planning::ScrewSegment seg_retreat;
  seg_retreat.start_theta = 0.0;
  seg_retreat.end_theta = 0.15;
  seg_retreat.screw_msg.header.frame_id = TAG_FRAME_NAME;
  seg_retreat.screw_msg.is_pure_translation = true;
  seg_retreat.screw_msg.origin = output.start_pose.pose.position;
  seg_retreat.screw_msg.axis = getXAxis(output.start_pose.pose.orientation);
  seg_retreat.screw_msg.axis.x *= -1;
  seg_retreat.screw_msg.axis.y *= -1;
  seg_retreat.screw_msg.axis.z *= -1;

  output.screw_path.push_back(seg_approach);
  output.screw_path.push_back(seg_turn);
  output.screw_path.push_back(seg_retreat);
  return output;
}

// Transform from tag frame to planning frame
bool transformRequest(tf2_ros::Buffer& tfBuffer,
                      ap_planning::APPlanningRequest& req) {
  geometry_msgs::TransformStamped tfmsg_planning_to_affordance;
  try {
    tfmsg_planning_to_affordance =
        tfBuffer.lookupTransform(PLANNING_FRAME, TAG_FRAME_NAME, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  auto tf_planning_to_affordance =
      tf2::transformToEigen(tfmsg_planning_to_affordance.transform);

  // Move the reference pose
  Eigen::Isometry3d tf_start_pose;
  tf2::fromMsg(req.start_pose.pose, tf_start_pose);
  tf_start_pose = tf_planning_to_affordance * tf_start_pose;
  req.start_pose.pose = tf2::toMsg(tf_start_pose);
  req.start_pose.header.frame_id = PLANNING_FRAME;

  // Now transform each screw
  for (auto& segment : req.screw_path) {
    segment.screw_msg = affordance_primitives::transformScrew(
        segment.screw_msg, tf_planning_to_affordance.inverse());

    segment.screw_msg.header.frame_id = PLANNING_FRAME;
  }

  return true;
}

void show_multi_screw(const ap_planning::APPlanningRequest& req,
                      moveit_visual_tools::MoveItVisualTools& visual_tools) {
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Get constraint
  auto constraint = req.toConstraint();

  visual_tools.publishAxis(constraint->referenceFrame());

  // Plot each screw
  const auto viz_screws = constraint->getVisualScrews();
  for (const auto& screw : viz_screws) {
    // Create points for plotting
    Eigen::Vector3d origin, axis;
    tf2::fromMsg(screw.origin, origin);
    tf2::fromMsg(screw.axis, axis);
    Eigen::Vector3d end = origin + 0.2 * axis.normalized();
    geometry_msgs::Point end_point = tf2::toMsg(end);
    geometry_msgs::Point origin_point = tf2::toMsg(origin);

    // Plot
    auto color = screw.is_pure_translation ? rviz_visual_tools::PURPLE
                                           : rviz_visual_tools::ORANGE;
    visual_tools.publishArrow(origin_point, end_point, color,
                              rviz_visual_tools::LARGE);
  }
  visual_tools.trigger();
}

void show_trajectory(const trajectory_msgs::JointTrajectory& traj,
                     moveit_visual_tools::MoveItVisualTools& visual_tools) {
  moveit_msgs::DisplayTrajectory joint_traj;
  joint_traj.model_id = PLANNING_GROUP;
  joint_traj.trajectory.push_back(moveit_msgs::RobotTrajectory());
  joint_traj.trajectory.at(0).joint_trajectory = traj;

  moveit_msgs::RobotState start_msg;
  start_msg.joint_state.name = traj.joint_names;
  auto first_waypoint = traj.points.at(0);
  start_msg.joint_state.position = first_waypoint.positions;
  joint_traj.trajectory_start = start_msg;

  joint_traj.trajectory.at(0).joint_trajectory.header.frame_id = PLANNING_FRAME;

  int time = 0;

  for (auto& wp : joint_traj.trajectory.at(0).joint_trajectory.points) {
    wp.time_from_start.sec = time;
    ++time;
  }

  visual_tools.publishTrajectoryPath(joint_traj);
  visual_tools.trigger();
}

/*
 * GRIPPER STUFF
 */
bool moveGripper(
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction>& ac,
    const double cmd) {
  control_msgs::GripperCommandGoal goal;
  goal.command.position = cmd;
  ac.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(8.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Gripper action finished: %s", state.toString().c_str());
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    }
  } else {
    ROS_INFO("Gripper action failed");
  }
  return false;
}

bool openGripper(
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction>& ac) {
  return moveGripper(ac, 490.0);
}

bool closeGripper(
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction>& ac) {
  return moveGripper(ac, 4000.0);
}

/*
 * MOVE GROUP STUFF
 */
bool executeMoveitMove(moveit::planning_interface::MoveGroupInterface& mgi) {
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  size_t i = 0;
  constexpr size_t max_attempts = 5;

  auto success = (mgi.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (!success) {
    ROS_WARN_STREAM("Failed to plan");
    return false;
  }

  ROS_INFO_STREAM("Starting move");
  i = 0;
  moveit::core::MoveItErrorCode result = moveit::core::MoveItErrorCode::FAILURE;
  while (ros::ok() && i++ < max_attempts &&
         result != moveit::core::MoveItErrorCode::SUCCESS) {
    ROS_INFO_STREAM("Starting exec attempt: " << i);
    result = mgi.execute(my_plan);
  }
  return result == moveit::core::MoveItErrorCode::SUCCESS;
}

/*
 * SCREW EXECUTOR
 */
affordance_primitives::APRobotParameter getDefaultParameters() {
  affordance_primitives::APRobotParameter default_parameters;

  // Admittance
  default_parameters.admittance.trans_x = 0.001;
  default_parameters.admittance.trans_y = 0.001;
  default_parameters.admittance.trans_z = 0.001;
  default_parameters.admittance.rot_x = 0.06;
  default_parameters.admittance.rot_y = 0;
  default_parameters.admittance.rot_z = 0;

  // Max forces
  default_parameters.max_force = 80;
  default_parameters.max_torque = 5;
  default_parameters.max_wrench.trans_x = 50;
  default_parameters.max_wrench.trans_y = 50;
  default_parameters.max_wrench.trans_z = 50;
  default_parameters.max_wrench.rot_x = 8;
  default_parameters.max_wrench.rot_y = 8;
  default_parameters.max_wrench.rot_z = 8;

  return default_parameters;
}

bool sendScrewExec(actionlib::SimpleActionClient<
                       affordance_primitives::AffordancePrimitiveAction>& ac,
                   const ap_planning::ScrewSegment& segment,
                   const double theta_dot) {
  // Set up goal
  affordance_primitives::AffordancePrimitiveGoal goal;
  goal.moving_frame_name = goal.LOOKUP;
  goal.moving_frame_name = EE_NAME;
  goal.robot_params = getDefaultParameters();

  if (!segment.screw_msg.is_pure_translation) {
    goal.task_impedance_rotation = 20;
  }

  goal.theta_dot = theta_dot;
  goal.screw = segment.screw_msg;
  goal.theta_start = segment.start_theta;
  goal.theta_end = segment.end_theta;

  // Send the command
  ac.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Screw executor action finished: %s", state.toString().c_str());
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    }
  } else {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO_STREAM(
        "Screw executor action failed: " << state.toString().c_str());
  }
  return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vb_screw_planning");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface(
      PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP);

  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      gripper_client(GRIPPER_ACTION, true);

  ROS_INFO("Waiting for gripper action server to start.");
  if (!gripper_client.waitForServer(ros::Duration(5.0))) {
    ROS_ERROR_STREAM("Gripper client not connected");
    return EXIT_FAILURE;
  }

  actionlib::SimpleActionClient<
      affordance_primitives::AffordancePrimitiveAction>
      screw_client(SCREW_EXEC_ACTION, true);
  ROS_INFO("Waiting for screw executor action server to start.");
  if (!screw_client.waitForServer(ros::Duration(5.0))) {
    ROS_ERROR_STREAM("Screw executor client not connected");
    return EXIT_FAILURE;
  }

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(PLANNING_FRAME);
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a
  // high level script via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // TF listeners
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Create the planners
  std::string description_name = nh.getNamespace() + "/robot_description";
  ap_planning::DSSPlanner dss_planner(PLANNING_GROUP, description_name);

  ap_planning::SequentialStepPlanner sps_planner(PLANNING_GROUP,
                                                 description_name);
  if (!sps_planner.initialize()) {
    ROS_ERROR_STREAM("Init failed");
    return EXIT_FAILURE;
  }

  std::stringstream ss;

  ros::Duration(3.0).sleep();
  constexpr size_t trials = 3;
  for (size_t num_trial = 0; num_trial < trials; ++num_trial) {
    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to start trial");

    ROS_INFO_STREAM("Starting trial " << num_trial);
    ss << "Starting trial " << num_trial << "\n";

    // Display screw path
    auto request = getScrewPath();
    if (!transformRequest(tfBuffer, request)) {
      return EXIT_FAILURE;
    }
    show_multi_screw(request, visual_tools);

    auto start_time = ros::Time::now();

    // Attempt planning
    size_t i = 0;
    constexpr size_t max_attempts = 5;
    bool get_to_start = false;
    while (ros::ok() && i++ < max_attempts && !get_to_start) {
      //
      ROS_INFO_STREAM("Starting SPS plan");
      ap_planning::APPlanningResponse response;
      auto result = sps_planner.plan(request, response);
      ROS_WARN_STREAM(ap_planning::toStr(result));
      ROS_WARN_STREAM("SPS plan is valid: " << response.trajectory_is_valid);
      ROS_WARN_STREAM(
          "SPS plan size: " << response.joint_trajectory.points.size());
      if (result == ap_planning::Result::SUCCESS) {
        ss << "SPS success\n";
        show_trajectory(response.joint_trajectory, visual_tools);
      } else {
        ROS_INFO_STREAM("SPS failed, starting DSS plan");
        result = dss_planner.plan(request, response);
        ROS_WARN_STREAM(ap_planning::toStr(result));
        ROS_WARN_STREAM("DSS plan is valid: " << response.trajectory_is_valid);
        ROS_WARN_STREAM(
            "DSS plan size: " << response.joint_trajectory.points.size());
        if (result == ap_planning::Result::SUCCESS &&
            response.trajectory_is_valid) {
          ss << "DSS success\n";
          show_trajectory(response.joint_trajectory, visual_tools);
        } else {
          ROS_ERROR_STREAM("Both planners failed, exiting");
          ss << "SPS and DSS both failed\n";
          break;
        }
      }

      auto planning_duration = ros::Time::now() - start_time;
      ROS_INFO_STREAM("Done planning in: " << planning_duration.toSec());
      ss << "Done planning in: " << planning_duration.toSec() << "\n";

      // Extract first joint state from solution, send robot there
      const auto& starting_joint_pos = response.joint_trajectory.points.front();
      const auto& joint_names = response.joint_trajectory.joint_names;

      move_group_interface.setJointValueTarget(joint_names,
                                               starting_joint_pos.positions);

      if (!executeMoveitMove(move_group_interface)) {
        ROS_ERROR_STREAM("Moveit move failed");
      } else {
        get_to_start = true;
      }
    }

    if (!get_to_start) {
      ROS_ERROR_STREAM("Trial " << num_trial << " failed to get to start");
      continue;
    }

    auto ap_exec_start = ros::Time::now();

    // Tinker with the first start/end
    request.screw_path.at(0).end_theta = 0.185;
    request.screw_path.at(0).start_theta = 0.0;
    sendScrewExec(screw_client, request.screw_path.at(0), 0.07);

    closeGripper(gripper_client);
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("Starting turn");
    bool turn_succes =
        sendScrewExec(screw_client, request.screw_path.at(1), 0.2);
    ROS_INFO_STREAM("Done with turn");
    openGripper(gripper_client);
    ros::Duration(0.5).sleep();

    sendScrewExec(screw_client, request.screw_path.at(2), 0.07);

    if (turn_succes) {
      ROS_INFO_STREAM("Trial " << num_trial << " success!");
      ss << "Trial " << num_trial << " success!\n";
    } else {
      ROS_ERROR_STREAM("Trial " << num_trial << " failed during turn");
      ss << "Trial " << num_trial << " failed during turn\n";
    }

    auto total_duration = ros::Time::now() - start_time;
    auto ap_exec_duration = ros::Time::now() - ap_exec_start;

    ROS_INFO_STREAM("AP exec duration: " << ap_exec_duration.toSec());
    ROS_INFO_STREAM("Total duration: " << total_duration.toSec());

    ss << "AP exec duration: " << ap_exec_duration.toSec() << "\n";
    ss << "Total duration: " << total_duration.toSec() << "\n";

    move_group_interface.setNamedTarget("right_ur5_stow");
    if (!executeMoveitMove(move_group_interface)) {
      ROS_ERROR_STREAM("Moveit move failed");
      continue;
    }
  }

  ROS_INFO_STREAM("All trials done, ending!");
  std::cout << ss.str();

  return 0;
}
