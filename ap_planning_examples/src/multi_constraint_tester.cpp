#include <affordance_primitives/screw_planning/chained_screws.hpp>
#include <affordance_primitives/screw_planning/unchained_screws.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <mutex>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace visualization_msgs;

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::shared_ptr<geometry_msgs::Pose> robot_pose;
std::shared_ptr<affordance_primitives::UnchainedScrews> constraints;
std::shared_ptr<geometry_msgs::Transform> grasp_location;

std::mutex mutex;

visualization_msgs::InteractiveMarkerControl &
makeFrameControl(visualization_msgs::InteractiveMarker &msg,
                 const geometry_msgs::Pose &pose) {
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;

  // Make visual marker x-axis
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = msg.scale * 0.75;
  marker.scale.y = msg.scale * 0.05;
  marker.scale.z = msg.scale * 0.05;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.pose = pose;
  control.markers.push_back(marker);

  // Make visual marker y-axis
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  Eigen::Quaterniond rotation(0.5 * sqrt(2), 0, 0,
                              0.5 * sqrt(2)); // w, x, y, z
  Eigen::Quaterniond eig_quat;
  tf2::fromMsg(pose.orientation, eig_quat);
  auto axis_orientation = eig_quat * rotation;
  marker.pose.orientation = tf2::toMsg(axis_orientation);
  control.markers.push_back(marker);

  // Make visual marker z-axis
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  rotation =
      Eigen::Quaterniond(0.5 * sqrt(2), 0, -0.5 * sqrt(2), 0); // w, x, y, z
  axis_orientation = eig_quat * rotation;
  marker.pose.orientation = tf2::toMsg(axis_orientation);
  control.markers.push_back(marker);

  msg.controls.push_back(control);

  return msg.controls.back();
}

void processMovingFeedback(const InteractiveMarkerFeedbackConstPtr &feedback) {
  const std::lock_guard<std::mutex> lock(mutex);
  *robot_pose = feedback->pose;
  server->applyChanges();
}

void makeMovingFrameMarker() {
  visualization_msgs::InteractiveMarker moving_frame;
  moving_frame.header.frame_id = "map";
  moving_frame.scale = 0.25;

  moving_frame.name = "moving_frame";
  moving_frame.description = "Moving Frame";

  moving_frame.pose.position.x = 1.0;
  moving_frame.pose.position.y = 0.5;

  // insert the arrow
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  makeFrameControl(moving_frame, start_pose);
  moving_frame.controls.at(0).interaction_mode =
      visualization_msgs::InteractiveMarkerControl::NONE;

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.x = 0.5 * sqrt(2);
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.orientation.w = 0.5 * sqrt(2);
  control.name = "rotate_x";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  moving_frame.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  moving_frame.controls.push_back(control);

  control.orientation.x = 0;
  control.orientation.y = 0.5 * sqrt(2);
  control.orientation.z = 0;
  control.orientation.w = 0.5 * sqrt(2);
  control.name = "rotate_z";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  moving_frame.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  moving_frame.controls.push_back(control);

  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.5 * sqrt(2);
  control.orientation.w = 0.5 * sqrt(2);
  control.name = "rotate_y";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  moving_frame.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  moving_frame.controls.push_back(control);

  server->insert(moving_frame);
  server->setCallback(moving_frame.name, &processMovingFeedback);
}

void timerFn(const ros::TimerEvent &) {
  const std::lock_guard<std::mutex> lock(mutex);

  // Get the robot pose right now
  Eigen::Isometry3d tf_m_to_q;
  tf2::fromMsg(*robot_pose, tf_m_to_q);

  // Set up the output
  affordance_primitives::ScrewConstraintSolution sol;

  // Call the constraint function
  auto res = constraints->constraintFn(tf_m_to_q, sol);
  if (!res) {
    ROS_ERROR_STREAM("Failed Constraints Fn");
    return;
  }

  // Print the solution
  std::stringstream ss;
  ss << "Solved phi: [";
  for (const auto &val : sol.solved_phi) {
    ss << val << ", ";
  }
  ss << "]";
  ROS_WARN_STREAM(ss.str());
  ROS_WARN_STREAM("Solved error: " << sol.error);

  // Get the solved pose
  Eigen::Isometry3d solved_closest = constraints->getPose(sol.solved_phi);

  // Display the closest pose
  visualization_msgs::InteractiveMarker waypoints;
  waypoints.header.frame_id = "map";
  waypoints.scale = 0.5;
  waypoints.name = "waypoints";
  makeFrameControl(waypoints, tf2::toMsg(solved_closest));
  waypoints.controls.back().interaction_mode =
      visualization_msgs::InteractiveMarkerControl::NONE;

  server->erase("waypoints");
  server->insert(waypoints);
  server->applyChanges();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "multi_constraint_tester");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  server = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "multi_constraint_tester", "map", false);

  robot_pose = std::make_shared<geometry_msgs::Pose>();

  // Make the constraints
  constraints = std::make_shared<affordance_primitives::UnchainedScrews>();

  // Add some screw axes
  affordance_primitive_msgs::ScrewStamped axis_msg;
  axis_msg.header.frame_id = "map";
  axis_msg.is_pure_translation = true;
  axis_msg.axis.x = 1;
  constraints->addScrewAxis(axis_msg, 0, 1);

  axis_msg.axis.x = 0;
  axis_msg.axis.y = 1;
  constraints->addScrewAxis(axis_msg, 0, 1);

  axis_msg.origin.y = 0.5;
  axis_msg.axis.x = 0;
  axis_msg.axis.y = 0;
  axis_msg.axis.z = 1;
  axis_msg.is_pure_translation = false;
  axis_msg.pitch = 0;
  constraints->addScrewAxis(axis_msg, 0, 0.5 * M_PI);

  // Add the frame information
  geometry_msgs::Transform grasp;
  grasp.rotation.w = 1;
  grasp_location = std::make_shared<geometry_msgs::Transform>(grasp);
  constraints->setReferenceFrame(tf2::transformToEigen(*grasp_location));

  // Visualize the axes, grasp, etc
  rviz_visual_tools::RvizVisualTools rvt("map");
  ros::Duration(1.5).sleep();
  rvt.publishAxis(tf2::transformToEigen(grasp));

  std::vector<affordance_primitives::ScrewStamped> viz_screws =
      constraints->getVisualScrews();
  for (const auto &screw : viz_screws) {
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
    rvt.publishArrow(origin_point, end_point, color, rviz_visual_tools::LARGE);
  }
  rvt.trigger();

  makeMovingFrameMarker();

  server->applyChanges();

  ros::Timer timer = nh.createTimer(ros::Duration(3), timerFn);

  ros::waitForShutdown();
  server.reset();
  return 0;
}
