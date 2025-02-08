// #include <ompl/base/SpaceInformation.h>
// #include <ompl/base/goals/GoalStates.h>
// #include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
// #include <ompl/base/spaces/SE3StateSpace.h>
// #include <ompl/config.h>
// #include <ompl/geometric/SimpleSetup.h>
// #include <ompl/geometric/planners/prm/PRM.h>
// #include <ompl/geometric/planners/prm/PRMstar.h>
// #include <ompl/geometric/planners/rrt/RRT.h>
// #include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <algorithm>
#include <iostream>
#include <queue>
#include <thread>
#include <utility>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

#include <affordance_primitive_msgs/ScrewStamped.h>
#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/screw_model/screw_axis.hpp>
#include <ap_planning/ap_planning.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <traj_serv_to_action/JointTraj.h>  


namespace ob = ompl::base;
namespace og = ompl::geometric;

void show_multi_screw(const ap_planning::APPlanningRequest &req, moveit_visual_tools::MoveItVisualTools &visual_tools)
{
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    // Get constraint
    auto constraint = req.toConstraint();

    visual_tools.publishAxis(constraint->referenceFrame());

    // Plot each screw
    const auto viz_screws = constraint->getVisualScrews();
    for (const auto &screw : viz_screws)
    {
        // Create points for plotting
        Eigen::Vector3d origin, axis;
        tf2::fromMsg(screw.origin, origin);
        tf2::fromMsg(screw.axis, axis);
        Eigen::Vector3d end = origin + 0.2 * axis.normalized();
        geometry_msgs::Point end_point = tf2::toMsg(end);
        geometry_msgs::Point origin_point = tf2::toMsg(origin);

        // Plot
        auto color = screw.is_pure_translation ? rviz_visual_tools::PURPLE : rviz_visual_tools::ORANGE;
        visual_tools.publishArrow(origin_point, end_point, color, rviz_visual_tools::LARGE);
    }
    visual_tools.trigger();
}

void show_trajectory(trajectory_msgs::JointTrajectory &traj, moveit_visual_tools::MoveItVisualTools &visual_tools)
{
    moveit_msgs::DisplayTrajectory joint_traj;
    joint_traj.model_id = "spota";
    joint_traj.trajectory.push_back(moveit_msgs::RobotTrajectory());

    // Set up left arm in stow
    // std::vector<double> left_stow{1.7453, -2.8863, 2.793, -1.6581, -3.0543, 0.0};
    // traj.joint_names.push_back("left_ur5_shoulder_pan_joint");
    // traj.joint_names.push_back("left_ur5_shoulder_lift_joint");
    // traj.joint_names.push_back("left_ur5_elbow_joint");
    // traj.joint_names.push_back("left_ur5_wrist_1_joint");
    // traj.joint_names.push_back("left_ur5_wrist_2_joint");
    // traj.joint_names.push_back("left_ur5_wrist_3_joint");

    joint_traj.trajectory.at(0).joint_trajectory = traj;

    moveit_msgs::RobotState start_msg;
    start_msg.joint_state.name = traj.joint_names;

    auto first_waypoint = traj.points.at(0);
    start_msg.joint_state.position = first_waypoint.positions;
    // for (const auto &val : left_stow)
    // {
    //     start_msg.joint_state.position.push_back(val);
    // }
    joint_traj.trajectory_start = start_msg;

    joint_traj.trajectory.at(0).joint_trajectory.header.frame_id = "base_link";

    int time = 0;

    for (auto &wp : joint_traj.trajectory.at(0).joint_trajectory.points)
    {
        wp.time_from_start.sec = time;
        ++time;
        // for (const auto &val : left_stow)
        // {
        //     wp.positions.push_back(val);
        // }
    }

    visual_tools.publishTrajectoryPath(joint_traj);
}

std::queue<moveit_msgs::CollisionObject> get_collision_objects()
{
    // Set up stuff same across all test cases
    std::queue<moveit_msgs::CollisionObject> output;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.operation = collision_object.ADD;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);

    // Screw one
    collision_object.id = "screw1_box1";
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.4;
    box_pose.position.x = 0.25;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    // Add screw one object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    output.push(collision_object);
    collision_object.primitives.clear();
    collision_object.primitive_poses.clear();

    return output;
}

double calculate_joint_distance(const trajectory_msgs::JointTrajectory &traj)
{
    double total_joint_space_distance = 0.0;
    for (size_t i = 1; i < traj.points.size(); ++i)
    {
        const auto &prev_wp = traj.points[i - 1];
        const auto &curr_wp = traj.points[i];

        double distance = 0.0;
        for (size_t j = 0; j < prev_wp.positions.size(); ++j)
        {
            double diff = curr_wp.positions[j] - prev_wp.positions[j];
            distance += diff * diff;
        }
        total_joint_space_distance += std::sqrt(distance);
    }
    return total_joint_space_distance;
}

struct TaskInfo
{
    Eigen::Matrix<double, 6, 1> start_state;
    Eigen::Vector3d screw_axis;
    Eigen::Vector3d screw_location;
    double screw_goal;
    double pitch;
    int trajectory_density;
};


class JointStateSubscriber {
public:
    Eigen::VectorXd joint_positions;

    JointStateSubscriber(ros::NodeHandle& nodeHandle) : nh(nodeHandle) {
        sub = nh.subscribe("/joint_states", 10, &JointStateSubscriber::jointStateCallback, this);
        
        // Initialize joint_positions to the correct size and set to NaN
        joint_positions.conservativeResize(joint_order.size());
        joint_positions.setConstant(std::numeric_limits<double>::quiet_NaN()); // default
       
        // Initialize service client for the "/arm_controller/follow_joint_trajectory" service
        joint_trajectory_client = nh.serviceClient<traj_serv_to_action::JointTraj>("/arm_controller/follow_joint_trajectory");
    }
    
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (msg->position.size() < joint_order.size()) {
            ROS_ERROR_STREAM("Unexpected number of joints in the joint callback: "
                             << msg->position.size());
            return; 
        }
        
        for (size_t i = 0; i < joint_order.size(); ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_order[i]);
            if (it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);
                joint_positions(i) = msg->position[index];
            } else {           
                ROS_ERROR_STREAM("Joint " << joint_order[i] << " not found in received joint states.");
            }
        }
    }

    // Function to send the joint trajectory to the service
    bool sendJointTrajectory(const trajectory_msgs::JointTrajectory& joint_traj) {
        // Create a service request and populate it with the joint trajectory
        traj_serv_to_action::JointTraj srv;
        srv.request.joint_traj = joint_traj;

        // Send the request and wait for a response
        if (joint_trajectory_client.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("Joint trajectory executed successfully.");
                return true;
            } else {
                ROS_ERROR("Failed to execute joint trajectory.");
                return false;
            }
        } else {
            ROS_ERROR("Failed to call service follow_joint_trajectory");
            return false;
        }
    }
    
private:
    ros::NodeHandle nh;        
    ros::Subscriber sub;       
    ros::ServiceClient joint_trajectory_client;
    
    std::vector<std::string> joint_order = {
        "arm0_shoulder_yaw",   
        "arm0_shoulder_pitch", 
        "arm0_elbow_pitch",    
        "arm0_elbow_roll",     
        "arm0_wrist_pitch",    
        "arm0_wrist_roll"      
    };  

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ap_planning");
    ros::NodeHandle nh;
    // construct the joint_state subscriber
    JointStateSubscriber js_sub(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    //--------------------------------------//
    // Define task
    TaskInfo task_info;

    // Retrieve current state
    // if ((js_sub.joint_positions.array().isNaN()).any()) {
    //         ROS_ERROR_STREAM("Unable to read joint states");
	    // return -1;
    // }
    // else {
    const double TIMEOUT_SEC = 10.0;
auto start_time = std::chrono::steady_clock::now();

while ((js_sub.joint_positions.array().isNaN()).any() || js_sub.joint_positions.isZero()) {
    // Check elapsed time
    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    double elapsed_sec = std::chrono::duration<double>(elapsed_time).count();

    if (elapsed_sec > TIMEOUT_SEC) {
        ROS_FATAL_STREAM("Timeout reached while waiting for valid joint states.");
        return -1;
    }

    // Add a small delay to avoid high CPU usage
    ros::Duration(0.1).sleep();
}

    task_info.start_state = js_sub.joint_positions;
            ROS_FATAL_STREAM("Read joint states: "<<js_sub.joint_positions);
        // (Eigen::VectorXd(6) << 0.01769, -1.27994, 2.13614, 0.04380, -0.84493, -0.07706).finished(); // VALVE TURN CASE 3
    // }


    // task_info.screw_axis = Eigen::Vector3d(-1, 0, 0);
    // task_info.screw_location = Eigen::Vector3d(0.617247, 0.0635829, 0.224735);
    // task_info.screw_goal = 3.0 / 4.0 * M_PI;
    // task_info.screw_goal = 1.0/4.0 * M_PI;
    // task_info.trajectory_density = 200;

    // double waypoint_ang = task_info.screw_goal / task_info.trajectory_density;
    // std::string n_name = ros::this_node::getName();
    // nh.setParam(n_name + "/waypoint_ang", waypoint_ang);

    //--------------------------------------//

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    // std::vector<double> default_joint_state{-0.00015592575073242188, -0.8980185389518738,
    //                                         1.8094338178634644,      0.000377655029296875,
    //                                         -0.8991076946258545,     0.0015475749969482422}; // READY
    // std::vector<double> default_joint_state = {-0.00415325, -1.3191, 2.291, 0.067544, -1.8325, -0.0605698}; //
    // NAMASTE
    std::vector<double> default_joint_state(task_info.start_state.data(),
                                            task_info.start_state.data() + task_info.start_state.size());

    kinematic_state->setJointGroupPositions("arm", default_joint_state);

    auto fk = kinematic_state->getGlobalLinkTransform("arm0_tool0");
    const Eigen::Vector3d fk_pos = fk.translation();
    const Eigen::Quaterniond fk_quat(fk.rotation());
    //---------------------------------------------//
    const double pelican_x = 0.29;
    const Eigen::Vector3d pelican_case_hinge(pelican_x, 0, 0);
    task_info.screw_axis = Eigen::Vector3d(0, 1, 0);
    task_info.screw_location = fk_pos + pelican_case_hinge;
    task_info.screw_goal = 1.0/6.0 * M_PI;
    //---------------------------------------------//

    // Print results
    ROS_INFO_STREAM("fk_pos: " << fk_pos.transpose());
    ROS_INFO_STREAM("fk_quat: [x: " << fk_quat.x() << ", y: " << fk_quat.y() << ", z: " << fk_quat.z()
                                    << ", w: " << fk_quat.w() << "]");

    int num_sample, num_sps;
    nh.param<int>(ros::this_node::getName() + "/num_sampling", num_sample, 2);
    nh.param<int>(ros::this_node::getName() + "/num_sps", num_sps, 2);

    bool show_trajectories;
    nh.param<bool>(ros::this_node::getName() + "/show_trajectories", show_trajectories, true);

    bool use_obstacles;
    nh.param<bool>(ros::this_node::getName() + "/add_collision_objects", use_obstacles, true);

    // Set planner type
    std::string planner_name;
    nh.param<std::string>(ros::this_node::getName() + "/planner", planner_name, "prm");
    std::transform(planner_name.begin(), planner_name.end(), planner_name.begin(), ::tolower);

    ap_planning::PlannerType planner_type;
    if (planner_name == "prm")
    {
        planner_type = ap_planning::PlannerType::PRM;
    }
    else if (planner_name == "prmstar")
    {
        planner_type = ap_planning::PlannerType::PRMstar;
    }
    else if (planner_name == "rrt")
    {
        planner_type = ap_planning::PlannerType::RRT;
    }
    else if (planner_name == "rrtconnect")
    {
        planner_type = ap_planning::PlannerType::RRTconnect;
    }
    else
    {
        ROS_WARN_STREAM("Unknown planner type: '" << planner_name << "', using PRM");
        planner_type = ap_planning::PlannerType::PRM;
    }

    // Add collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto collision_objects = get_collision_objects();

    ros::Duration(2.0).sleep();

    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.setRobotStateTopic("/display_robot_state");
    visual_tools.trigger();

    std::queue<ap_planning::APPlanningRequest> planning_queue;
    ap_planning::APPlanningRequest single_request;
    single_request.ee_frame_name = "arm0_tool0";
    single_request.planning_time = 10;
    single_request.planner = planner_type;

    ap_planning::ScrewSegment screw1, screw2, screw3;

    // We'll plan from the default joint states for these experiments
    single_request.start_pose.pose.position.x = fk_pos[0];
    single_request.start_pose.pose.position.y = fk_pos[1];
    single_request.start_pose.pose.position.z = fk_pos[2];
    single_request.start_pose.pose.orientation.x = fk_quat.x();
    single_request.start_pose.pose.orientation.y = fk_quat.y();
    single_request.start_pose.pose.orientation.z = fk_quat.z();
    single_request.start_pose.pose.orientation.w = fk_quat.w();

    // Add some test cases
    screw1.screw_msg.header.frame_id = "base_link";
    screw1.start_theta = 0.0;
    screw1.end_theta = task_info.screw_goal;
    screw1.screw_msg.origin.x = task_info.screw_location[0]; // VALVE TURN CASE 3
    screw1.screw_msg.origin.y = task_info.screw_location[1]; // VALVE TURN CASE 3
    screw1.screw_msg.origin.z = task_info.screw_location[2]; // VALVE TURN CASE 3
    screw1.screw_msg.axis.x = task_info.screw_axis[0];
    screw1.screw_msg.axis.y = task_info.screw_axis[1];
    screw1.screw_msg.axis.z = task_info.screw_axis[2];
    single_request.screw_path.push_back(screw1);
    planning_queue.push(single_request);

    ap_planning::DSSPlanner ap_planner("arm");
    ap_planning::SequentialStepPlanner sequential_step_planner("arm");
    if (!sequential_step_planner.initialize())
    {
        ROS_ERROR_STREAM("Init failed");
        return EXIT_FAILURE;
    }

    std::stringstream ss_dssp, ss_sps;
    ss_dssp << "Start of output\n";
    ss_sps << "Start of output\n";
    size_t sample = 0;
    ap_planning::APPlanningResponse last_plan;
    bool collision_obj_exists = false;

    // visual_tools.prompt("\n\n\nStow the left arm!\n\n\n");

    // Plan each screw request
    while (planning_queue.size() > 0 && ros::ok())
    {
        sample++;
        auto req = planning_queue.front();
        planning_queue.pop();

        if (show_trajectories)
        {
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan next screw");
        }

        if (use_obstacles && !collision_objects.empty())
        {
            if (collision_obj_exists)
            {
                std::vector<std::string> remove_objs;
                remove_objs.push_back(collision_objects.front().id);
                planning_scene_interface.removeCollisionObjects(remove_objs);
                collision_objects.pop();
            }
            std::vector<moveit_msgs::CollisionObject> collision_obj_vec;
            collision_obj_vec.push_back(collision_objects.front());
            planning_scene_interface.addCollisionObjects(collision_obj_vec);
            collision_obj_exists = true;
            std::cout << "\n\n\nUSING OBSTACLES!!!!!!!!\n\n";
        }

        show_multi_screw(req, visual_tools);

        for (size_t i = 0; i < num_sample; ++i)
        {
            std::cout << "Starting i = " << i << "\n";
            ap_planning::APPlanningResponse result;
            auto start = std::chrono::high_resolution_clock::now();
            ap_planning::Result success = ap_planner.plan(req, result);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            if (success == ap_planning::SUCCESS)
            {
                std::cout << "\n\n\nDSS planning: Success!!\n\n";
                std::cout << "Trajectory is: " << result.percentage_complete * 100
                          << "% complete, and has length: " << result.path_length
                          << ", is valid: " << result.trajectory_is_valid << "\n";
                ROS_WARN("Planning time: %.6f seconds", duration.count() / 1e6);
                auto joint_distance = calculate_joint_distance(result.joint_trajectory);
                ROS_WARN("Joint distance: %.6f", joint_distance);
                ROS_WARN("Number of waypoints: %zu", result.joint_trajectory.points.size());
                ROS_WARN("Trying to execute trajectory on the robot");

                if (show_trajectories)
                {
                    show_trajectory(result.joint_trajectory, visual_tools);
                }
		if (!js_sub.sendJointTrajectory(result.joint_trajectory)){return -1;}
                last_plan = result;
            }
            else
            {
                std::cout << "\n\n\nDSS planning: Fail (" << ap_planning::toStr(success) << ")\n\n";
            }

            ss_dssp << sample << ", DSS, " << ap_planning::toStr(success) << ", " << result.percentage_complete * 100
                    << ", " << duration.count() << ", " << result.path_length << ",\n";
        }

        // Now move to SPS planner
        if (show_trajectories)
        {
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan again using "
                                "SPS planner");
        }

        // Try planning
        for (size_t i = 0; i < num_sps; ++i)
        {
            ap_planning::APPlanningResponse sps_output;
            auto start = std::chrono::high_resolution_clock::now();
            auto sps_res = sequential_step_planner.plan(req, sps_output);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            if (sps_res == ap_planning::SUCCESS)
            {
                std::cout << "\n\n\nSPS planning: Success!!\n\n";
                std::cout << "Trajectory is: " << sps_output.percentage_complete * 100
                          << "% complete, and has length: " << sps_output.path_length << "\n";
                ROS_WARN("Planning time: %.6f seconds", duration.count() / 1e6);
                auto joint_distance = calculate_joint_distance(sps_output.joint_trajectory);
                ROS_WARN("Joint distance: %.6f", joint_distance);
                ROS_WARN("Number of waypoints: %zu", sps_output.joint_trajectory.points.size());
                ROS_WARN("Trying to execute trajectory on the robot");
                if (show_trajectories)
                {
                    show_trajectory(sps_output.joint_trajectory, visual_tools);
                }
		if (!js_sub.sendJointTrajectory(sps_output.joint_trajectory)){return -1;}
                last_plan = sps_output;
            }
            else
            {
                std::cout << "\n\n\nSPS planning: Fail (" << ap_planning::toStr(sps_res) << ")\n\n";
                std::cout << "Trajectory is: " << sps_output.percentage_complete * 100
                          << "% complete, and has length: " << sps_output.path_length << "\n";
            }
            ss_sps << sample << ", SPS, " << ap_planning::toStr(sps_res) << ", " << sps_output.percentage_complete * 100
                   << ", " << duration.count() << ",\n";

            // if (sps_res == ap_planning::NO_IK_SOLUTION) {
            //   sequential_step_planner = ap_planning::SequentialStepPlanner(nh);
            //   sequential_step_planner.initialize();
            // }
        }
    }

    ROS_ERROR_STREAM("\n\nAll done with trajectories\n\n");

    ss_dssp << "End output\n";
    ss_sps << "End output\n";
    std::cout << ss_dssp.str();
    std::cout << ss_sps.str();

    ros::shutdown();
    return 0;
}
