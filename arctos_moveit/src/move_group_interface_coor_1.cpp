#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("arm_controller");
    
    // Create executor for spinning
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    //----------------------------
    //Setup
    //----------------------------

    static const std::string PLANNING_GROUP = "arm";

    // The move_group_interface::MoveGroupInterface can be easily
    // setup using just the name of the planning group you would like to control and plan for
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // Using planning_scene_interface::PlanningSceneInterface class to deal directly with the world
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    move_group.setEndEffectorLink("Link_6_1");
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    
    // Create publisher for robot pose
    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 10);
    
    // We can print the name of the reference frame for this robot.
    // also printing the current position and orientation of the robot.
    RCLCPP_INFO(node->get_logger(), "x position: %f", current_pose.pose.position.x);
    RCLCPP_INFO(node->get_logger(), "y position: %f", current_pose.pose.position.y);
    RCLCPP_INFO(node->get_logger(), "z position: %f", current_pose.pose.position.z);
    RCLCPP_INFO(node->get_logger(), "x orientation: %f", current_pose.pose.orientation.x);
    RCLCPP_INFO(node->get_logger(), "y orientation: %f", current_pose.pose.orientation.y);
    RCLCPP_INFO(node->get_logger(), "z orientation: %f", current_pose.pose.orientation.z);
    RCLCPP_INFO(node->get_logger(), "w orientation: %f", current_pose.pose.orientation.w);
 
    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "move_group_tutorial", move_group.getRobotModel());
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Arctos Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    //-----------------------------
    //Getting Basic Information
    //-----------------------------

    // We can print the name of the reference frame for this robot.
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    RCLCPP_INFO(node->get_logger(), "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(),
              move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    //-----------------------------
    //Planning to a Pose Goal
    //-----------------------------

    // Plan a motion for this group to a desired pose for end-effector
    geometry_msgs::msg::Pose target_pose1;
    
    // Default pose
    target_pose1.position.x = 0.120679;
    target_pose1.position.y = 0.072992;
    target_pose1.position.z = 0.569166;
    target_pose1.orientation.x = -0.386473;
    target_pose1.orientation.y = -0.418023;
    target_pose1.orientation.z = -0.760978;
    target_pose1.orientation.w = 0.311139;

    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(node->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");

    // Visualizing plans
    RCLCPP_INFO(node->get_logger(), "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    
    // Wait for user input
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute trajectory");
    
    // Execute the plan
    if (success) {
        move_group.execute(my_plan);
        RCLCPP_INFO(node->get_logger(), "Motion executed successfully");
    }

    // Clean up
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
