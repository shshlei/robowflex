/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>
#include <robowflex_library/constants.h>

using namespace robowflex;

/* \file fetch_visualization.cpp
 * A simple script that demonstrates how to use RViz with Robowflex with the
 * Fetch robot. See https://kavrakilab.github.io/robowflex/rviz.html for how to
 * use RViz visualization. Here, the scene, the pose goal, and motion plan
 * displayed in RViz.
 */

static const std::string GROUP = "arm_with_base_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    IO::RVIZHelper rviz(fetch);
    IO::RobotBroadcaster bc(fetch);
    bc.start();
    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromURDFFile("package://robowflex_resources/scenes/messy/messy.xacro");
    scene->getScene()->setSafetyDistance(0.03);
    scene->getScene()->setContactDistanceThreshold(0.05);
    scene->getScene()->setNegativeDistanceThreshold(-0.05);
    scene->setCollisionDetector("FCLNew");
    // Visualize the scene in RViz.
    rviz.updateScene(scene);
    RBX_INFO("Setted planning scene! Press enter to continue ...");
    std::cin.get();

    // Create the default planner for the fetch.
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
    planner->initialize();

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, GROUP);
    request.setAllowedPlanningTime(500.0);
    request.setWorkspaceBounds(Eigen::Vector3d(-5.0, -5.0, 2.0), Eigen::Vector3d(5.0, 5.0, 2.0));

    fetch->setBasePose(2.0, 3.5, constants::pi);
    fetch->setGroupState("arm_with_torso", {0.0, -0.000043, 1.358958, 3.141547, 1.384234, 0.000307, -0.025282, 3.141288});  // catch a shelves2-can2_0_1 1 
    if (false) // start
    {
        Robot::IKQuery ik_query("arm", "gripper_link", Eigen::Vector3d(1.15, 3.5, 0.45), Eigen::Quaterniond(0, 0, 0, 1)); // shelves2-can2_0_1
        ik_query.verbose = true;
        ik_query.timeout = 100.0;
        ik_query.setScene(scene);

        bool success = fetch->setFromIKWithoutCollision(ik_query);
        
        if (success)
            ROS_INFO("The start state is valid");
        else 
            ROS_INFO("The start state is invalid");
    }
    robot_model::RobotStatePtr robot_state;
    robot_state = fetch->getScratchState();
    robot_state->enforceBounds();
    scene->getScene()->setCurrentState(*robot_state.get());
    scene->attachObject(*robot_state.get(), "shelves2-can2_0_1", "gripper_link", {"gripper_link", "r_gripper_finger_link", "l_gripper_finger_link"});
    request.setStartConfiguration(robot_state);

    std::vector<double> joint_values;
    const std::vector<std::string>& joint_names = fetch->getModelConst()->getJointModelGroup("arm_with_torso")->getVariableNames();
    robot_state->copyJointGroupPositions("arm_with_torso", joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Start Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    RBX_INFO("Setted the start position! Press enter to continue...");
    std::cin.get();

    fetch->setBasePose(-3.0, -3.5, -0.5 * constants::pi);
    fetch->setGroupState("arm_with_torso", {0.0, 0.390447, 0.985810, -0.029607, -1.914817, -0.500756, 0.997199, 0.306547});  // detach a can 4 
    if (false) // goal 
    {
        Robot::IKQuery ik_query("arm", "gripper_link", Eigen::Vector3d(-2.8, -4.3, 0.75), Eigen::Quaterniond(std::sqrt(2.0)/2.0, 0, 0, -std::sqrt(2.0)/2.0));
        ik_query.verbose = true;
        ik_query.timeout = 100.0;
        ik_query.setScene(scene);

        bool success = fetch->setFromIKWithoutCollision(ik_query);
        
        if (success)
            ROS_INFO("The goal state is valid");
        else 
            ROS_INFO("The goal state is invalid");
    }
    robot_state = fetch->getScratchState();
    robot_state->enforceBounds();
    scene->getScene()->setCurrentState(*robot_state.get());
    request.setGoalConfiguration(fetch->getScratchState());
    request.toYAMLFile("fetch_messy_request.yaml");
    bc.stop();
    std::vector<double> joint_values_goal;
    robot_state->copyJointGroupPositions("arm_with_torso", joint_values_goal);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Goal Joint %s: %f", joint_names[i].c_str(), joint_values_goal[i]);
    }
    RBX_INFO("Setted the goal position! Press enter to continue...");
    std::cin.get();

    fetch->setBasePose(2.0, 3.5, constants::pi);
    fetch->setGroupState("arm_with_torso", joint_values);
    rviz.updateScene(scene);
    RBX_INFO("Setted the planning problem! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
//    request.setConfig("BiHSC");
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz.
    rviz.updateTrajectory(res);
    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);
    // Output path to a file for visualization.
    trajectory->toYAMLFile("fetch_pick.yaml");

    RBX_INFO("Press enter to remove goal and scene.");
    std::cin.get();
    rviz.removeScene();

    return 0;
}
