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

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);
    ros::NodeHandle pnh("~");

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by
    // default.
    IO::RVIZHelper rviz(fetch);
    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromURDFFile("package://robowflex_resources/scenes/messy/shelf.xacro");
    scene->getScene()->setSafetyDistance(0.03);
    scene->getScene()->setContactDistanceThreshold(0.03);
    scene->getScene()->setNegativeDistanceThreshold(-0.05);
    scene->setCollisionDetector("FCLNew");
    // Visualize the scene in RViz.
    rviz.updateScene(scene);
    RBX_INFO("Setted the planning scene! Press enter to continue ...");
    std::cin.get();

    // Create the default planner for the fetch.
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
    planner->initialize();

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(fetch);
    request.setPlanner(planner);
    request.fromYAMLFile("/home/shshlei/mine/motion/moveit_exam/src/robowflex_resources/requests/messy/fetch/fetch_manipulator_shelf_request.yaml");
//    request.setAllowedPlanningTime(500.0);
//    request.setWorkspaceBounds(Eigen::Vector3d(-5.0, -5.0, 2.0), Eigen::Vector3d(5.0, 5.0, 2.0));

    robot_model::RobotStatePtr robot_state = request.getStartConfiguration();
    scene->getScene()->setCurrentState(*robot_state.get());
    rviz.updateScene(scene);

    const planning_interface::MotionPlanRequest req = request.getRequestConst();
    for (auto & obj : req.start_state.attached_collision_objects)
    {
        if (scene->hasObject(obj.object.id))
            scene->removeCollisionObject(obj.object.id);
    }
    RBX_INFO("Setted the planning problem! Press enter to continue ...");
    std::cin.get();

    // Do motion planning!
    std::string planner_id;
    pnh.getParam("planner_id", planner_id);
//    request.setConfig(planner_id);
    for (int i = 0; i < 10; i++)
    {
        planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            return 1;

        // Publish the trajectory to a topic to display in RViz.
        rviz.updateTrajectory(res);
        // Create a trajectory object for better manipulation.
        auto trajectory = std::make_shared<Trajectory>(res.trajectory_);
        // Output path to a file for visualization.
        trajectory->toYAMLFile("fetch_manipulator_path.yaml");
    }

    RBX_INFO("Press enter to remove goal and scene.");
    std::cin.get();
    rviz.removeScene();

    return 0;
}
