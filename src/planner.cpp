#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include "robowflex.h"

using namespace robowflex;

MotionRequestBuilder::MotionRequestBuilder(const Robot &robot, const std::string &group_name)
  : robot_(robot), group_name_(group_name), jmg_(robot.getModel()->getJointModelGroup(group_name))
{
    request_.group_name = group_name_;

    // Default workspace
    moveit_msgs::WorkspaceParameters &wp = request_.workspace_parameters;
    wp.min_corner.x = wp.min_corner.y = wp.min_corner.z = -10;
    wp.max_corner.x = wp.max_corner.y = wp.max_corner.z = 10;

    // Default planning time
    request_.allowed_planning_time = 5.0;
}

void MotionRequestBuilder::setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &wp)
{
    request_.workspace_parameters = wp;
}

void MotionRequestBuilder::setStartConfiguration(const std::vector<double> &joints)
{
    robot_state::RobotState start_state(robot_.getModel());
    start_state.setJointGroupPositions(jmg_, joints);

    moveit::core::robotStateToRobotStateMsg(start_state, request_.start_state);
}

void MotionRequestBuilder::setGoalConfiguration(const std::vector<double> &joints)
{
    robot_state::RobotState goal_state(robot_.getModel());
    goal_state.setJointGroupPositions(jmg_, joints);

    request_.goal_constraints.clear();
    request_.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg_));
}

void MotionRequestBuilder::setGoalRegion(const std::string &ee_name, const std::string &base_name,
                                         const Eigen::Affine3d &pose, const Geometry &geometry,
                                         const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances)
{
    moveit_msgs::Constraints constraints;

    constraints.position_constraints.push_back(TF::getPositionConstraint(ee_name, base_name, pose, geometry));
    constraints.orientation_constraints.push_back(
        TF::getOrientationConstraint(ee_name, base_name, orientation, tolerances));

    request_.goal_constraints.clear();
    request_.goal_constraints.push_back(constraints);
}

const planning_interface::MotionPlanRequest &MotionRequestBuilder::getRequest()
{
    return request_;
}

planning_interface::MotionPlanResponse PipelinePlanner::plan(Scene &scene,
                                                             const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse response;
    if (pipeline_)
        pipeline_->generatePlan(scene.getScene(), request, response);

    return response;
}

void OMPL::Settings::setParam(IO::Handler &handler) const
{
    const std::string prefix = "ompl/";
    handler.setParam(prefix + "max_goal_samples", max_goal_samples);
    handler.setParam(prefix + "max_goal_sampling_attempts", max_goal_sampling_attempts);
    handler.setParam(prefix + "max_planning_threads", max_planning_threads);
    handler.setParam(prefix + "max_solution_segment_length", max_solution_segment_length);
    handler.setParam(prefix + "max_state_sampling_attempts", max_state_sampling_attempts);
    handler.setParam(prefix + "minimum_waypoint_count", minimum_waypoint_count);
    handler.setParam(prefix + "simplify_solutions", simplify_solutions);
    handler.setParam(prefix + "use_constraints_approximations", use_constraints_approximations);
    handler.setParam(prefix + "display_random_valid_states", display_random_valid_states);
    handler.setParam(prefix + "link_for_exploration_tree", link_for_exploration_tree);
    handler.setParam(prefix + "maximum_waypoint_distance", maximum_waypoint_distance);
}

const std::vector<std::string>                                                                                //
    OMPL::OMPLPipelinePlanner::DEFAULT_ADAPTERS({"default_planner_request_adapters/AddTimeParameterization",  //
                                                 "default_planner_request_adapters/FixWorkspaceBounds",       //
                                                 "default_planner_request_adapters/FixStartStateBounds",      //
                                                 "default_planner_request_adapters/FixStartStateCollision",   //
                                                 "default_planner_request_adapters/FixStartStatePathConstraints"});

const std::string PLANNER_CONFIGS = "planner_configs";

OMPL::OMPLPipelinePlanner::OMPLPipelinePlanner(Robot &robot) : PipelinePlanner(robot)
{
}

bool OMPL::OMPLPipelinePlanner::initialize(const std::string &config_file, const OMPL::Settings settings,
                                           const std::string &plugin, const std::vector<std::string> &adapters)
{
    if (!config_file.empty())
    {
        auto &config = IO::loadFileToYAML(config_file);
        if (!config.first)
        {
            ROS_ERROR("Failed to load planner configs.");
            return false;
        }

        handler_.loadYAMLtoROS(config.second);
    }

    handler_.setParam("planning_plugin", plugin);

    std::stringstream ss;
    for (std::size_t i = 0; i < adapters.size(); ++i)
    {
        ss << adapters[i];
        if (i < adapters.size() - 1)
            ss << " ";
    }

    handler_.setParam("request_adapters", ss.str());
    settings.setParam(handler_);

    pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_.getModel(), handler_.getHandle(), "planning_plugin",
                                                            "request_adapters"));

    return true;
}

OMPL::OMPLInterfacePlanner::OMPLInterfacePlanner(Robot &robot)
  : Planner(robot), interface_(robot.getModel(), robot.getHandler().getHandle())
{
}

bool OMPL::OMPLInterfacePlanner::initialize(const std::string &config_file, const OMPL::Settings settings)
{
    if (!config_file.empty())
    {
        auto &config = IO::loadFileToYAML(config_file);
        if (!config.first)
        {
            ROS_ERROR("Failed to load planner configs.");
            return false;
        }

        handler_.loadYAMLtoROS(config.second);
    }

    settings.setParam(handler_);
    return true;
}

planning_interface::MotionPlanResponse
OMPL::OMPLInterfacePlanner::plan(Scene &scene, const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse response;
    response.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;

    ompl_interface::ModelBasedPlanningContextPtr context = interface_.getPlanningContext(scene.getScene(), request);

    if (!context)
        return response;

    context->clear();
    bool result = context->solve(response);

    return response;
}
