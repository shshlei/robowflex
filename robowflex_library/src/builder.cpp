/* Author: Zachary Kingston */

#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <robowflex_library/builder.h>
#include <robowflex_library/constants.h>
#include <robowflex_library/random.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/util.h>

using namespace robowflex;

// Typical name for RRTConnect configuration in MoveIt
const std::string MotionRequestBuilder::DEFAULT_CONFIG = "RRTConnectkConfigDefault";

MotionRequestBuilder::MotionRequestBuilder(const RobotConstPtr &robot) : robot_(robot)
{
    initialize();
}

MotionRequestBuilder::MotionRequestBuilder(const RobotConstPtr &robot, const std::string &group_name,
                                           const std::string &planner_config)
  : MotionRequestBuilder(robot)
{
    setPlanningGroup(group_name);

    if (not planner_config.empty())
        setConfig(planner_config);
}

MotionRequestBuilder::MotionRequestBuilder(const PlannerConstPtr &planner, const std::string &group_name,
                                           const std::string &planner_config)
  : MotionRequestBuilder(planner->getRobot())
{
    setPlanningGroup(group_name);
    setPlanner(planner);

    if (not planner_config.empty())
        setConfig(planner_config);
}

MotionRequestBuilder::MotionRequestBuilder(const MotionRequestBuilder &other)
  : MotionRequestBuilder(other.getRobot())
{
    request_ = other.getRequestConst();

    const auto &planner = other.getPlanner();
    if (planner)
        setPlanner(planner);
}

MotionRequestBuilderPtr MotionRequestBuilder::clone() const
{
    return std::make_shared<MotionRequestBuilder>(*this);
}

void MotionRequestBuilder::initialize()
{
    setConfig(DEFAULT_CONFIG);

    setWorkspaceBounds(Eigen::Vector3d::Constant(-constants::default_workspace_bound),
                       Eigen::Vector3d::Constant(constants::default_workspace_bound));
    request_.allowed_planning_time = constants::default_allowed_planning_time;
}

void MotionRequestBuilder::setPlanner(const PlannerConstPtr &planner)
{
    const auto &rname = robot_->getName();
    const auto &pname = planner->getRobot()->getName();

    if (rname != pname)
    {
        RBX_ERROR("Conflicting robots `%s` and `%s` in request builder!", rname, pname);
        throw Exception(1, "Invalid planner!");
    }

    planner_ = planner;
}

void MotionRequestBuilder::setPlanningGroup(const std::string &group_name)
{
    const auto &model = robot_->getModelConst();

    if (model->hasJointModelGroup(group_name))
    {
        group_name_ = group_name;
        jmg_ = robot_->getModelConst()->getJointModelGroup(group_name_);

        request_.group_name = group_name_;
    }
    else
    {
        RBX_ERROR("Joint group `%s` does not exist in robot!", group_name);
        throw Exception(1, "Invalid joint group name!");
    }
}

bool MotionRequestBuilder::setConfig(const std::string &requested_config)
{
    if (not planner_)
    {
        RBX_INFO("No planner set! Using requested config `%s`", requested_config);
        request_.planner_id = requested_config;
        return true;
    }

    const auto &configs = planner_->getPlannerConfigs();

    std::vector<std::reference_wrapper<const std::string>> matches;
    for (const auto &config : configs)
    {
        if (config.find(requested_config) != std::string::npos)
            matches.emplace_back(config);
    }

    if (matches.empty())
        return false;

    const auto &found =
        std::min_element(matches.begin(), matches.end(),
                         [](const std::string &a, const std::string &b) { return a.size() < b.size(); });

    incrementVersion();

    request_.planner_id = *found;
    RBX_INFO("Requested Config: `%s`: Using planning config `%s`", requested_config, request_.planner_id);
    return true;
}

void MotionRequestBuilder::setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &wp)
{
    incrementVersion();
    request_.workspace_parameters = wp;
}

void MotionRequestBuilder::setWorkspaceBounds(const Eigen::Ref<const Eigen::VectorXd> &min,
                                              const Eigen::Ref<const Eigen::VectorXd> &max)
{
    moveit_msgs::WorkspaceParameters wp;
    wp.min_corner.x = min[0];
    wp.min_corner.y = min[1];
    wp.min_corner.z = min[2];
    wp.max_corner.x = max[0];
    wp.max_corner.y = max[1];
    wp.max_corner.z = max[2];

    setWorkspaceBounds(wp);
}

bool MotionRequestBuilder::swapStartWithGoal()
{
    if (request_.goal_constraints.size() != 1)
    {
        RBX_ERROR("Multiple goal constraints exist, cannot swap start with goal");
        return false;
    }

    if (request_.goal_constraints[0].joint_constraints.empty())
    {
        RBX_ERROR("No joint goal is specified, cannot swap start with goal");
        return false;
    }

    const auto &start = getStartConfiguration();
    const auto &goal = getGoalConfiguration();
    clearGoals();

    setStartConfiguration(goal);
    setGoalConfiguration(start);
    return true;
}

void MotionRequestBuilder::setStartConfiguration(const std::vector<double> &joints)
{
    if (not jmg_)
    {
        RBX_ERROR("No planning group set!");
        throw Exception(1, "No planning group set!");
    }

    incrementVersion();

    robot_state::RobotState start_state(robot_->getModelConst());
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(jmg_, joints);

    moveit::core::robotStateToRobotStateMsg(start_state, request_.start_state);
}

void MotionRequestBuilder::setStartConfiguration(const robot_state::RobotState &state)
{
    incrementVersion();
    moveit::core::robotStateToRobotStateMsg(state, request_.start_state);
}

void MotionRequestBuilder::setStartConfiguration(const robot_state::RobotStatePtr &state)
{
    setStartConfiguration(*state);
}

void MotionRequestBuilder::useSceneStateAsStart(const SceneConstPtr &scene)
{
    setStartConfiguration(scene->getCurrentStateConst());
}

bool MotionRequestBuilder::attachObjectToStart(ScenePtr scene, const std::string &object)
{
    // Attach object to current start configuration.
    const auto &start = getStartConfiguration();
    if (not scene->attachObject(*start, object))
        return false;

    useSceneStateAsStart(scene);
    return true;
}

bool MotionRequestBuilder::attachObjectToStartConst(const SceneConstPtr &scene, const std::string &object)
{
    auto copy = scene->deepCopy();
    return attachObjectToStart(copy, object);
}

void MotionRequestBuilder::addGoalConfiguration(const std::vector<double> &joints)
{
    if (not jmg_)
    {
        RBX_ERROR("No planning group set!");
        throw Exception(1, "No planning group set!");
    }

    incrementVersion();

    robot_state::RobotStatePtr state;
    state.reset(new robot_state::RobotState(robot_->getModelConst()));
    state->setJointGroupPositions(jmg_, joints);

    addGoalConfiguration(state);
}

void MotionRequestBuilder::addGoalConfiguration(const robot_state::RobotStatePtr &state)
{
    addGoalConfiguration(*state);
}

void MotionRequestBuilder::addGoalConfiguration(const robot_state::RobotState &state)
{
    if (not jmg_)
    {
        RBX_ERROR("No planning group set!");
        throw Exception(1, "No planning group set!");
    }

    incrementVersion();
    request_.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(state, jmg_));
}

void MotionRequestBuilder::addGoalFromIKQuery(const Robot::IKQuery &query)
{
    if (not jmg_)
    {
        RBX_ERROR("No planning group set!");
        throw Exception(1, "No planning group set!");
    }

    if (group_name_ != query.group)
    {
        RBX_ERROR("Planning group in IK query `%1%` not the same as request `%2%`", query.group, group_name_);
        throw Exception(1, "Mismatched query groups!");
    }

    if (query.regions.size() > 1)
    {
        RBX_ERROR("Cannot set goal request from IK query with multiple targets!");
        throw Exception(1, "Tried to set goal from multi-target request!");
    }

    std::string tip_to_use = query.tips[0];
    if (tip_to_use.empty())
    {
        const auto &tips = robot_->getSolverTipFrames(group_name_);
        if (tips.empty() or tips.size() > 1)
        {
            RBX_ERROR("Unable to find tip frame for request.");
            throw Exception(1, "Unable to find tip frame for request.");
        }

        tip_to_use = tips[0];
    }

    const std::string &base = robot_->getSolverBaseFrame(group_name_);
    if (base.empty())
    {
        RBX_ERROR("Failed to get base frame for request.");
        throw Exception(1, "Unable to find base frame for request.");
    }

    addGoalRegion(tip_to_use, base, query.region_poses[0], query.regions[0], query.orientations[0],
                  query.tolerances[0]);
}

void MotionRequestBuilder::addGoalPose(const std::string &ee_name, const std::string &base_name,
                                       const RobotPose &pose, double tolerance)
{
    auto copy = pose;
    Eigen::Quaterniond orientation(pose.rotation());
    copy.linear() = Eigen::Matrix3d::Identity();
    addGoalRegion(ee_name, base_name,                     //
                  copy, Geometry::makeSphere(tolerance),  //
                  orientation, {tolerance, tolerance, tolerance});
}

void MotionRequestBuilder::addGoalRegion(const std::string &ee_name, const std::string &base_name,
                                         const RobotPose &pose, const GeometryConstPtr &geometry,
                                         const Eigen::Quaterniond &orientation,
                                         const Eigen::Vector3d &tolerances)
{
    incrementVersion();

    moveit_msgs::Constraints constraints;

    constraints.position_constraints.push_back(TF::getPositionConstraint(ee_name, base_name, pose, geometry));
    constraints.orientation_constraints.push_back(
        TF::getOrientationConstraint(ee_name, base_name, orientation, tolerances));

    request_.goal_constraints.push_back(constraints);
}

void MotionRequestBuilder::addGoalRotaryTile(const std::string &ee_name, const std::string &base_name,
                                             const RobotPose &pose, const GeometryConstPtr &geometry,
                                             const Eigen::Quaterniond &orientation,
                                             const Eigen::Vector3d &tolerances, const RobotPose &offset,
                                             const Eigen::Vector3d &axis, unsigned int n)
{
    for (double angle = 0; angle < constants::two_pi; angle += constants::two_pi / n)
    {
        Eigen::Quaterniond rotation(Eigen::AngleAxisd(angle, axis));
        RobotPose new_pose = pose * rotation * offset;
        Eigen::Quaterniond new_orientation(rotation * orientation);

        addGoalRegion(ee_name, base_name, new_pose, geometry, new_orientation, tolerances);
    }
}

void MotionRequestBuilder::addCylinderSideGrasp(const std::string &ee_name, const std::string &base_name,
                                                const RobotPose &pose, const GeometryConstPtr &cylinder,
                                                double distance, double depth, unsigned int n)
{
    // Grasping region to tile
    auto box = Geometry::makeBox(depth, depth, cylinder->getDimensions()[1]);
    RobotPose offset(Eigen::Translation3d(cylinder->getDimensions()[0] + distance, 0, 0));

    Eigen::Quaterniond orientation = Eigen::AngleAxisd(-constants::pi, Eigen::Vector3d::UnitX())  //
                                     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())             //
                                     * Eigen::AngleAxisd(constants::pi, Eigen::Vector3d::UnitZ());

    addGoalRotaryTile(ee_name, base_name,                          //
                      pose, box, orientation, {0.01, 0.01, 0.01},  //
                      offset, Eigen::Vector3d::UnitZ(), n);
}

void MotionRequestBuilder::setGoalConfiguration(const std::vector<double> &joints)
{
    clearGoals();
    addGoalConfiguration(joints);
}

void MotionRequestBuilder::setGoalConfiguration(const robot_state::RobotStatePtr &state)
{
    clearGoals();
    addGoalConfiguration(state);
}

void MotionRequestBuilder::setGoalConfiguration(const robot_state::RobotState &state)
{
    clearGoals();
    addGoalConfiguration(state);
}

void MotionRequestBuilder::setGoalFromIKQuery(const Robot::IKQuery &query)
{
    clearGoals();
    addGoalFromIKQuery(query);
}

void MotionRequestBuilder::setGoalPose(const std::string &ee_name, const std::string &base_name,
                                       const RobotPose &pose, double tolerance)
{
    clearGoals();
    addGoalPose(ee_name, base_name, pose, tolerance);
}

void MotionRequestBuilder::setGoalRegion(const std::string &ee_name, const std::string &base_name,
                                         const RobotPose &pose, const GeometryConstPtr &geometry,
                                         const Eigen::Quaterniond &orientation,
                                         const Eigen::Vector3d &tolerances)
{
    clearGoals();
    addGoalRegion(ee_name, base_name, pose, geometry, orientation, tolerances);
}

void MotionRequestBuilder::precomputeGoalConfigurations(std::size_t n_samples, const ScenePtr &scene,
                                                        const ConfigurationValidityCallback &callback)
{
    // Allocate samplers for each region
    constraint_samplers::ConstraintSamplerManager manager;
    std::vector<constraint_samplers::ConstraintSamplerPtr> samplers;
    for (const auto &goal : request_.goal_constraints)
    {
        samplers.emplace_back(manager.selectSampler(scene->getSceneConst(), group_name_, goal));
        samplers.back()->setGroupStateValidityCallback(scene->getGSVCF(false));
    }

    clearGoals();

    // Clone start
    robot_state::RobotState state = *robot_->getScratchStateConst();

    // Sample n_samples to add to new request
    std::size_t n = n_samples;
    while (n)
    {
        auto sampler = RNG::uniformSample(samplers);
        if (sampler->sample(state) and (not callback or callback(state)))
        {
            addGoalConfiguration(state);
            n--;
        }
    }
}

void MotionRequestBuilder::clearGoals()
{
    incrementVersion();
    request_.goal_constraints.clear();
}

void MotionRequestBuilder::setAllowedPlanningTime(double allowed_planning_time)
{
    incrementVersion();
    request_.allowed_planning_time = allowed_planning_time;
}

void MotionRequestBuilder::setNumPlanningAttempts(unsigned int num_planning_attempts)
{
    incrementVersion();
    request_.num_planning_attempts = num_planning_attempts;
}

void MotionRequestBuilder::addPathPoseConstraint(const std::string &ee_name, const std::string &base_name,
                                                 const RobotPose &pose, const GeometryConstPtr &geometry,
                                                 const Eigen::Quaterniond &orientation,
                                                 const Eigen::Vector3d &tolerances)
{
    addPathPositionConstraint(ee_name, base_name, pose, geometry);
    addPathOrientationConstraint(ee_name, base_name, orientation, tolerances);
}

void MotionRequestBuilder::addPathPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                                     const RobotPose &pose, const GeometryConstPtr &geometry)
{
    incrementVersion();
    request_.path_constraints.position_constraints.push_back(
        TF::getPositionConstraint(ee_name, base_name, pose, geometry));
}

void MotionRequestBuilder::addPathOrientationConstraint(const std::string &ee_name,
                                                        const std::string &base_name,
                                                        const Eigen::Quaterniond &orientation,
                                                        const Eigen::Vector3d &tolerances)
{
    incrementVersion();
    request_.path_constraints.orientation_constraints.push_back(
        TF::getOrientationConstraint(ee_name, base_name, orientation, tolerances));
}

moveit_msgs::Constraints &MotionRequestBuilder::getPathConstraints()
{
    incrementVersion();
    return request_.path_constraints;
}

planning_interface::MotionPlanRequest &MotionRequestBuilder::getRequest()
{
    incrementVersion();
    return request_;
}

robot_state::RobotStatePtr MotionRequestBuilder::getStartConfiguration() const
{
    auto start_state = robot_->allocState();

    moveit::core::robotStateMsgToRobotState(request_.start_state, *start_state);
    start_state->update(true);

    return start_state;
}

robot_state::RobotStatePtr MotionRequestBuilder::getGoalConfiguration() const
{
    auto goal_state = robot_->allocState();

    if (request_.goal_constraints.size() != 1)
    {
        RBX_ERROR("Ambiguous goal, %lu goal goal_constraints exist, returning default goal",
                  request_.goal_constraints.size());
        return goal_state;
    }

    if (request_.goal_constraints[0].joint_constraints.empty())
    {
        RBX_ERROR("No joint constraints specified, returning default goal");
        return goal_state;
    }

    std::map<std::string, double> variable_map;
    for (const auto &joint : request_.goal_constraints[0].joint_constraints)
        variable_map[joint.joint_name] = joint.position;

    // Start state includes attached objects and values for the non-group links.
    moveit::core::robotStateMsgToRobotState(request_.start_state, *goal_state);
    goal_state->setVariablePositions(variable_map);
    goal_state->update(true);

    return goal_state;
}

const planning_interface::MotionPlanRequest &MotionRequestBuilder::getRequestConst() const
{
    return request_;
}

bool MotionRequestBuilder::toYAMLFile(const std::string &file) const
{
    return IO::YAMLToFile(IO::toNode(request_), file);
}

bool MotionRequestBuilder::fromYAMLFile(const std::string &file)
{
    incrementVersion();
    return IO::fromYAMLFile(request_, file);
}

bool MotionRequestBuilder::fromYAMLNode(const YAML::Node& node)
{
    incrementVersion();
    request_ = node.as<moveit_msgs::MotionPlanRequest>();
    return true;
}

const RobotConstPtr &MotionRequestBuilder::getRobot() const
{
    return robot_;
}

const PlannerConstPtr &MotionRequestBuilder::getPlanner() const
{
    return planner_;
}

const std::string &MotionRequestBuilder::getPlanningGroup() const
{
    return group_name_;
}

const std::string &MotionRequestBuilder::getPlannerConfig() const
{
    return request_.planner_id;
}

Builder::~Builder()
{
}

void Builder::reset()
{
    node_map_.clear();
}

const std::map<std::string, YAML::Node>& Builder::getResources() const
{
    return node_map_;
}

void Builder::insertResource(const std::string name, const YAML::Node& node)
{
    auto it = node_map_.find(name);
    if (it == node_map_.end())
        node_map_.insert({ name, node });
    else
        ROS_WARN("Resource name `%s` already in map.", name.c_str());
}

bool Builder::validateResource(const YAML::Node& node)
{
    return true;
}

bool Builder::cloneResource(const std::string& src, const std::string& dst)
{
    auto it_src = node_map_.find(src);
    if (it_src == node_map_.end())
    {
        ROS_WARN("Invalid resource name `%s`.", src.c_str());
        return false;
    }

    auto it_dst = node_map_.find(dst);
    if (it_dst != node_map_.end())
    {
        ROS_WARN("Resource name exists already `%s`.", dst.c_str());
        return false;
    }

    node_map_.insert({ dst, YAML::Clone(it_src->second) });
    return true;
}

bool Builder::deleteResource(const std::string& name)
{
    node_map_.erase(name);
    return true;
}

void Builder::decodeResourceTag(const YAML::Node& source, YAML::Node& target)
{
    // resource is a filename or a namespace
    if (source.IsScalar())
    {
        const auto& filename = source.as<std::string>();

        std::string path = IO::resolvePath(filename);

        // resource is a namespace
        if (path.empty())
        {
            IO::Handler handler("");
            handler.loadROStoYAML(filename, target);

            if (target.IsNull())
                ROS_WARN("Cannot resolve PATH or ROS NAMESPACE from resource `%s`.", filename.c_str());
        }
        // resource is either a YAML or XML file
        else
        {
            // try YAML first (check extension)
            auto pair = IO::loadFileToYAML(filename);
            if (pair.first)
                target = pair.second;
            else
            {
                // try XML or XACRO (cannot check extension)
                std::string xml_str = IO::loadXMLToString(filename);
                if (xml_str.empty())
                    ROS_WARN("Cannot load YAML, XML or XACRO from resource `%s`.", filename.c_str());
                else
                    target = xml_str;
            }
        }
    }
    // keep resource as a YAML node
    else
        target = source;
}

void Builder::loadResources(const YAML::Node& node)
{
    if (node.IsMap())
        loadResource(node);
    else if (node.IsSequence())
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
            loadResource(*it);
    else
        ROS_WARN("YAML node MUST be a sequence or a map");
}

void Builder::loadResource(const YAML::Node& node)
{
    YAML::Node n;
    std::string name;

    if (node["name"])
        name = node["name"].as<std::string>();
    else
        return;

    // Load resource/s key
    if (node["resource"])
    {
        YAML::Node temp;
        decodeResourceTag(node["resource"], temp);
        if (!temp.IsNull())
            n["resource"] = temp;
    }
    else if (node["resources"])
    {
        for (YAML::const_iterator it2 = node["resources"].begin(); it2 != node["resources"].end(); ++it2)
        {
            YAML::Node temp;

            const auto& key = it2->first.as<std::string>();
            decodeResourceTag(it2->second, temp);
            if (!temp.IsNull())
                n["resources"][key] = temp;
        }
    }

    // Load parameters key, keep as YAML
    if (node["parameters"])
        n["parameters"] = node["parameters"];

    if (node["settings"])
        n["settings"] = node["settings"];

    if (node["plugin"])
        n["plugin"] = node["plugin"];

    if (node["adapters"])
        n["adapters"] = node["adapters"];

    if (!validateResource(n))
    {
        ROS_WARN("Resource name `%s` did not pass validation", name.c_str());
        return;
    }

    if (!n.IsNull())
        insertResource(name, n);
}

void Builder::mergeResources(const YAML::Node& node)
{
    for (auto& resource : node_map_)
        mergeResource(resource.first, node);
}

void Builder::mergeResource(const std::string& name, const YAML::Node& node)
{
    auto it = node_map_.find(name);
    if (it == node_map_.end())
    {
        ROS_WARN("Invalid resource name `%s`, cannot merge node.", name.c_str());
        return;
    }

    if (it->second["resource"])
        yaml::merge_node(it->second["resource"], node);
    else if (it->second["resources"])
        yaml::merge_node(it->second["resources"], node);

    if (!validateResource(it->second))
        ROS_ERROR("Merge failed for resource name `%s`.", name.c_str());
}

void Builder::extendResources(const YAML::Node& node)
{
    std::vector<std::string> resource_names;

    if (node.IsMap())
        extendResource(node, resource_names);
    else if (node.IsSequence())
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
            extendResource(*it, resource_names);
    else if (node.IsDefined())
        ROS_WARN("Resource extension failed, YAML node MUST be a sequence or a map");

    // Remove original resource used for extension
    for (const auto& name : resource_names)
        node_map_.erase(name);
}

void Builder::extendResource(const YAML::Node& node, std::vector<std::string>& resource_names)
{
    if (!node["name"] || !node["resources"] || !node["resources"].IsSequence())
    {
        ROS_ERROR("Invalid extend config. Needs `name` and `resources` keys and the later MUST be a YAML sequence.");
        return;
    }

    std::size_t seq_idx = 0;
    const auto& name = node["name"].as<std::string>();

    // name must be in the map
    auto it1 = node_map_.find(name);
    if (it1 == node_map_.end())
    {
        ROS_ERROR("Cannot extend missing resource name `%s`", name.c_str());
        return;
    }

    resource_names.push_back(name);

    for (YAML::const_iterator it2 = node["resources"].begin(); it2 != node["resources"].end(); ++it2)
    {
        YAML::Node source;
        YAML::Node target = YAML::Clone(it1->second);
        ++seq_idx;

        decodeResourceTag(*it2, source);

        // merge
        if (target["resource"])
            yaml::merge_node(target["resource"], source);
        else if (target["resources"])
            yaml::merge_node(target["resources"], source);
        else
        {
            ROS_WARN("Resource has no resource/s key");
            continue;
        }

        if (source.IsNull())
            return;

        if (!source.IsNull())
        {
            if (!validateResource(target))
            {
                ROS_ERROR("Failed to extend resource name `%s` (sequence number %zu). Check config file.", name.c_str(),
                        seq_idx);
                return;
            }

            // insert with new name
            std::string extended_name = name + "_" + std::to_string(seq_idx);

            if (node_map_.find(extended_name) != node_map_.end())
            {
                ROS_ERROR("Cannot extend resource name `%s`. Name already.", extended_name.c_str());
                return;
            }

            insertResource(extended_name, target);
        }
    }
}

//
// RobotBuilder
//

RobotPtr RobotBuilder::generateResult() const
{
    RobotPtr result;
    const auto& node_map = getResources();

    for (const auto& pair : node_map)
    {
        auto robot = std::make_shared<Robot>(pair.first);

        if (robot->initializeFromYAML(pair.second["resources"]))
        {
            result = robot;
            break;
        }
    }

    return result;
}

std::map<std::string, RobotPtr> RobotBuilder::generateResults() const
{
    std::map<std::string, RobotPtr> results;
    const auto& node_map = getResources();

    for (const auto& pair : node_map)
    {
        auto robot = std::make_shared<Robot>(pair.first);

        if (robot->initializeFromYAML(pair.second["resources"]))
            results.insert({ pair.first, robot });
    }

    return results;
}

//
// SceneBuilder
//

ScenePtr SceneBuilder::generateResult(const RobotPtr& robot, const std::string& collision_detector) const
{
    ScenePtr result;
    const auto& node_map = getResources();

    for (const auto& pair : node_map)
    {
        bool success = true;

        auto scene = std::make_shared<Scene>(robot->getModelConst());
        scene->setName(pair.first);
        if (pair.second["resource"].IsScalar())
            success &= scene->fromURDFString(pair.second["resource"].as<std::string>());
        else
            success &= scene->fromYAMLNode(pair.second["resource"]);
        if (success)
        {
            if (pair.second["parameters"])
            {
                if (pair.second["parameters"]["safety_distance"])
                    scene->getScene()->setSafetyDistance(pair.second["parameters"]["safety_distance"].as<double>());
                if (pair.second["parameters"]["contact_distance"])
                    scene->getScene()->setContactDistanceThreshold(pair.second["parameters"]["contact_distance"].as<double>());
                if (pair.second["parameters"]["negative_distance"])
                    scene->getScene()->setNegativeDistanceThreshold(pair.second["parameters"]["negative_distance"].as<double>());
            }
            success &= scene->setCollisionDetector(collision_detector);
            if (success)
            {
                result = scene;
                break;
            }
        }
    }

    return result;
}

std::map<std::string, ScenePtr> SceneBuilder::generateResults(const RobotPtr& robot, const std::string& collision_detector) const
{
    std::map<std::string, ScenePtr> results;
    const auto& node_map = getResources();
    for (const auto& pair : node_map)
    {
        bool success = true;
        auto scene = std::make_shared<Scene>(robot->getModelConst());
        scene->setName(pair.first);
        if (pair.second["resource"].IsScalar())
            success &= scene->fromURDFString(pair.second["resource"].as<std::string>());
        else
            success &= scene->fromYAMLNode(pair.second["resource"]);
        if (success)
        {
            if (pair.second["parameters"])
            {
                if (pair.second["parameters"]["safety_distance"])
                    scene->getScene()->setSafetyDistance(pair.second["parameters"]["safety_distance"].as<double>());
                if (pair.second["parameters"]["contact_distance"])
                    scene->getScene()->setContactDistanceThreshold(pair.second["parameters"]["contact_distance"].as<double>());
                if (pair.second["parameters"]["negative_distance"])
                    scene->getScene()->setNegativeDistanceThreshold(pair.second["parameters"]["negative_distance"].as<double>());
            }
            success &= scene->setCollisionDetector(collision_detector);
            if (success)
                results.insert({ pair.first, scene });
        }
    }
    return results;
}

//
// Pipelineplannerbuilder 
//

PipelinePlannerPtr PipelinePlannerBuilder::generateResult(const RobotPtr& robot) const
{
    PipelinePlannerPtr result;
    const auto& node_map = getResources();

    for (const auto& pair : node_map)
    {
        if (pair.first == "ompl")
        {
            auto planner = std::make_shared<OMPL::OMPLPipelinePlanner>(robot, pair.first);

            OMPL::Settings settings;
            if (IO::isNode(pair.second["settings"]))
                settings.fromYAMLNode(pair.second["settings"]);

            bool user_plugin = false;
            bool user_adapters = false;
            std::string plugin;
            std::vector<std::string> adapters;
            if (pair.second["plugin"])
            {
                user_plugin = true;
                plugin = pair.second["plugin"].as<std::string>();
            }
            if (pair.second["adapters"])
            {
                user_adapters = true;
                std::vector<std::string> ads = pair.second["adapters"].as<std::vector<std::string>>();
                for (auto & ad : ads)
                    adapters.push_back("default_planner_request_adapters/" + ad);
            }

            if (user_plugin && user_adapters)
            {
                if (planner->initialize(pair.second["resource"], settings, plugin, adapters))
                {
                    result = planner;
                    break;
                }
            }
            else
            {
                if (planner->initialize(pair.second["resource"], settings))
                {
                    result = planner;
                    break;
                }
            }
        }
        else if (pair.first == "chomp")
        {
            auto planner = std::make_shared<opt::CHOMPPipelinePlanner>(robot, pair.first);

            if (planner->initialize(pair.second["resource"]))
            {
                result = planner;
                break;
            }
        }
        else if (pair.first == "trajopt")
        {
            auto planner = std::make_shared<opt::TrajOptPipelinePlanner>(robot, pair.first);

            if (planner->initialize(pair.second["resource"]))
            {
                result = planner;
                break;
            }
        }
    }

    return result;
}

std::map<std::string, PipelinePlannerPtr> PipelinePlannerBuilder::generateResults(const RobotPtr& robot) const
{
    std::map<std::string, PipelinePlannerPtr> results;
    const auto& node_map = getResources();

    for (const auto& pair : node_map)
    {
        if (pair.first == "ompl")
        {
            auto planner = std::make_shared<OMPL::OMPLPipelinePlanner>(robot, pair.first);

            OMPL::Settings settings;
            if (IO::isNode(pair.second["settings"]))
                settings.fromYAMLNode(pair.second["settings"]);

            if (planner->initialize(pair.second["resource"], settings))
                results.insert({ pair.first, planner });
        }
        else if (pair.first == "chomp")
        {
            auto planner = std::make_shared<opt::CHOMPPipelinePlanner>(robot, pair.first);

            if (planner->initialize(pair.second["resource"]))
                results.insert({ pair.first, planner });
        }
        else if (pair.first == "trajopt")
        {
            auto planner = std::make_shared<opt::TrajOptPipelinePlanner>(robot, pair.first);

            if (planner->initialize(pair.second["resource"]))
                results.insert({ pair.first, planner });
        }
    }

    return results;
}

//
// BenchmarkBuilder
//

ExperimentPtr BenchmarkBuilder::generateResult() const
{
    ExperimentPtr result;
    const auto& node_map = getResources();

    for (const auto& pair : node_map)
    {
        auto experiment = std::make_shared<Experiment>(pair.first);

        if (experiment->initializeFromYAML(pair.second["parameters"]))
        {
            result = experiment;
            break;
        }
    }

    return result;
}

std::map<std::string, ExperimentPtr> BenchmarkBuilder::generateResults() const
{
    std::map<std::string, ExperimentPtr> results;
    const auto& node_map = getResources();

    for (const auto& pair : node_map)
    {
        auto experiment = std::make_shared<Experiment>(pair.first);

        if (experiment->initializeFromYAML(pair.second["parameters"]))
            results.insert({ pair.first, experiment });
    }

    return results;
}
