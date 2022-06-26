/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Captain Yoshi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Shi Shenglei 
   Desc: Motion planning benchmark node
*/

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io.h>
#include <robowflex_library/log.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>

using namespace robowflex;

int main(int argc, char** argv)
{
    // Startup ROS
    ROS ros(argc, argv, "motion_planning_benchmark");
    ros::NodeHandle pnh("~");

    // Get config
    std::string filename;
    pnh.getParam("config_file", filename);
    YAML::Node node;
    if (!IO::loadFileToYAML(filename, node))
    {
        RBX_INFO("Fail to load the config file.");
        return 0;
    }

    RobotPtr robot;
    ScenePtr scene;
    PlannerPtr planner;
    MotionRequestBuilderPtr request;
    std::vector<std::string> collision_detectors;
    ExperimentPtr experiment;

    {
        // Build robots
        RobotBuilder builder;
        builder.loadResources(node["profiler_config"]["robot"]);
        builder.extendResources(node["extend_resource_config"]["robot"]);
        robot = builder.generateResult();
    }

    {
        // Build collision detectors
        collision_detectors = node["profiler_config"]["collision_detectors"].as<std::vector<std::string>>();
    }

    {  // Build scenes
        SceneBuilder scene_builder;
        scene_builder.loadResources(node["profiler_config"]["scenes"]);
        scene_builder.extendResources(node["extend_resource_config"]["scenes"]);
        scene = scene_builder.generateResult(robot, collision_detectors.back());
    }

    {
        PipelinePlannerBuilder builder;
        builder.loadResources(node["profiler_config"]["planning_pipelines"]);
        builder.extendResources(node["extend_resource_config"]["planning_pipelines"]);
        planner = builder.generateResult(robot);
    }

    request = std::make_shared<MotionRequestBuilder>(robot);
    request->setPlanner(planner);
    {
        // Build MotionPlanRequest
        YAMLDeserializerBuilder<moveit_msgs::MotionPlanRequest> builder;
        builder.loadResources(node["profiler_config"]["requests"]);
        builder.mergeResources(node["profiler_config"]["requests_override"]);
        builder.extendResources(node["extend_resource_config"]["requests"]);
        request->getRequest() = builder.generateResult();

        const planning_interface::MotionPlanRequest req = request->getRequestConst();
        for (auto & obj : req.start_state.attached_collision_objects)
        {
            if (scene->hasObject(obj.object.id))
                scene->removeCollisionObject(obj.object.id);
        }
    }

    {
        BenchmarkBuilder builder;
        builder.loadResources(node["benchmark_config"]);
        experiment = builder.generateResult();
    }

    bool visualize = true;
    pnh.getParam("visualize", visualize);
    if (visualize)
    {
        RBX_INFO("Visualize the problem.");
        IO::RVIZHelper rviz(robot);

        robot_model::RobotStatePtr robot_state = request->getStartConfiguration();
        scene->getScene()->setCurrentState(*robot_state.get());
        rviz.updateScene(scene);
        RBX_INFO("Setted the start position! Press enter to continue ...");
        std::cin.get();

        robot_state = request->getGoalConfiguration();
        scene->getScene()->setCurrentState(*robot_state.get());
        rviz.updateScene(scene);
        RBX_INFO("Setted the goal position! Press enter to continue ...");
        std::cin.get();

        robot_state = request->getStartConfiguration();
        scene->getScene()->setCurrentState(*robot_state.get());
        rviz.updateScene(scene);
        RBX_INFO("Setted the planning problem! Press enter to continue ...");
        std::cin.get();

        planning_interface::MotionPlanResponse res = planner->plan(scene, request->getRequestConst());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            RBX_INFO("No solution found.");
        else 
        {
            RBX_INFO("Solution found and visualize it.");
            rviz.updateTrajectory(res);
        }

        RBX_INFO("Press enter to remove the scene and start the benchmarks.");
        std::cin.get();
        rviz.removeScene();
    }

    std::vector<std::string> planner_ids = node["profiler_config"]["planner_ids"].as<std::vector<std::string>>();
    for (auto & planner_id : planner_ids)
    {
        request->setConfig(planner_id);
        experiment->addQuery(planner_id, scene, planner, request->getRequestConst());
    }

    auto dataset = experiment->benchmark();

    RBX_INFO("Benchmark process finished. Save the dataset.");
    std::string savefilename;
    pnh.getParam("save_file", savefilename);
    OMPLPlanDataSetOutputter output(savefilename);
    output.dump(*dataset);

    return 0;
}
