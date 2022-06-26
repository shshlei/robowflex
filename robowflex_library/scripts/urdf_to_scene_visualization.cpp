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
Desc:
*/

#include <ros/ros.h>

#include <urdf_to_scene/scene_parser.h>

#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/util.h>

using namespace robowflex;

int main(int argc, char** argv)
{
    // Startup ROS
    ROS ros(argc, argv, "urdf_to_scene_visualization");

    IO::RVIZHelper rviz;

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    ros::NodeHandle nh("~");

    // Parse URDF into a planning scene
    SceneParser parser;
    parser.loadURDF(nh, "/scene_urdf");
    parser.parseURDF();

    // Load a scene from a YAML file.
//    auto scene = std::make_shared<Scene>(fetch);
//    scene->fromYAMLFile("package://robowflex_library/yaml/test_fetch.yml");

    // Visualize the scene in RViz.
    rviz.updateScene(parser.getPlanningScene());

    RBX_INFO("Scene displayed! Press enter to plan...");
    std::cin.get();

    RBX_INFO("Press enter to remove scene.");
    std::cin.get();

    // Clean up RViz.
    rviz.removeScene();

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
