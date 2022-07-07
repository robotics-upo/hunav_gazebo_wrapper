/***********************************************************************/
/**                                                                    */
/** WorldGenerator.cpp                                                 */
/**                                                                    */
/** Copyright (c) 2022, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#include "hunav_gazebo_wrapper/WorldGenerator.hpp"
//#include <ament_index_cpp/get_package_prefix.hpp>
//#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hunav {

using namespace std::chrono_literals;

WorldGenerator::WorldGenerator() : Node("hunav_gazebo_world_generator") {

  // fill with the names of agent parameters
  params_ = {".id",          ".skin",        ".behavior",    ".group_id",
             ".max_vel",     ".radius",      ".init_pose.x", ".init_pose.y",
             ".init_pose.z", ".init_pose.h", ".goal_radius", ".cyclic_goals",
             ".goals"};
  // names of the goal parameters
  goal_params_ = {".x", ".y", ".h"};

  // Get the path to the worlds directory
  // std::string pkg_shared_dir;
  // try {
  //   pkg_shared_dir =
  //       ament_index_cpp::get_package_share_directory("hunav_gazebo_wrapper");

  // } catch (ament_index_cpp::PackageNotFoundError) {
  //   RCLCPP_ERROR(this->get_logger(),
  //                "Package hunav_gazebo_wrapper not found in dir: %s!!!",
  //                pkg_shared_dir.c_str());
  // }
  // pkg_shared_dir = pkg_shared_dir + "/worlds/";

  // get the world name
  // std::string world_file = this->declare_parameter<std::string>(
  //    "base_world", std::string("empty_cafe.world"));
  // base_world_ = pkg_shared_dir + world_file;

  // Plugin parameters
  base_world_ = this->declare_parameter<std::string>(
      "base_world", std::string("empty_cafe.world"));
  plug_use_gazebo_obs_ = this->declare_parameter<bool>("use_gazebo_obs", false);
  plug_update_rate_ = this->declare_parameter<double>("update_rate", 100.0);
  plug_robot_name_ =
      this->declare_parameter<std::string>("robot_name", std::string("robot"));
  plug_global_frame_ = this->declare_parameter<std::string>(
      "global_frame_to_publish", std::string("map"));
  this->declare_parameter("ignore_models");
  rclcpp::Parameter ig_models = this->get_parameter("ignore_models");
  std::string models = ig_models.as_string();
  RCLCPP_INFO(this->get_logger(), "Ignore_models string: %s", models.c_str());
  const char delim = ' ';
  std::vector<std::string> out;
  tokenize(models, delim, plug_ignore_models_);
  for (std::string st : plug_ignore_models_) {
    RCLCPP_INFO(this->get_logger(), "Ignore_model: %s", st.c_str());
  }
  // call readAgentParams here only for testing!
  readAgentParams();
}

WorldGenerator::~WorldGenerator() {}

void WorldGenerator::readAgentParams() {
  auto parameters_client =
      std::make_shared<rclcpp::SyncParametersClient>(this, "hunav_loader");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(this->get_logger(), "Reading parameters...");
  auto parameters = parameters_client->get_parameters({"map", "agents"});

  std::string map = parameters[0].value_to_string();

  std::cout << "map parameter: " << map << std::endl;
  std::cout << "agent names: " << parameters[1].value_to_string() << std::endl
            << std::endl;

  //   for (auto &parameter : parameters) {
  //     std::cout << "\nParameter name: " << parameter.get_name() << std::endl;
  //     std::cout << "Parameter value (" << parameter.get_type_name()
  //               << "): " << parameter.value_to_string() << std::endl;
  //   }

  hunav_msgs::msg::Agents agents;

  auto agent_names = parameters[1].as_string_array();
  for (std::string an : agent_names) {
    std::cout << "agent name: " << an << std::endl;
    std::vector<std::string> agent_params = params_;
    for (unsigned int i = 0; i < params_.size(); i++) {
      agent_params[i] = an + agent_params[i];
      // std::cout << "agent_params " << i << ": " << agent_params[i] <<
      // std::endl;
    }
    auto aparams = parameters_client->get_parameters(agent_params);
    hunav_msgs::msg::Agent a;
    a.id = aparams[0].as_int();
    a.type = hunav_msgs::msg::Agent::PERSON;
    a.behavior_state = hunav_msgs::msg::Agent::BEH_NO_ACTIVE;
    a.skin = aparams[1].as_int();
    a.behavior = aparams[2].as_int();
    a.group_id = aparams[3].as_int();
    a.desired_velocity = aparams[4].as_double();
    a.radius = aparams[5].as_double();
    a.position.position.x = aparams[6].as_double();
    a.position.position.y = aparams[7].as_double();
    a.position.position.z = aparams[8].as_double();
    a.yaw = aparams[9].as_double();
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, aparams[9].as_double());
    a.position.orientation = tf2::toMsg(myQuaternion);
    a.goal_radius = aparams[10].as_double();
    a.cyclic_goals = aparams[11].as_bool();

    std::cout << "id: " << a.id << " skin:" << (int)a.skin
              << " behavior:" << (int)a.behavior
              << " group_id:" << (int)a.group_id
              << " max_vel:" << a.desired_velocity << " radius:" << a.radius
              << " initpose.x:" << a.position.position.x << std::endl;

    auto goal_names = aparams[12].as_string_array();
    for (std::string goal : goal_names) {
      std::vector<std::string> gnames = goal_params_;
      for (unsigned int i = 0; i < goal_params_.size(); i++) {
        gnames[i] = an + "." + goal + goal_params_[i];
      }
      auto gparams = parameters_client->get_parameters({gnames});
      geometry_msgs::msg::Pose p;
      p.position.x = gparams[0].as_double();
      p.position.y = gparams[1].as_double();
      tf2::Quaternion quat;
      quat.setRPY(0, 0, gparams[2].as_double());
      p.orientation = tf2::toMsg(quat);
      a.goals.push_back(p);
      std::cout << "goal: " << goal << " x:" << p.position.x
                << " y:" << p.position.y << std::endl;
    }

    agents.agents.push_back(a);
  }
}

} // namespace hunav

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hunav::WorldGenerator>());

  rclcpp::shutdown();
  return 0;
}
