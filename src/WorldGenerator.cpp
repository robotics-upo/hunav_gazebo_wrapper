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
/** of the MIT license. See the LICENSE file for details.              */
/**                                                                    */
/**                                                                    */
/***********************************************************************/

#include "hunav_gazebo_wrapper/WorldGenerator.hpp"
//#include <ament_index_cpp/get_package_prefix.hpp>
//#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace tinyxml2;

namespace hunav {

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

WorldGenerator::WorldGenerator() : Node("hunav_gazebo_world_generator") {

  // fill with the names of agent parameters
  params_ = {".id",          ".skin",        ".behavior",    ".group_id",
             ".max_vel",     ".radius",      ".init_pose.x", ".init_pose.y",
             ".init_pose.z", ".init_pose.h", ".goal_radius", ".cyclic_goals",
             ".goals"};
  // names of the goal parameters
  goal_params_ = {".x", ".y", ".h"};

  agents_srv_ = this->create_service<hunav_msgs::srv::GetAgents>(
      std::string("get_agents"),
      std::bind(&hunav::WorldGenerator::getAgentsService, this, _1, _2));
  // agents_srv_ = this->create_service<hunav_msgs::srv::GetAgents>(
  //    std::string("get_agents"), &hunav::WorldGenerator::getAgentsService);

  // Read the plugin parameters
  readPluginParams();
  // Read the agents parameters
  readAgentParams();
  // Generate the world
  processXML();
}

WorldGenerator::~WorldGenerator() {}

void WorldGenerator::readPluginParams() {

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
      "base_world", std::string("bookstore.world"));
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
  tokenize(models, delim, plug_ignore_models_);
  for (std::string st : plug_ignore_models_) {
    RCLCPP_INFO(this->get_logger(), "Ignore_model: %s", st.c_str());
  }
}

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
    a.name = an;
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

    agents_.agents.push_back(a);
  }
}

void WorldGenerator::getAgentsService(
    const std::shared_ptr<hunav_msgs::srv::GetAgents::Request> request,
    std::shared_ptr<hunav_msgs::srv::GetAgents::Response> response) {

  int r = request->empty;
  response->agents = agents_;
  // turn off the node since we do not use it anymore
  rclcpp::shutdown();
}

bool WorldGenerator::processXML() {
  // std::cout << base_world_ << std::endl;

  std::string skin_filename[] = {"walk.dae", "walk-green.dae", "walk-blue.dae",
                                 "walk-red.dae", "stand.dae"};
  std::string animation_filename[] = {
      "07_01-walk.bvh",         "142_04-walk_cool.bvh",
      "141_20-waiting.bvh",     "142_01-walk_childist.bvh",
      "120_19-walk_slow.bvh",   "142_08-walk_happy.bvh",
      "142_17-walk_scared.bvh", "17_01-walk_with_anger.bvh"};

  // load the base world file
  tinyxml2::XMLDocument doc;
  const char *path = base_world_.c_str();
  auto err = doc.LoadFile(path);

  if (err != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "\nCould not open world file: %s",
                 base_world_.c_str());
    RCLCPP_ERROR(
        this->get_logger(),
        "Please, check that the world file does not contain any comment!\n",
        base_world_.c_str());
    return false;
  }

  // CREATE PHYSICS TAG
  // <physics type="ode">
  //   <max_step_size>0.01</max_step_size>
  //   <real_time_factor>1</real_time_factor>
  //   <real_time_update_rate>100</real_time_update_rate>
  // </physics>
  tinyxml2::XMLElement *physics_tag = doc.NewElement("physics");
  physics_tag->SetAttribute("type", "ode");
  tinyxml2::XMLElement *max_step = doc.NewElement("max_step_size");
  max_step->SetText(0.01);
  tinyxml2::XMLElement *time_factor = doc.NewElement("real_time_factor");
  time_factor->SetText(1);
  tinyxml2::XMLElement *time_rate = doc.NewElement("real_time_update_rate");
  time_rate->SetText(100);

  // Check if the tag <physics> exists
  tinyxml2::XMLElement *physics = doc.FirstChildElement("sdf")
                                      ->FirstChildElement("world")
                                      ->FirstChildElement("physics");

  // if does not exist, we add it
  if (physics == nullptr) {
    // we add it
    // Insert plugin in the XML
    doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertFirstChild(
        physics_tag);

    tinyxml2::XMLElement *phy = doc.FirstChildElement("sdf")
                                    ->FirstChildElement("world")
                                    ->FirstChildElement("physics");
    phy->InsertFirstChild(max_step);
    phy->InsertAfterChild(max_step, time_factor);
    phy->InsertAfterChild(time_factor, time_rate);

  } else {
    tinyxml2::XMLElement *phy = doc.FirstChildElement("sdf")
                                    ->FirstChildElement("world")
                                    ->FirstChildElement("physics");
    XMLElement *mss = phy->FirstChildElement("max_step_size");
    if (mss == nullptr) {
      phy->InsertFirstChild(max_step);
      mss = phy->FirstChildElement("max_step_size");
    }
    mss->SetText(0.01);

    XMLElement *rtf = phy->FirstChildElement("real_time_factor");
    if (rtf == nullptr) {
      phy->InsertAfterChild(max_step, time_factor);
      rtf = phy->FirstChildElement("real_time_factor");
    }
    rtf->SetText(1);

    XMLElement *rtur = phy->FirstChildElement("real_time_update_rate");
    if (rtur == nullptr) {
      phy->InsertAfterChild(time_factor, time_rate);
      rtur = phy->FirstChildElement("real_time_update_rate");
    }
    rtur->SetText(100);

  }

  // CREATE PLUGIN TAG
  tinyxml2::XMLElement *pNewPlugin = doc.NewElement("plugin");
  pNewPlugin->SetAttribute("name", "hunav_plugin");
  pNewPlugin->SetAttribute("filename", "libHuNavPlugin.so");

  tinyxml2::XMLElement *pUpdate = doc.NewElement("update_rate");
  pUpdate->SetText(plug_update_rate_);

  tinyxml2::XMLElement *pRobot = doc.NewElement("robot_name");
  pRobot->SetText(plug_robot_name_.c_str());

  tinyxml2::XMLElement *pGazebo = doc.NewElement("use_gazebo_obs");
  pGazebo->SetText(plug_use_gazebo_obs_);

  tinyxml2::XMLElement *pGlobalFrame =
      doc.NewElement("global_frame_to_publish");
  pGlobalFrame->SetText(plug_global_frame_.c_str());

  tinyxml2::XMLElement *pIgnoreModels = doc.NewElement("ignore_models");
  std::vector<tinyxml2::XMLElement *> pModels;
  for (std::string st : plug_ignore_models_) {
    // tinyxml2::XMLElement *pm = doc.NewElement("model");
    pModels.push_back(doc.NewElement("model"));
    pModels.back()->SetText(st.c_str());
  }

  // Insert plugin in the XML
  doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertFirstChild(
      pNewPlugin);

  tinyxml2::XMLElement *plugin = doc.FirstChildElement("sdf")
                                     ->FirstChildElement("world")
                                     ->FirstChildElement("plugin");
  plugin->InsertFirstChild(pUpdate);
  plugin->InsertAfterChild(pUpdate, pRobot);
  plugin->InsertAfterChild(pRobot, pGazebo);
  plugin->InsertAfterChild(pGazebo, pGlobalFrame);
  plugin->InsertAfterChild(pGlobalFrame, pIgnoreModels);
  tinyxml2::XMLElement *ignore = doc.FirstChildElement("sdf")
                                     ->FirstChildElement("world")
                                     ->FirstChildElement("plugin")
                                     ->FirstChildElement("ignore_models");

  if (!pModels.empty()) {
    ignore->InsertFirstChild(pModels[0]);
    for (size_t i = 1; i < pModels.size(); i++)
      ignore->InsertAfterChild(pModels[i - 1], pModels[i]);
  }

  // CREATE ACTORS
  bool first = true;
  for (auto a : agents_.agents) {
    tinyxml2::XMLElement *pNewActor = doc.NewElement("actor");
    pNewActor->SetAttribute("name", a.name.c_str());

    // pose format = "x y z r p y"
    tinyxml2::XMLElement *pPose = doc.NewElement("pose");
    std::string pose = std::to_string(a.position.position.x) + " " +
                       std::to_string(a.position.position.y) + " " +
                       std::to_string(a.position.position.z) + " 0 0 " +
                       std::to_string(a.yaw);
    pPose->SetText(pose.c_str());

    tinyxml2::XMLElement *pSkin = doc.NewElement("skin");
    tinyxml2::XMLElement *pFilename = doc.NewElement("filename");
    pFilename->SetText(skin_filename[a.skin].c_str());
    tinyxml2::XMLElement *pScale = doc.NewElement("scale");
    pScale->SetText(1.0f);

    // Set no_active animation
    tinyxml2::XMLElement *pAnimation = doc.NewElement("animation");
    pAnimation->SetAttribute("name", "no_active");

    tinyxml2::XMLElement *pFilename1 = doc.NewElement("filename");

    if (a.behavior == hunav_msgs::msg::Agent::BEH_REGULAR ||
        a.behavior == hunav_msgs::msg::Agent::BEH_SURPRISED ||
        a.behavior == hunav_msgs::msg::Agent::BEH_THREATENING) {
      pFilename1->SetText(animation_filename[0].c_str());
    } else if (a.behavior == hunav_msgs::msg::Agent::BEH_IMPASSIVE) {
      pFilename1->SetText(animation_filename[1].c_str());
    } else if (a.behavior == hunav_msgs::msg::Agent::BEH_SCARED) {
      pFilename1->SetText(animation_filename[5].c_str());
    } else if (a.behavior == hunav_msgs::msg::Agent::BEH_CURIOUS) {
      pFilename1->SetText(animation_filename[1].c_str());
    }

    tinyxml2::XMLElement *pScale1 = doc.NewElement("scale");
    pScale1->SetText("1.0");
    tinyxml2::XMLElement *pInterpolate = doc.NewElement("interpolate_x");
    pInterpolate->SetText("true");

    // Set active animation
    tinyxml2::XMLElement *pAnimation1 = doc.NewElement("animation");
    pAnimation1->SetAttribute("name", "active");

    tinyxml2::XMLElement *pFilename2 = doc.NewElement("filename");

    if (a.behavior == hunav_msgs::msg::Agent::BEH_REGULAR) {
      pFilename2->SetText(animation_filename[0].c_str());
    } else if (a.behavior == hunav_msgs::msg::Agent::BEH_IMPASSIVE) {
      pFilename2->SetText(animation_filename[1].c_str());
    } else if (a.behavior == hunav_msgs::msg::Agent::BEH_SURPRISED) {
      pFilename2->SetText(animation_filename[2].c_str());
    } else if (a.behavior == hunav_msgs::msg::Agent::BEH_THREATENING) {
      pFilename2->SetText(animation_filename[7].c_str());
    } else if (a.behavior == hunav_msgs::msg::Agent::BEH_SCARED) {
      pFilename2->SetText(animation_filename[6].c_str());
    } else if (a.behavior == hunav_msgs::msg::Agent::BEH_CURIOUS) {
      pFilename2->SetText(animation_filename[4].c_str());
    }

    tinyxml2::XMLElement *pScale2 = doc.NewElement("scale");
    pScale2->SetText("1.0");
    tinyxml2::XMLElement *pInterpolate1 = doc.NewElement("interpolate_x");
    pInterpolate1->SetText("true");

    // Insert actor in the XML
    if (first) {
      first = false;
      tinyxml2::XMLElement *pInclude = doc.FirstChildElement("sdf")
                                           ->FirstChildElement("world")
                                           ->LastChildElement("physics");
      doc.FirstChildElement("sdf")
          ->FirstChildElement("world")
          ->InsertAfterChild(pInclude, pNewActor);

      tinyxml2::XMLElement *actors = doc.FirstChildElement("sdf")
                                         ->FirstChildElement("world")
                                         ->FirstChildElement("actor");
      actors->InsertFirstChild(pPose);
      actors->InsertAfterChild(pPose, pSkin);
      tinyxml2::XMLElement *skin = doc.FirstChildElement("sdf")
                                       ->FirstChildElement("world")
                                       ->FirstChildElement("actor")
                                       ->FirstChildElement("skin");
      skin->InsertFirstChild(pFilename);
      skin->InsertAfterChild(pFilename, pScale);

      // Insert no_active animation
      actors->InsertAfterChild(pSkin, pAnimation);
      tinyxml2::XMLElement *animation = doc.FirstChildElement("sdf")
                                            ->FirstChildElement("world")
                                            ->FirstChildElement("actor")
                                            ->FirstChildElement("animation");
      animation->InsertFirstChild(pFilename1);
      animation->InsertAfterChild(pFilename1, pScale1);

      // Insert active animation
      actors->InsertAfterChild(animation, pAnimation1);
      tinyxml2::XMLElement *animation_active =
          doc.FirstChildElement("sdf")
              ->FirstChildElement("world")
              ->FirstChildElement("actor")
              ->LastChildElement("animation");
      animation_active->InsertFirstChild(pFilename2);
      animation_active->InsertAfterChild(pFilename2, pScale2);
      animation_active->InsertAfterChild(pScale2, pInterpolate1);
    } else {
      tinyxml2::XMLElement *pInclude = doc.FirstChildElement("sdf")
                                           ->FirstChildElement("world")
                                           ->LastChildElement("actor");
      doc.FirstChildElement("sdf")
          ->FirstChildElement("world")
          ->InsertAfterChild(pInclude, pNewActor);

      tinyxml2::XMLElement *actors = doc.FirstChildElement("sdf")
                                         ->FirstChildElement("world")
                                         ->LastChildElement("actor");
      actors->InsertFirstChild(pPose);
      actors->InsertAfterChild(pPose, pSkin);
      tinyxml2::XMLElement *skin = doc.FirstChildElement("sdf")
                                       ->FirstChildElement("world")
                                       ->LastChildElement("actor")
                                       ->FirstChildElement("skin");
      skin->InsertFirstChild(pFilename);
      skin->InsertAfterChild(pFilename, pScale);

      // Insert no_active animation
      actors->InsertAfterChild(pSkin, pAnimation);
      tinyxml2::XMLElement *animation = doc.FirstChildElement("sdf")
                                            ->FirstChildElement("world")
                                            ->LastChildElement("actor")
                                            ->FirstChildElement("animation");
      animation->InsertFirstChild(pFilename1);
      animation->InsertAfterChild(pFilename1, pScale1);

      // Insert active animation
      actors->InsertAfterChild(animation, pAnimation1);
      tinyxml2::XMLElement *animation_active =
          doc.FirstChildElement("sdf")
              ->FirstChildElement("world")
              ->LastChildElement("actor")
              ->LastChildElement("animation");
      animation_active->InsertFirstChild(pFilename2);
      animation_active->InsertAfterChild(pFilename2, pScale2);
      animation_active->InsertAfterChild(pScale2, pInterpolate1);
    }
  }

  // save new world file
  std::size_t found = base_world_.find_last_of("/\\");
  // std::cout << " path: " << base_world_.substr(0, found + 1) << '\n';
  // std::cout << " file: " << base_world_.substr(found + 1) << '\n';
  std::string new_world =
      base_world_.substr(0, found) + "/" + "generatedWorld.world";
  if (doc.SaveFile(new_world.c_str()) == tinyxml2::XML_SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "New world file created! (%s)",
                new_world.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error while saving new world file!");
    return false;
  }
  return true;
}

} // namespace hunav

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hunav::WorldGenerator>());

  rclcpp::shutdown();
  return 0;
}
