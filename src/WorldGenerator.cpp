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

namespace hunav
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

WorldGenerator::WorldGenerator() : Node("hunav_gazebo_world_generator")
{
  // fill with the names of agent parameters
  params_ = { ".id",
              ".skin",
              ".behavior.type",
              ".behavior.configuration",
              ".behavior.duration",
              ".behavior.once",
              ".behavior.vel",
              ".behavior.dist",
              ".behavior.social_force_factor",
              ".behavior.goal_force_factor",
              ".behavior.obstacle_force_factor",
              ".behavior.other_force_factor",
              ".group_id",
              ".max_vel",
              ".radius",
              ".init_pose.x",
              ".init_pose.y",
              ".init_pose.z",
              ".init_pose.h",
              ".goal_radius",
              ".cyclic_goals",
              ".goals" };
  // names of the goal parameters
  goal_params_ = {".x", ".y"}; //.h

  agents_srv_ = this->create_service<hunav_msgs::srv::GetAgents>(
      std::string("get_agents"), std::bind(&hunav::WorldGenerator::getAgentsService, this, _1, _2));
  // agents_srv_ = this->create_service<hunav_msgs::srv::GetAgents>(
  //    std::string("get_agents"), &hunav::WorldGenerator::getAgentsService);

  // Read the plugin parameters
  readPluginParams();
  // Read the agents parameters
  readAgentParams();
  // Generate the world
  processXML();
}

WorldGenerator::~WorldGenerator()
{
}

void WorldGenerator::readPluginParams()
{
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
  base_world_ = this->declare_parameter<std::string>("base_world", std::string("bookstore.world"));
  plug_use_gazebo_obs_ = this->declare_parameter<bool>("use_gazebo_obs", false);
  plug_use_collision_ = this->declare_parameter<bool>("use_collision", false);
  plug_update_rate_ = this->declare_parameter<double>("update_rate", 100.0);
  plug_robot_name_ = this->declare_parameter<std::string>("robot_name", std::string("robot"));
  RCLCPP_INFO(this->get_logger(), "Robot name: %s", plug_robot_name_.c_str());
  plug_global_frame_ = this->declare_parameter<std::string>("global_frame_to_publish", std::string("map"));
  plug_use_navgoal_to_start_ = this->declare_parameter<bool>("use_navgoal_to_start", false);
  plug_navgoal_topic_ = this->declare_parameter<std::string>("navgoal_topic", std::string("goal_pose"));
  this->declare_parameter(std::string("ignore_models"), rclcpp::ParameterType::PARAMETER_STRING);
  rclcpp::Parameter ig_models = this->get_parameter("ignore_models");
  std::string models = ig_models.as_string();
  // RCLCPP_INFO(this->get_logger(), "Ignore_models string: %s", models.c_str());
  const char delim = ' ';
  tokenize(models, delim, plug_ignore_models_);
  for (std::string st : plug_ignore_models_)
  {
    RCLCPP_INFO(this->get_logger(), "Ignore_model: %s", st.c_str());
  }
}

void WorldGenerator::readAgentParams()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "hunav_loader");
  while (!parameters_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(this->get_logger(), "Reading parameters...");

  // To check the available parameters
  // auto result = parameters_client->list_parameters({}, 3);
  // for (auto &name : result.names) {
  //   RCLCPP_INFO(this->get_logger(), "Disponible: %s", name.c_str());
  // }


  auto parameters = parameters_client->get_parameters({ "map", "agents"});

  std::string map = parameters[0].value_to_string();

  //std::cout << "map parameter: " << map << std::endl;
  RCLCPP_INFO(this->get_logger(), "map parameter: %s", map.c_str());
  //std::cout << "agent names: " << parameters[1].value_to_string() << std::endl << std::endl;
  RCLCPP_INFO(this->get_logger(), "agent names: %s", parameters[1].value_to_string().c_str());

  //   for (auto &parameter : parameters) {
  //     std::cout << "\nParameter name: " << parameter.get_name() << std::endl;
  //     std::cout << "Parameter value (" << parameter.get_type_name()
  //               << "): " << parameter.value_to_string() << std::endl;
  //   }

  //auto global_goals = parameters[2].as_string_array();

  auto agent_names = parameters[1].as_string_array();
  for (std::string an : agent_names)
  {
    //std::cout << "agent name: " << an << std::endl;
    RCLCPP_INFO(this->get_logger(), "agent name: %s", an.c_str());
    std::vector<std::string> agent_params = params_;
    for (unsigned int i = 0; i < params_.size(); i++)
    {
      agent_params[i] = an + agent_params[i];
      // std::cout << "agent_params " << i << ": " << agent_params[i] <<
      // std::endl;
      RCLCPP_INFO(this->get_logger(), "agent_params %d: %s", i, agent_params[i].c_str());
    }
    auto aparams = parameters_client->get_parameters({agent_params});
    //auto aparams = parameters_client->get_parameters({an});
    

    // {"agent1.id": {"type": "integer", "value": "1"},
    //"agent1.skin": {"type": "integer", "value": "0"},
    //"agent1.behavior.type": {"type": "integer", "value": "4"},
    //"agent1.behavior.configuration": {"type": "integer", "value": "0"},
    //"agent1.behavior.duration": {"type": "double", "value": "40.000000"},
    //"agent1.behavior.once": {"type": "bool", "value": "true"},
    //"agent1.behavior.vel": {"type": "double", "value": "0.600000"},
    //"agent1.behavior.dist": {"type": "double", "value": "3.000000"},
    //"agent1.behavior.social_force_factor": {"type": "double", "value": "5.000000"},
    //"agent1.behavior.goal_force_factor": {"type": "double", "value": "2.000000"},
    //"agent1.behavior.obstacle_force_factor": {"type": "double", "value": "10.000000"},
    //"agent1.behavior.other_force_factor": {"type": "double", "value": "20.000000"},
    //"agent1.group_id": {"type": "integer", "value": "-1"},
    //"agent1.max_vel": {"type": "double", "value": "1.500000"},
    //"agent1.radius": {"type": "double", "value": "0.400000"},
    //"agent1.init_pose.x": {"type": "double", "value": "3.484565"},
    //"agent1.init_pose.y": {"type": "double", "value": "6.934117"},
    //"agent1.init_pose.z": {"type": "double", "value": "1.250000"},
    //"agent1.init_pose.h": {"type": "double", "value": "0.000000"},
    //"agent1.goal_radius": {"type": "double", "value": "0.300000"},
    //"agent1.cyclic_goals": {"type": "bool", "value": "true"},
    //"agent1.goals": {"type": "string_array", "value": "[g0, g1, g2]"}}

    RCLCPP_INFO(this->get_logger(), "Agent parameters for %s:", an.c_str());
    RCLCPP_INFO(this->get_logger(), "  %li parameters found", aparams.size());
    for (unsigned int i = 0; i < aparams.size(); i++)
    {
      RCLCPP_INFO(this->get_logger(), "  %s: %s", aparams[i].get_name().c_str(),
                  aparams[i].value_to_string().c_str());
      // std::cout << "  " << aparams[i].get_name() << ": " << aparams[i].value_to_string() << std::endl;
    }
    // std::cout << "aparams: " << aparams << std::endl;
    hunav_msgs::msg::Agent a;
    a.name = an;
    // std::cout << "aparams[0]: " << aparams[0] << std::endl;
    a.id = aparams[0].as_int();
    // std::cout << "id: " << a.id << std::endl;
    RCLCPP_INFO(this->get_logger(), "Agent id: %d", a.id);
    a.type = hunav_msgs::msg::Agent::PERSON;
    // std::cout << "aparams[1]: " << aparams[1] << std::endl;
    a.skin = aparams[1].as_int();
    // std::cout << "skin: " << a.skin << std::endl;
    RCLCPP_INFO(this->get_logger(), "Agent skin: %d", a.skin);

    // behavior
    //type
    std::string behavior_type = aparams[2].as_string();
    if (behavior_type == "Regular")
        a.behavior.type = hunav_msgs::msg::AgentBehavior::BEH_REGULAR;
    else if(behavior_type == "Impassive")
        a.behavior.type = hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE;
    else if(behavior_type == "Surprised")
        a.behavior.type = hunav_msgs::msg::AgentBehavior::BEH_SURPRISED;
    else if(behavior_type == "Scared")
        a.behavior.type = hunav_msgs::msg::AgentBehavior::BEH_SCARED;
    else if(behavior_type == "Curious")
        a.behavior.type = hunav_msgs::msg::AgentBehavior::BEH_CURIOUS;
    else if(behavior_type == "Threatening")
        a.behavior.type = hunav_msgs::msg::AgentBehavior::BEH_THREATENING;
    else{
        RCLCPP_WARN(this->get_logger(), "Unknown behavior type: %s, defaulting to Regular", behavior_type.c_str());
        a.behavior.type = hunav_msgs::msg::AgentBehavior::BEH_REGULAR;
    }
    //a.behavior.type = aparams[2].as_int();
    a.behavior.configuration = aparams[3].as_int();
    a.behavior.duration = aparams[4].as_double();
    a.behavior.once = aparams[5].as_bool();
    a.behavior.vel = aparams[6].as_double();
    a.behavior.dist = aparams[7].as_double();
    a.behavior.social_force_factor = aparams[8].as_double();
    a.behavior.goal_force_factor = aparams[9].as_double();
    a.behavior.obstacle_force_factor = aparams[10].as_double();
    a.behavior.other_force_factor = aparams[11].as_double();

    // std::cout << "aparams[12]: " << aparams[12] << std::endl;
    a.group_id = aparams[12].as_int();
    // std::cout << "group_id: " << a.group_id << std::endl;
    // std::cout << "aparams[13]: " << aparams[13] << std::endl;
    a.desired_velocity = aparams[13].as_double();
    // std::cout << "desired vel: " << a.desired_velocity << std::endl;
    // std::cout << "aparams[14]: " << aparams[14] << std::endl;
    a.radius = aparams[14].as_double();
    // std::cout << "radius: " << a.radius << std::endl;

    // init pose
    a.position.position.x = aparams[15].as_double();
    a.position.position.y = aparams[16].as_double();
    a.position.position.z = aparams[17].as_double();
    a.yaw = aparams[18].as_double();
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, a.yaw);
    a.position.orientation = tf2::toMsg(myQuaternion);
    a.goal_radius = aparams[19].as_double();
    a.cyclic_goals = aparams[20].as_bool();

    std::cout << "id: " << a.id << " skin:" << (int)a.skin << " group_id:" << (int)a.group_id
              << " max_vel:" << a.desired_velocity << " radius:" << a.radius << std::endl
              << " initpose.x:" << a.position.position.x << " initpose.y:" << a.position.position.y << std::endl;
    std::cout << "Behavior:" << std::endl;
    std::cout << "type:" << (int)a.behavior.type << " configuration:" << (int)a.behavior.configuration
              << " duration:" << a.behavior.duration << " once:" << a.behavior.once << " vel:" << a.behavior.vel
              << " dist:" << a.behavior.dist << " goal_force_factor:" << a.behavior.goal_force_factor
              << " obstacle_force_factor:" << a.behavior.obstacle_force_factor
              << " social_force_factor:" << a.behavior.social_force_factor
              << " other_force_factor:" << a.behavior.other_force_factor << std::endl;

    //auto goal_names = aparams[21].as_string_array();
    auto goal_names = aparams[21].as_integer_array();
    for (auto goal : goal_names)
    {
      std::vector<std::string> gnames = goal_params_;
      for (unsigned int i = 0; i < goal_params_.size(); i++)
      {
        gnames[i] = "global_goals." + std::to_string(goal) + goal_params_[i];
      }
      auto gparams = parameters_client->get_parameters({ gnames });
      geometry_msgs::msg::Pose p;
      p.position.x = gparams[0].as_double();
      p.position.y = gparams[1].as_double();
      tf2::Quaternion quat;
      quat.setRPY(0, 0, 0); //gparams[2].as_double()
      p.orientation = tf2::toMsg(quat);
      a.goals.push_back(p);
      std::cout << "goal: " << goal << " x:" << p.position.x << " y:" << p.position.y << std::endl;
    }

    agents_.agents.push_back(a);
  }
}

void WorldGenerator::getAgentsService(const std::shared_ptr<hunav_msgs::srv::GetAgents::Request> ,
                                      std::shared_ptr<hunav_msgs::srv::GetAgents::Response> response)
{
  // int r = request->empty;
  response->agents = agents_;
  std::cout << "Sending " << agents_.agents.size() << " agents to agent_manager" << std::endl;
  std::cout << "Shutting down WorldGenerator..." << std::endl;
  // turn off the node since we do not use it anymore
  rclcpp::shutdown();
}

bool WorldGenerator::processXML()
{
  // std::cout << base_world_ << std::endl;

  std::string skin_filename[] = { "elegant_man.dae", "casual_man.dae", "elegant_woman.dae", "regular_man.dae",
                                  "worker_man.dae",  "walk.dae",       "walk-green.dae",    "walk-blue.dae",
                                  "walk-red.dae",    "stand.dae" };
  std::string animation_filename[] = { "07_01-walk.bvh",           "69_02_walk_forward.bvh",   "137_28-normal_wait.bvh",
                                       "142_01-walk_childist.bvh", "07_04-slow_walk.bvh",      "02_01-walk.bvh",
                                       "142_17-walk_scared.bvh",   "17_01-walk_with_anger.bvh" };

  // load the base world file
  tinyxml2::XMLDocument doc;
  const char* path = base_world_.c_str();
  auto err = doc.LoadFile(path);

  if (err != tinyxml2::XML_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "\nCould not open world file: %s", base_world_.c_str());
    RCLCPP_ERROR(this->get_logger(), "Please, check that the world file does not contain any comment!\n");
    return false;
  }

  // CREATE PHYSICS TAG
  // <physics type="ode">
  //   <max_step_size>0.01</max_step_size>
  //   <real_time_factor>1</real_time_factor>
  //   <real_time_update_rate>100</real_time_update_rate>
  // </physics>
  tinyxml2::XMLElement* physics_tag = doc.NewElement("physics");
  physics_tag->SetAttribute("type", "ode");
  tinyxml2::XMLElement* max_step = doc.NewElement("max_step_size");
  max_step->SetText(0.001);  // 0.01
  tinyxml2::XMLElement* time_factor = doc.NewElement("real_time_factor");
  time_factor->SetText(1);
  tinyxml2::XMLElement* time_rate = doc.NewElement("real_time_update_rate");
  time_rate->SetText(1000);  // 100

  // Check if the tag <physics> exists
  tinyxml2::XMLElement* physics =
      doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("physics");

  // if does not exist, we add it
  if (physics == nullptr)
  {
    // we add it
    // Insert plugin in the XML
    doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertFirstChild(physics_tag);

    tinyxml2::XMLElement* phy = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("physics");
    phy->InsertFirstChild(max_step);
    phy->InsertAfterChild(max_step, time_factor);
    phy->InsertAfterChild(time_factor, time_rate);
  }
  else
  {
    tinyxml2::XMLElement* phy = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("physics");
    XMLElement* mss = phy->FirstChildElement("max_step_size");
    if (mss == nullptr)
    {
      phy->InsertFirstChild(max_step);
      mss = phy->FirstChildElement("max_step_size");
    }
    mss->SetText(0.001);  // 0.01

    XMLElement* rtf = phy->FirstChildElement("real_time_factor");
    if (rtf == nullptr)
    {
      phy->InsertAfterChild(max_step, time_factor);
      rtf = phy->FirstChildElement("real_time_factor");
    }
    rtf->SetText(1);

    XMLElement* rtur = phy->FirstChildElement("real_time_update_rate");
    if (rtur == nullptr)
    {
      phy->InsertAfterChild(time_factor, time_rate);
      rtur = phy->FirstChildElement("real_time_update_rate");
    }
    rtur->SetText(1000);  // 100
  }

  // CREATE PLUGIN TAG
  tinyxml2::XMLElement* pNewPlugin = doc.NewElement("plugin");
  pNewPlugin->SetAttribute("name", "hunav_plugin");
  pNewPlugin->SetAttribute("filename", "libHuNavPlugin.so");

  tinyxml2::XMLElement* pUpdate = doc.NewElement("update_rate");
  pUpdate->SetText(plug_update_rate_);

  tinyxml2::XMLElement* pRobot = doc.NewElement("robot_name");
  pRobot->SetText(plug_robot_name_.c_str());

  tinyxml2::XMLElement* pGazebo = doc.NewElement("use_gazebo_obs");
  pGazebo->SetText(plug_use_gazebo_obs_);

  tinyxml2::XMLElement* pCollision = doc.NewElement("use_collision");
  pCollision->SetText(plug_use_collision_);

  tinyxml2::XMLElement* pGlobalFrame = doc.NewElement("global_frame_to_publish");
  pGlobalFrame->SetText(plug_global_frame_.c_str());

  tinyxml2::XMLElement* pUseGoal = doc.NewElement("use_navgoal_to_start");
  pUseGoal->SetText(plug_use_navgoal_to_start_);
  tinyxml2::XMLElement* pGoalTopic = doc.NewElement("navgoal_topic");
  pGoalTopic->SetText(plug_navgoal_topic_.c_str());

  tinyxml2::XMLElement* pIgnoreModels = doc.NewElement("ignore_models");
  std::vector<tinyxml2::XMLElement*> pModels;
  for (std::string st : plug_ignore_models_)
  {
    // tinyxml2::XMLElement *pm = doc.NewElement("model");
    pModels.push_back(doc.NewElement("model"));
    pModels.back()->SetText(st.c_str());
  }

  // Insert plugin in the XML
  doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertFirstChild(pNewPlugin);

  tinyxml2::XMLElement* plugin = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("plugin");
  plugin->InsertFirstChild(pUpdate);
  plugin->InsertAfterChild(pUpdate, pRobot);
  plugin->InsertAfterChild(pRobot, pGazebo);
  plugin->InsertAfterChild(pGazebo, pCollision);
  plugin->InsertAfterChild(pGazebo, pGlobalFrame);
  plugin->InsertAfterChild(pGlobalFrame, pUseGoal);
  plugin->InsertAfterChild(pUseGoal, pGoalTopic);
  plugin->InsertAfterChild(pGoalTopic, pIgnoreModels);
  tinyxml2::XMLElement* ignore =
      doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("plugin")->FirstChildElement(
          "ignore_models");

  if (!pModels.empty())
  {
    ignore->InsertFirstChild(pModels[0]);
    for (size_t i = 1; i < pModels.size(); i++)
      ignore->InsertAfterChild(pModels[i - 1], pModels[i]);
  }

  // CREATE ACTORS
  bool first = true;
  for (auto a : agents_.agents)
  {
    tinyxml2::XMLElement* pNewActor = doc.NewElement("actor");
    pNewActor->SetAttribute("name", a.name.c_str());

    // pose format = "x y z r p y"
    tinyxml2::XMLElement* pPose = doc.NewElement("pose");
    std::string pose = std::to_string(a.position.position.x) + " " + std::to_string(a.position.position.y) + " " +
                       std::to_string(a.position.position.z) + " 0 0 " + std::to_string(a.yaw);
    pPose->SetText(pose.c_str());

    tinyxml2::XMLElement* pSkin = doc.NewElement("skin");
    tinyxml2::XMLElement* pFilename = doc.NewElement("filename");
    pFilename->SetText(skin_filename[a.skin].c_str());
    tinyxml2::XMLElement* pScale = doc.NewElement("scale");
    pScale->SetText(1.0f);

    // Set no_active animation
    tinyxml2::XMLElement* pAnimation = doc.NewElement("animation");
    pAnimation->SetAttribute("name", "no_active");

    tinyxml2::XMLElement* pFilename1 = doc.NewElement("filename");

    if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_REGULAR ||
        a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED ||
        a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
    {
      pFilename1->SetText(animation_filename[0].c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE)
    {
      pFilename1->SetText(animation_filename[1].c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
    {
      pFilename1->SetText(animation_filename[5].c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
    {
      pFilename1->SetText(animation_filename[1].c_str());
    }

    tinyxml2::XMLElement* pScale1 = doc.NewElement("scale");
    pScale1->SetText("1.0");
    tinyxml2::XMLElement* pInterpolate = doc.NewElement("interpolate_x");
    pInterpolate->SetText("true");

    // Set active animation
    tinyxml2::XMLElement* pAnimation1 = doc.NewElement("animation");
    pAnimation1->SetAttribute("name", "active");

    tinyxml2::XMLElement* pFilename2 = doc.NewElement("filename");

    if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_REGULAR)
    {
      pFilename2->SetText(animation_filename[0].c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE)
    {
      pFilename2->SetText(animation_filename[1].c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED)
    {
      pFilename2->SetText(animation_filename[2].c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
    {
      pFilename2->SetText(animation_filename[7].c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
    {
      pFilename2->SetText(animation_filename[6].c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
    {
      pFilename2->SetText(animation_filename[4].c_str());
    }

    tinyxml2::XMLElement* pScale2 = doc.NewElement("scale");
    pScale2->SetText("1.0");
    tinyxml2::XMLElement* pInterpolate1 = doc.NewElement("interpolate_x");
    pInterpolate1->SetText("true");

    // Insert actor in the XML
    if (first)
    {
      first = false;
      tinyxml2::XMLElement* pInclude =
          doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("physics");
      doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertAfterChild(pInclude, pNewActor);

      tinyxml2::XMLElement* actors =
          doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("actor");
      actors->InsertFirstChild(pPose);
      actors->InsertAfterChild(pPose, pSkin);
      tinyxml2::XMLElement* skin =
          doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("actor")->FirstChildElement(
              "skin");
      skin->InsertFirstChild(pFilename);
      skin->InsertAfterChild(pFilename, pScale);

      // Insert no_active animation
      actors->InsertAfterChild(pSkin, pAnimation);
      tinyxml2::XMLElement* animation =
          doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("actor")->FirstChildElement(
              "animation");
      animation->InsertFirstChild(pFilename1);
      animation->InsertAfterChild(pFilename1, pScale1);

      // Insert active animation
      actors->InsertAfterChild(animation, pAnimation1);
      tinyxml2::XMLElement* animation_active =
          doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("actor")->LastChildElement(
              "animation");
      animation_active->InsertFirstChild(pFilename2);
      animation_active->InsertAfterChild(pFilename2, pScale2);
      animation_active->InsertAfterChild(pScale2, pInterpolate1);
    }
    else
    {
      tinyxml2::XMLElement* pInclude =
          doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor");
      doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertAfterChild(pInclude, pNewActor);

      tinyxml2::XMLElement* actors =
          doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor");
      actors->InsertFirstChild(pPose);
      actors->InsertAfterChild(pPose, pSkin);
      tinyxml2::XMLElement* skin =
          doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor")->FirstChildElement(
              "skin");
      skin->InsertFirstChild(pFilename);
      skin->InsertAfterChild(pFilename, pScale);

      // Insert no_active animation
      actors->InsertAfterChild(pSkin, pAnimation);
      tinyxml2::XMLElement* animation =
          doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor")->FirstChildElement(
              "animation");
      animation->InsertFirstChild(pFilename1);
      animation->InsertAfterChild(pFilename1, pScale1);

      // Insert active animation
      actors->InsertAfterChild(animation, pAnimation1);
      tinyxml2::XMLElement* animation_active =
          doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor")->LastChildElement(
              "animation");
      animation_active->InsertFirstChild(pFilename2);
      animation_active->InsertAfterChild(pFilename2, pScale2);
      animation_active->InsertAfterChild(pScale2, pInterpolate1);
    }
    if (plug_use_collision_)
    {
      
      tinyxml2::XMLElement* pNewModel = doc.NewElement("model");
      pNewModel->SetAttribute("name", (a.name + "_body").c_str());
    
      tinyxml2::XMLElement* modelPose = doc.NewElement("pose");
      modelPose->SetText(pose.c_str());
      pNewModel->InsertEndChild(modelPose);
    
      tinyxml2::XMLElement* link = doc.NewElement("link");
      link->SetAttribute("name", "base_link");
    
      tinyxml2::XMLElement* inertial = doc.NewElement("inertial");
      tinyxml2::XMLElement* mass = doc.NewElement("mass");
      mass->SetText("5.0");
      inertial->InsertEndChild(mass);
      link->InsertEndChild(inertial);
    
      tinyxml2::XMLElement* collision = doc.NewElement("collision");
      collision->SetAttribute("name", "collision");
      tinyxml2::XMLElement* geom = doc.NewElement("geometry");
      tinyxml2::XMLElement* cylinder = doc.NewElement("cylinder");
      tinyxml2::XMLElement* radius = doc.NewElement("radius");
      tinyxml2::XMLElement* length = doc.NewElement("length");
      radius->SetText(a.radius);  // agent radius
      cylinder->InsertEndChild(radius);
      cylinder->InsertEndChild(length);
      length->SetText("1.60");  // agent height
      geom->InsertEndChild(cylinder);
      collision->InsertEndChild(geom);

      link->InsertEndChild(collision);
    
      pNewModel->InsertEndChild(link);
      doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertEndChild(pNewModel);
    }
  }

  // save new world file
  std::size_t found = base_world_.find_last_of("/\\");
  // std::cout << " path: " << base_world_.substr(0, found + 1) << '\n';
  // std::cout << " file: " << base_world_.substr(found + 1) << '\n';
  std::string new_world = base_world_.substr(0, found) + "/" + "generatedWorld.world";
  if (doc.SaveFile(new_world.c_str()) == tinyxml2::XML_SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "New world file created! (%s)", new_world.c_str());
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Error while saving new world file!");
    return false;
  }
  return true;
}

}  // namespace hunav

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hunav::WorldGenerator>());

  rclcpp::shutdown();
  return 0;
}
