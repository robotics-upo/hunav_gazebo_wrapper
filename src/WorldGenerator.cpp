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

using namespace tinyxml2;

namespace hunav
{

  using namespace std::chrono_literals;

  WorldGenerator::WorldGenerator() : Node("hunav_gazebo_world_generator")
  {

    // fill with the names of agent parameters
    params_ = {".id", ".skin", ".behavior", ".group_id",
               ".max_vel", ".radius", ".init_pose.x", ".init_pose.y",
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
    for (std::string st : plug_ignore_models_)
    {
      RCLCPP_INFO(this->get_logger(), "Ignore_model: %s", st.c_str());
    }
    // call readAgentParams here only for testing!
    readAgentParams();
    processXML();
  }

  WorldGenerator::~WorldGenerator() {}

  void WorldGenerator::readAgentParams()
  {

    auto parameters_client =
        std::make_shared<rclcpp::SyncParametersClient>(this, "hunav_loader");
    while (!parameters_client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
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
    for (std::string an : agent_names)
    {
      std::cout << "agent name: " << an << std::endl;
      std::vector<std::string> agent_params = params_;
      for (unsigned int i = 0; i < params_.size(); i++)
      {
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
      for (std::string goal : goal_names)
      {
        std::vector<std::string> gnames = goal_params_;
        for (unsigned int i = 0; i < goal_params_.size(); i++)
        {
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

  bool WorldGenerator::processXML()
  {
    std::cout << base_world_ << std::endl;

    std::string pose[] = {"-1 2 1.25 0 0 0", "-1 -1 1.25 0 0 0", "1 0 1.25 0 0 0", "3 0 1.25 0 0 1.57"};
    std::string skin_filename[] = {"walk-green.dae", "walk-blue.dae", "stand.dae", "walk-red.dae"};
    std::string animation_filename[] = {"walk.dae", "stand.dae", "142_17-walk_scared.bvh", "walk.dae"};

    tinyxml2::XMLDocument doc;
    const char *path = base_world_.c_str();
    auto err = doc.LoadFile(path);

    if (err != tinyxml2::XML_SUCCESS)
    {
      std::cout << "Could not open" << std::endl;
    }
    else
    {
      std::cout << "Start" << std::endl;

      // PLUGIN
      tinyxml2::XMLElement *pNewPlugin = doc.NewElement("plugin");
      pNewPlugin->SetAttribute("name", "hunav_plugin");
      pNewPlugin->SetAttribute("filename", "libHuNavPlugin.so");

      tinyxml2::XMLElement *pUpdate = doc.NewElement("update_rate");
      pUpdate->SetText(100.0f);

      tinyxml2::XMLElement *pRobot = doc.NewElement("robot_name");
      pRobot->SetText("actor3");

      tinyxml2::XMLElement *pGazebo = doc.NewElement("use_gazebo_obs");
      pGazebo->SetText("True");

      tinyxml2::XMLElement *pGlobalFrame = doc.NewElement("global_frame_to_publish");
      pGlobalFrame->SetText("map");

      tinyxml2::XMLElement *pIgnoreModels = doc.NewElement("ignore_models");
      tinyxml2::XMLElement *pModels = doc.NewElement("model");
      pModels->SetText("cafe");
      tinyxml2::XMLElement *pModels1 = doc.NewElement("model");
      pModels1->SetText("ground_plane");

      // Insertar en XML
      doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertFirstChild(pNewPlugin);

      tinyxml2::XMLElement *plugin = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("plugin");
      plugin->InsertFirstChild(pUpdate);
      plugin->InsertAfterChild(pUpdate, pRobot);
      plugin->InsertAfterChild(pRobot, pGazebo);
      plugin->InsertAfterChild(pGazebo, pGlobalFrame);
      plugin->InsertAfterChild(pGlobalFrame, pIgnoreModels);
      tinyxml2::XMLElement *ignore = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("plugin")->FirstChildElement("ignore_models");
      ignore->InsertFirstChild(pModels);
      ignore->InsertAfterChild(pModels, pModels1);

      // ACTORS

      for (int num = 0; num < 4; num++)
      {
        tinyxml2::XMLElement *pNewActor = doc.NewElement("actor");
        std::string aux = "actor" + std::to_string(num);
        pNewActor->SetAttribute("name", aux.c_str());

        tinyxml2::XMLElement *pPose = doc.NewElement("pose");
        pPose->SetText(pose[num].c_str());

        tinyxml2::XMLElement *pSkin = doc.NewElement("skin");
        tinyxml2::XMLElement *pFilename = doc.NewElement("filename");
        pFilename->SetText(skin_filename[num].c_str());
        tinyxml2::XMLElement *pScale = doc.NewElement("scale");
        pScale->SetText(1.0f);

        tinyxml2::XMLElement *pAnimation = doc.NewElement("animation");
        pAnimation->SetAttribute("name", "no_active");
        tinyxml2::XMLElement *pFilename1 = doc.NewElement("filename");
        pFilename1->SetText(animation_filename[num].c_str());
        tinyxml2::XMLElement *pScale1 = doc.NewElement("scale");
        pScale1->SetText("1.000000");
        tinyxml2::XMLElement *pInterpolate = doc.NewElement("interpolate_x");
        pInterpolate->SetText("true");

        if (num == 0)
        {
          tinyxml2::XMLElement *pInclude = doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("include");
          doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertAfterChild(pInclude, pNewActor);

          tinyxml2::XMLElement *actors = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("actor");
          actors->InsertFirstChild(pPose);
          actors->InsertAfterChild(pPose, pSkin);
          tinyxml2::XMLElement *skin = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("actor")->FirstChildElement("skin");
          skin->InsertFirstChild(pFilename);
          skin->InsertAfterChild(pFilename, pScale);

          actors->InsertAfterChild(pSkin, pAnimation);
          tinyxml2::XMLElement *animation = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("actor")->FirstChildElement("animation");
          animation->InsertFirstChild(pFilename1);
          animation->InsertAfterChild(pFilename1, pScale1);
        }
        else
        {
          tinyxml2::XMLElement *pInclude = doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor");
          doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertAfterChild(pInclude, pNewActor);

          tinyxml2::XMLElement *actors = doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor");
          actors->InsertFirstChild(pPose);
          actors->InsertAfterChild(pPose, pSkin);
          tinyxml2::XMLElement *skin = doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor")->FirstChildElement("skin");
          skin->InsertFirstChild(pFilename);
          skin->InsertAfterChild(pFilename, pScale);

          actors->InsertAfterChild(pSkin, pAnimation);
          tinyxml2::XMLElement *animation = doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor")->FirstChildElement("animation");
          animation->InsertFirstChild(pFilename1);
          animation->InsertAfterChild(pFilename1, pScale1);
        }
      }
      
      if (doc.SaveFile("/home/roberott/Desktop/prueba.world") == tinyxml2::XML_SUCCESS)
      {
        std::cout << "Saved" << std::endl;
      }
      else
      {
        std::cout << "Not Saved";
      }
    }
    return true;
  }

} // namespace hunav

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hunav::WorldGenerator>());

  rclcpp::shutdown();
  return 0;
}
