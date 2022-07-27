/***********************************************************************/
/**                                                                    */
/** HuNavPlugin.cpp                                                 */
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

//#include <functional>
#include <stdio.h>
#include <string>

//#include <ignition/math.hh>
//#include <ignition/math/gzmath.hh>
#include <hunav_gazebo_wrapper/HuNavPlugin.h>

#define DEF_WALKING_ANIMATION "walking"

// namespace gazebo_ros {
namespace hunav {

// GZ_REGISTER_MODEL_PLUGIN(HumanNavPlugin)

using namespace std::chrono_literals;

class HuNavPluginPrivate {

public:
  /// Callback when world is updated.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo &_info);
  /// \brief Helper function to collect the pedestrians data from Gazebo
  bool GetPedestrians();
  /// \brief Helper function to collect the robot data from Gazebo
  bool GetRobot();
  /// \brief Helper function to collect obtacles from Gazebo
  void HandleObstacles();
  /// \brief Helper function to update the Gazebo actors
  void UpdateGazeboPedestrians(const gazebo::common::UpdateInfo &_info,
                               const hunav_msgs::msg::Agents &_agents);
  /// \brief load the stored animations for the agents behaviors
  // void LoadAnimations();
  /// \brief Update the animation of the agent according to the behavior
  // void UpdateAnimation(const human_nav_msgs::msg::Agent &_agent);
  /// \brief Helper function to initialize the agents at the initial step
  void InitializeAgents();
  /// \brief Helper function to initialize the robot the initial step
  bool InitializeRobot();
  inline double normalizeAngle(double a) {
    double value = a;
    while (value <= -M_PI)
      value += 2 * M_PI;
    while (value > M_PI)
      value -= 2 * M_PI;
    return value;
  }

  /// \brief ros node required to call the service
  // std::shared_ptr<rclcpp::Node> rosnode;
  gazebo_ros::Node::SharedPtr rosnode;

  /// \brief ros service to update the actor states
  rclcpp::Client<hunav_msgs::srv::ComputeAgents>::SharedPtr rosSrvClient;

  /// \brief ros service to get the initial agents setup
  rclcpp::Client<hunav_msgs::srv::GetAgents>::SharedPtr rosSrvGetAgentsClient;

  /// \brief the robot as a agent msg
  hunav_msgs::msg::Agent robotAgent;

  /// \brief vector of pedestrians detected.
  std::vector<hunav_msgs::msg::Agent> pedestrians;
  // std::vector<ped> pedestrians;

  /// \brief Pointer to the parent actor.
  gazebo::physics::ModelPtr robotModel;

  /// \brief Pointer to the world, for convenience.
  gazebo::physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
  sdf::ElementPtr sdf;

  /// \brief connection to world iteration
  gazebo::event::ConnectionPtr connection;

  /// \brief Time of the last update.
  gazebo::common::Time lastUpdate;

  rclcpp::Time rostime;
  double dt;
  double update_rate_secs;

  std::string robotName;
  std::string globalFrame;

  /// \brief List of models to ignore. Used for vector field
  std::vector<std::string> ignoreModels;
};

/////////////////////////////////////////////////
HuNavPlugin::HuNavPlugin() : hnav_(std::make_unique<HuNavPluginPrivate>()) {}

HuNavPlugin::~HuNavPlugin() {}

/////////////////////////////////////////////////
void HuNavPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) {

  hnav_->world = _world;

  hnav_->rosnode = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(hnav_->rosnode->get_logger(), "Loading HuNav plugin...");

  // Get QoS profiles
  // const gazebo_ros::QoS &qos = hnav_->rosnode->get_qos();
  hnav_->sdf = _sdf;

  if (_sdf->HasElement("robot_name"))
    hnav_->robotName = _sdf->Get<std::string>("robot_name");
  else
    hnav_->robotName = "robot";

  if (_sdf->HasElement("global_frame_to_publish"))
    hnav_->globalFrame = _sdf->Get<std::string>("global_frame_to_publish");
  else
    hnav_->globalFrame = "map";

  // Read models to be ignored
  if (_sdf->HasElement("ignore_models")) {
    sdf::ElementPtr modelElem =
        _sdf->GetElement("ignore_models")->GetElement("model");
    while (modelElem) {
      hnav_->ignoreModels.push_back(modelElem->Get<std::string>());
      RCLCPP_INFO(hnav_->rosnode->get_logger(), "Ignoring model: %s",
                  (modelElem->Get<std::string>()).c_str());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  hnav_->rostime = hnav_->rosnode->get_clock()->now();
  hnav_->dt = 0;
  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    hnav_->update_rate_secs = 1.0 / update_rate;
  } else {
    hnav_->update_rate_secs = 0.0;
  }
  RCLCPP_INFO(hnav_->rosnode->get_logger(), "update_rate: %.2f, secs:%.4f",
              update_rate, hnav_->update_rate_secs);

  hnav_->rosSrvClient =
      hnav_->rosnode->create_client<hunav_msgs::srv::ComputeAgents>(
          "compute_agents");

  hnav_->rosSrvGetAgentsClient =
      hnav_->rosnode->create_client<hunav_msgs::srv::GetAgents>("get_agents");

  hnav_->connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(
      &HuNavPluginPrivate::OnUpdate, hnav_.get(), std::placeholders::_1));

  // Reset();
  hnav_->InitializeAgents();
  hnav_->HandleObstacles();

  hnav_->lastUpdate = _world->SimTime();

  // Reset trajectory so we can move the agents!
  for (size_t i = 0; i < hnav_->pedestrians.size(); ++i) {

    gazebo::physics::ModelPtr model =
        hnav_->world->ModelByName(hnav_->pedestrians[i].name);
    gazebo::physics::ActorPtr actor =
        boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

    gazebo::physics::TrajectoryInfoPtr trajectoryInfo;
    trajectoryInfo.reset(new gazebo::physics::TrajectoryInfo());
    trajectoryInfo->id = 0;
    auto skelAnims = actor->SkeletonAnimations();
    if (!skelAnims.empty()) {
      for (auto it = skelAnims.begin(); it != skelAnims.end(); ++it) {
        RCLCPP_INFO(hnav_->rosnode->get_logger(), "Agent %s has animation: %s",
                    actor->GetName().c_str(), it->first.c_str());
      }
      trajectoryInfo->type = "no_active"; // skelAnims.begin()->first;
    } else {
      RCLCPP_INFO(hnav_->rosnode->get_logger(), "Animation default type: %s",
                  DEF_WALKING_ANIMATION);
      trajectoryInfo->type = DEF_WALKING_ANIMATION;
    }
    trajectoryInfo->duration = 1.0;

    /// \brief Set a custom trajectory for the actor, using one of the
    /// existing animations. This will override any trajectories previously
    /// defined. When a custom trajectory is defined, the script time must
    /// be set with `SetScriptTime` in order to play the animation.
    /// \param[in] _trajInfo Information about custom trajectory.
    /// \sa ResetCustomTrajectory, SetScriptTime
    /// public: void SetCustomTrajectory(TrajectoryInfoPtr &_trajInfo);
    actor->SetCustomTrajectory(trajectoryInfo);
  }
}

// /////////////////////////////////////////////////
// void PedestriansPlugin::Reset() {
//   // this->velocity = 0.8;
//   lastUpdate = 0;

//   // auto skelAnims = this->actor->SkeletonAnimations();
//   // if (skelAnims.find(this->animationName) == skelAnims.end()) {
//   //   gzerr << "Skeleton animation " << this->animationName << " not
//   found.\n";
//   // } else {
//   //   // Create custom trajectory
//   //   this->trajectoryInfo.reset(new physics::TrajectoryInfo());
//   //   this->trajectoryInfo->type = this->animationName;
//   //   this->trajectoryInfo->duration = 1.0;

//   //   this->actor->SetCustomTrajectory(this->trajectoryInfo);
//   // }
// }

bool HuNavPluginPrivate::InitializeRobot() {
  RCLCPP_INFO(rosnode->get_logger(), "Initializing robot...");
  robotModel = world->ModelByName(robotName);
  if (!robotModel) {
    RCLCPP_ERROR(rosnode->get_logger(), "Robot model %s not found (yet)!!!!",
                 robotName.c_str());
    return false;
  } else {
    // Initialize robot agent
    ignition::math::Vector3d pos = robotModel->WorldPose().Pos();
    ignition::math::Vector3d rpy = robotModel->WorldPose().Rot().Euler();
    robotAgent.id = robotModel->GetId();
    robotAgent.type = hunav_msgs::msg::Agent::ROBOT;
    robotAgent.behavior = hunav_msgs::msg::Agent::BEH_REGULAR;
    robotAgent.behavior_state = hunav_msgs::msg::Agent::BEH_NO_ACTIVE;
    robotAgent.name = robotName;
    robotAgent.group_id = -1;
    robotAgent.radius = 0.35;
    robotAgent.position.position.x = pos.X();
    robotAgent.position.position.y = pos.Y();
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, rpy.Z());
    robotAgent.position.orientation = tf2::toMsg(myQuaternion);
    robotAgent.yaw = rpy.Z();
    ignition::math::Vector3d linvel = robotModel->WorldLinearVel();
    robotAgent.velocity.linear.x = linvel.X();
    robotAgent.velocity.linear.y = linvel.Y();
    robotAgent.linear_vel = linvel.Length();
    ignition::math::Vector3d angvel = robotModel->WorldAngularVel();
    robotAgent.velocity.angular.z = angvel.Z();
    robotAgent.angular_vel = angvel.Z();
    return true;
  }
}

void HuNavPluginPrivate::InitializeAgents() {
  RCLCPP_INFO(rosnode->get_logger(), "Initializing agents...");

  InitializeRobot();

  // Fill the service request
  auto request = std::make_shared<hunav_msgs::srv::GetAgents::Request>();
  request->empty = 0;

  // Wait for the service to be available
  while (!rosSrvGetAgentsClient->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rosnode->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(rosnode->get_logger(),
                "service not available, waiting again...");
  }
  // Call the service
  auto result = rosSrvGetAgentsClient->async_send_request(request);
  std::chrono::duration<int, std::milli> ms(200);
  if (result.wait_for(ms) == std::future_status::ready) {

    // Initialize the Gazebo actors
    const hunav_msgs::msg::Agents &agents = result.get()->agents;
    pedestrians.clear();

    for (auto agent : agents.agents) {

      hunav_msgs::msg::Agent ag = agent;

      gazebo::physics::ModelPtr model = world->ModelByName(agent.name);
      if (!model) {
        RCLCPP_ERROR(rosnode->get_logger(), "Pedestrian model %s not found!!!!",
                     agent.name.c_str());
        return;
      }
      ignition::math::Vector3d pos = model->WorldPose().Pos();

      // The front (x axis) of the actors in Gazebo is
      // looking to the person left, we rotate it
      double yaw = normalizeAngle(model->WorldPose().Rot().Yaw() - M_PI_2);
      ag.position.position.x = pos.X();
      ag.position.position.y = pos.Y();
      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, yaw);
      ag.position.orientation = tf2::toMsg(myQuaternion);
      ag.yaw = yaw;
      ignition::math::Vector3d linvel = model->WorldLinearVel();
      ag.velocity.linear.x = linvel.X();
      ag.velocity.linear.y = linvel.Y();
      ag.linear_vel = linvel.Length();
      ignition::math::Vector3d angvel = model->WorldAngularVel();
      ag.velocity.angular.z = angvel.Z();
      ag.angular_vel = angvel.Z();

      pedestrians.push_back(ag);
      RCLCPP_INFO(rosnode->get_logger(),
                  "Adding agent: %s, type: %i, id:%i, beh:%i, x:%.2f, "
                  "y:%.2f, th:%.2f",
                  ag.name.c_str(), ag.type, ag.id, ag.behavior, pos.X(),
                  pos.Y(), yaw);
    }

  } else {
    RCLCPP_ERROR(rosnode->get_logger(), "Failed to call service get_agents");
  }
}

/////////////////////////////////////////////////
void HuNavPluginPrivate::HandleObstacles() {

  for (size_t i = 0; i < pedestrians.size(); ++i) {
    gazebo::physics::ModelPtr agent = world->ModelByName(pedestrians[i].name);

    double minDist = 10000.0;
    ignition::math::Vector3d closest_obstacle;
    // ignition::math::Vector3d closest_obs2;
    pedestrians[i].closest_obs.clear();

    for (size_t j = 0; j < world->ModelCount(); ++j) {
      gazebo::physics::ModelPtr modelObstacle = world->ModelByIndex(j);

      // Avoid to compute the other actors as obstacles
      for (size_t k = 0; k < pedestrians.size(); ++k) {
        if ((int)modelObstacle->GetId() == pedestrians[k].id)
          break;
      }

      // Avoid the agent itself and the indicated models
      if (agent->GetId() != modelObstacle->GetId() &&
          std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                    modelObstacle->GetName()) == this->ignoreModels.end()) {

        ignition::math::Vector3d actorPos = agent->WorldPose().Pos();
        ignition::math::Vector3d obsPos = modelObstacle->WorldPose().Pos();
        std::tuple<bool, double, ignition::math::Vector3d> intersect =
            modelObstacle->BoundingBox().Intersect(obsPos, actorPos, 0.05, 8.0);

        if (std::get<0>(intersect) == true) {

          // ignition::math::Vector3d = model->BoundingBox().Center();
          // double approximated_radius =
          // std::max(model->BoundingBox().XLength(),model->BoundingBox().YLength());

          // ignition::math::Vector3d offset1 = modelPos - actorPos;
          // double modelDist1 = offset1.Length();
          // double dist1 = actorPos.Distance(modelPos);

          ignition::math::Vector3d offset = std::get<2>(intersect) - actorPos;
          double modelDist = offset.Length(); //-approximated_radius;
          // double dist2 = actorPos.Distance(std::get<2>(intersect));

          if (modelDist < minDist) {
            minDist = modelDist;
            // closest_obs = offset;
            closest_obstacle = std::get<2>(intersect);
          }
        }
      }
    }

    // printf("Actor %s x: %.2f y: %.2f\n", this->actor->GetName().c_str(),
    //        this->actor->WorldPose().Pos().X(),
    //        this->actor->WorldPose().Pos().Y());
    // printf("Model offset x: %.2f y: %.2f\n", closest_obs.X(),
    // closest_obs.Y());
    // printf("Model intersec x: %.2f y: %.2f\n\n", closest_obs2.X(),
    //        closest_obs2.Y());
    if (minDist <= 10.0) {
      geometry_msgs::msg::Point p;
      p.x = closest_obstacle.X();
      p.y = closest_obstacle.Y();
      p.z = 0.0;
      pedestrians[i].closest_obs.push_back(p);
    }
  }
}

bool HuNavPluginPrivate::GetRobot() {

  if (!robotModel) {
    if (!InitializeRobot()) {
      RCLCPP_ERROR(rosnode->get_logger(), "Robot model %s not found!!!!",
                   robotName.c_str());
      return false;
    }
    return true;
  }
  // Update robot agent
  ignition::math::Vector3d pos = robotModel->WorldPose().Pos();
  // ignition::math::Vector3d rpy = robotModel->WorldPose().Rot().Euler();
  robotAgent.position.position.x = pos.X();
  robotAgent.position.position.y = pos.Y();
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, robotModel->WorldPose().Rot().Yaw());
  robotAgent.position.orientation = tf2::toMsg(myQuaternion);
  robotAgent.yaw = robotModel->WorldPose().Rot().Yaw();
  ignition::math::Vector3d linvel = robotModel->WorldLinearVel();
  robotAgent.velocity.linear.x = linvel.X();
  robotAgent.velocity.linear.y = linvel.Y();
  robotAgent.linear_vel = linvel.Length();
  ignition::math::Vector3d angvel = robotModel->WorldAngularVel();
  robotAgent.velocity.angular.z = angvel.Z();
  robotAgent.angular_vel = angvel.Z();
  return true;
}

/////////////////////////////////////////////////
bool HuNavPluginPrivate::GetPedestrians() {
  for (unsigned int i = 0; i < pedestrians.size(); ++i) {
    gazebo::physics::ModelPtr model = world->ModelByName(pedestrians[i].name);

    if (!model) {
      RCLCPP_ERROR(rosnode->get_logger(), "Pedestrian model %s not found!!!!",
                   pedestrians[i].name.c_str());
      return false;
    }

    double yaw = normalizeAngle(model->WorldPose().Rot().Yaw() - M_PI_2);
    ignition::math::Vector3d pos = model->WorldPose().Pos();

    // Velocities
    // ignition::math::Vector3d linvel = model->WorldLinearVel();
    // pedestrians[i].velocity.linear.x = linvel.X();
    // pedestrians[i].velocity.linear.y = linvel.Y();
    // pedestrians[i].linear_vel = linvel.Length();
    // ignition::math::Vector3d angvel = model->WorldAngularVel();
    // pedestrians[i].velocity.angular.z = angvel.Z();
    // pedestrians[i].angular_vel = angvel.Z();

    // I do not know the reason, but Gazebo WorldVels are throwing zero
    // velocities, so I compute them by myself
    double anvel =
        normalizeAngle(yaw - pedestrians[i].yaw) / (update_rate_secs);
    double xi = pedestrians[i].position.position.x;
    double yi = pedestrians[i].position.position.y;
    double xf = pos.X();
    double yf = pos.Y();
    double dist = sqrt((xf - xi) * (xf - xi) + (yf - yi) * (yf - yi));
    double linearVelocity = dist / (update_rate_secs);
    double vx = (xf - xi) / (update_rate_secs);
    double vy = (yf - yi) / (update_rate_secs);
    pedestrians[i].velocity.linear.x = vx;
    pedestrians[i].velocity.linear.y = vy;
    pedestrians[i].linear_vel = linearVelocity;
    pedestrians[i].velocity.angular.z = anvel;
    pedestrians[i].angular_vel = anvel;

    // Pose
    pedestrians[i].position.position.x = pos.X();
    pedestrians[i].position.position.y = pos.Y();
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw);
    pedestrians[i].position.orientation = tf2::toMsg(myQuaternion);
    pedestrians[i].yaw = yaw;

    // if (pedestrians[i].name == "actor2") { //// (dt * step_count),
    //   RCLCPP_INFO(rosnode->get_logger(),
    //               "%s xf:%.3f, xi:%.3f, yf:%.3f, yi:%.3f\ncomputed lv: %
    //               .3f " "av: % .3f, vx: % .3f, vy: % .3f ",
    //               pedestrians[i].name.c_str(), xf, xi, yf, yi,
    //               pedestrians[i].linear_vel, pedestrians[i].angular_vel,
    //               pedestrians[i].velocity.linear.x,
    //               pedestrians[i].velocity.linear.y);
    // }

    // RCLCPP_INFO(rosnode->get_logger(),
    //             "Getting agent: %s from simulator, type: %i, id:%i, x:%.2f,
    //             " "y:%.2f, th:%.2f, lv:%.3f, av:%.3f",
    //             model->GetName().c_str(), model->GetType(), model->GetId(),
    //             pos.X(), pos.Y(), yaw, pedestrians[i].linear_vel,
    //             pedestrians[i].angular_vel);
  }
  return true;
}

/////////////////////////////////////////////////
void HuNavPluginPrivate::UpdateGazeboPedestrians(
    const gazebo::common::UpdateInfo &_info,
    const hunav_msgs::msg::Agents &_agents) {
  // update the Gazebo actors
  // for (unsigned int i = 0; i < this->pedestrians.size(); i++)
  for (auto a : _agents.agents) {

    // RCLCPP_INFO(rosnode->get_logger(),
    //             "UpdateGazeboPeds ped received... actor id:%i, pose x:%.2f,
    //             " "y:%.2f, th:%.2f, lv: %.3f, av:%.3f", a.id,
    //             a.position.position.x, a.position.position.y, a.yaw,
    //             a.linear_vel, a.angular_vel);

    // auto entity = world->EntityByName(a.name);
    // auto model =
    // boost::dynamic_pointer_cast<gazebo::physics::Model>(entity);
    gazebo::physics::ModelPtr model = world->ModelByName(a.name);
    gazebo::physics::ActorPtr actor =
        boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

    ignition::math::Pose3d actorPose = actor->WorldPose();
    double yaw = normalizeAngle(a.yaw + M_PI_2);
    double currAngle = actorPose.Rot().Yaw();
    double diff = normalizeAngle(yaw - currAngle);
    if (std::fabs(diff) > IGN_DTOR(10)) {
      yaw = normalizeAngle(currAngle + (diff * 0.01)); // 0.005
    }

    // auto entity_pos =
    // Convert<ignition::math::Vector3d>(a.position.position);
    // entity_pos.Z(1.2138);
    // auto entity_rot = ignition::math::Quaterniond(1.5707, 0, yaw);
    // // Convert<ignition::math::Quaterniond>(a.position.orientation);
    // // Eliminate invalid rotation (0, 0, 0, 0)
    // entity_rot.Normalize();
    // ignition::math::Pose3d entity_pose(entity_pos, entity_rot);

    auto entity_lin_vel =
        gazebo_ros::Convert<ignition::math::Vector3d>(a.velocity.linear);
    auto entity_ang_vel =
        gazebo_ros::Convert<ignition::math::Vector3d>(a.velocity.angular);
    // RCLCPP_INFO(rosnode->get_logger(), "linvel: x:%.3f, y:%.3f - angvel
    // z:%.3f",
    //             entity_lin_vel.X(), entity_lin_vel.Y(),
    //             entity_ang_vel.Z());

    actorPose.Pos().X(a.position.position.x);
    actorPose.Pos().Y(a.position.position.y);
    actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, yaw);
    actorPose.Pos().Z(1.2138);

    // Distance traveled is used to coordinate motion with the walking
    // animation
    double distanceTraveled =
        (actorPose.Pos() - actor->WorldPose().Pos()).Length();

    // RCLCPP_INFO(rosnode->get_logger(),
    //             "UpdateGazeboPeds updating... actor id:%i, pose x:%.2f, "
    //             "y:%.2f, th:%.2f",
    //             actor->GetId(), actorPose.Pos().X(), actorPose.Pos().Y(),
    //             actorPose.Rot().Euler().Z());
    bool is_paused = world->IsPaused();
    world->SetPaused(true);
    model->SetWorldPose(actorPose); //, true, true); // false, false);
    world->SetPaused(is_paused);

    // actor->SetLinearVel(entity_lin_vel);
    // actor->SetAngularVel(entity_ang_vel);
    // RCLCPP_INFO(
    //     rosnode->get_logger(),
    //     "UpdateGazeboPeds updated.. actor id:%i, pose x:%.2f, y:%.2f,
    //     th:%.2f", actor->GetId(), actor->WorldPose().Pos().X(),
    //     actor->WorldPose().Pos().Y(),
    //     actor->WorldPose().Rot().Euler().Z());

    int index = -1;
    for (unsigned int i = 0; i < pedestrians.size(); i++) {
      if (a.id == pedestrians[i].id &&
          a.behavior_state != this->pedestrians[i].behavior_state) {
        this->pedestrians[i].behavior_state = a.behavior_state;
        index = i;
        break;
      }
    }

    // TODO: select better animations for each behavior
    // and adjust the animationFactor value for each case
    double animationFactor;

    if (a.behavior_state == hunav_msgs::msg::Agent::BEH_NO_ACTIVE) {
      if (a.behavior == hunav_msgs::msg::Agent::BEH_REGULAR ||
          a.behavior == hunav_msgs::msg::Agent::BEH_SURPRISED ||
          a.behavior == hunav_msgs::msg::Agent::BEH_THREATENING) {

        animationFactor = 1.5;
      } else if (a.behavior == hunav_msgs::msg::Agent::BEH_IMPASSIVE) {
        animationFactor = 1.5;
      } else if (a.behavior == hunav_msgs::msg::Agent::BEH_SCARED) {
        animationFactor = 2.5;
      } else if (a.behavior == hunav_msgs::msg::Agent::BEH_CURIOUS) {
        animationFactor = 1.5;
      } else {
        animationFactor = 1.0;
      }
    } else {
      if (a.behavior == hunav_msgs::msg::Agent::BEH_REGULAR) {
        animationFactor = 1.5;
      } else if (a.behavior == hunav_msgs::msg::Agent::BEH_IMPASSIVE) {
        animationFactor = 1.5;
      } else if (a.behavior == hunav_msgs::msg::Agent::BEH_SURPRISED) {
        animationFactor = 1.0;
      } else if (a.behavior == hunav_msgs::msg::Agent::BEH_THREATENING) {
        animationFactor = 1.0;
      } else if (a.behavior == hunav_msgs::msg::Agent::BEH_SCARED) {
        animationFactor = 1.5;
      } else if (a.behavior == hunav_msgs::msg::Agent::BEH_CURIOUS) {
        animationFactor = 4.0;
      } else {
        animationFactor = 1.0;
      }
    }

    // change the animation
    if (index > -1) {
      gazebo::physics::TrajectoryInfoPtr trajectoryInfo;
      trajectoryInfo.reset(new gazebo::physics::TrajectoryInfo());
      trajectoryInfo->id = a.id;
      trajectoryInfo->duration = 1.0;
      if (a.behavior_state == hunav_msgs::msg::Agent::BEH_NO_ACTIVE) {
        trajectoryInfo->type = "no_active";
        // RCLCPP_INFO(rosnode->get_logger(),
        //            "changing behavior %i of %s to 'no_active'",
        //            (int)a.behavior, actor->GetName().c_str());

      } else {
        trajectoryInfo->type = "active";
        // RCLCPP_INFO(rosnode->get_logger(),
        //            "changing behavior %i of %s to 'active'", (int)a.behavior,
        //            actor->GetName().c_str());
      }
      actor->Stop();
      actor->SetCustomTrajectory(trajectoryInfo);
      actor->Play();
    }
    actor->SetScriptTime(actor->ScriptTime() +
                         (distanceTraveled * animationFactor));
    // lastUpdate = _info.simTime;
  }
}

/////////////////////////////////////////////////
void HuNavPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info) {
  // Time delta
  // double dt = (_info.simTime - this->lastUpdate).Double();

  // rclcpp::Time now = rosnode->get_clock()->now();

  double seconds_since_last_update = (_info.simTime - lastUpdate).Double();
  // RCLCPP_INFO(rosnode->get_logger(), "seconds since last update: %.4f",
  //             seconds_since_last_update);

  // update_rate_secs = 0.1;
  if (seconds_since_last_update < update_rate_secs) {
    // get pedestrian states (fill pedestrians)
    // GetPedestrians();
    // hunav_msgs::msg::Agents ags;
    // ags.agents = pedestrians;
    // ags.header.frame_id = "world";
    // ags.header.stamp = now;
    // UpdateGazeboPedestrians(_info, ags);
    RCLCPP_INFO(rosnode->get_logger(), "skipping!!!");
    return;
  }

  // update closest obstacle (fill pedestrians obstacles)
  HandleObstacles();
  // get robot state (fill robotAgent)
  if (!GetRobot())
    return;
  // get pedestrian states (fill pedestrians)
  if (!GetPedestrians())
    return;

  // Fill the service request
  auto request = std::make_shared<hunav_msgs::srv::ComputeAgents::Request>();

  hunav_msgs::msg::Agents agents;
  agents.header.frame_id = globalFrame;
  agents.header.stamp =
      gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);
  // agents.header.stamp = now;
  agents.agents = pedestrians;
  request->robot = robotAgent;
  request->current_agents = agents;

  // Wait for the service to be available
  while (!rosSrvClient->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rosnode->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(rosnode->get_logger(),
                "service not available, waiting again...");
  }
  // Call the service
  auto result = rosSrvClient->async_send_request(request);
  // Wait for the result.
  // std::chrono::duration<int, std::milli> ms(500);
  // RCLCPP_INFO(rosnode->get_logger(), "Waiting for service result...");
  // This provokes a lock here so we can not use it
  // if (rclcpp::spin_until_future_complete(rosnode, result, ms) ==
  //     // if (rclcpp::spin_until_future_complete(rosnode, result) ==
  //     rclcpp::FutureReturnCode::SUCCESS) {
  //   // Update the agents data

  std::chrono::duration<int, std::milli> ms(200);
  if (result.wait_for(ms) == std::future_status::ready) {
    // RCLCPP_INFO(rosnode->get_logger(), "Service result received!");
    // update the Gazebo actors
    UpdateGazeboPedestrians(_info, result.get()->updated_agents);

  } else {
    RCLCPP_ERROR(rosnode->get_logger(),
                 "Failed to call service compute_agents");
  }
  lastUpdate = _info.simTime;
}

GZ_REGISTER_WORLD_PLUGIN(HuNavPlugin)
} // namespace hunav
