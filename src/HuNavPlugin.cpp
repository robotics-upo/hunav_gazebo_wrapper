/***********************************************************************/
/**                                                                    */
/** HuNavPlugin.cpp                                                    */
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

// #include <functional>
#include <stdio.h>
#include <string>

// #include <ignition/math.hh>
// #include <ignition/math/gzmath.hh>
#include <hunav_gazebo_wrapper/HuNavPlugin.h>

#define DEF_WALKING_ANIMATION "walking"

// namespace gazebo_ros {
namespace hunav
{

// GZ_REGISTER_MODEL_PLUGIN(HumanNavPlugin)

using namespace std::chrono_literals;

class HuNavPluginPrivate
{
public:
  /// Callback when world is updated.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo& _info);

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /// \brief Helper function to collect the pedestrians data from Gazebo
  bool GetPedestrians();
  /// \brief Helper function to collect the robot data from Gazebo
  bool GetRobot();
  /// \brief Helper function to collect obtacles from Gazebo
  void HandleObstacles();
  /// \brief Helper function to collect obtacles from Gazebo
  void HandleObstacles2();
  /// \brief Helper function to update the Gazebo actors
  void UpdateGazeboPedestrians(const gazebo::common::UpdateInfo& _info, const hunav_msgs::msg::Agents& _agents);
  /// @brief  Helper function to get the closest point on a bounding box
  /// @param _point 
  /// @param _box 
  /// @return closest point on the bounding box
  ignition::math::Vector3d GetClosestPointOnBoundingBox(const ignition::math::Vector3d& _point,
                                                        const ignition::math::AxisAlignedBox& _box);
  /// \brief load the stored animations for the agents behaviors
  // void LoadAnimations();
  /// \brief Update the animation of the agent according to the behavior
  // void UpdateAnimation(const human_nav_msgs::msg::Agent &_agent);
  /// \brief Helper function to initialize the agents at the initial step
  void InitializeAgents();
  /// \brief Helper function to initialize the robot the initial step
  bool InitializeRobot();
  inline double normalizeAngle(double a)
  {
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

  /// \brief ros service to reset the agents in the hunav_manager
  rclcpp::Client<hunav_msgs::srv::ResetAgents>::SharedPtr rosSrvResetClient;

  /// \brief ros service to reset the agents in the hunav_manager
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;

  /// \brief the robot as a agent msg
  hunav_msgs::msg::Agent robotAgent;
  hunav_msgs::msg::Agent init_robotAgent;

  /// \brief vector of pedestrians detected.
  std::vector<hunav_msgs::msg::Agent> pedestrians;
  // init state of agents
  hunav_msgs::msg::Agents init_pedestrians;

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
  bool reset;

  std::string robotName;
  std::string globalFrame;

  bool waitForGoal;
  bool goalReceived;
  bool useCollision;
  std::string goalTopic;

  /// \brief List of models to ignore. Used for vector field
  std::vector<std::string> ignoreModels;
};

/////////////////////////////////////////////////
HuNavPlugin::HuNavPlugin() : hnav_(std::make_unique<HuNavPluginPrivate>())
{
}

HuNavPlugin::~HuNavPlugin()
{
}

/////////////////////////////////////////////////
void HuNavPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
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

  if (_sdf->HasElement("use_navgoal_to_start"))
    hnav_->waitForGoal = _sdf->Get<bool>("use_navgoal_to_start");
  else
  {
    hnav_->waitForGoal = false;
    RCLCPP_INFO(hnav_->rosnode->get_logger(), "PARAMETER USE_NAVGOAL_TO_START IS NOT IN THE WORLD FILE!!");
  }
  if (_sdf->HasElement("use_collision"))
    hnav_->useCollision = _sdf->Get<bool>("use_collision");
  else
  {
    hnav_->useCollision = false;
    RCLCPP_INFO(hnav_->rosnode->get_logger(), "PARAMETER USE_COLLISION IS NOT IN THE WORLD FILE!!");
  }

  if (hnav_->waitForGoal)
  {
    hnav_->goalReceived = false;
    if (_sdf->HasElement("navgoal_topic"))
      hnav_->goalTopic = _sdf->Get<std::string>("navgoal_topic");
    else
      hnav_->goalTopic = "goal_pose";
  }
  else
  {
    hnav_->goalReceived = true;
  }

  // Read models to be ignored
  if (_sdf->HasElement("ignore_models"))
  {
    sdf::ElementPtr modelElem = _sdf->GetElement("ignore_models")->GetElement("model");
    while (modelElem)
    {
      hnav_->ignoreModels.push_back(modelElem->Get<std::string>());
      RCLCPP_INFO(hnav_->rosnode->get_logger(), "Ignoring model: %s", (modelElem->Get<std::string>()).c_str());
      modelElem = modelElem->GetNextElement("model");
    }
  }
  hnav_->reset = false;
  hnav_->rostime = hnav_->rosnode->get_clock()->now();
  hnav_->dt = 0;
  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0)
  {
    hnav_->update_rate_secs = 1.0 / update_rate;
  }
  else
  {
    hnav_->update_rate_secs = 0.0;
  }
  RCLCPP_INFO(hnav_->rosnode->get_logger(), "update_rate: %.2f, secs:%.4f", update_rate, hnav_->update_rate_secs);

  hnav_->rosSrvClient = hnav_->rosnode->create_client<hunav_msgs::srv::ComputeAgents>("compute_agents");

  hnav_->rosSrvGetAgentsClient = hnav_->rosnode->create_client<hunav_msgs::srv::GetAgents>("get_agents");

  hnav_->rosSrvResetClient = hnav_->rosnode->create_client<hunav_msgs::srv::ResetAgents>("reset_agents");

  if (hnav_->waitForGoal)
  {
    hnav_->goal_sub = hnav_->rosnode->create_subscription<geometry_msgs::msg::PoseStamped>(
        hnav_->goalTopic, 1, std::bind(&HuNavPluginPrivate::goalCallback, hnav_.get(), std::placeholders::_1));
  }

  hnav_->connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&HuNavPluginPrivate::OnUpdate, hnav_.get(), std::placeholders::_1));

  // Reset();
  hnav_->InitializeAgents();
  hnav_->HandleObstacles2();

  hnav_->lastUpdate = _world->SimTime();

  // Reset trajectory so we can move the agents!
  for (size_t i = 0; i < hnav_->pedestrians.size(); ++i)
  {
    gazebo::physics::ModelPtr model = hnav_->world->ModelByName(hnav_->pedestrians[i].name);
    gazebo::physics::ActorPtr actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

    gazebo::physics::TrajectoryInfoPtr trajectoryInfo;
    trajectoryInfo.reset(new gazebo::physics::TrajectoryInfo());
    trajectoryInfo->id = 0;
    auto skelAnims = actor->SkeletonAnimations();
    if (!skelAnims.empty())
    {
      for (auto it = skelAnims.begin(); it != skelAnims.end(); ++it)
      {
        RCLCPP_INFO(hnav_->rosnode->get_logger(), "Agent %s has animation: %s", actor->GetName().c_str(),
                    it->first.c_str());
      }
      trajectoryInfo->type = "no_active";  // skelAnims.begin()->first;
    }
    else
    {
      RCLCPP_INFO(hnav_->rosnode->get_logger(), "Animation default type: %s", DEF_WALKING_ANIMATION);
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

  // Reset();
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

void HuNavPlugin::Reset()
{
  RCLCPP_INFO(hnav_->rosnode->get_logger(), "\n\n---------World reset---------\n");

  hnav_->reset = true;
  hnav_->lastUpdate = 0.0;  // hnav_->world->SimTime();
  hnav_->rostime = hnav_->rosnode->get_clock()->now();

  hnav_->InitializeRobot();

  for (size_t i = 0; i < hnav_->init_pedestrians.agents.size(); ++i)
  {
    gazebo::physics::ModelPtr model = hnav_->world->ModelByName(hnav_->init_pedestrians.agents[i].name);
    gazebo::physics::ActorPtr actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

    // // Gazebo models positions----------------
    // ignition::math::Pose3d actorPose = actor->WorldPose();
    // double yaw =
    //     hnav_->normalizeAngle(hnav_->init_pedestrians.agents[i].yaw +
    //     M_PI_2);
    // auto entity_lin_vel = gazebo_ros::Convert<ignition::math::Vector3d>(
    //     hnav_->init_pedestrians.agents[i].velocity.linear);
    // auto entity_ang_vel = gazebo_ros::Convert<ignition::math::Vector3d>(
    //     hnav_->init_pedestrians.agents[i].velocity.angular);
    // actorPose.Pos().X(hnav_->init_pedestrians.agents[i].position.position.x);
    // actorPose.Pos().Y(hnav_->init_pedestrians.agents[i].position.position.y);
    // actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, yaw);
    // model->SetWorldPose(actorPose);
    // model->ResetPhysicsStates();
    // //----------------------------------------

    //---Reset trajectories-----------------------------
    gazebo::physics::TrajectoryInfoPtr trajectoryInfo;
    trajectoryInfo.reset(new gazebo::physics::TrajectoryInfo());
    // trajectoryInfo->id = 0;
    auto skelAnims = actor->SkeletonAnimations();
    if (!skelAnims.empty())
    {
      trajectoryInfo->type = "no_active";
    }
    else
    {
      RCLCPP_INFO(hnav_->rosnode->get_logger(), "Animation default type: %s", DEF_WALKING_ANIMATION);
      trajectoryInfo->type = DEF_WALKING_ANIMATION;
    }
    trajectoryInfo->duration = 1.0;

    actor->SetCustomTrajectory(trajectoryInfo);
  }
}

bool HuNavPluginPrivate::InitializeRobot()
{
  RCLCPP_INFO(rosnode->get_logger(), "Initializing robot...");
  robotModel = world->ModelByName(robotName);
  if (!robotModel)
  {
    RCLCPP_ERROR(rosnode->get_logger(), "Robot model %s not found (yet)!!!!", robotName.c_str());
    return false;
  }
  else
  {
    RCLCPP_INFO(rosnode->get_logger(), "Robot %s detected!!! initializing params...", robotName.c_str());
    // Initialize robot agent
    ignition::math::Vector3d pos = robotModel->WorldPose().Pos();
    ignition::math::Vector3d rpy = robotModel->WorldPose().Rot().Euler();
    robotAgent.id = robotModel->GetId();
    robotAgent.type = hunav_msgs::msg::Agent::ROBOT;
    robotAgent.behavior.type = hunav_msgs::msg::AgentBehavior::BEH_REGULAR;
    robotAgent.behavior.state = hunav_msgs::msg::AgentBehavior::BEH_NO_ACTIVE;
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
    init_robotAgent = robotAgent;
    return true;
  }
}

void HuNavPluginPrivate::InitializeAgents()
{
  RCLCPP_INFO(rosnode->get_logger(), "Initializing agents...");

  InitializeRobot();

  // Fill the service request
  auto request = std::make_shared<hunav_msgs::srv::GetAgents::Request>();
  request->empty = 0;

  // Wait for the service to be available
  while (!rosSrvGetAgentsClient->wait_for_service(5s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rosnode->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(rosnode->get_logger(), "service /get_agents not available, waiting again...");
  }
  // Call the service
  auto result = rosSrvGetAgentsClient->async_send_request(request);
  std::chrono::duration<int, std::milli> ms(200);
  if (result.wait_for(ms) == std::future_status::ready)
  {
    // Initialize the actors
    auto res = *result.get();
    init_pedestrians = res.agents;

    // if (result.success) {
    // init_pedestrians = result.get()->agents;
    RCLCPP_INFO(rosnode->get_logger(), "Received %i agents from service /get_agents", (int)init_pedestrians.agents.size());
    pedestrians.clear();

    for (auto agent : init_pedestrians.agents)
    {
      hunav_msgs::msg::Agent ag = agent;

      gazebo::physics::ModelPtr model = world->ModelByName(agent.name);
      if (!model)
      {
        RCLCPP_ERROR(rosnode->get_logger(), "Pedestrian model '%s' not found!!!!", agent.name.c_str());
        return;
      }

      // ---- update Gazebo models ---
      gazebo::physics::ActorPtr actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

      ignition::math::Pose3d actorPose = actor->WorldPose();
      double yaw = normalizeAngle(agent.yaw + M_PI_2);
      auto entity_lin_vel = gazebo_ros::Convert<ignition::math::Vector3d>(agent.velocity.linear);
      auto entity_ang_vel = gazebo_ros::Convert<ignition::math::Vector3d>(agent.velocity.angular);
      actorPose.Pos().X(agent.position.position.x);
      actorPose.Pos().Y(agent.position.position.y);
      actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, yaw);
      model->SetWorldPose(actorPose);
      if (useCollision)
      {
        gazebo::physics::ModelPtr body_model = world->ModelByName(agent.name + "_body");
        actorPose.Rot() = ignition::math::Quaterniond(0, 0, yaw);
        body_model->SetWorldPose(actorPose);
      }
      //-----------------------------------

      ignition::math::Vector3d pos = model->WorldPose().Pos();
      // The front (x axis) of the actors in Gazebo is
      // looking to the person left, we rotate it
      yaw = normalizeAngle(model->WorldPose().Rot().Yaw() - M_PI_2);
      ag.position.position.x = pos.X();
      ag.position.position.y = pos.Y();
      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, yaw);
      ag.position.orientation = tf2::toMsg(myQuaternion);
      ag.yaw = yaw;
      ignition::math::Vector3d linvel = model->WorldLinearVel();
      ag.desired_velocity = agent.desired_velocity;
      ag.velocity.linear.x = linvel.X();
      ag.velocity.linear.y = linvel.Y();
      ag.linear_vel = linvel.Length();
      ignition::math::Vector3d angvel = model->WorldAngularVel();
      ag.velocity.angular.z = angvel.Z();
      ag.angular_vel = angvel.Z();

      pedestrians.push_back(ag);
      RCLCPP_INFO(rosnode->get_logger(),
                  "Adding agent: %s, type: %i, id:%i, beh:%i, x:%.2f, "
                  "y:%.2f, th:%.2f, dvel:%.2f",
                  ag.name.c_str(), ag.type, ag.id, (int)ag.behavior.type, pos.X(), pos.Y(), yaw,
                  pedestrians.back().desired_velocity);
    }
    //} else {
    //  RCLCPP_ERROR(rosnode->get_logger(), "Call service /get_agents not success!!");
    //}
  }
  else
  {
    RCLCPP_ERROR(rosnode->get_logger(), "Failed to call service get_agents");
  }
}

void HuNavPluginPrivate::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(rosnode->get_logger(), "\n\nPlugin! GOAL RECEIVED!!!!\n\n");
  goalReceived = true;
}

/////////////////////////////////////////////////
// void HuNavPluginPrivate::HandleObstacles()
// {
//   for (size_t i = 0; i < pedestrians.size(); ++i)
//   {
//     gazebo::physics::ModelPtr agent = world->ModelByName(pedestrians[i].name);

//     double minDist = 10000.0;
//     ignition::math::Vector3d closest_obstacle;
//     // ignition::math::Vector3d closest_obs2;
//     pedestrians[i].closest_obs.clear();

//     for (size_t j = 0; j < world->ModelCount(); ++j)
//     {
//       gazebo::physics::ModelPtr modelObstacle = world->ModelByIndex(j);

//       // Avoid to compute the other actors as obstacles
//       for (size_t k = 0; k < pedestrians.size(); ++k)
//       {
//         if ((int)modelObstacle->GetId() == pedestrians[k].id)
//           break;
//       }

//       // Avoid the agent itself and the indicated models
//       if (agent->GetId() != modelObstacle->GetId() && std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
//                                                                 modelObstacle->GetName()) == this->ignoreModels.end())
//       {
//         ignition::math::Vector3d actorPos = agent->WorldPose().Pos();
//         ignition::math::Vector3d obsPos = modelObstacle->WorldPose().Pos();

//         // This if is used to avoid an invisible obstacle that Gazebo is setting at (0,0). We don't know why it's
//         // happening.
//         if (obsPos.X() != 0.0 && obsPos.Y() != 0.0)
//         {
//           // RCLCPP_INFO(rosnode->get_logger(), "\n\nX --> %.2f and Y --> %.2f\n\n", obsPos.X(), obsPos.Y());
//           ignition::math::Line3d act_obs_line(actorPos, obsPos);
//           std::tuple<bool, double, ignition::math::Vector3d> obs_intersect =
//               modelObstacle->BoundingBox().Intersect(act_obs_line);

//           ignition::math::Vector3d intersecPos;
//           double dist = -1;

//           if (std::get<0>(obs_intersect) == true)
//           {
//             intersecPos = std::get<2>(obs_intersect);
//             dist = std::get<1>(obs_intersect);
//           }

//           /*ignition::math::Vector3d goalPos(pedestrians[i].goals[0].position.x,
//                                           pedestrians[i].goals[0].position.y,
//                                           actorPos.Z());
//           ignition::math::Line3d act_goal_line(actorPos, goalPos);
//           std::tuple<bool, double, ignition::math::Vector3d> goal_intersect =
//               modelObstacle->BoundingBox().Intersect(act_goal_line);
//           if (std::get<0>(goal_intersect) == true) {
//             if (dist > 0.0 && std::get<1>(goal_intersect) < dist) {
//               intersecPos = std::get<2>(goal_intersect);
//               dist = std::get<1>(goal_intersect);
//             }
//           }*/

//           if (dist > 0)
//           {
//             ignition::math::Vector3d offset = intersecPos - actorPos;
//             double modelDist = offset.Length();  //-approximated_radius;
//             // double dist2 = actorPos.Distance(std::get<2>(intersect));

//             if (modelDist < minDist)
//             {
//               minDist = modelDist;
//               // closest_obs = offset;
//               closest_obstacle = intersecPos;
//             }
//           }
//         }
//         // std::tuple<bool, double, ignition::math::Vector3d> intersect =
//         //     modelObstacle->BoundingBox().Intersect(obsPos, actorPos,
//         //     0.05, 8.0);
//       }
//     }
//     if (minDist <= 10.0)
//     {
//       geometry_msgs::msg::Point p;
//       p.x = closest_obstacle.X();
//       p.y = closest_obstacle.Y();
//       p.z = 0.0;
//       pedestrians[i].closest_obs.push_back(p);
//     }
//   }
// }



ignition::math::Vector3d ClosestPointUsingCollisions(
  const ignition::math::Vector3d &point,
  const gazebo::physics::WorldPtr &world,
  const std::vector<std::string> &ignoreModels)
{
  double minDist = std::numeric_limits<double>::max();
  ignition::math::Vector3d closestPoint;

  // Iterate through all models in the world
  for (size_t i = 0; i < world->ModelCount(); ++i)
  {
      gazebo::physics::ModelPtr model = world->ModelByIndex(i);

      // Skip ignored models
      if (std::find(ignoreModels.begin(), ignoreModels.end(), model->GetName()) != ignoreModels.end())
          continue;

      // Iterate through all links in the model
      for (auto link : model->GetLinks())
      {
          // Iterate through all collisions in the link
          for (auto collision : link->GetCollisions())
          {
              // Get the bounding box of the collision
              auto bbox = collision->BoundingBox();

              // Compute the closest point on the bounding box
              auto minCorner = bbox.Min();
              auto maxCorner = bbox.Max();
              double x = std::max(minCorner.X(), std::min(point.X(), maxCorner.X()));
              double y = std::max(minCorner.Y(), std::min(point.Y(), maxCorner.Y()));
              double z = std::max(minCorner.Z(), std::min(point.Z(), maxCorner.Z()));
              ignition::math::Vector3d candidatePoint(x, y, z);

              // Compute the distance to the point
              double dist = (candidatePoint - point).Length();
              if (dist < minDist)
              {
                  minDist = dist;
                  closestPoint = candidatePoint;
              }
          }
      }
  }

  return closestPoint;
}



void HuNavPluginPrivate::HandleObstacles2()
{
  for (size_t i = 0; i < pedestrians.size(); ++i)
  {
    gazebo::physics::ModelPtr agent = world->ModelByName(pedestrians[i].name);

    double minDist = 5.0; //10000.0;
    pedestrians[i].closest_obs.clear();

    for (size_t j = 0; j < world->ModelCount(); ++j)
    {
      gazebo::physics::ModelPtr modelObstacle = world->ModelByIndex(j);

      // Avoid to compute the other actors as obstacles
      for (size_t k = 0; k < pedestrians.size(); ++k)
      {
        if ((int)modelObstacle->GetId() == pedestrians[k].id)
          break;
      }

      // Avoid the agent itself and the indicated models
      if (agent->GetId() != modelObstacle->GetId() && std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                                                                modelObstacle->GetName()) == this->ignoreModels.end())
      {

        // Iterate through all links in the model
        for (auto link : modelObstacle->GetLinks())
        {
          // Iterate through all collisions in the link
          for (auto collision : link->GetCollisions())
          {
            // Get the bounding box of the collision
            auto bbox = collision->BoundingBox();

            ignition::math::Vector3d actorPos = agent->WorldPose().Pos();
            auto closestObs = GetClosestPointOnBoundingBox(actorPos, modelObstacle->BoundingBox()); 
            ignition::math::Vector3d offset = closestObs - actorPos;
            double dist = offset.Length();
            if (dist > 0 && dist < minDist)
            {
              geometry_msgs::msg::Point p;
              p.x = closestObs.X();
              p.y = closestObs.Y();
              p.z = 0.0;
              pedestrians[i].closest_obs.push_back(p);
            }
          }
        }
      }
    }
  }
}







ignition::math::Vector3d HuNavPluginPrivate::GetClosestPointOnBoundingBox(const ignition::math::Vector3d& _point,
  const ignition::math::AxisAlignedBox& _box)
{
  // Get the minimum and maximum corners of the bounding box
  auto minCorner = _box.Min();
  auto maxCorner = _box.Max();

  // Clamp the point to the bounding box
  double x = std::max(minCorner.X(), std::min(_point.X(), maxCorner.X()));
  double y = std::max(minCorner.Y(), std::min(_point.Y(), maxCorner.Y()));
  double z = std::max(minCorner.Z(), std::min(_point.Z(), maxCorner.Z()));

  // Return the closest point
  return ignition::math::Vector3d(x, y, z);
}


void HuNavPluginPrivate::HandleObstacles()
{
  for (size_t i = 0; i < pedestrians.size(); ++i)
  {
    gazebo::physics::ModelPtr agent = world->ModelByName(pedestrians[i].name);

    double minDist = 5.0; //10000.0;
    pedestrians[i].closest_obs.clear();

    for (size_t j = 0; j < world->ModelCount(); ++j)
    {
      gazebo::physics::ModelPtr modelObstacle = world->ModelByIndex(j);

      // Avoid to compute the other actors as obstacles
      for (size_t k = 0; k < pedestrians.size(); ++k)
      {
        if ((int)modelObstacle->GetId() == pedestrians[k].id)
          break;
      }

      // Avoid the agent itself and the indicated models
      if (agent->GetId() != modelObstacle->GetId() && std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                                                                modelObstacle->GetName()) == this->ignoreModels.end())
      {
        ignition::math::Vector3d actorPos = agent->WorldPose().Pos();
        ignition::math::Vector3d obsPos = modelObstacle->WorldPose().Pos();

        //RCLCPP_INFO(rosnode->get_logger(), "\n model %s, X --> %.2f and Y --> %.2f\n\n", modelObstacle->GetName().c_str(), obsPos.X(), obsPos.Y());

        // This if is used to avoid an invisible obstacle that Gazebo is setting at (0,0). We don't know why it's
        // happening.
        if (obsPos.X() != 0.0 && obsPos.Y() != 0.0)
        {
          auto closestObs = GetClosestPointOnBoundingBox(actorPos, modelObstacle->BoundingBox()); 
          ignition::math::Vector3d offset = closestObs - actorPos;
          double dist = offset.Length();
          if (dist > 0 && dist < minDist)
          {
            geometry_msgs::msg::Point p;
            p.x = closestObs.X();
            p.y = closestObs.Y();
            p.z = 0.0;
            pedestrians[i].closest_obs.push_back(p);
          }
        }
      }
    }
  }
}




/**
 * \brief Get the robot position and velocity
 */
bool HuNavPluginPrivate::GetRobot()
{
  if (!robotModel)
  {
    if (!InitializeRobot())
    {
      //RCLCPP_ERROR(rosnode->get_logger(), "Robot model %s not found!!!!", robotName.c_str());
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
bool HuNavPluginPrivate::GetPedestrians()
{
  for (unsigned int i = 0; i < pedestrians.size(); ++i)
  {
    gazebo::physics::ModelPtr model = world->ModelByName(pedestrians[i].name);

    if (!model)
    {
      RCLCPP_ERROR(rosnode->get_logger(), "Pedestrian model %s not found!!!!", pedestrians[i].name.c_str());
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
    double xi = pedestrians[i].position.position.x;
    double yi = pedestrians[i].position.position.y;
    double xf = pos.X();
    double yf = pos.Y();
    double dist = sqrt((xf - xi) * (xf - xi) + (yf - yi) * (yf - yi));
    double linearVelocity = dist / (update_rate_secs);
    double anvel = normalizeAngle(yaw - pedestrians[i].yaw) / (update_rate_secs);
    double vx = (xf - xi) / (update_rate_secs);
    double vy = (yf - yi) / (update_rate_secs);

    if (reset)
    {
      linearVelocity = 0.0;
      vx = 0.0;
      vy = 0.0;
      anvel = 0.0;
    }
    else if (linearVelocity > pedestrians[i].desired_velocity)
    {
      linearVelocity = pedestrians[i].desired_velocity;
      double maxd = linearVelocity * update_rate_secs;
      if (fabs(xf - xi) > maxd)
        vx = ((xf - xi) / fabs(xf - xi)) * maxd / update_rate_secs;
      if (fabs(yf - yi) > maxd)
        vy = ((yf - yi) / fabs(yf - yi)) * maxd / update_rate_secs;
    }
    pedestrians[i].velocity.linear.x = vx;
    pedestrians[i].velocity.linear.y = vy;
    pedestrians[i].linear_vel = linearVelocity;
    pedestrians[i].velocity.angular.z = anvel;
    pedestrians[i].angular_vel = anvel;

    // Pose
    pedestrians[i].position.position.x = xf;  // pos.X();
    pedestrians[i].position.position.y = yf;  // pos.Y();
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw);
    pedestrians[i].position.orientation = tf2::toMsg(myQuaternion);
    pedestrians[i].yaw = yaw;

    // RCLCPP_INFO(rosnode->get_logger(),
    //             "getPedestrians. Actor %s pose x: %.2f, y: %.2f, vx:%.2f, "
    //             "vy:%.2f, lv: %.2f, dvel: %.2f",
    //             pedestrians[i].name.c_str(),
    //             pedestrians[i].position.position.x,
    //             pedestrians[i].position.position.y,
    //             pedestrians[i].velocity.linear.x,
    //             pedestrians[i].velocity.linear.y, pedestrians[i].linear_vel,
    //             pedestrians[i].desired_velocity);

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
    //             "GetPedestrians. Agent: %s from simulator, type: %i, id:%i,
    //             " "x:%.2f, y: % .2f, th: % .2f, lv: % .3f, av: % .3f ",
    //             model->GetName().c_str(), model->GetType(), model->GetId(),
    //             pos.X(), pos.Y(), yaw, pedestrians[i].linear_vel,
    //             pedestrians[i].angular_vel);
  }
  reset = false;
  return true;
}

/////////////////////////////////////////////////
void HuNavPluginPrivate::UpdateGazeboPedestrians(const gazebo::common::UpdateInfo& _info,
                                                 const hunav_msgs::msg::Agents& _agents)
{
  if (goalReceived == false)
  {
    RCLCPP_INFO(rosnode->get_logger(), "HuNavPlugin. Waiting to receive the robot navigation goal...");
    return;
  }

  // update the Gazebo actors
  for (auto a : _agents.agents)
  {
    // auto model =
    // boost::dynamic_pointer_cast<gazebo::physics::Model>(entity);
    gazebo::physics::ModelPtr model = world->ModelByName(a.name);
    gazebo::physics::ActorPtr actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);

    ignition::math::Pose3d actorPose = actor->WorldPose();
    double yaw = normalizeAngle(a.yaw + M_PI_2);
    double currAngle = actorPose.Rot().Yaw();
    double diff = normalizeAngle(yaw - currAngle);
    if (std::fabs(diff) > IGN_DTOR(10))
    {
      yaw = normalizeAngle(currAngle + (diff * 0.01));  // ok: 01,  tested: 0.01, 0.005
    }

    auto entity_lin_vel = gazebo_ros::Convert<ignition::math::Vector3d>(a.velocity.linear);
    auto entity_ang_vel = gazebo_ros::Convert<ignition::math::Vector3d>(a.velocity.angular);
    // RCLCPP_INFO(rosnode->get_logger(), "linvel: x:%.3f, y:%.3f - angvel
    // z:%.3f",
    //             entity_lin_vel.X(), entity_lin_vel.Y(),
    //             entity_ang_vel.Z());

    actorPose.Pos().X(a.position.position.x);
    actorPose.Pos().Y(a.position.position.y);
    actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, yaw);

    // The human agents do not have collisions.
    // When the simulation starts, they move to zero height
    // (in the middle of their bodies approximately)
    // So, we need to add an extra height (Z) according to the height
    // of each model in order to put them on the floor
    // (the floor must be at zero height)
    for (auto pedestrian : pedestrians)
    {
      if (a.id == pedestrian.id)
      {
        switch (pedestrian.skin)
        {
          // Elegant man
          case 0:
            actorPose.Pos().Z(0.96);
            break;
          // Casual man
          case 1:
            actorPose.Pos().Z(0.97);
            break;
          // Elegant woman
          case 2:
            actorPose.Pos().Z(0.93);
            break;
          // Regular man
          case 3:
            actorPose.Pos().Z(0.93);
            break;
          // Worker man
          case 4:
            actorPose.Pos().Z(0.97);
            break;
          // Balds
          case 5:
            actorPose.Pos().Z(1.05);
            break;
          case 6:
            actorPose.Pos().Z(1.05);
            break;
          case 7:
            actorPose.Pos().Z(1.05);
            break;
          case 8:
            actorPose.Pos().Z(1.05);
            break;
          default:
            break;
        }
      }
    }

    // Distance traveled is used to coordinate motion with the walking
    // animation: newPose(actorPose) - prevPose(actor->WorldPose)
    double distanceTraveled = (actorPose.Pos() - actor->WorldPose().Pos()).Length();

    // RCLCPP_INFO(rosnode->get_logger(),
    //             "UpdateGazeboPeds updating... actor id:%i, pose x:%.2f, "
    //             "y:%.2f, th:%.2f",
    //             actor->GetId(), actorPose.Pos().X(), actorPose.Pos().Y(),
    //             actorPose.Rot().Euler().Z());
    bool is_paused = world->IsPaused();
    world->SetPaused(true);
    model->SetWorldPose(actorPose);  //, true, true); // false, false);
    if (useCollision)
    {
      gazebo::physics::ModelPtr body_model = world->ModelByName(a.name + "_body");
      actorPose.Rot() = ignition::math::Quaterniond(0,0, yaw);
      body_model->SetWorldPose(actorPose);  //, true, true); // false, false);
    }
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
    for (unsigned int i = 0; i < pedestrians.size(); i++)
    {
      if (a.id == pedestrians[i].id)
      {
        // desiredVel
        // this->pedestrians[i].desired_velocity = a.desired_velocity;
        // update pedestrians' goals
        this->pedestrians[i].goals.clear();
        this->pedestrians[i].goals = a.goals;

        // update behavior state
        if (a.behavior.state != this->pedestrians[i].behavior.state)
        {
          this->pedestrians[i].behavior.state = a.behavior.state;
          index = i;
          break;
        }
      }
    }

    // TODO: select better animations for each behavior
    // and adjust the animationFactor value for each case
    double animationFactor;

    if (a.behavior.state == hunav_msgs::msg::AgentBehavior::BEH_NO_ACTIVE)
    {
      if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_REGULAR ||
          a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED ||
          a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
      {
        animationFactor = 1.0;
      }
      else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE)
      {
        animationFactor = 1.0;
      }
      else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
      {
        animationFactor = 1.0;
      }
      else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
      {
        animationFactor = 1.0;
      }
      else
      {
        animationFactor = 1.0;
      }
    }
    else
    {
      if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_REGULAR)
      {
        animationFactor = 1.5;
      }
      else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE)
      {
        animationFactor = 1.5;
      }
      else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED)
      {
        animationFactor = 1.0;
      }
      else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
      {
        animationFactor = 1.0;
      }
      else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
      {
        animationFactor = 1.5;
      }
      else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
      {
        animationFactor = 1.0;
      }
      else
      {
        animationFactor = 1.0;
      }
    }

    // change the animation
    if (index > -1)
    {
      gazebo::physics::TrajectoryInfoPtr trajectoryInfo;
      trajectoryInfo.reset(new gazebo::physics::TrajectoryInfo());
      trajectoryInfo->id = a.id;
      trajectoryInfo->duration = 1.0;
      if (a.behavior.state == hunav_msgs::msg::AgentBehavior::BEH_NO_ACTIVE)
      {
        trajectoryInfo->type = "no_active";
        // RCLCPP_INFO(rosnode->get_logger(),
        //            "changing behavior %i of %s to 'no_active'",
        //            (int)a.behavior, actor->GetName().c_str());
      }
      else
      {
        trajectoryInfo->type = "active";
        // RCLCPP_INFO(rosnode->get_logger(),
        //            "changing behavior %i of %s to 'active'",
        //            (int)a.behavior, actor->GetName().c_str());
      }
      actor->Stop();
      actor->SetCustomTrajectory(trajectoryInfo);
      actor->Play();
    }
    actor->SetScriptTime(actor->ScriptTime() + (distanceTraveled * animationFactor));
    // lastUpdate = _info.simTime;
  }
}

/////////////////////////////////////////////////
void HuNavPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  // Time delta
  // double dt = (_info.simTime - this->lastUpdate).Double();

  // rclcpp::Time now = rosnode->get_clock()->now();

  double seconds_since_last_update = (_info.simTime - lastUpdate).Double();
  // // RCLCPP_INFO(rosnode->get_logger(), "seconds since last update: %.4f",
  // //             seconds_since_last_update);

  if (seconds_since_last_update < update_rate_secs)
    return;
  // // update_rate_secs = 0.1;
  // if (seconds_since_last_update < update_rate_secs) {
  //   // get pedestrian states (fill pedestrians)
  //   // GetPedestrians();
  //   // hunav_msgs::msg::Agents ags;
  //   // ags.agents = pedestrians;
  //   // ags.header.frame_id = "world";
  //   // ags.header.stamp = now;
  //   // UpdateGazeboPedestrians(_info, ags);
  //   RCLCPP_INFO(rosnode->get_logger(), "skipping!!!");
  //   return;
  // }

  // update closest obstacle (fill pedestrians obstacles)
  HandleObstacles2();
  // get robot state (fill robotAgent)
  if (!GetRobot())
    return;
  // get pedestrian states (fill pedestrians)
  if (!GetPedestrians())
    return;

  // Fill the service request
  auto request = std::make_shared<hunav_msgs::srv::ComputeAgents::Request>();

  // Wait for the service to be available
  while (!rosSrvClient->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rosnode->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(rosnode->get_logger(), "Service /compute_agents not available, waiting again...");
  }

  hunav_msgs::msg::Agents agents;
  agents.header.frame_id = globalFrame;
  agents.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);
  // agents.header.stamp = now;
  agents.agents = pedestrians;
  request->robot = robotAgent;
  request->current_agents = agents;

  // Call the service
  auto result = rosSrvClient->async_send_request(request);
  // Wait for the result.
  std::chrono::duration<int, std::milli> ms(150);
  // RCLCPP_INFO(rosnode->get_logger(), "Waiting for service result...");
  //  This provokes a lock here so we can not use it
  //  if (rclcpp::spin_until_future_complete(rosnode, result, ms) ==
  //     rclcpp::FutureReturnCode::SUCCESS)
  //  {
  //  Update the agents data

  // std::chrono::duration<int, std::milli> ms(300);
  auto res = result.wait_for(ms);
  if (res == std::future_status::ready)
  {
    //  RCLCPP_INFO(rosnode->get_logger(), "Service result received!");

    // for (auto ped : pedestrians) {
    //   for (auto a : result.get()->updated_agents.agents)
    //     if (ped.id == a.id) {
    //       RCLCPP_INFO(rosnode->get_logger(),
    //                   "OnUpdate Actor %s:\npx:%.2f, py:%.2f, plv:%.2f, "
    //                   "pvx:%.2f, pvy:%.2f, pva:%.2f\nnx:%.2f, ny:%.2f, "
    //                   "nlv:%.2f, nvx:%.2f, nvy:%.2f, nva: %.2f ",
    //                   ped.name.c_str(), ped.position.position.x,
    //                   ped.position.position.y, ped.linear_vel,
    //                   ped.velocity.linear.x, ped.velocity.linear.y,
    //                   ped.velocity.angular.z, a.position.position.x,
    //                   a.position.position.y, a.linear_vel,
    //                   a.velocity.linear.x, a.velocity.linear.y,
    //                   a.velocity.angular.z);
    //     }
    // }

    // update the Gazebo actors
    UpdateGazeboPedestrians(_info, result.get()->updated_agents);
  }
  else if (res == std::future_status::timeout)
  {
    RCLCPP_ERROR(rosnode->get_logger(), "Service /compute_agents timeout!!");
  }
  else
  {
    RCLCPP_ERROR(rosnode->get_logger(), "Failed to call service /compute_agents");
  }
  lastUpdate = _info.simTime;
}

GZ_REGISTER_WORLD_PLUGIN(HuNavPlugin)
}  // namespace hunav
