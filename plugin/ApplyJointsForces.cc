/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "ApplyJointsForces.hh"

#include <gz/msgs/double.pb.h>

#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include <gz/msgs/map_named_joints_forces.pb.h>

using namespace gz;
using namespace sim;
using namespace systems;


/// \brief A single 1-axis joint that is controlled by JointTrajectoryController
/// plugin
class ActuatedJoint
{
  /// \brief Default contructor
  public: ActuatedJoint() = default;

  /// \brief Constructor that properly configures the actuated joint
  /// \param[in] _entity Entity of the joint
  /// configuration
 public: ActuatedJoint(const Entity &an_entity): entity(an_entity) {};


  /// \brief Entity of the joint
  public: Entity entity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief The jointForceCmd.
  double jointForceCmd;

};

class gz::sim::systems::ApplyJointsForcesPrivate
{

    /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
  /// joint names are specified in the plugin configuration, all valid 1-axis
  /// joints are returned
  /// \param[in] _entity Entity of the model that the plugin is being
  /// configured for
  /// \param[in] _sdf SDF reference used to determine enabled joints
  /// \param[in] _ecm Gazebo Entity Component Manager
  /// \return List of entities containinig all enabled joints
  public: std::vector<Entity> GetEnabledJoints(
              const Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              EntityComponentManager &_ecm) const;


  /// \brief Reset internals of the plugin, without affecting already created
  /// components
  public: void Reset();

  /// \brief Callback for joint force subscription
  /// \param[in] _msg Joint force message
 public: void CallbackCmdForce(const gz::msgs::MapNamedJointsForces &_msg);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  //! [jointEntityDeclaration]

  /// \brief Commanded joint force
  //! [forceDeclaration]
  public: std::map<std::string, ActuatedJoint> actuatedJoints;
  //! [forceDeclaration]

  /// \brief mutex to protect jointForceCmd
  public: std::mutex jointForceCmdMutex;

  /// \brief Model interface
  //! [modelDeclaration]
  public: Model model{kNullEntity};
  //! [modelDeclaration]<q

  public:
  template <typename T>
    std::vector<T> Parse(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_parameterName) const;

};

//////////////////////////////////////////////////
ApplyJointsForces::ApplyJointsForces()
  : dataPtr(std::make_unique<ApplyJointsForcesPrivate>())
{
}

//////////////////////////////////////////////////
template <typename T>
std::vector<T> ApplyJointsForcesPrivate::Parse(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_parameterName) const
{
  std::vector<T> output;

  if (_sdf->HasElement(_parameterName))
  {
    auto param = _sdf->FindElement(_parameterName);
    while (param)
    {
      output.push_back(param->Get<T>());
      param = param->GetNextElement(_parameterName);
    }
  }
  return output;
}

//////////////////////////////////////////////////
//! [Configure]
void ApplyJointsForces::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  //! [Configure]

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "ApplyJoinstForces plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  gzmsg << "[ApplyJointsForces] Setting up controller for ["
         << this->dataPtr->model.Name(_ecm) << "(Entity=" << _entity << ")].\n";

  auto sdfClone = _sdf->Clone();

  // Get list of enabled joints
  const auto enabledJoints = this->dataPtr->GetEnabledJoints(_entity,
                                                             _sdf,
                                                             _ecm);


  // Iterate over all enabled joints and create/configure them
  for (const auto &jointEntity : enabledJoints)
  {
    const auto jointName =
        _ecm.Component<components::Name>(jointEntity)->Data();
    this->dataPtr->actuatedJoints[jointName] =
        ActuatedJoint(jointEntity);
    gzmsg << "[ApplyJointsForces] Configured joint ["
           << jointName << "(Entity=" << jointEntity << ")].\n";
  }

  // Make sure at least one joint is configured
  if (this->dataPtr->actuatedJoints.empty())
  {
    gzerr << "[JointTrajectoryController] Failed to initialize because ["
           << this->dataPtr->model.Name(_ecm) << "(Entity=" << _entity
           << ")] has no supported joints.\n";
    return;
  }


  // Subscribe to commands
  //! [cmdTopic]
  auto topic = _sdf->Get<std::string>("topic");
  //! [cmdTopic]
  // TODO: Use the model namespace.
  topic = "/model/" + this->dataPtr->model.Name(_ecm) + "/joints/" + topic;


  // Make sure the topic is valid
  const auto validApplyJointsForcesTopic = transport::TopicUtils::AsValidTopic(
      topic);
  if (validApplyJointsForcesTopic.empty())
  {
    gzerr << "[ApplyJointsForces] Cannot subscribe to invalid topic ["
           << topic << "].\n";
    gzerr << "/model/" +
        this->dataPtr->model.Name(_ecm) + "/joints/cmd_forces\n";
    return;
  }
 // Subscribe
  gzmsg << "[ApplyJointsForces] Subscribing to ApplyJointsForces"
            " commands on topic [" << validApplyJointsForcesTopic << "].\n";

  //! [cmdSub]
  this->dataPtr->node.Subscribe(validApplyJointsForcesTopic,
                                &ApplyJointsForcesPrivate::CallbackCmdForce,
                                this->dataPtr.get());
  //! [cmdSub]

  gzmsg << "ApplyJointsForces subscribing to Double messages on [" << topic
         << "]" << std::endl;
}

//////////////////////////////////////////////////
void ApplyJointsForces::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ApplyJointsForces::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;

  }


  // Iterator over all the actuated joints
  for (auto an_actuated_joint = this->dataPtr->actuatedJoints.begin();
       an_actuated_joint != this->dataPtr->actuatedJoints.end();
       an_actuated_joint++)
  {
    // If the joint hasn't been identified yet, look for it
    //! [findJoint]
    if (an_actuated_joint->second.entity == kNullEntity)
    {
      an_actuated_joint->second.entity =
          this->dataPtr->model.JointByName(_ecm, an_actuated_joint->first);
    }
    //! [findJoint]

    if (an_actuated_joint->second.entity == kNullEntity)
      continue;


    // Update joint force
    //! [jointForceComponent]
    auto force = _ecm.Component<components::JointForceCmd>(
        an_actuated_joint->second.entity);
    //! [jointForceComponent]

    std::lock_guard<std::mutex> lock(this->dataPtr->jointForceCmdMutex);

    //! [modifyComponent]
    if (force == nullptr)
    {
      _ecm.CreateComponent(
          an_actuated_joint->second.entity,
          components::JointForceCmd({an_actuated_joint->second.jointForceCmd}));
    }
    else
    {
      force->Data()[0] = an_actuated_joint->second.jointForceCmd;
    }
  }
    //! [modifyComponent]
}

//////////////////////////////////////////////////
//! [setForce]
void ApplyJointsForcesPrivate::CallbackCmdForce(
    const gz::msgs::MapNamedJointsForces &mnjf_msg)
{
  std::lock_guard<std::mutex> lock(this->jointForceCmdMutex);
  for ( auto mnjf_it =  mnjf_msg.jointsforces().begin();
        mnjf_it !=  mnjf_msg.jointsforces().end();
        mnjf_it++)
  {
    actuatedJoints[mnjf_it->first].jointForceCmd = mnjf_it->second;
  }
}
//! [setForce]

std::vector<Entity> ApplyJointsForcesPrivate::GetEnabledJoints(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm) const
{
  std::vector<Entity> output;

  // Get list of user-enabled joint names. If empty, enable all 1-axis joints
  const auto enabledJoints = Parse<std::string>(_sdf,
                                                "joint_name");

  // Get list of joint entities of the model
  // If there are joints explicitely enabled by the user, get only those
  std::vector<Entity> jointEntities;
  if (!enabledJoints.empty())
  {
    for (const auto &enabledJointName : enabledJoints)
    {
      auto enabledJointEntity = _ecm.ChildrenByComponents(
          _entity, components::Joint(), components::Name(enabledJointName));
      // Check that model has exactly one joint that matches the name
      if (enabledJointEntity.empty())
      {
        gzerr << "[ApplyJointsForces] Model does not contain joint ["
               << enabledJointName << "], which was explicitly enabled.\n";
        continue;
      }
      else if (enabledJointEntity.size() > 1)
      {
        gzwarn << "[ApplyJointsForces] Model has "
                << enabledJointEntity.size() << " duplicate joints named ["
                << enabledJointName << "]. Only the first (Entity="
                << enabledJointEntity[0] << ") will be configured.\n";
      }
      // Add entity to the list of enabled joints
      jointEntities.push_back(enabledJointEntity[0]);
    }
  }
  else
  {
    jointEntities = _ecm.ChildrenByComponents(_entity, components::Joint());
  }

  // Iterate over all joints and verify whether they can be enabled or not
  for (const auto &jointEntity : jointEntities)
  {
    const auto jointName = _ecm.Component<components::Name>(
                                   jointEntity)->Data();

    // Ignore duplicate joints
    for (const auto &actuatedJoint : this->actuatedJoints)
    {
      if (actuatedJoint.second.entity == jointEntity)
      {
        gzwarn << "[ApplyJointsForces] Ignoring duplicate joint ["
                << jointName << "(Entity=" << jointEntity << ")].\n";
        continue;
      }
    }

    // Make sure the joint type is supported, i.e. it has exactly one
    // actuated axis
    const auto *jointType = _ecm.Component<components::JointType>(jointEntity);
    switch (jointType->Data())
    {
      case sdf::JointType::PRISMATIC:
      case sdf::JointType::REVOLUTE:
      case sdf::JointType::CONTINUOUS:
      case sdf::JointType::GEARBOX:
      {
        // Supported joint type
        break;
      }
      case sdf::JointType::FIXED:
      {
        gzdbg << "[ApplyJointsForces] Fixed joint [" << jointName
               << "(Entity=" << jointEntity << ")] is skipped.\n";
        continue;
      }
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::BALL:
      case sdf::JointType::UNIVERSAL:
      {
        gzwarn << "[ApplyJointsForces] Joint [" << jointName
                << "(Entity=" << jointEntity
                << ")] is of unsupported type. Only joints with a single axis"
                   " are supported.\n";
        continue;
      }
      default:
      {
        gzwarn << "[ApplyJointsForces] Joint [" << jointName
                << "(Entity=" << jointEntity << ")] is of unknown type.\n";
        continue;
      }
    }
    output.push_back(jointEntity);
  }

  return output;
}



GZ_ADD_PLUGIN(ApplyJointsForces,
                    System,
                    ApplyJointsForces::ISystemConfigure,
                    ApplyJointsForces::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ApplyJointsForces,
                    "gz::sim::systems::ApplyJointsForces")
