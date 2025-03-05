#include "gz_gep_tools/joint_state_interface.hh"
#include <cmath>
#include <cstdint>
#include <cstring>

#include <gz/math/Pose3.hh>
#include <gz/msgs/empty.pb.h>
#include <gz_gep_tools/control_over_gz.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>

namespace gz_transport_hw_tools {

ControlOverGz::ControlOverGz(std::string &world_prefix, bool debug_level)
    : world_prefix_(world_prefix), debug_level_(debug_level)
{
  /// Create control_gzsim topic name
  control_service_gzsim_ = world_prefix+std::string("/control");

  /// State gzsim service
  state_service_gzsim_ = world_prefix+std::string("/state");

  /// Set pose service
  set_pose_service_gzsim_ = world_prefix + std::string("/set_pose");

  std::string gazebo_clock_topic_name;
  gazebo_clock_topic_name = world_prefix + std::string("/clock");
  node_.Subscribe<ControlOverGz,gz::msgs::Clock>(gazebo_clock_topic_name,
                                                 &ControlOverGz::CallbackClock,
                                                 this);
  gz_sim_time_ = std::nan("1");

  wrld_ctrl_state_srv_gzsim_ = control_service_gzsim_ + std::string("/state");
}

void ControlOverGz::CallbackClock(const gz::msgs::Clock &a_gz_time_msg) {
  if (a_gz_time_msg.has_sim())
  {
    gz_sim_time_ = (double)a_gz_time_msg.sim().sec() +
        1e-9*(double)a_gz_time_msg.sim().nsec();
    if (debug_level_)
      std::cerr << "ControlOverGz::CallbackClock: Time: "  << a_gz_time_msg.sim().sec()
                << " "  <<  a_gz_time_msg.sim().nsec()
                << std::endl;
  }
}

double ControlOverGz::GetSimTime() {
  return gz_sim_time_;
}

bool ControlOverGz::Step()
{
  gz::msgs::WorldControl req_world_ctrl;
  gz::msgs::Boolean rep_bool;
  unsigned int timeout = 3000;

  /// Set reset for everything
  req_world_ctrl.set_multi_step(1);
  req_world_ctrl.set_pause(true);

  /// Do request.
  bool result=true;
  if (!node_.Request( control_service_gzsim_, req_world_ctrl, timeout, rep_bool, result)) {

    std::cerr << "Unable to send reset request !"
              << std::endl;
    result =false;
  }
      ///set_allocated_reset(&req_world_reset);
  return result;
}

bool ControlOverGz::Reset()
{
  gz::msgs::WorldControl req_world_ctrl;
  gz::msgs::WorldReset * req_world_reset;
  gz::msgs::Boolean rep_bool;
  unsigned int timeout = 3000;

  /// Set reset for everything
  req_world_reset = new gz::msgs::WorldReset();
  req_world_reset->set_all(true);
  req_world_ctrl.set_pause(true);
  /// Set the world reset inside the world control reset.
  req_world_ctrl.set_allocated_reset(req_world_reset);
  req_world_ctrl.set_multi_step(0);
  req_world_ctrl.set_step (false);
  req_world_ctrl.set_pause(true);
  req_world_ctrl.set_seed(0);

  /// Do request.
  bool result;
  if (!node_.Request( control_service_gzsim_, req_world_ctrl, timeout, rep_bool, result)) {

    std::cerr << "Unable to send reset request !"
              << std::endl;
    result =false;
  }
  /// World Reset will be released because out of scope.
  return result;
}

bool ControlOverGz::SendPauseRequest(bool abool)
{
  gz::msgs::WorldControl req_world_ctrl;
  gz::msgs::Boolean rep_bool;
  unsigned int timeout = 5000;

  /// Set the world reset inside the world control reset.
  req_world_ctrl.set_pause(abool);

  /// Do request.
  bool result;
  node_.Request( control_service_gzsim_, req_world_ctrl, timeout, rep_bool, result);
  return result;
}

bool ControlOverGz::Pause() { return SendPauseRequest(true); }

bool ControlOverGz::Start() { return SendPauseRequest(false); }

bool ControlOverGz::ReadWorldStateToInitECM()
{
  /// Gazebo messages
  /// Response as Serialized map of the entities
  gz::msgs::SerializedStepMap rep_serialized_step_map;
  /// Request does not have request argument.
  gz::msgs::Empty req_empty_msg;
  unsigned int timeout = 3000;
  gz::msgs::Boolean rep_bool;
  bool result=true;

  /// Get World Control State.
  /// by calling the /prefix_world/state Gazebo service.
  if (!node_.Request(state_service_gzsim_,
                     req_empty_msg,
                     timeout,
                     rep_serialized_step_map,
                     result)) {

    std::cerr << "Unable to send reset request ! timeout "
              << state_service_gzsim_ << " "
              << result
              << std::endl;
    result =false;
  }

  /// Initialization of the robot's Entity Component Manager.
  Robot_ECM_.SetState(*rep_serialized_step_map.mutable_state());

  return result;
}

void ControlOverGz::DisplayLinkValues()
{
  auto entities = Robot_ECM_.EntitiesByComponents( gz::sim::components::Link());

  for (auto it_entity = entities.begin();
       it_entity != entities.end();
       it_entity++)
  {
    gz::sim::Link a_link(*it_entity);
    std::string default_link_name("noname");
    std::cout << "Link name:" << a_link.Name(Robot_ECM_).value_or(default_link_name);
    std::vector<gz::sim::Entity> a_link_collisions = a_link.Collisions(Robot_ECM_);
    std::cout << "Collisions:" << a_link_collisions.size() << std::endl;
  }
}

void ControlOverGz::DisplayJointValues()
{
  auto entities = Robot_ECM_.EntitiesByComponents( gz::sim::components::Joint());

  for (auto it_Entity = entities.begin();
       it_Entity != entities.end();
       it_Entity++)
  {
    gz::sim::Joint aJoint(*it_Entity);
    //    std::cout << "Joint name:" << aJoint.Name(Robot_ECM_).value_or("No joint name");
    std::vector<double> empty_vecd;
    std::vector<double> pos_vecd = aJoint.Position(Robot_ECM_).value_or(empty_vecd);
    if (pos_vecd.size()!=0)
      std::cout << " " << "Position: " << pos_vecd[0];
    else
      std::cout << " no pos ";
    std::vector<double> vel_vecd = aJoint.Velocity(Robot_ECM_).value_or(empty_vecd);
    if (vel_vecd.size()!=0)
      std::cout << " " << "Velocity: " << vel_vecd[1];
    else
      std::cout << " no vel ";
    std::cout << std::endl;
  }
}

bool ControlOverGz::SendWorldControlStateToInitECM(const RobotCtrlJointInfos &rbt_ctrl_joint_infos,
                                                   std::vector<double> &aPose3d)
{
  gz::msgs::WorldControlState req_world_ctrl_state;
  gz::msgs::SerializedState * req_serialized_state;
  unsigned int timeout = 3000;
  gz::msgs::Boolean rep_bool;
  bool result=true;


  // Modify joint entities in Robot_ECM.
  auto entities = Robot_ECM_.EntitiesByComponents( gz::sim::components::Joint());

  for (auto it_Entity = entities.begin();
       it_Entity != entities.end();
       it_Entity++)
  {
    gz::sim::Joint aJoint(*it_Entity);
    std::string jointName =  aJoint.Name(Robot_ECM_).value_or("No joint name");
    //    std::cout << "Joint name:" << jointName;

    std::vector<double> one_vecd;
    auto a_ctrl_joint_info = rbt_ctrl_joint_infos.find(jointName);
    if (a_ctrl_joint_info != rbt_ctrl_joint_infos.end())
    {
      one_vecd.push_back(a_ctrl_joint_info->second.PosDes());
      //    std::cout << " " << one_vecd[0] << std::endl;
      aJoint.ResetPosition(Robot_ECM_,one_vecd);
    }
  }

  auto mentities = Robot_ECM_.EntitiesByComponents( gz::sim::components::Model());
  for (auto it_Entity = mentities.begin();
       it_Entity != mentities.end();
       it_Entity++)
  {
    gz::sim::Model aModel(*it_Entity);
    std::string modelName =  aModel.Name(Robot_ECM_);
    std::cout << "Model name:" << modelName << std::endl;

    if (modelName=="Pyrene")
    {
      gz::math::Pose3d aPose;
      aPose.Set( aPose3d[0], aPose3d[1], aPose3d[2],
                 aPose3d[3], aPose3d[4], aPose3d[5]);
      aModel.SetWorldPoseCmd(Robot_ECM_,aPose);
    }
  }


  /// Create the serialized state.
  req_serialized_state = new gz::msgs::SerializedState();
  ///  Generate the serialized state msg from the ECM.
  *req_serialized_state = Robot_ECM_.State();
  /// Link it in the WorldControlState msg
  req_world_ctrl_state.set_allocated_state(req_serialized_state);

  /// Set World Control State.
  /// by calling the /prefix_world/state/control Gazebo service.
  if (!node_.Request(wrld_ctrl_state_srv_gzsim_,
                     req_world_ctrl_state,
                     timeout,
                     rep_bool,
                     result)) {

    std::cerr << "Unable to send reset request ! timeout "
              << wrld_ctrl_state_srv_gzsim_ << " "
              << result
              << std::endl;
    result =false;
  }

  return true;
}

bool ControlOverGz::SetPose(double x, double y, double z,
                            double qx, double qy, double qz, double qw)
{
  gz::msgs::Pose pose_msg;
  gz::msgs::Boolean rep_bool;
  unsigned int timeout = 3000;
  bool result;

  gz::msgs::Vector3d * aPosition = pose_msg.mutable_position();
  aPosition->set_x(x); aPosition->set_y(y); aPosition->set_z(z);

  gz::msgs::Quaternion * aQuaternion  = pose_msg.mutable_orientation();
  aQuaternion->set_x(qx); aQuaternion->set_y(qy); aQuaternion->set_z(qz); aQuaternion->set_w(qw);

  /// Set Robot Pose
  /// by calling the /prefix_world/set_pose Gazebo service.
  if (!node_.Request(set_pose_service_gzsim_,
                     pose_msg,
                     timeout,
                     rep_bool,
                     result)) {

    std::cerr << "Unable to send reset request ! timeout "
              << wrld_ctrl_state_srv_gzsim_ << " "
              << result
              << std::endl;
    result =false;
  }

  return true;
}
}
