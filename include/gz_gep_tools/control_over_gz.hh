#pragma once
/// Standard includes
#include <gz/sim/EntityComponentManager.hh>
#include <vector>
#include <string>

/// GZ includes
#include <gz/msgs.hh>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/world_control_state.pb.h>
#include <gz/transport.hh>

#include "joint_state_interface.hh"

namespace gz_transport_hw_tools {

class ControlOverGz {
 public:

  /// Provides the gazebo name.
  ControlOverGz(std::string &world_prefix, bool debug_level=false);

  /// Pause Gazebosim
  bool Pause();

  /// Start Gazebo
  bool Start();

  /// Reset Gazebosim
  bool Reset();

  /// Performing a simulation step
  bool Step();

  /// Getting the simualted time
  double GetSimTime();

  /// Send World Control state to Init ECM.
  bool SendWorldControlStateToInitECM(const RobotCtrlJointInfos &rbt_ctrl_joint_infos);

  /// Set Pose
  bool SetPose(double x, double y, double z,
               double qx, double qy, double qz, double qw);

  /// Initialization of the ECM by reading the world control state
  bool ReadWorldStateToInitECM();

  /// Display joint values
  void DisplayJointValues();

  /// Display link values
  void DisplayLinkValues();

 protected:


  /// Reading GZ sim time
  void CallbackClock(const gz::msgs::Clock &);

  /// Send a Pause request
  bool SendPauseRequest(bool abool);

  /// World prefix
  std::string world_prefix_;

  /// Name of the control service
  std::string control_service_gzsim_;

  /// Name of state service
  std::string state_service_gzsim_;

  /// Name of world control state service.
  std::string wrld_ctrl_state_srv_gzsim_;

  /// Name of set pose service.
  std::string set_pose_service_gzsim_;

  /// GZ node
  gz::transport::Node node_;

  /// Time
  double gz_sim_time_;

  bool debug_level_;

  /// Robot Entity Component Manager
  gz::sim::EntityComponentManager Robot_ECM_;
};

}
