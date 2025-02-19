#include <string>

#include "gz_gep_tools/joint_state_interface.hh"
#include "gz_gep_tools/control_over_gz.hh"

namespace gz_transport_hw_tools {

class PerceptionActionLoop {

 public:
  // Default constructore
  PerceptionActionLoop(gz_transport_hw_tools::RobotCtrlJointInfos & a_robot_ctrl_joint_infos,
                       std::string & a_prefix_model_root,
                       std::string & a_prefix_world,
                       bool debug_level);

  /// Reset the robot to the desired state inside robot_ctrl_joint_infos_
  int InitGz();

  /// Main loop
  int MainLoop();
  
 protected:
  /// Information for control
  gz_transport_hw_tools::RobotCtrlJointInfos & robot_ctrl_joint_infos_;
  
  /// Informmation from perception and the simulator
  gz_transport_hw_tools::JointStateInterface joint_state_interface_;

  /// Informmation from perception and the simulator
  gz_transport_hw_tools::ControlOverGz control_over_gz_;

  /// Time related variables
  double state_gz_time_;
  double pre_state_gz_time_;
  unsigned long long int local_time_;
  
};

}
