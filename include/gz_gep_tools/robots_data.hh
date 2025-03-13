#pragma once
#include <vector>
#include <gz_gep_tools/joint_state_interface.hh>

namespace gz_transport_hw_tools
{

/// Data related to TALOS
extern RobotCtrlJointInfos talos_ctrl_joint_infos;
extern std::vector<double> talos_pose3d;

/// Data related to H1_v2 full model
extern RobotCtrlJointInfos h1_v2_ctrl_joint_infos;
extern std::vector<double> h1_v2_pose3d;

extern RobotCtrlJointInfos h1_v2_woh_ctrl_joint_infos;
extern std::vector<double> h1_v2_woh_pose3d;

extern std::string talos_prefix_model_root;
extern std::string talos_prefix_world;

extern std::string h1_v2_prefix_model_root;
extern std::string h1_v2_prefix_world;

extern std::string h1_v2_woh_prefix_model_root;
extern std::string h1_v2_woh_prefix_world;



};
