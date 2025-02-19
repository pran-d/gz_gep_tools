#pragma once
/// Standard includes
#include <vector>
#include <string>
#include <memory>

/// GZ includes
#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace gz_transport_hw_tools {

class ControlJointValue {
 public:
  /// Desired quantities
  double pos_des, vel_des;

  // Control gains
  double Kp, Kd;

  // Measured quantities
  double pos_mes, vel_mes;

  double cmd;

  void compute_cmd()
  {
    cmd = Kp*(pos_des - pos_mes) + Kd*(vel_des -vel_mes);
  }
};

typedef std::map<std::string, ControlJointValue> RobotCtrlJointInfos;

bool SaveCmd(const RobotCtrlJointInfos & a_rbt_ctrl_joint_infos,
                   const std::string &afilename);
bool SavePos(const RobotCtrlJointInfos & a_rbt_ctrl_joint_infos,
                   const std::string &afilename);                     


class JointValues {
 public:
  /// Measured quantities
  double pos_mes;
  double vel_mes;
  /// Control
  double force_ctrl;

  std::string cmd_force_topic;
  gz::transport::Node::Publisher gz_pub_cmd_force;
};


class RobotJoints {
 public:
  /// Time
  int64_t time_sec_;
  int64_t time_nsec_;

  /// Map of joints values
  std::map<std::string, JointValues> dict_joint_values;
  
  std::mutex lock_state_access_;
  
  RobotJoints();
};


/// This class handles the interface to the joints and the sensors
/// of a robot simulated by Gazebo (Harmonic)
class JointStateInterface {
 public:
  /// Constructor
  JointStateInterface(std::string &a_prefix_model_root,
                      std::string &a_prefix_world,
                      bool debug_level=false);
  /// Destructor
  ~JointStateInterface();

  /// Specify the list of joints.
  void SetListOfJoints(const RobotCtrlJointInfos & rbt_ctrl_joint_infos);


  bool SetCmd(const RobotCtrlJointInfos &rbt_ctrl_joint_infos);

  bool GetPosVel(RobotCtrlJointInfos &rbt_ctrl_joint_infos,
                 double &time);

  private:

  /// Callback function for state model update.
  void CallbackJointState(const gz::msgs::Model &a_gz_model_msg);

  /// Root of the prefix model
  std::string prefix_model_root_;

  /// Prefix world
  std::string prefix_world_;

  /// List of joints
  std::vector<std::string> list_of_joints_;

  /// Map joints to index in list
  std::map<std::string, std::size_t> map_name_2_indx_;

  /// Topic name joint state
  std::string joint_state_topic_;

  /// GZ node
  gz::transport::Node node_;

  /// Last state of the robot
  RobotJoints robot_joints_;

  bool debug_level_;
};

}
