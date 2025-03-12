#pragma once
/// Standard includes
#include <vector>
#include <string>
#include <memory>

/// GZ includes
#include <gz/msgs/model.pb.h>
#include <gz/transport.hh>

namespace gz_transport_hw_tools {
/// This class is to be used by the user to provide:
/// @param Kp: Gain to reach the desired position pos_des
/// @param Kd: Gain to reach the desired velocity vel_des
/// @param Ki; Gain for the integral error
/// @param i_clamp: Clamping the integral action
/// @param pos_des: Desired position (used for the initailization phase, i.e. at
/// start up)
/// @param vel_des: Velocity position (used for the initailization phase, i.e.
/// at start up)
/// Each joint has such a structure which can initialized in a container here a map.
class ControlJointValue {
 public:

  ControlJointValue( double Kp,
                     double Kd,
                     double Ki,
                     double i_clamp,
                     double pos_des,
                     double vel_des);

  ControlJointValue(const ControlJointValue &other);

  ControlJointValue(const ControlJointValue &&other) noexcept;

  double Cmd() const { return cmd_;}
  double PosMes() const { return pos_mes_;}
  double VelMes() const { return vel_mes_;}
  double ForceMes() const { return force_mes_;}
  double PosDes() const { return pos_des_;}
  double VelDes() const { return vel_des_;}

  void SetPosMes(double &pos_mes) { pos_mes_ = pos_mes;}
  void SetVelMes(double &vel_mes) { vel_mes_ = vel_mes;}
  void SetForceMes(double &force_mes) { force_mes_ = force_mes;}

  void ComputeCmd();

 private:

  /// Desired quantities
  double pos_des_, vel_des_;

  // Control gains
  double Kp_, Kd_, Ki_;

  // Clamping i value to i_clamp
  double i_clamp_;

  // Measured quantities
  double pos_mes_, vel_mes_, force_mes_;

  /// Command
  double cmd_;

  /// Cumulative error
  double cumul_err_;


};

/// Definition of the type handling a map of controlled joint
typedef std::map<std::string, ControlJointValue> RobotCtrlJointInfos;

bool SaveCmd(const RobotCtrlJointInfos & a_rbt_ctrl_joint_infos,
             const std::string &afilename);
bool SavePos(const RobotCtrlJointInfos & a_rbt_ctrl_joint_infos,
             const std::string &afilename);
bool SavePosDes(const RobotCtrlJointInfos & a_rbt_ctrl_joint_infos,
                const std::string &afilename);
bool SaveListOfNamedJoints(const RobotCtrlJointInfos &a_rbt_ctrl_joint_infos,
                           const std::string &afilename);

/// Intermediate structure used to read information from Gazebo "joint_name/joint_state" topic.
///
class GZJointValues {
 public:
  /// Measured quantities
  double pos_mes;
  double vel_mes;
  double force_mes;
  /// Control
  double force_ctrl;
};


class GZRobotJoints {
 public:
  /// Time
  int64_t time_sec_;
  int64_t time_nsec_;

  /// Map of joints values
  std::map<std::string, GZJointValues> dict_joint_values;

  std::mutex lock_state_access_;

  GZRobotJoints();
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

  /// Topic name joint state
  std::string joint_state_topic_;

  /// GZ node
  gz::transport::Node node_;

  /// Last state of the robot from Gazebo
  GZRobotJoints gz_robot_joints_;

  bool debug_level_;

  std::string topic_name_gz_pub_named_joints_forces_;
  gz::transport::Node::Publisher gz_pub_named_joints_forces_;
};

}
