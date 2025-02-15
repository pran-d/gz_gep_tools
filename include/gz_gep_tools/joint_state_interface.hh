#pragma once
/// Standard includes
#include <vector>
#include <string>

/// GZ includes
#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace gz_transport_hw_tools {

class LastState {
 public:
  /// Time
  int64_t time_sec_;
  int64_t time_nsec_;

  /// Position
  std::vector<double> positions_;

  /// Velocity
  std::vector<double> velocities_;

  void resize(std::size_t asize);

  std::mutex lock_state_access_;
  LastState();
};

class JointValues {
  double pos_mes;
  double vel_mes;
  double force_ctrl;
  std::string cmd_force_topic;
  gz::transport::Node::Publisher;
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
  void SetListOfJoints(const std::vector<std::string> & a_list_of_joints);


  bool SetCmd(const std::vector<double> &cmd_vec);

  bool GetPosVel(std::vector<double> &pos_vecd,
                 std::vector<double> &vel_vecd,
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

  /// Vector of string describing the topics to command forces on actuators.
  std::vector<std::string> cmd_force_topics_;

  /// Topic name joint state
  std::string joint_state_topic_;

  /// GZ published for the cmd_force.
  std::vector<gz::transport::Node::Publisher> gz_pub_cmd_forces_;

  /// GZ node
  gz::transport::Node node_;

  /// Last state of the robot
  LastState last_state_;

  bool debug_level_;
};

}
