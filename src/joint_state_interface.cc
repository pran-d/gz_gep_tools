#include <gz_gep_tools/joint_state_interface.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace gz_transport_hw_tools {

LastState::LastState() :
    time_sec_(0),
    time_nsec_(0)
{}

void LastState::resize(size_t asize) {
  positions_.clear();
  positions_.resize(asize);
  velocities_.clear();
  velocities_.resize(asize);
  for(unsigned i=0;i<positions_.size();i++)
  {
    positions_[i]  = 0.0;
    velocities_[i] = 0.0;
  }
}

JointStateInterface::JointStateInterface(std::string &a_prefix_model_root,
                                         std::string &a_prefix_world,
                                         bool debug_level):
    debug_level_(debug_level) {
  prefix_model_root_ = a_prefix_model_root;
  prefix_world_ = a_prefix_world ;
  joint_state_topic_ =a_prefix_world +  a_prefix_model_root + "joint_state";

  node_.Subscribe<JointStateInterface,gz::msgs::Model>(joint_state_topic_,
                                                       &JointStateInterface::CallbackJointState,
                                                       this);
}

JointStateInterface::~JointStateInterface() {}

void JointStateInterface::SetListOfJoints(
    const std::vector<std::string> &a_list_of_joints)
{
  list_of_joints_.clear();
  list_of_joints_ = a_list_of_joints;

  cmd_force_topics_.clear();
  gz_pub_cmd_forces_.clear();

  unsigned int idx=0;
  for (auto jointItr = a_list_of_joints.begin();
       jointItr != a_list_of_joints.end();
       jointItr++) {
    /// Build a new string
    std::string a_new_cmd_force = prefix_model_root_ + std::string("joint/") +
        *jointItr + std::string("/cmd_force");
    cmd_force_topics_.push_back(a_new_cmd_force);
    /// Build advertise
    gz::transport::Node::Publisher a_gz_pub_cmd_force =
        node_.Advertise<gz::msgs::Double>(a_new_cmd_force);
    gz_pub_cmd_forces_.push_back(a_gz_pub_cmd_force);

    // Set map relationship between name and idx.
    map_name_2_indx_[*jointItr] = idx;
    idx++;
  }

  last_state_.resize(a_list_of_joints.size());

}

void JointStateInterface::CallbackJointState(
    const gz::msgs::Model&a_gz_model_msg
    ) {

  std::lock_guard(last_state_.lock_state_access_);
  unsigned int idx_joints=0;
  for (auto jointItr = a_gz_model_msg.joint().begin();
       jointItr != a_gz_model_msg.joint().end();
       jointItr++) {


    const ::gz::msgs::Axis &axis1 = jointItr->axis1();

    long unsigned int local_joint_idx = map_name_2_indx_[jointItr->name()];

    if (last_state_.positions_.size()!=0)
      last_state_.positions_[local_joint_idx] = axis1.position();
    if (last_state_.velocities_.size()!=0)
      last_state_.velocities_[local_joint_idx] = axis1.velocity();

    idx_joints++;
  }

  if (debug_level_)
  {
    std::cerr << "JointStateInterface::CallbackJointState: time: "
              << " " << last_state_.time_sec_
              << " " << last_state_.time_nsec_;
    if (last_state_.positions_.size()>1)
      std::cerr << " " << last_state_.positions_[1];

    std::cerr << std::endl;
  }
  last_state_.time_sec_ = a_gz_model_msg.header().stamp().sec();
  last_state_.time_nsec_ = a_gz_model_msg.header().stamp().nsec();


}

bool JointStateInterface::SetCmd( const std::vector<double> &a_cmd_vec_d)
{
  if (a_cmd_vec_d.size()!=gz_pub_cmd_forces_.size())
    return false;

  for (unsigned int i = 0; i < a_cmd_vec_d.size(); i++)
  {
    gz::msgs::Double msg;
    msg.set_data(a_cmd_vec_d[i]);

    if (!gz_pub_cmd_forces_[i].Publish(msg)){
      std::cerr << "Unable to publish on "
               << cmd_force_topics_[i]
               << std::endl;
      return false;
    }
  }
  return true;
}

bool JointStateInterface::GetPosVel(std::vector<double> &pos_vecd,
                                    std::vector<double> &vel_vecd,
                                    double &time)
{
  if ((pos_vecd.size()!= last_state_.positions_.size()) ||
      (vel_vecd.size()!= last_state_.velocities_.size()))
      return false;

  std::lock_guard(last_state_.lock_state_access_);
  time=last_state_.time_sec_ + 1e-9*last_state_.time_nsec_;
  for(std::vector<double>::size_type idx=0;
      idx < pos_vecd.size();
      idx++)
  {
    pos_vecd[idx] = last_state_.positions_[idx];
    vel_vecd[idx] = last_state_.velocities_[idx];
  }
  if (debug_level_)
    std::cerr << "JointStateInterface::GetPosVel: time: "
              << time << " "
              << pos_vecd[1] << " "
              << std::endl;

  return true;
}


};
