#include <gz_gep_tools/joint_state_interface.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace gz_transport_hw_tools {

JointStateInterface::JointStateInterface(std::string &a_prefix_model_root,
                                         std::string &a_prefix_world) {
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

  for (auto jointItr = a_list_of_joints.begin();
       jointItr != a_list_of_joints.end();
       jointItr++) {
    std::string a_new_cmd_force = prefix_model_root_ + std::string("joint/") +
        *jointItr + std::string("/cmd_force");
    cmd_force_topics_.push_back(a_new_cmd_force);
    gz::transport::Node::Publisher a_gz_pub_cmd_force =
        node_.Advertise<gz::msgs::Double>(a_new_cmd_force);
    gz_pub_cmd_forces_.push_back(a_gz_pub_cmd_force);
  }
  positions_.clear();
  positions_.resize(a_list_of_joints.size());
  velocities_.clear();
  velocities_.resize(a_list_of_joints.size());
}

void JointStateInterface::CallbackJointState(
    const gz::msgs::Model&a_gz_model_msg
    ) {
  unsigned int idx_joints=0;
  for (auto jointItr = a_gz_model_msg.joint().begin();
       jointItr != a_gz_model_msg.joint().end();
       jointItr++) {
    idx_joints++;

    const ::gz::msgs::Axis &axis1 = jointItr->axis1();


    if (positions_.size()!=0)
      positions_[idx_joints] = axis1.position();
    if (velocities_.size()!=0)
      velocities_[idx_joints] = axis1.velocity();

  }
  
}

bool JointStateInterface::SetCmd( const std::vector<double> &a_cmd_vec_d)
{
  if (a_cmd_vec_d.size()!=gz_pub_cmd_forces_.size())
    return false;
  
  for (unsigned int i = 0; i < a_cmd_vec_d.size(); i++) {
    gz::msgs::Double msg;
    msg.set_data(a_cmd_vec_d[i]);
    
    if (!gz_pub_cmd_forces_[i].Publish(msg)){
      std::cerr << "Unable to publish on "
               << cmd_force_topics_[i]
               << std::endl;
    }
  }
  return true;
}

void JointStateInterface::GetPosVel(std::vector<double> &pos_vecd,
                                    std::vector<double> &vel_vecd)
{
  for(std::vector<double>::size_type idx=0;
      idx < pos_vecd.size();
      idx++)
  {
    pos_vecd[idx] = positions_[idx];
    vel_vecd[idx] = velocities_[idx];    
  }
}
};
