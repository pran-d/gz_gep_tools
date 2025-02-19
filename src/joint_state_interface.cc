#include <fstream>
#include <iostream>

#include <gz_gep_tools/joint_state_interface.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace gz_transport_hw_tools {

bool SavePos(const RobotCtrlJointInfos &a_rbt_ctrl_joint_infos,
             const std::string &afilename) 
{
  std::ofstream ofs_position_robot(afilename.c_str(), std::ios::app);
  for(auto it_joint_info = a_rbt_ctrl_joint_infos.begin();
      it_joint_info != a_rbt_ctrl_joint_infos.end();
      it_joint_info++)
    ofs_position_robot << it_joint_info->second.pos_mes << " ";
  ofs_position_robot << std::endl;
  ofs_position_robot.close();
  return true;
}

bool SaveCmd(const RobotCtrlJointInfos &a_rbt_ctrl_joint_infos,
             const std::string &afilename) 
{
  std::ofstream ofs_position_robot(afilename.c_str(), std::ios::app);
  for(auto it_joint_info = a_rbt_ctrl_joint_infos.begin();
      it_joint_info != a_rbt_ctrl_joint_infos.end();
      it_joint_info++)
    ofs_position_robot << it_joint_info->second.cmd << " ";
  ofs_position_robot << std::endl;
  ofs_position_robot.close();
  return true;
}

RobotJoints::RobotJoints() :
    time_sec_(0),
    time_nsec_(0)
{}


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
    const RobotCtrlJointInfos &rbt_ctrl_joint_infos)
{
  robot_joints_.dict_joint_values.clear();

  for (auto jointItr = rbt_ctrl_joint_infos.begin();
       jointItr != rbt_ctrl_joint_infos.end();
       jointItr++) {
    /// Build a new string
    std::string a_new_cmd_force = prefix_model_root_ + std::string("joint/") +
        jointItr->first + std::string("/cmd_force");
    
    /// Build advertise
    robot_joints_.dict_joint_values[jointItr->first].gz_pub_cmd_force = 
        node_.Advertise<gz::msgs::Double>(a_new_cmd_force);
    
    robot_joints_.dict_joint_values[jointItr->first].cmd_force_topic = a_new_cmd_force;    
    
  }

}

void JointStateInterface::CallbackJointState(
    const gz::msgs::Model&a_gz_model_msg
    ) {

  std::lock_guard(robot_joints_.lock_state_access_);

  for (auto jointItr = a_gz_model_msg.joint().begin();
       jointItr != a_gz_model_msg.joint().end();
       jointItr++) {

    const ::gz::msgs::Axis &axis1 = jointItr->axis1();

    robot_joints_.dict_joint_values[jointItr->name()].pos_mes = axis1.position();
    robot_joints_.dict_joint_values[jointItr->name()].vel_mes = axis1.velocity();

  }

  if (debug_level_)
  {
    std::cerr << "JointStateInterface::CallbackJointState: time: "
              << " " << robot_joints_.time_sec_
              << " " << robot_joints_.time_nsec_;
    if (robot_joints_.dict_joint_values["arm_left_1_joint"].pos_mes)
      std::cerr << " " << robot_joints_.dict_joint_values["arm_left_1_joint"].pos_mes;

    std::cerr << std::endl;
  }
  robot_joints_.time_sec_ = a_gz_model_msg.header().stamp().sec();
  robot_joints_.time_nsec_ = a_gz_model_msg.header().stamp().nsec();


}

bool JointStateInterface::SetCmd( const RobotCtrlJointInfos &rbt_ctrl_joint_infos)
{
  if (rbt_ctrl_joint_infos.size()>robot_joints_.dict_joint_values.size())
  {
    std::cerr << "rbt_ctrl_joint_infos.size(): "
              << rbt_ctrl_joint_infos.size()
              << " robot_joints_.dict_joint_values.size()"
              << robot_joints_.dict_joint_values.size()
              << std::endl;
    return false;
  }
  // Iterate over the cmd map.
  for (auto cmd_it = rbt_ctrl_joint_infos.begin();
       cmd_it != rbt_ctrl_joint_infos.end();
       cmd_it++)    
  {
    gz::msgs::Double msg;
    msg.set_data(cmd_it->second.cmd);

    auto it_robot_joint_value = robot_joints_.dict_joint_values.find(cmd_it->first);
    // If the joint is not found go to the next iteration.
    if (it_robot_joint_value == robot_joints_.dict_joint_values.end())
      continue;
    
    if (!it_robot_joint_value->second.gz_pub_cmd_force.Publish(msg)){
      std::cerr << "Unable to publish on "
                << it_robot_joint_value->second.cmd_force_topic
                << std::endl;
      continue;
    } 
    if (debug_level_)
    {
      std::cout << " Publish " << cmd_it->second.cmd << " on "
                << robot_joints_.dict_joint_values[cmd_it->first].cmd_force_topic
                << std::endl;
    }
  }
  return true;
}

bool JointStateInterface::GetPosVel(RobotCtrlJointInfos &rbt_ctrl_joint_infos,
                                    double &time)
{
  if (rbt_ctrl_joint_infos.size()> robot_joints_.dict_joint_values.size())
  {
    std::cerr << "rbt_ctrl_joint_infos.size(): " << rbt_ctrl_joint_infos.size()
              << " robot_joints_.dict_joint_values.size():"  << robot_joints_.dict_joint_values.size()
              << std::endl;
      return false;
  }
  std::lock_guard(robot_joints_.lock_state_access_);
  time=(double)robot_joints_.time_sec_ + 1e-9*(double)robot_joints_.time_nsec_;
  for (auto joint_it = rbt_ctrl_joint_infos.begin();
       joint_it != rbt_ctrl_joint_infos.end();
       joint_it++)
  {
    joint_it->second.pos_mes = robot_joints_.dict_joint_values[joint_it->first].pos_mes;
    joint_it->second.vel_des = robot_joints_.dict_joint_values[joint_it->first].vel_mes;
  }
  if (debug_level_)
    std::cerr << "JointStateInterface::GetPosVel: time: "
              << time << " "
              << rbt_ctrl_joint_infos["arm_left_1_joint"].pos_mes << " "
              << std::endl;

  return true;
}


};
