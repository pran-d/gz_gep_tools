#include <fstream>
#include <iostream>

#include <gz_gep_tools/joint_state_interface.hh>
#include <gz/msgs.hh>
#include <gz/msgs/model.pb.h>
#include <gz/transport.hh>

#include <gz/msgs/map_named_joints_forces.pb.h>

namespace gz_transport_hw_tools {

ControlJointValue::ControlJointValue(double Kp,
                                     double Kd,
                                     double Ki,
                                     double i_clamp,
                                     double pos_des,
                                     double vel_des):
    pos_des_(pos_des), vel_des_(vel_des), Kp_(Kp),
    Kd_(Kd), Ki_(Ki), i_clamp_(i_clamp), pos_mes_(0.0),
    vel_mes_(0.0), cmd_(0.0), cumul_err_(0.0) {
}
ControlJointValue::ControlJointValue(const ControlJointValue &other)
    : ControlJointValue(other.Kp_, other.Kd_, other.Ki_,
                        other.i_clamp_,
                        other.pos_des_, other.vel_des_)
{
  pos_mes_ = other.pos_mes_;
  vel_mes_ = other.vel_mes_;
  cmd_ = other.cmd_;
  cumul_err_ = other.cumul_err_;
}

ControlJointValue::ControlJointValue(const ControlJointValue &&other) noexcept
    :  ControlJointValue(other.Kp_, other.Kd_, other.Ki_,
                         other.i_clamp_,
                         other.pos_des_, other.vel_des_ )
{
  pos_mes_ = other.pos_mes_;
  vel_mes_ = other.vel_mes_;
  cmd_ = other.cmd_;
  cumul_err_ = other.cumul_err_;
}

void ControlJointValue::ComputeCmd()
{
  double err_pos = pos_des_ - pos_mes_;
  double i_term = Ki_*cumul_err_;
  if (i_term > i_clamp_)
    i_term = i_clamp_;
  else if (i_term < -i_clamp_)
    i_term = - i_clamp_;

  cmd_ = Kp_* err_pos + Kd_*(vel_des_ -vel_mes_) + i_term;
  cumul_err_ += err_pos;
}

bool SavePos(const RobotCtrlJointInfos &a_rbt_ctrl_joint_infos,
             const std::string &afilename)
{
  std::ofstream ofs_position_robot(afilename.c_str(), std::ios::app);
  for(auto it_joint_info = a_rbt_ctrl_joint_infos.begin();
      it_joint_info != a_rbt_ctrl_joint_infos.end();
      it_joint_info++)
    ofs_position_robot << it_joint_info->second.PosMes() << " ";
  ofs_position_robot << std::endl;
  ofs_position_robot.close();
  return true;
}

bool SavePosDes(const RobotCtrlJointInfos &a_rbt_ctrl_joint_infos,
             const std::string &afilename)
{
  std::ofstream ofs_position_robot(afilename.c_str(), std::ios::app);
  for(auto it_joint_info = a_rbt_ctrl_joint_infos.begin();
      it_joint_info != a_rbt_ctrl_joint_infos.end();
      it_joint_info++)
    ofs_position_robot << it_joint_info->second.PosDes() << " ";
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
    ofs_position_robot << it_joint_info->second.Cmd() << " ";
  ofs_position_robot << std::endl;
  ofs_position_robot.close();
  return true;
}

bool SaveListOfNamedJoints(const RobotCtrlJointInfos &a_rbt_ctrl_joint_infos,
                           const std::string &afilename)
{
  std::ofstream ofs_position_robot(afilename.c_str(), std::ios::app);
  for(auto it_joint_info = a_rbt_ctrl_joint_infos.begin();
      it_joint_info != a_rbt_ctrl_joint_infos.end();
      it_joint_info++)
    ofs_position_robot << it_joint_info->first << " ";
  ofs_position_robot << std::endl;
  ofs_position_robot.close();
  return true;
}

GZRobotJoints::GZRobotJoints() :
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

  if (debug_level)
    std::cout << "Subscribe to " << joint_state_topic_ << std::endl;

  node_.Subscribe<JointStateInterface,gz::msgs::Model>(joint_state_topic_,
                                                       &JointStateInterface::CallbackJointState,
                                                       this);

  topic_name_gz_pub_named_joints_forces_ = a_prefix_model_root + "joints/cmd_forces";
  gz_pub_named_joints_forces_ =
      node_.Advertise<gz::msgs::MapNamedJointsForces>(topic_name_gz_pub_named_joints_forces_);
}

JointStateInterface::~JointStateInterface() {}

void JointStateInterface::SetListOfJoints(
    const RobotCtrlJointInfos &rbt_ctrl_joint_infos)
{
  gz_robot_joints_.dict_joint_values.clear();

  for (auto jointItr = rbt_ctrl_joint_infos.begin();
       jointItr != rbt_ctrl_joint_infos.end();
       jointItr++) {
    /// Build advertise
    GZJointValues & agzjointvalues = gz_robot_joints_.dict_joint_values[jointItr->first];
    agzjointvalues.pos_mes = 0.0;
    agzjointvalues.vel_mes = 0.0;
    agzjointvalues.force_ctrl = 0.0;

  }

}

void JointStateInterface::CallbackJointState(
    const gz::msgs::Model&a_gz_model_msg
    ) {

  std::lock_guard(gz_robot_joints_.lock_state_access_);

  for (auto jointItr = a_gz_model_msg.joint().begin();
       jointItr != a_gz_model_msg.joint().end();
       jointItr++) {

    const ::gz::msgs::Axis &axis1 = jointItr->axis1();

    gz_robot_joints_.dict_joint_values[jointItr->name()].pos_mes = axis1.position();
    gz_robot_joints_.dict_joint_values[jointItr->name()].vel_mes = axis1.velocity();

  }

  //  if (debug_level_)
  {
    std::cerr << "JointStateInterface::CallbackJointState: time: "
              << " " << gz_robot_joints_.time_sec_
              << " " << gz_robot_joints_.time_nsec_;
    if (gz_robot_joints_.dict_joint_values["arm_left_1_joint"].pos_mes)
      std::cerr << " " << gz_robot_joints_.dict_joint_values["arm_left_1_joint"].pos_mes;

    std::cerr << std::endl;
  }
  gz_robot_joints_.time_sec_ = a_gz_model_msg.header().stamp().sec();
  gz_robot_joints_.time_nsec_ = a_gz_model_msg.header().stamp().nsec();


}

bool JointStateInterface::SetCmd( const RobotCtrlJointInfos &rbt_ctrl_joint_infos)
{
  if (rbt_ctrl_joint_infos.size()>gz_robot_joints_.dict_joint_values.size())
  {
    std::cerr << "rbt_ctrl_joint_infos.size(): "
              << rbt_ctrl_joint_infos.size()
              << " gz_robot_joints_.dict_joint_values.size()"
              << gz_robot_joints_.dict_joint_values.size()
              << std::endl;
    return false;
  }

  gz::msgs::MapNamedJointsForces mnjf_msg;
  auto ljointsforces = mnjf_msg.mutable_jointsforces();

  // Iterate over the cmd map.
  for (auto cmd_it = rbt_ctrl_joint_infos.begin();
       cmd_it != rbt_ctrl_joint_infos.end();
       cmd_it++)
  {

    (*ljointsforces)[cmd_it->first]= cmd_it->second.Cmd();
  }

  if (!gz_pub_named_joints_forces_.Publish(mnjf_msg)) {
       std::cerr << "Unable to publish on "
                 << topic_name_gz_pub_named_joints_forces_
                 << std::endl;
       return false;
  }
  return true;
}

bool JointStateInterface::GetPosVel(RobotCtrlJointInfos &rbt_ctrl_joint_infos,
                                    double &time)
{
  if (rbt_ctrl_joint_infos.size()> gz_robot_joints_.dict_joint_values.size())
  {
    std::cerr << "rbt_ctrl_joint_infos.size(): " << rbt_ctrl_joint_infos.size()
              << " gz_robot_joints_.dict_joint_values.size():"  << gz_robot_joints_.dict_joint_values.size()
              << std::endl;
      return false;
  }
  std::lock_guard(gz_robot_joints_.lock_state_access_);
  time=(double)gz_robot_joints_.time_sec_ + 1e-9*(double)gz_robot_joints_.time_nsec_;
  for (auto ctrl_joint_it = rbt_ctrl_joint_infos.begin();
       ctrl_joint_it != rbt_ctrl_joint_infos.end();
       ctrl_joint_it++)
  {
    ctrl_joint_it->second.SetPosMes(gz_robot_joints_.dict_joint_values[ctrl_joint_it->first].pos_mes);
    ctrl_joint_it->second.SetVelMes(gz_robot_joints_.dict_joint_values[ctrl_joint_it->first].vel_mes);
  }
  if (debug_level_)
  {
    std::cerr << "JointStateInterface::GetPosVel: time: "
              << time << " "
              << std::endl;
    for (auto ctrl_joint_it = rbt_ctrl_joint_infos.begin();
         ctrl_joint_it != rbt_ctrl_joint_infos.end();
         ctrl_joint_it++)
    {

      std::cerr << ctrl_joint_it->second.PosMes() << " ";

    }
    std::cerr << std::endl;
  }
  return true;
}


};
