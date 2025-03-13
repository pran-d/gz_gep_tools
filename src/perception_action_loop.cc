#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <iostream>
#include <string>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "gz_gep_tools/perception_action_loop.hh"

/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic<bool> g_terminatePub(false);

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}

namespace gz_transport_hw_tools {

PerceptionActionLoop::PerceptionActionLoop(
    gz_transport_hw_tools::RobotCtrlJointInfos &a_robot_ctrl_joint_infos,
    std::vector<double> &a_pose3d,
    std::string &a_prefix_model_root, std::string &a_prefix_world,
    bool debug_level)
    : robot_ctrl_joint_infos_(a_robot_ctrl_joint_infos),
      joint_state_interface_(a_prefix_model_root, a_prefix_world, debug_level),
      control_over_gz_(a_prefix_world, debug_level),
      state_gz_time_(0.0),
      pre_state_gz_time_(0.0),
      gz_time_(0.0),
      pre_gz_time_(0.0),
      local_time_(0),
      pose3d_(a_pose3d),
      debug_level_(debug_level)
      {

  joint_state_interface_.SetListOfJoints(a_robot_ctrl_joint_infos);

  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Save list of joint named
  std::string filename_list_named_joints("/tmp/list_of_joints.dat");
  SaveListOfNamedJoints(robot_ctrl_joint_infos_,filename_list_named_joints);
}

int PerceptionActionLoop::InitGz()
{
  /// Starting simulation.
  local_time_=0;

  // First reset to read the robot state
  if (!control_over_gz_.Reset())  {
    std::cerr << "Reset failed" << std::endl;
  }
  gz_time_=0.0;
  pre_gz_time_=control_over_gz_.GetSimTime();

  using namespace std::chrono_literals;
  // Wait 1 ms to perform reset.
  //std::this_thread::sleep_for(std::chrono::nanoseconds(1ms));
  // Start simulation.
  control_over_gz_.Step();
  control_over_gz_.ReadWorldStateToInitECM();

  while (pre_gz_time_==gz_time_) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(2ms));
    gz_time_=control_over_gz_.GetSimTime();
  }
  // Second reset to set the robot state to a specific position.
  gz_time_=0.0;
  pre_gz_time_=control_over_gz_.GetSimTime();
  if (!control_over_gz_.Reset())  {
    std::cerr << "Reset failed" << std::endl;
  }

  while (pre_gz_time_== gz_time_) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(2ms));
    gz_time_=control_over_gz_.GetSimTime();
  }

  control_over_gz_.SendWorldControlStateToInitECM(robot_ctrl_joint_infos_,
                                                  pose3d_);
  if (debug_level_)
    control_over_gz_.DisplayLinkValues();
  control_over_gz_.Step();

  /// Synchronize simulation wait for starting.
  unsigned long int i=0;
  while(std::isnan(pre_gz_time_) ||
         (pre_gz_time_>0.001))
  { ///  Wait (2ms)
    std::this_thread::sleep_for(std::chrono::nanoseconds(2ms));
    pre_gz_time_=control_over_gz_.GetSimTime();
    if ((i%10==0) && (i>1)){
      control_over_gz_.Reset();
      //  std::this_thread::sleep_for(std::chrono::nanoseconds(2ms));
      control_over_gz_.Step();
      //std::this_thread::sleep_for(std::chrono::nanoseconds(2ms));
      if (i>100) {
        std::cerr << "control_loop stuck in the waiting loop for starting."
                  << std::endl
                  << pre_gz_time_ << " "
                  << i << " "
                  << std::endl;
        return -1;
      }
    }
    i++;
  }
  return 0;
}

int PerceptionActionLoop::MainLoop(unsigned long long int &duration)
{
  unsigned long int internal_timer = 0;
  std::string filename_pos("/tmp/position.dat");
  std::string filename_pos_des("/tmp/position_des.dat");
  std::string filename_cmd("/tmp/cmd.dat");
  std::ofstream ofs_position_robot(filename_pos),
      ofs_cmd_robot(filename_cmd),
      ofs_position_des_robot(filename_pos_des);
  ofs_position_robot.close();
  ofs_cmd_robot.close();
  ofs_position_des_robot.close();

  /// CTRL-C loop
  while (!g_terminatePub)
  {
    /// Check if Gazebo time has been updated.
    gz_time_ = control_over_gz_.GetSimTime();

    /// By default sim is not ready
    bool is_sim_ready = false;

    /// Test if the simulator clock has been updated.
    if (pre_gz_time_ < gz_time_)
      /// Sense
      is_sim_ready = joint_state_interface_.GetPosVel(robot_ctrl_joint_infos_,
                                                           state_gz_time_);


    if (debug_level_)
      std::cerr << "control_loop: "
                << is_sim_ready << " "
                << pre_gz_time_<< " "
                << gz_time_ << " "
                << pre_state_gz_time_<< " "
                << state_gz_time_ << " "
                << internal_timer
                << std::endl;
    if ((is_sim_ready) && // If the simulation is ready
        (pre_gz_time_<gz_time_) && // If the pre_gz_time_ is before gz_time_
        (fabs(pre_gz_time_-gz_time_) < 0.005) // The update of gz_time_ might happen before
        // reset and then putting gz_time_ to far ahead.
        )
    {

      {
        /// Control computation.
        for(auto it_ctrl_joint_i=robot_ctrl_joint_infos_.begin();
            it_ctrl_joint_i!=robot_ctrl_joint_infos_.end();
            it_ctrl_joint_i++)
        {
          it_ctrl_joint_i->second.ComputeCmd();
        }
      }

      /// Store Gazebo time
      pre_gz_time_ = gz_time_;


      if (joint_state_interface_.SetCmd(robot_ctrl_joint_infos_))
      {
        if (true)
        {
          SavePos(robot_ctrl_joint_infos_,filename_pos);
          SavePosDes(robot_ctrl_joint_infos_,filename_pos_des);
          SaveCmd(robot_ctrl_joint_infos_,filename_cmd);
        }
      }

      // Start simulation.
      // TODO : Verify that step has converged before returning.
      control_over_gz_.Step();

      local_time_++;

      internal_timer=0;
      /// Stop after 0.2 seconds.
      if (local_time_>duration)
        break;
    } else
    {

      // We have missed the Step for one sec.
      if ((internal_timer>=1000) &&
          (pre_gz_time_>=gz_time_))
      {
        std::cerr << "internal_timer: " << internal_timer << std::endl;
        control_over_gz_.Step();
        std::cout << "Step in internal timer" << std::endl;
        internal_timer=0;
      }

    }
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(2ms);
    if ((local_time_%1000==0) && (internal_timer==0))
      std::cout << "local_time:" << ((long double)local_time_)/1000.0 << " s " << std::endl
                << " internal_timer:" << internal_timer << std::endl;
    internal_timer++;
  }
  //  aControlOverGz.SendWorldControlState();

  control_over_gz_.Pause();

  ofs_position_robot.close();
  ofs_cmd_robot.close();

  return 0;
}

}
