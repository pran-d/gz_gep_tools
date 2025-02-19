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

PerceptionActionLoop::PerceptionActionLoop(gz_transport_hw_tools::RobotCtrlJointInfos & a_robot_ctrl_joint_infos,
                                           std::string &a_prefix_model_root,
                                           std::string &a_prefix_world,
                                           bool debug_level):
    robot_ctrl_joint_infos_(a_robot_ctrl_joint_infos),
    joint_state_interface_(a_prefix_model_root,a_prefix_world,debug_level),
    control_over_gz_(a_prefix_world,debug_level) {

  joint_state_interface_.SetListOfJoints(a_robot_ctrl_joint_infos);

  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

}

int PerceptionActionLoop::InitGz()
{
  /// Starting simulation.
  local_time_=0;

  // First reset to read the robot state
  if (!control_over_gz_.Reset())  {
    std::cerr << "Reset failed" << std::endl;
  }
  state_gz_time_=0.0;
  pre_state_gz_time_=control_over_gz_.GetSimTime();

  using namespace std::chrono_literals;
  // Wait 1 ms to perform reset.
  std::this_thread::sleep_for(std::chrono::nanoseconds(1ms));
  // Start simulation.
  control_over_gz_.Step();
  control_over_gz_.ReadWorldStateToInitECM();

  // Second reset to set the robot state to a specific position.
  if (!control_over_gz_.Reset())  {
    std::cerr << "Reset failed" << std::endl;
  }  
  control_over_gz_.SendWorldControlStateToInitECM(robot_ctrl_joint_infos_);
  control_over_gz_.DisplayLinkValues();
  control_over_gz_.Step();
  
  /// Synchronize simulation wait for starting.
  unsigned long int i=0;
  while(std::isnan(pre_state_gz_time_) ||
         (pre_state_gz_time_>0.001))
  { ///  Wait (1ms)
    std::this_thread::sleep_for(std::chrono::nanoseconds(2ms));
    pre_state_gz_time_=control_over_gz_.GetSimTime();
    if ((i%10==0) && (i>1)){
      control_over_gz_.Reset();
      std::this_thread::sleep_for(std::chrono::nanoseconds(2ms));
      control_over_gz_.Step();
      std::this_thread::sleep_for(std::chrono::nanoseconds(2ms));      
      if (i>100) {
        std::cerr << "control_loop stuck in the waiting loop for starting."
                  << std::endl
                  << pre_state_gz_time_ << " "
                  << i << " "
                  << std::endl;
        return -1;
      }
    }
    i++;
  }
  return 0;
}

int PerceptionActionLoop::MainLoop()
{    
  unsigned long int internal_timer = 0;
  std::string filename_pos("/tmp/position.dat");
  std::string filename_cmd("/tmp/cmd.dat");
  std::ofstream ofs_position_robot(filename_pos),
      ofs_cmd_robot(filename_cmd);  
  
  /// CTRL-C loop
  while (!g_terminatePub)
  {
    /// Sense
    bool is_sim_ready = joint_state_interface_.GetPosVel(robot_ctrl_joint_infos_,
                                                         state_gz_time_);

    std::cout << "state_gz_time:" << state_gz_time_ << std::endl;
    //    if (debug_level)
      std::cerr << "control_loop: "
                << is_sim_ready << " "
                << pre_state_gz_time_<< " "
                << state_gz_time_ << " "
                << std::endl;
    if ((is_sim_ready) && // If the simulation is ready
        (pre_state_gz_time_<state_gz_time_) && // If the pre_state_gz_time_ is before state_gz_time_
        (fabs(pre_state_gz_time_-state_gz_time_) < 0.005) // The update of state_gz_time_ might happen before
        // reset and then putting state_gz_time_ to far ahead.
        ) {

      {
        /// Control computation.
        for(auto it_ctrl_joint_i=robot_ctrl_joint_infos_.begin();
            it_ctrl_joint_i!=robot_ctrl_joint_infos_.end();
            it_ctrl_joint_i++)
        {
          it_ctrl_joint_i->second.compute_cmd();
        }
      }

      /// Store Gazebo time
      pre_state_gz_time_ = state_gz_time_;


      if (joint_state_interface_.SetCmd(robot_ctrl_joint_infos_))
      {
        SavePos(robot_ctrl_joint_infos_,filename_pos);
        SaveCmd(robot_ctrl_joint_infos_,filename_cmd);            
      }
      
      // Start simulation.
      // TODO : Verify that step has converged before returning.
      control_over_gz_.Step();

      local_time_++;

      internal_timer=0;
      /// Stop after 0.2 seconds.
      if (local_time_>2000)
        break;
    } else
    {

      // We have missed the Step for one sec.
      if ((internal_timer>=1000) &&
          (pre_state_gz_time_==state_gz_time_))
      {
        std::cerr << "internal_timer: " << internal_timer << std::endl;
        control_over_gz_.Step();
        internal_timer=0;
      }

    }
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1ms);
    internal_timer++;
  }
  //  aControlOverGz.SendWorldControlState();

  control_over_gz_.Pause();

  ofs_position_robot.close();
  ofs_cmd_robot.close();

  return 0;
}

}
