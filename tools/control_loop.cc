/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

//! [complete]
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <gz_gep_tools/gz_gep_tools.hh>

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

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  using namespace std::chrono_literals;

  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create a transport node and advertise a topic.
  gz::transport::Node node;
  std::vector<std::string> talos_list_of_joints {
    "arm_left_1_joint",
    "arm_left_2_joint",
    "arm_left_3_joint",
    "arm_left_4_joint",
    "arm_left_5_joint",
    "arm_left_6_joint",
    "arm_left_7_joint",
    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
    "gripper_left_joint",
    "gripper_right_joint",
    "head_1_joint",
    "head_2_joint",
    "leg_left_1_joint",
    "leg_left_2_joint",
    "leg_left_3_joint",
    "leg_left_4_joint",
    "leg_left_5_joint",
    "leg_left_6_joint",
    "leg_right_1_joint",
    "leg_right_2_joint",
    "leg_right_3_joint",
    "leg_right_4_joint",
    "leg_right_5_joint",
    "leg_right_6_joint",
    "torso_1_joint",
    "torso_2_joint",
  };

  std::string a_prefix_model_root("/model/Pyrene/");
  std::string a_prefix_world("/world/empty_talos_gz");

  bool debug_level = false;
  gz_transport_hw_tools::JointStateInterface
      a_joint_state_inter(a_prefix_model_root,
                          a_prefix_world,
                          debug_level);

  gz_transport_hw_tools::ControlOverGz aControlOverGz(a_prefix_world,debug_level);

  a_joint_state_inter.SetListOfJoints(talos_list_of_joints);

  std::vector<double> cmd_vec_d;
  cmd_vec_d.resize(talos_list_of_joints.size());
  for(unsigned int i=0;i<talos_list_of_joints.size();i++)
    cmd_vec_d[i]=100.0;

  // Publish messages at 1Hz.
  std::vector<double> pos_mes_d,vel_mes_d, pos_err_d,vel_err_d;
  pos_mes_d.resize(talos_list_of_joints.size());
  vel_mes_d.resize(talos_list_of_joints.size());
  pos_err_d.resize(talos_list_of_joints.size());
  vel_err_d.resize(talos_list_of_joints.size());

  // Desired position and velocity
  std::vector<double> pos_des_d  { 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, // arm_left
    -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, // arm_right
    0, 0, // gripper
    0, 0, // head
    0, 0, -0.411354, 0.859395, -0.448041, -0.001708, // leg_left
    0, 0, -0.411354, 0.859395, -0.448041, -0.001708, // leg_right
    0, 0.006761 // torso
  };
  std::map<std::string, double> named_pos_des_d;
  if (talos_list_of_joints.size()!=pos_des_d.size())
    std::cerr << "Size between talos_list_of_joints and pos_des_d do not match." << std::endl;

  for(unsigned int i=0;i<talos_list_of_joints.size();i++)
    named_pos_des_d[talos_list_of_joints[i]]= pos_des_d[i];

  std::vector<double> vel_des_d  { 0, 0, 0, 0, 0, 0, 0, // arm_left
    0, 0, 0, 0, 0, 0, 0, // arm_right
    0, 0, // gripper
    0, 0, // head
    0, 0, 0, 0, 0, 0, 0, // leg_left
    0, 0, 0, 0, 0, 0, 0, // leg_right
    0, 0 // torso
  };

  std::vector<double> Kp { /* 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, // arm_left
    1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, // arm_right
    100.0, 100.0, // gripper
    100.0, 100.0, // head*/
    1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, // arm_left
    1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, // arm_right
    0.0, 0.0, // gripper
    0.0, 0.0, // head
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // leg_left
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // leg_right
    1000.0, 1000.0 // torso
  };

  std::vector<double> Kd { 10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, // arm_left
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // arm_right
    0.0, 0.0, // gripper
    0.0, 0.0, // head
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // leg_left
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // leg_right
    0.0, 0.0 // torso
  };
  unsigned long long int local_time=0;

  if (!aControlOverGz.Reset())  {
    std::cerr << "Reset failed" << std::endl;
  }
  double state_gz_time=0.0;
  double pre_state_gz_time=aControlOverGz.GetSimTime();
  // Wait 1 ms to perform reset.
  std::this_thread::sleep_for(std::chrono::nanoseconds(1ms));
  // Start simulation.
  aControlOverGz.Step();

  /// Synchronize simulation wait for starting.
  unsigned long int i=0;
  while(std::isnan(pre_state_gz_time) ||
         (pre_state_gz_time>0.001))
  { ///  Wait (1ms)
    std::this_thread::sleep_for(std::chrono::nanoseconds(1ms));
    pre_state_gz_time=aControlOverGz.GetSimTime();
    if ((i%100==0) && (i>1)){
      aControlOverGz.Reset();
      std::this_thread::sleep_for(std::chrono::nanoseconds(1ms));
      aControlOverGz.Step();
      /* std::cerr << "control_loop stuck in the waiting loop for starting."
                << std::endl
                << pre_state_gz_time << " "
                << i << " "
                << std::endl; */
    }
    i++;
  }

  aControlOverGz.ReadWorldStateToInitECM();

  unsigned long int internal_timer = 0;
  /// CTRL-C loop
  while (!g_terminatePub)
  {
    /// Sense
    bool is_sim_ready = a_joint_state_inter.GetPosVel(pos_mes_d, vel_mes_d,state_gz_time);

    if (debug_level)
      std::cerr << "control_loop: "
                << is_sim_ready << " "
                << pre_state_gz_time << " "
                << state_gz_time << " "
                << std::endl;
    if ((is_sim_ready) && // If the simulation is ready
        (pre_state_gz_time<state_gz_time) && // If the pre_state_gz_time is before state_gz_time
        (fabs(pre_state_gz_time-state_gz_time) < 0.005) // The update of state_gz_time might happen before
        // reset and then putting state_gz_time to far ahead.
        ) {
      /// Control
      for(unsigned int i=0;i<talos_list_of_joints.size();i++)
      {
        pos_err_d[i] = pos_des_d[i] - pos_mes_d[i];
        vel_err_d[i] = vel_des_d[i] - vel_mes_d[i];

        cmd_vec_d[i] = Kp[i] * pos_err_d[i] + Kd[i]* vel_err_d[i];

      }

      /// Store Gazebo time
      pre_state_gz_time = state_gz_time;


      if (local_time==1) {
        aControlOverGz.ReadWorldStateToInitECM();
        aControlOverGz.SendWorldControlStateToInitECM(named_pos_des_d);
      }
      else {

        if (local_time>1)
        {
          if (a_joint_state_inter.SetCmd(cmd_vec_d))
          {
            // std::cout   << pos_err_d[1] << " "
            //             << vel_err_d[1] << " "
            //             << pos_des_d[1] << " "
            //             << pos_mes_d[1] << std::endl;
            for(unsigned int i=0;i<talos_list_of_joints.size();i++)
              std::cout << pos_mes_d[i] << " ";
            // std::cout << Kp[1] << " "
            //           << Kd[1] << " "
            //           << state_gz_time << " "
            //           << pre_state_gz_time << " "
            std::cout  << std::endl;
          }
        }

        // Start simulation.
        // TODO : Verify that step has converged before returning.
        aControlOverGz.Step();
      }

      local_time++;

      internal_timer=0;
      /// Stop after 0.2 seconds.
      if (local_time>2000)
        break;
    } else
    {

      // We have missed the Step for one sec.
      if ((internal_timer>=1000) &&
          (pre_state_gz_time==state_gz_time))
      {
        std::cerr << "internal_timer: " << internal_timer << std::endl;
        aControlOverGz.Step();
        internal_timer=0;
      }

    }
    std::this_thread::sleep_for(1ms);
    internal_timer++;
  }
  //  aControlOverGz.SendWorldControlState();

  aControlOverGz.Pause();

  return 0;
}
//! [complete]
