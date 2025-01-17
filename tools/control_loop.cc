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
  
  gz_transport_hw_tools::JointStateInterface
      a_joint_state_inter(a_prefix_model_root,
                          a_prefix_world);

  gz_transport_hw_tools::ControlOverGz aControlOverGz(a_prefix_world);
  
  a_joint_state_inter.SetListOfJoints(talos_list_of_joints);

  std::vector<double> cmd_vec_d;
  cmd_vec_d.resize(talos_list_of_joints.size());
  for(unsigned int i=0;i<talos_list_of_joints.size();i++)
    cmd_vec_d[i]=100.0;
  
  // Publish messages at 1Hz.
  std::vector<double> pos_mes_d,vel_mes_d;
  pos_mes_d.resize(talos_list_of_joints.size());
  vel_mes_d.resize(talos_list_of_joints.size());
  
  // Desired position and velocity
  std::vector<double> pos_des_d  { 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, // arm_left
    -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, // arm_right
    0, 0, // gripper
    0, 0, // head
    0, 0, -0.411354, 0.859395, -0.448041, -0.001708, // leg_left
    0, 0, -0.411354, 0.859395, -0.448041, -0.001708, // leg_right
    0, 006761 // torso
  };
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
    900.0, 300.0, 900.0, 900.0, 900.0, 900.0, 900.0, // arm_left
    900.0, 900.0, 900.0, 900.0, 900.0, 900.0, 900.0, // arm_right
    0.0, 0.0, // gripper
    0.0, 0.0, // head
    900.0, 900.0, 900.0, 900.0, 900.0, 900.0, 900.0, // leg_left
    900.0, 900.0, 900.0, 900.0, 900.0, 900.0, 900.0, // leg_right
    900.0, 900.0 // torso
  };

  std::vector<double> Kd { 10.0, 0.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm_left
    10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm_right
    1.0, 1.0, // gripper
    1.0, 1.0, // head
    5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, // leg_left
    5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, // leg_right
    10.0, 10.0 // torso
  };
  unsigned long long int local_time=0;

  aControlOverGz.Reset();
  aControlOverGz.Start();
  while (!g_terminatePub)
  {
    /// Sense
    a_joint_state_inter.GetPosVel(pos_mes_d, vel_mes_d);
    
    /// Control
    for(unsigned int i=0;i<talos_list_of_joints.size();i++)
    {
      double pos_err = pos_des_d[i] - pos_mes_d[i];
      double vel_err = vel_des_d[i] - vel_mes_d[i];
      std::string nb_joint = std::to_string(i);

      cmd_vec_d[i] = Kp[i] * pos_err + Kd[i]* vel_err ;
      
#if 0
      if (local_time%10==1)      
        std::cout << "Time : " << local_time
                  << " pos_err[" + nb_joint +" ] : " << pos_err
                  << " vel_err[" + nb_joint +" ] :" << vel_err << std::endl;
#else
      if (i==1)
        std::cout   << pos_err << " " 
                    << pos_des_d[i] << " "
                    << pos_mes_d[i] << " "
                    << cmd_vec_d[i] << " "
                    << Kp[i] << " "
                    << Kd[i] << std::endl;
#endif
    }
    a_joint_state_inter.SetCmd(cmd_vec_d);
    
    std::this_thread::sleep_for(std::chrono::nanoseconds(1000));
    local_time++;
    if (local_time>10000)
      break;
  }
  aControlOverGz.Pause();

  return 0;
}
//! [complete]
