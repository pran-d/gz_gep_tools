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

#include <gz_gep_tools/joint_state_interface.hh>

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

  a_joint_state_inter.SetListOfJoints(talos_list_of_joints);

  std::vector<double> cmd_vec_d;
  cmd_vec_d.resize(talos_list_of_joints.size());
  for(unsigned int i=0;i<talos_list_of_joints.size();i++)
    cmd_vec_d[i]=100.0;
  
  // Publish messages at 1Hz.
  while (!g_terminatePub)
  {
    a_joint_state_inter.SetCmd(cmd_vec_d);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}
//! [complete]
