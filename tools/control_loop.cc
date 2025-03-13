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
#include <fstream>
#include <iostream>
#include <string>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <gz_gep_tools/gz_gep_tools.hh>


using namespace gz_transport_hw_tools;
//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Dealing with the arguments.
  int choice = 0;

  if (argc!=2) {
    std::cerr << argv[0] << " h|t|w" << std::endl
              << " h : for the H1 robot (default)" << std::endl
              << " w : for the H1 robot without hands" << std::endl
              << " t : for the TALOS robot" << std::endl;
    return -1;
  }
  else if (std::string(argv[1])==std::string("w"))
    choice = 1;
  else if (std::string(argv[1])==std::string("t"))
    choice = 2;

  std::cout << "Choice :" << choice << std::endl;
  // Create a transport node and advertise a topic.
  gz::transport::Node node;



  std::string & a_prefix_model_root =  gz_transport_hw_tools::h1_v2_prefix_model_root;
  std::string & a_prefix_world = gz_transport_hw_tools::h1_v2_prefix_world;
  std::vector<double> & a_pose3d = gz_transport_hw_tools::h1_v2_pose3d;
  gz_transport_hw_tools::RobotCtrlJointInfos &
      a_robot_ctrl_joint_infos =  gz_transport_hw_tools::h1_v2_ctrl_joint_infos;

  switch (choice) {
    case (1):
      a_prefix_model_root = gz_transport_hw_tools::h1_v2_woh_prefix_model_root;
      a_prefix_world = gz_transport_hw_tools::h1_v2_woh_prefix_world;
      a_pose3d = gz_transport_hw_tools::h1_v2_woh_pose3d;
      a_robot_ctrl_joint_infos = gz_transport_hw_tools::h1_v2_woh_ctrl_joint_infos ;
      break;
    case(2) :
      a_prefix_model_root = gz_transport_hw_tools::talos_prefix_model_root;
      a_prefix_world = gz_transport_hw_tools::talos_prefix_world;
      a_pose3d = gz_transport_hw_tools::talos_pose3d;
      a_robot_ctrl_joint_infos = gz_transport_hw_tools::talos_ctrl_joint_infos ;
      break;
  }

  bool debug_level = false;


  gz_transport_hw_tools::PerceptionActionLoop a_pal(a_robot_ctrl_joint_infos,
                                                    a_pose3d,
                                                    a_prefix_model_root,
                                                    a_prefix_world,
                                                    debug_level);


  a_pal.InitGz();

  unsigned long long int duration= 20000;
  a_pal.MainLoop(duration);

  return 0;
}
//! [complete]
