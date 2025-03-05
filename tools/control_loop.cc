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
#include <gz_gep_tools/perception_action_loop.hh>


using namespace gz_transport_hw_tools;
//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Dealing with the arguments.
  int choice = 0;

  if (argc!=2) {
    std::cerr << argv[0] << " h|t" << std::endl
              << " h : for the H1 robot (default)" << std::endl
              << " t : for the TALOS robot" << std::endl;
    return -1;
  }
  else if (std::string(argv[1])==std::string("t"))
    choice = 1;

  // Create a transport node and advertise a topic.
  gz::transport::Node node;
  RobotCtrlJointInfos talos_ctrl_joint_infos {
    { "arm_left_1_joint", ControlJointValue(  10000.0, 0.01, 1.0, 14.0, 0.25847 ,  0.0)},
    { "arm_left_2_joint", ControlJointValue(  10000.0, 0.01, 1.0, 14.0, 0.173046,  0.0)},
    { "arm_left_3_joint", ControlJointValue(   5000.0, 0.0 , 1.0,  9.0, -0.0002  , 0.0)},
    { "arm_left_4_joint", ControlJointValue(   5000.0, 0.0 , 1.0,  9.0, -0.525366, 0.0)},
    { "arm_left_5_joint", ControlJointValue(    500.0, 1.0 , 0.1,  5.0,  0.0,      0.0)},
    { "arm_left_6_joint", ControlJointValue(    500.0, 1.0 , 0.1,  5.0,  0.0,      0.0)},
    { "arm_left_7_joint", ControlJointValue(    100.0, 1.0 , 0.0,  3.0,  0.0,      0.0)},
    { "arm_right_1_joint", ControlJointValue( 10000.0, 0.01, 1.0, 14.0, -0.25847 , 0.0)},
    { "arm_right_2_joint", ControlJointValue( 10000.0, 0.01, 1.0, 14.0, -0.173046, 0.0)},
    { "arm_right_3_joint", ControlJointValue(  5000.0, 0.0 , 1.0,  9.0,  0.0002  , 0.0)},
    { "arm_right_4_joint", ControlJointValue(  5000.0, 0.0 , 1.0,  9.0, -0.525366, 0.0)},
    { "arm_right_5_joint", ControlJointValue(   500.0, 1.0 , 0.1,  5.0,  0.0,      0.0)},
    { "arm_right_6_joint", ControlJointValue(   500.0, 1.0 , 0.1,  5.0,  0.0,      0.0)},
    { "arm_right_7_joint", ControlJointValue(   500.0, 1.0 , 0.1,  5.0,  0.0,      0.0)},
    { "gripper_left_joint",  ControlJointValue(1000.0,10.0 , 1.0, 10.0,  0.0,      0.0)},
    { "gripper_right_joint", ControlJointValue(1000.0,10.0 , 1.0, 10.0,  0.0,      0.0)},
    { "head_1_joint", ControlJointValue(        300.0, 0.1,  1.0,  5.0,  0.0,      0.0)},
    { "head_2_joint", ControlJointValue(        300.0, 0.1,  1.0,  1.5,  0.0,      0.0)},
    { "leg_left_1_joint", ControlJointValue(   5000.0,20.0,  5.0,   7.0, 0.0,      0.0)},
    { "leg_left_2_joint", ControlJointValue(   5000.0,20.0,  5.0,  14.0, 0.0,      0.0)},
    { "leg_left_3_joint", ControlJointValue(   5000.0,20.0,  5.0,  14.0,-0.448041, 0.0)},
    { "leg_left_4_joint", ControlJointValue(   5000.0,20.0,  5.0,  25.0, 0.896082, 0.0)},
    { "leg_left_5_joint", ControlJointValue(   5000.0,20.0,  5.0,  14.0,-0.448041, 0.0)},
    { "leg_left_6_joint", ControlJointValue(   5000.0,20.0,  5.0,   9.0, 0.0,      0.0)},
    { "leg_right_1_joint", ControlJointValue(  5000.0,20.0,  5.0,   7.0, 0.0,      0.0)},
    { "leg_right_2_joint", ControlJointValue(  5000.0,20.0,  5.0,  14.0, 0.0,      0.0)},
    { "leg_right_3_joint", ControlJointValue(  5000.0,20.0,  5.0,  14.0,-0.448041, 0.0)},
    { "leg_right_4_joint", ControlJointValue(  5000.0,20.0,  5.0,  25.0, 0.896082, 0.0)},
    { "leg_right_5_joint", ControlJointValue(  5000.0,20.0,  5.0,   9.0,-0.448041, 0.0)},
    { "leg_right_6_joint", ControlJointValue(  5000.0,20.0,  5.0,   7.0, 0.0,      0.0)},
    { "torso_1_joint",     ControlJointValue( 10000.0,10.0,  1.0,  10.0, 0.0,      0.0)},
    { "torso_2_joint",     ControlJointValue( 10000.0,10.0,  1.0,  10.0, 0.0,      0.0)}
  };
  std::vector<double> talos_pose3d {0.0 , 0.0, 1.02, 0.0, 0.0, 0.0};

  gz_transport_hw_tools::RobotCtrlJointInfos h1_2_ctrl_joint_infos {
    { "left_shoulder_pitch_joint", ControlJointValue( 10000.0, 0.01, 1.0, 14.0,  0.0, 0.0) },
    { "left_shoulder_roll_joint",  ControlJointValue( 10000.0, 0.01, 1.0, 14.0,  0.0, 0.0) },
    { "left_shoulder_yaw_joint",   ControlJointValue(  5000.0, 0.0,  1.0,  9.0,  0.0, 0.0) },
    { "left_elbow_joint",          ControlJointValue(  5000.0, 0.0,  1.0,  9.0,  0.0, 0.0) },
    { "left_wrist_roll_joint",     ControlJointValue(  500.0,  1.0,  0.1,  5.0,  0.0, 0.0) },
    { "left_wrist_pitch_joint",    ControlJointValue(  500.0,  1.0,  0.1,  5.0,  0.0, 0.0) },
    { "left_wrist_yaw_joint",      ControlJointValue(  500.0,  1.0,  0.1,  3.0,  0.0, 0.0) },
    { "right_shoulder_pitch_joint",ControlJointValue(10000.0, 0.01,  1.0,  0.0,  0.0, 0.0) },
    { "right_shoulder_roll_joint", ControlJointValue(10000.0, 0.01,  1.0,  0.0,  0.0, 0.0) },
    { "right_shoulder_yaw_joint",  ControlJointValue( 5000.0,  0.0,  1.0,  0.0,  0.0, 0.0) },
    { "right_elbow_joint",         ControlJointValue( 5000.0,  0.0,  1.0,  0.0,  0.0, 0.0) },
    { "right_wrist_roll_joint",    ControlJointValue(  500.0,  1.0,  0.1,  0.0,  0.0, 0.0) },
    { "right_wrist_pitch_joint",   ControlJointValue(  500.0,  1.0,  0.1,  0.0,  0.0, 0.0) },
    { "right_wrist_yaw_joint",     ControlJointValue(  500.0,  1.0,  0.1,  0.0,  0.0, 0.0) },
    { "left_hip_yaw_joint",        ControlJointValue( 5000.0, 20.0,  5.0,  7.0,  0.0, 0.0) },
    { "left_hip_pitch_joint",      ControlJointValue( 5000.0, 20.0,  5.0, 14.0,  0.0, 0.0) },
    { "left_hip_roll_joint",       ControlJointValue( 5000.0, 20.0,  5.0, 14.0,  0.0, 0.0) },
    { "left_knee_joint",           ControlJointValue( 5000.0, 20.0,  5.0, 25.0,  0.0, 0.0) },
    { "left_ankle_pitch_joint",    ControlJointValue( 5000.0, 20.0,  5.0, 14.0,  0.0, 0.0) },
    { "left_ankle_roll_joint",     ControlJointValue( 5000.0, 20.0,  5.0,  9.0,  0.0, 0.0) },
    { "right_hip_yaw_joint",       ControlJointValue( 5000.0, 20.0,  5.0,  7.0,  0.0, 0.0) },
    { "right_hip_pitch_joint",     ControlJointValue( 5000.0, 20.0,  5.0, 14.0,  0.0, 0.0) },
    { "right_hip_roll_joint",      ControlJointValue( 5000.0, 20.0,  5.0, 14.0,  0.0, 0.0) },
    { "right_knee_joint",          ControlJointValue( 5000.0, 20.0,  5.0, 25.0,  0.0, 0.0) },
    { "right_ankle_pitch_joint",   ControlJointValue( 5000.0, 20.0,  5.0, 14.0,  0.0, 0.0) },
    { "right_ankle_roll_joint",    ControlJointValue( 5000.0, 20.0,  5.0,  9.0,  0.0, 0.0) },
    { "torso_joint",               ControlJointValue(10000.0, 20.0,  1.0, 10.0,  0.0, 0.0) },
  };
    std::vector<double> h1_v2_pose3d {0.0 , 0.0, 0.0, 0.0, 0.0, 0.0};


  std::string talos_prefix_model_root("/model/Pyrene/");
  std::string talos_prefix_world("/world/empty_talos_gz");

  std::string h1_v2_prefix_model_root("/model/H1/");
  std::string h1_v2_prefix_world("/world/empty_h1_2_gz");


  std::string & a_prefix_model_root = ( choice ==0 ) ?
      h1_v2_prefix_model_root : talos_prefix_model_root;

  std::string & a_prefix_world = (choice == 0) ?
      h1_v2_prefix_world: talos_prefix_world;

  std::vector<double> & a_pose3d = (choice ==0) ?
      h1_v2_pose3d : talos_pose3d;

  gz_transport_hw_tools::RobotCtrlJointInfos &
    a_robot_ctrl_joint_infos = (choice==0) ?
      h1_2_ctrl_joint_infos: talos_ctrl_joint_infos ;

  bool debug_level = false;


  gz_transport_hw_tools::PerceptionActionLoop a_pal(a_robot_ctrl_joint_infos,
                                                    a_pose3d,
                                                    a_prefix_model_root,
                                                    a_prefix_world,
                                                    debug_level);


  a_pal.InitGz();

  a_pal.MainLoop();

  return 0;
}
//! [complete]
