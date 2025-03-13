#include <gz_gep_tools/robots_data.hh>

namespace gz_transport_hw_tools {

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

RobotCtrlJointInfos h1_v2_ctrl_joint_infos {
    /// Left hand
    { "L_index_intermediate_joint",  ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_index_proximal_joint",      ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_middle_intermediate_joint", ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_middle_proximal_joint",     ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_pinky_intermediate_joint",  ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_pinky_proximal_joint",      ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_ring_intermediate_joint",   ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_ring_proximal_joint",       ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_thumb_intermediate_joint",  ControlJointValue(    150.0,   0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_thumb_proximal_yaw_joint",  ControlJointValue(    150.0,   0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_thumb_distal_joint",        ControlJointValue(    150.0,   0.0,   0.0,  14.0,  0.0, 0.0) },
    { "L_thumb_proximal_pitch_joint",ControlJointValue(    150.0,   0.0,   0.0,  14.0,  0.0, 0.0) },
    /// Right hand
    { "R_index_intermediate_joint",  ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_index_proximal_joint",      ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_middle_intermediate_joint", ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_middle_proximal_joint",     ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_pinky_intermediate_joint",  ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_pinky_proximal_joint",      ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_ring_intermediate_joint",   ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_ring_proximal_joint",       ControlJointValue(    100.0,  0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_thumb_intermediate_joint",  ControlJointValue(    150.0,   0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_thumb_proximal_yaw_joint",  ControlJointValue(    150.0,   0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_thumb_distal_joint",        ControlJointValue(    150.0,   0.0,   0.0,  14.0,  0.0, 0.0) },
    { "R_thumb_proximal_pitch_joint",ControlJointValue(    150.0,   0.0,   0.0,  14.0,  0.0, 0.0) },
    /// Left arm
    { "left_shoulder_pitch_joint", ControlJointValue(10000.0, 0.01, 1.0, 14.0,  0.0, 0.0) },
    { "left_shoulder_roll_joint",  ControlJointValue(10000.0, 0.01, 1.0, 14.0,  0.0, 0.0) },
    { "left_shoulder_yaw_joint",   ControlJointValue(10000.0, 0.01, 1.0,  9.0,  0.0, 0.0) },
    { "left_elbow_joint",          ControlJointValue( 5000.0, 0.7,  1.0, 14.0,  0.0, 0.0) },
    { "left_wrist_roll_joint",     ControlJointValue(   50.0, 1.0,  0.1,  5.0,  0.0, 0.0) },
    { "left_wrist_pitch_joint",    ControlJointValue(  100.0, 4.0,  2.0, 14.0,  0.0, 0.0) },
    { "left_wrist_yaw_joint",      ControlJointValue(   50.0, 1.0,  0.1,  5.0,  0.0, 0.0) },
    /// Right arm
    { "right_shoulder_pitch_joint",ControlJointValue(10000.0, 0.01, 1.0, 14.0,  0.0, 0.0) },
    { "right_shoulder_roll_joint", ControlJointValue(10000.0, 0.01, 1.0, 14.0,  0.0, 0.0) },
    { "right_shoulder_yaw_joint",  ControlJointValue(10000.0, 0.01, 1.0,  9.0,  0.0, 0.0) },
    { "right_elbow_joint",         ControlJointValue( 5000.0, 0.7,  1.0, 14.0,  0.0, 0.0) },
    { "right_wrist_roll_joint",    ControlJointValue(   50.0, 1.0,  0.1,  5.0,  0.0, 0.0) },
    { "right_wrist_pitch_joint",   ControlJointValue(   50.0, 4.0,  2.0, 14.0,  0.0, 0.0) },
    { "right_wrist_yaw_joint",     ControlJointValue(   50.0, 1.0,  0.1,  5.0,  0.0, 0.0) },
    /// Left leg
    { "left_hip_yaw_joint",        ControlJointValue( 5000.0,20.0,  5.0,  7.0,  0.0, 0.0) },
    { "left_hip_pitch_joint",      ControlJointValue(10000.0,20.0,  1.0, 14.0,  0.0, 0.0) },
    { "left_hip_roll_joint",       ControlJointValue(10000.0,20.0,  1.0, 14.0,  0.0, 0.0) },
    { "left_knee_joint",           ControlJointValue( 5000.0,20.0,  5.0, 25.0,  0.0, 0.0) },
    { "left_ankle_pitch_joint",    ControlJointValue(  200.0, 1.0,  0.1,  5.0,  0.0, 0.0) },
    { "left_ankle_roll_joint",     ControlJointValue(  200.0, 1.0,  0.1,  5.0,  0.0, 0.0) },
    /// Right leg
    { "right_hip_yaw_joint",       ControlJointValue( 5000.0,20.0,  5.0,  7.0,  0.0, 0.0) },
    { "right_hip_pitch_joint",     ControlJointValue(10000.0,20.0,  1.0, 14.0,  0.0, 0.0) },
    { "right_hip_roll_joint",      ControlJointValue(10000.0,20.0,  1.0, 14.0,  0.0, 0.0) },
    { "right_knee_joint",          ControlJointValue( 5000.0,20.0,  5.0, 25.0,  0.0, 0.0) },
    { "right_ankle_pitch_joint",   ControlJointValue(  200.0, 1.0,  0.1,  5.0,  0.0, 0.0) },
    { "right_ankle_roll_joint",    ControlJointValue(  200.0, 1.0,  0.1,  5.0,  0.0, 0.0) },
    /// Torso
    { "torso_joint",               ControlJointValue(10000.0,20.0,  1.0, 10.0,  0.0, 0.0) },
  };
std::vector<double> h1_v2_pose3d {0.0 , 0.0, 0.0, 0.0, 0.0, 0.0};

RobotCtrlJointInfos h1_v2_woh_ctrl_joint_infos {
    /// Left arm
    { "left_shoulder_pitch_joint", ControlJointValue(10000.0, 20.0, 5.0, 14.0,  0.0, 0.0) },
    { "left_shoulder_roll_joint",  ControlJointValue(1000.0,   2.0, 5.0, 14.0,  0.0, 0.0) },
    { "left_shoulder_yaw_joint",   ControlJointValue(10000.0, 20.0, 5.0, 14.0,  0.0, 0.0) },
    { "left_elbow_joint",          ControlJointValue(10000.0, 20.0, 5.0, 14.0,  0.0, 0.0) },
    { "left_wrist_roll_joint",     ControlJointValue(   5.0,  0.1, 0.0, 14.0,  0.0, 0.0) },
    { "left_wrist_pitch_joint",    ControlJointValue( 500.0,  0.0, 0.0, 14.0,  0.0, 0.0) },
    { "left_wrist_yaw_joint",      ControlJointValue(  50.0,  0.1, 0.0, 14.0,  0.0, 0.0) },
    /// Right arm
    { "right_shoulder_pitch_joint",ControlJointValue(10000.0, 20.0, 5.0, 14.0,  0.0, 0.0) },
    { "right_shoulder_roll_joint", ControlJointValue(1000.0,  2.0, 5.0, 14.0,  0.0, 0.0) },
    { "right_shoulder_yaw_joint",  ControlJointValue(10000.0, 20.0, 5.0, 14.0,  0.0, 0.0) },
    { "right_elbow_joint",         ControlJointValue(10000.0, 20.0, 5.0, 14.0,  0.0, 0.0) },
    { "right_wrist_roll_joint",    ControlJointValue(   5.0,  0.1, 0.0, 14.0,  0.0, 0.0) },
    { "right_wrist_pitch_joint",   ControlJointValue( 500.0,  0.0, 0.0, 14.0,  0.0, 0.0) },
    { "right_wrist_yaw_joint",     ControlJointValue(  50.0,  0.1, 0.0, 14.0,  0.0, 0.0) },
    /// Left leg
    { "left_hip_yaw_joint",        ControlJointValue(10000.0,20.0,  5.0,  7.0,  0.0, 0.0) },
    { "left_hip_pitch_joint",      ControlJointValue(10000.0,20.0,  1.0, 14.0,  0.0, 0.0) },
    { "left_hip_roll_joint",       ControlJointValue(10000.0,20.0,  1.0, 14.0,  0.0, 0.0) },
    { "left_knee_joint",           ControlJointValue( 500.0, 2.0,  1.0,  7.0,  0.0, 0.0) },
    { "left_ankle_pitch_joint",    ControlJointValue( 400.0, 2.0,  1.0,  7.0,  0.0, 0.0) },
    { "left_ankle_roll_joint",     ControlJointValue( 40.0, 1.0,  0.0,  7.0,  0.0, 0.0) },
    /// Right leg
    { "right_hip_yaw_joint",       ControlJointValue(10000.0,20.0,  5.0,  7.0,  0.0, 0.0) },
    { "right_hip_pitch_joint",     ControlJointValue(10000.0,20.0,  1.0, 14.0,  0.0, 0.0) },
    { "right_hip_roll_joint",      ControlJointValue(10000.0,20.0,  1.0, 14.0,  0.0, 0.0) },
    { "right_knee_joint",          ControlJointValue( 500.0, 2.0,  1.0,  7.0,  0.0, 0.0) },
    { "right_ankle_pitch_joint",   ControlJointValue( 400.0, 2.0,  1.0,  7.0,  0.0, 0.0) },
    { "right_ankle_roll_joint",    ControlJointValue( 40.0, 1.0,  0.0,  7.0,  0.0, 0.0) },
    /// Torso
    { "torso_joint",               ControlJointValue(10000.0,20.0,  1.0, 10.0,  0.0, 0.0) },
  };
std::vector<double> h1_v2_woh_pose3d {0.0 , 0.0, 1.02, 0.0, 0.0, 0.0};

std::string talos_prefix_model_root("/model/Pyrene/");
std::string talos_prefix_world("/world/empty_talos_gz");

std::string h1_v2_prefix_model_root("/model/H1/");
std::string h1_v2_prefix_world("/world/empty_h1_2_gz");

std::string h1_v2_woh_prefix_model_root("/model/H1/");
std::string h1_v2_woh_prefix_world("/world/empty_h1_2_gz");

};
