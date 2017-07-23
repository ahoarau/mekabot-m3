#ifndef _PcvCommand_h_
#define _PcvCommand_h_

#include "Params.h"

typedef struct Pcv_Command
{ 
  TrajMode trajMode_;
  CtrlFrame ctrlFrame_;
  CtrlMode controlMode_;
  bool internalForce_;
  bool accel_FF_;
  bool adjust_local_position;
  bool adjust_global_position;
  Float local_position[3];
  Float global_position[3];
  Float motor_angles_rad[MAX_NUM_CASTERS*2];  
  Float motor_velocities_rad[MAX_NUM_CASTERS*2];
  bool use_angles_instead_of_ticks;
  bool use_external_motor_velocities;
  int enc_ticks[MAX_NUM_CASTERS*2];  
  Float steer_torque_desired_Nm[MAX_NUM_CASTERS];
  Float roll_torque_desired_Nm[MAX_NUM_CASTERS];
  Float opspace_force_desired[3];
  Float local_position_desired[3];
  Float local_velocity_desired[3];
  Float local_acceleration_desired[3];
  Float global_position_desired[3];
  Float global_velocity_desired[3];
  Float global_acceleration_desired[3];
  Float max_linear_velocity;
  Float max_rotation_velocity;
  Float max_linear_acceleration;
  Float max_rotation_acceleration;
  Float traj_goal[3];
  double joystick_x;
  double joystick_y;
  double joystick_yaw;
  int joystick_button;
} Pcv_Command;



#endif 