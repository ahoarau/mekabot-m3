#ifndef _PcvStatus_h_
#define _PcvStatus_h_

#include "Params.h"

typedef struct Pcv_Status
{ 
    Float global_position[3];
    Float global_velocity[3];    
    Float local_position[3];
    Float local_velocity[3];
    Float local_acceleration[3];    
    Float local_force[3]; 
    Float position_desired[3];
    Float velocity_desired[3];
    Float position_error[3];
    Float velocity_error[3];    
    Float acceleration_desired[3];
    Float steer_angle_rad[MAX_NUM_CASTERS];
    Float steer_velocity_rad[MAX_NUM_CASTERS];
    Float roll_angle_rad[MAX_NUM_CASTERS];
    Float roll_velocity_rad[MAX_NUM_CASTERS];
    Float steer_torque_Nm[MAX_NUM_CASTERS];
    Float roll_torque_Nm[MAX_NUM_CASTERS];
    Float steer_torque_internal[MAX_NUM_CASTERS];
    Float roll_torque_internal[MAX_NUM_CASTERS];
    Float motor_torque_Nm[MAX_NUM_CASTERS*2];
    Float dac_ticks[MAX_NUM_CASTERS*2];
    Float truss_vel[6];
    bool traj_goal_reached;    
} Pcv_Status;  


#endif 