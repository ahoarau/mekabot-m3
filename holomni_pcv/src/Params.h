#ifndef _PcvParams_h_
#define _PcvParams_h_

#include "PrGlobalDefn.h"

#define MAX_NUM_CASTERS 4

#define ENC_CNT2REV (1.0/ENC_REV2CNT)
#define ENC_RAD2CNT (ENC_REV2CNT/(2.0*M_PI))
#define ENC_CNT2RAD ((2.0*M_PI)/ENC_REV2CNT)

typedef struct Gains
{ 
  Float KPx_;
  Float KPa_;
  Float KVx_;
  Float KVa_;  
  Float KpE_;
  Float KpC_;
} Gains;

typedef struct Mass
{   
  Float mass;
  Float inertia_flat;  
} Mass;

typedef struct Placement
{
  Float x;
  Float y;
  Float angle;
} Placement;

typedef struct Geometry
{
  Float steering_offsets[MAX_NUM_CASTERS]; // tics
  int num_casters;
  Placement caster_placement[MAX_NUM_CASTERS];
} Geometry;

typedef struct Kinematics
{
  Float abs_max_linear_velocity;
  Float abs_max_rotation_velocity;
  Float abs_max_linear_acceleration;
  Float abs_max_rotation_acceleration;
} Kinematics;


typedef struct Filter_Params
{   
  Float FQD;
  Float FGXD;
  Float FCXDD;
  Float FTE;
  Float FXD;  
  
} Filter_Params;


typedef struct Caster_Params
{ 
  Float ENC_REV2CNT;
  Float Nm_PER_AMP;
  Float AMPS_PER_VOLT;
  Float COUNTS_PER_VOLT;
  Float b;
  Float r;
  Float f;
  Float Mf;
  Float If;
  Float Ih;
  Float Ii;
  Float Is;
  Float It;
  Float Ij;
  Float Ns;
  Float Nt;
  Float Nw;
  Float px;
  Float py;
  Float Mp;
  Float Ip;  
} Caster_Params;

typedef struct Home_Params
{ 
  //( 1*6802 +  20),0,( 0*6802 +  85),0
  //(-1*6802 +  75),0,(-2*6802 +  80),0
  Float offset[8];
  Float e_err; //3000
  Float v_des; //6000
  Float KPz_; //2.0
  Float KVz_ ;// 0.04
} Home_Params;

typedef struct Pcv_Params
{ 
  
  Caster_Params caster;
  Filter_Params filter;
  Home_Params home;
  Gains gains;
  Mass mass;
  Geometry geometry;  
  Kinematics kinematics;
} Pcv_Params;


#endif // _Params_h_
