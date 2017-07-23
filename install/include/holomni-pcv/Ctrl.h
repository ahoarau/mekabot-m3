#ifndef _Ctrl_h_
#define _Ctrl_h_

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include "matrix/PrMatrix.h"
#include "matrix/PrVector.h"
#include "Filter.h"
#include "Vehicle.h"
#include "Caster.h"
#include "Motion.h"
#include "Params.h"
#include "Status.h"
#include "Command.h"
#include "Traj3.h"


#define MSG  printf
#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

using namespace std;

class Ctrl
{
 public:
  Ctrl( int step_freq, vector<Float> gx_init, Pcv_Params params_in);
  ~Ctrl();
  
  Pcv_Status Step(Pcv_Command pcv_cmd);  
  void JoystickInit(Pcv_Command pcv_cmd);
  void StepJoystick(Pcv_Command cmd);
  void TrajInit(Pcv_Command pcv_cmd);  
  Traj3    *traj;

 private:    
    TrajMode old_traj_mode;
    Float hw_period;     
    Float heading;
    PrMatrix * J;
    PrMatrix * Jt;
    PrMatrix Lambda;
    PrVector Mu;
    PrVector cf;
    PrVector   cxdd;
    PrVector  fcxdd;
    PrVector  cgxdd;
    PrMatrix      C;
    PrVector * qd_bar;
    PrVector qd_null;
    PrVector    * tq;
    PrVector    * motor_tq;        
    PrVector    * dac_ticks; 
    PrMatrix E,Et; // TODO: enable for variable caster numbers
    PrVector rtE,tE;// TODO: enable for variable caster numbers
    PrVector ctE;// TODO: enable for variable caster numbers
    PrVector tqE;// TODO: enable for variable caster numbers
    PrMatrix rot;
    Vehicle *veh;
    //M3DFilter2 * Fqd_meka[8];
    Filter Fqd;
    Filter Fcxdd;
    Filter Fgxd;
    Filter Fxd;    
    Filter FtE;    
    Pcv_Params params;
    Pcv_Command cmd;    
    Pcv_Status status;
    Float ENC_REV2CNT;    
    Float hw_freq;
    Float cur_time;  
    PrVector  * q;
    PrVector  * motor_vel;
    PrVector  * qd;
    PrVector * e;
    PrVector         rxd;
    PrVector        rgxd;
    PrVector   x,  xd;
    PrVector  gx, gxd, gxdd;
    PrVector  dx, dxd, dxdd;    
    PrVector cxd;       
    bool first_time;
    Float old_traj_goal[3];
    int tmp_cnt;
};

#endif // _Ctrl_h_