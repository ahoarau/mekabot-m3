/* =======================================================================
   (c) 2006, Robert Holmberg

   PROPRIETARY and CONFIDENTIAL

   This file contains source code that constitutes proprietary and
   confidential information owned by Robert Holmberg (RAH) that is
   considered a trade secret of RAH.

   RAH retains the title, ownership and intellectual property rights
   in and to the Software and all subsequent copies regardless of the
   form or media.  Copying or distributing any portion of this file
   without the written permission of RAH is prohibited.

   Use of this code is governed by the license agreement,
   confidentiality agreement, and/or other agreement under which it
   was distributed. When conflicts or ambiguities exist between this
   header and the written agreement, the agreement supersedes this header.
   ========================================================================*/
 
#ifndef _Vehicle_h_
#define _Vehicle_h_

#include "PrGlobalDefn.h"
#include "Home.h"
#include "matrix/PrVector.h"
#include "matrix/PrMatrix.h"
#include "Caster.h"


/*** CONSTANTS ***/
#define INLB2KGM  (2.93e-4)

#define NUM_TRUSS_LINKS  6

// Constraint Matrix

class
Vehicle
{
public:

  static Vehicle* HandleGet(Pcv_Params params_in); // SINGLETON
                               // CREATION DOES A Home()
    ~Vehicle();

  void Home( void );

  void JointRad(PrVector &jRad, PrVector e);
  void MotorTq(PrVector const &mtq, PrVector &dac_ticks);
  //void JointTq(PrVector const &jTq);
  void JointTq(PrVector const &jtq, PrVector &motor_tq);
  void JointVel(PrVector &jnt_vel, PrVector motor_vel);
  void JtTq2MotAmp(PrVector const &jtq, PrVector &motAmp);

  // ADD PARTS THAT ARE FIXED TO VEHICLE
  // USE x,y LOCATION OF CENTER OF MASS OF EACH PIECE
  // USE INERTIA ABOUT PIECE'S OWN CENTER OF MASS
  void Add_Solid(double _x, double _y, double _M, double _I);

  // FILL Lambda AND Mu FOR VEHICLE (THIS OBJECT)
  void Dyn(PrVector qd, double w);

  void Add_Caster(double Kx, double Ky, double ang,double enc_offset_fine);

  void Fill_C(PrMatrix &C);
  void Fill_Jcp(PrMatrix &J);
  void Fill_Jt_gamma(PrMatrix &Jt);
  void Fill_E_p(PrMatrix &E);
  void Fill_E_q(PrMatrix &E);
  int GetNumCasters(){return num_casters; }
  PrVector Fill_tqS(PrVector const &qd, PrVector const &tq,
                    PrVector &tqS);

  PrMatrix Lambda;
  PrVector Mu;

private:

  //ServoToGo* stg_;

  Vehicle(Pcv_Params params_in);
//  Vehicle(Vehicle const &) {} // DISALLOW COPY

  bool  IsValid() const;

  void Fill_Mu_veh( double w );

  
  double X_veh,Y_veh,M_veh,I_veh;

  Caster *cstr[4];
  int num_casters;

  PrMatrix L_veh;
  PrVector Mu_veh;
  Pcv_Params params;
  Float ENC_REV2CNT;
  int tmp_cnt;
};

#endif // _Vehicle_h_

