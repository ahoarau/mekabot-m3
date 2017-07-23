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

#include <math.h>

#include "matrix/PrMatrix.h"
#include "matrix/PrVector.h"

#include "Caster.h"

#ifndef SQ
#define SQ(X)  ((X)*(X))
#endif


Caster::Caster(
            double _Kx, double _Ky, double _ang, double enc_offset_fine,
	       Pcv_Params params_in)
            /*double  _b, double  _r,
            double  _f, double _Mf, double _If,
            double _Ih, double _Ii, double _Is, double _It, double _Ij,
            double _Ns, double _Nt, double _Nw,
            double _px, double _py, double _Mp, double _Ip) */
 :Lambda(3,3), Mu(3),
  Ce(3,3), Jdot(3,3), A(3,3), CC(3), L_Pk(3,3), Mu_Pk(3), tmp_cnt(0)
{
 Kx = _Kx;   Ky =  _Ky;
  b = params_in.caster.b;    bi= 1.0/b;   r = params_in.caster.r;    ri= 1.0/r;
  f = params_in.caster.f;    e = f-b;    Mf = params_in.caster.Mf;  If = params_in.caster.If;
 Ih = params_in.caster.Ih;  Ii = params_in.caster.Ii;    Is = params_in.caster.Is;  It = params_in.caster.It;    Ij = params_in.caster.Ij;
 Ns = params_in.caster.Ns;  Nt = params_in.caster.Nt;    Nw = params_in.caster.Nw;
 px = params_in.caster.px;  py = params_in.caster.py;    Mp = params_in.caster.Mp;  Ip = params_in.caster.Ip; 

 Px = Kx + px;
 Py = Ky + py;
 P2  = SQ(Px) + SQ(Py);
 ENC_REV2CNT = params_in.caster.ENC_REV2CNT;
// enc_offset = (_ang - M_PI_2) * Ns * ENC_RAD2CNT; //Add this later
 enc_offset = (_ang - M_PI_2) * Ns * ENC_RAD2CNT + enc_offset_fine;
//printf("%f, %f, %f, %f\n",_ang,enc_offset,Ns,ENC_RAD2CNT);
 Ce.at(2,2) = 1.0; // CONSTANT, SO SET IT NOW, FOREVER

 Fill_A();
 Fill_L_Pk();
 
}


void
Caster::Fill_A()
{
  double mfe2Ifih= Mf*e*e + If + Ii + Ih;
  double IsNs    = Is * Ns;
  double ItNt    = It * Nt;
  double ItNt2   = ItNt*Nt;
  
  //printf("%f, %f, %f, %f\n",Nt,It,ItNt,ItNt2);

  A.at(0,0) = mfe2Ifih + IsNs*Ns + ItNt;
  A.at(0,1) = (Ii - Ih - ItNt2) * Nw;
  A.at(0,2) = mfe2Ifih - IsNs - ItNt;

  A.at(1,0) = A.at(0,1);
  A.at(1,1) = Mf*r*r + (Ii + Ih + ItNt2)*Nw*Nw + Ij;
  A.at(1,2) = (Ii - Ih + ItNt ) * Nw;

  A.at(2,0) = A.at(0,2);
  A.at(2,1) = A.at(1,2);
  A.at(2,2) = mfe2Ifih + Is + It;
}


inline void
Caster::Fill_CC()
{
  register double  u_w = u + w;
  register double Mfre = Mf*r*e;

  CC.at(0) =  Mfre * v * u_w;
  CC.at(1) = -Mfre *  SQ(u_w);
  CC.at(2) =  CC[0];
}


void
Caster::Fill_L_Pk()
{
  L_Pk.at(0,0) =  Mp;
//L_Pk.at(0,1) =  0.0;            // ZERO AT CREATION
  L_Pk.at(0,2) = -Mp * Py;

//L_Pk.at(1,0) =  0.0;            // ZERO AT CREATION
  L_Pk.at(1,1) =  Mp;
  L_Pk.at(1,2) =  Mp * Px;

  L_Pk.at(2,0) = L_Pk.at(0,2);
  L_Pk.at(2,1) = L_Pk.at(1,2);
  L_Pk.at(2,2) = Ip + Mp*P2;
}


inline void
Caster::Fill_Mu_Pk()
{
  register double w2  = SQ(w);

  Mu_Pk.at(0) = -Mp * Px * w2;
  Mu_Pk.at(1) = -Mp * Py * w2;
//Mu_Pk.at(2) =  0.0;          // ZERO AT CREATION
}


inline void
Caster::Fill_Ce()  // Ce (C_theta) (a.k.a. Ji )
{
  Ce.at(0,0) =  bi*s;
  Ce.at(0,1) = -bi*c;
  Ce.at(0,2) = -bi*(Kx*c + Ky*s) - 1.0;

  Ce.at(1,0) =  ri*c;
  Ce.at(1,1) =  ri*s;
  Ce.at(1,2) =  ri*(Kx*s - Ky*c);

//   Ce.at(2,0) =  0.0;   // ZERO ON CREATION
//   Ce.at(2,1) =  0.0;   // ZERO ON CREATION
//   Ce.at(2,2) =  1.0;   // SET IN CONSTRUCTOR
}


inline void
Caster::Fill_Jdot()
{
  register double u_w = u + w;

  Jdot.at(0,0) =  b*c*u_w;
  Jdot.at(0,1) = -r*s*u_w;
  Jdot.at(0,2) =  Kx*w + b*c*u_w;

  Jdot.at(1,0) =  b*s*u_w;
  Jdot.at(1,1) =  r*c*u_w;
  Jdot.at(1,2) =  Ky*w + b*s*u_w;

//   Jdot.at(2,0) =  0.0;   // ZERO ON CREATION
//   Jdot.at(2,1) =  0.0;   // ZERO ON CREATION
//   Jdot.at(2,2) =  0.0;   // ZERO ON CREATION
}


void
Caster::Fill_LM( double const _u,   // sigma dot
                 double const _v,   //  rho  dot
                 double const _w)   // theta dot
{
  // TEMP VARIABLES FOR FAST COMPUTATION
  static PrVector   Qdot(3);

  static PrMatrix  Cet (3,3);

  static PrMatrix   ACe(3,3);
  static PrMatrix   L_0(3,3);

  static PrVector    JdQd(3);
  static PrVector  CeJdQd(3);
  static PrVector ACeJdQd(3);
  static PrVector CC_AJJQ(3);
  static PrVector    Mu_0(3);

  // SET STATE
  Qdot[0] = u = _u;
  Qdot[1] = v = _v;
  Qdot[2] = w = _w;
  

  // FILL JACOBIANS
  Fill_Ce();
  Ce.transpose( Cet );
  Fill_Jdot();

  // FILL JT-SPACE CC VECTOR (NOTE: 'A' IS CONSTANT)
  Fill_CC();
  // FILL OP-SPACE CC VECTOR (NOTE: 'L_Pk' IS CONSTANT)
  Lambda  = L_Pk;
  Fill_Mu_Pk();
  Mu  = Mu_Pk;

  //Lambda = (Cet * A * Ce) + L_Pk;
  A.multiply(Ce,ACe);
  Cet.multiply(ACe,L_0);
  Lambda += L_0;

  //Mu = [Cet * ( CC - (A*(Ce*(Jdot*Qdot))) )] + Mu_Pk;
  Jdot.multiply(Qdot,JdQd);
  Ce.multiply(JdQd,CeJdQd);
  A.multiply(CeJdQd,ACeJdQd);
  CC_AJJQ = CC - ACeJdQd;
  Cet.multiply(CC_AJJQ,Mu_0);
  Mu += Mu_0;
}

