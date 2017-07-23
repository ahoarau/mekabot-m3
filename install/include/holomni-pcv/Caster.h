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

#ifndef _Caster_h_
#define _Caster_h_

class PrVector;
class PrMatrix;

#include "Params.h"



class Caster
{
 public:
  Caster(double _Kx,       double _Ky,       double _ang,double enc_offset_fine,
	 Pcv_Params params);
         /*double  _b=XR_b,  double  _r=XR_r,
         double  _f=XR_f,  double _Mf=XR_Mf, double _If=XR_If,
         double _Ih=XR_Ih, double _Ii=XR_Ii, 
         double _Is=XR_Is, double _It=XR_It, double _Ij=XR_Ij,
         double _Ns=XR_Ns, double _Nt=XR_Nt, double _Nw=XR_Nw,
         double _px=XR_px, double _py=XR_py,
         double _Mp=XR_Mp, double _Ip=XR_Ip,*/	 
 ~Caster(){}

  PrMatrix Lambda;
  PrVector Mu;

  void Fill_LM( double const _u,   // sigma dot
                double const _v,   //  rho  dot
                double const _w);  // theta dot

  double Kx,Ky;   // x,y COORDS FROM VEHICLE CENTER TO STEER AXIS
  double b,bi;    // CASTER OFFSET (negative), b INVERSE
  double r,ri;    // WHEEL RADIUS  (positive), r INVERSE
  double f, e, Mf, If;  // FORK CG FROM STEER AXIS (negative), e, MASS, INERTIA
  double Ih, Ii, Is, It, Ij; // INERTIAS
  double Ns, Nt, Nw; // GEAR RATIOS, STEER, TRANSLATE, WHEEL
  double px,py,Mp,Ip;// PUMPKIN CG x,y COORDS FROM STEER AXIS, MASS, INERTIA

  double Px,Py,P2; // PUMPKIN GC x,y COORDS FROM VEHICLE CENTER

  double a,c,s;  // sigma, cos(sigma), sin(sigma)
  double u,v,w;  // sigma dot, rho dot, theta dot

  double   enc_offset; // FOR ARBITRARY ORIENTATION OF CASTER

 private:
         void Fill_A();
  inline void Fill_CC();

         void Fill_L_Pk();
  inline void Fill_Mu_Pk();

  inline void Fill_Ce( );
  inline void Fill_Jdot( );

  PrMatrix Ce;
  PrMatrix Jdot;
  PrMatrix A;
  PrVector CC;

  PrMatrix L_Pk;
  PrVector Mu_Pk;
  Float ENC_REV2CNT;
  int tmp_cnt;
};

#endif // _Caster_h_
