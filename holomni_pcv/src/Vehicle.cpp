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

#include <stdio.h>
#include <math.h>

#include "matrix/PrVector.h"
#include "matrix/PrMatrix.h"

#include "Caster.h"
#include "Vehicle.h"

#ifndef SQ
#define SQ(x) ((x)*(x))
#endif

#define N_2X2_MATRIX  \
    double N_00 = -cstr[i]->Ns; \
    double N_10 = -cstr[i]->Nt; \
    double N_11 =  cstr[i]->Nt * cstr[i]->Nw;

#define Ni_2X2_MATRIX  \
    double Ni_00 = -1.0/cstr[i]->Ns; \
    double Ni_10 = -1.0/(cstr[i]->Ns * cstr[i]->Nw); \
    double Ni_11 =  1.0/(cstr[i]->Nt * cstr[i]->Nw);


// SINGLETON
static Vehicle* gVehicle = NULL;

bool 
Vehicle::IsValid() const 
{ return gVehicle ? true : false; 
}

Vehicle* 
Vehicle::HandleGet(Pcv_Params params_in)
{
  if( gVehicle == NULL )
  {
    if( !( gVehicle = new Vehicle(params_in) ) ||
        !gVehicle->IsValid() )
    {
      return NULL;
    }
  }
  return gVehicle;
}


Vehicle::Vehicle(Pcv_Params params_in)
  :Lambda(3,3),Mu(3),
   X_veh(0),Y_veh(0),M_veh(0),I_veh(0),num_casters(0),
   L_veh(3,3),Mu_veh(3), params(params_in), tmp_cnt(0),
   ENC_REV2CNT(params.caster.ENC_REV2CNT)
{
  //stg_ = ServoToGo::HandleGet();

  return;
}

Vehicle::~Vehicle()
{
  // SHOULD SET DAC's TO ZERO HERE, 
  // BUT, FOR NOW, LET WATCHDOG DO IT FOR US

  //gVehicle = NULL; // SINGLETON

  for( int i=0; i<num_casters; i++ )
  {
    if (cstr[i] != NULL)
      delete cstr[i];
    cstr[i] = NULL;
  }
  num_casters = 0;
  gVehicle = NULL;
}


void
Vehicle::Home()
{
  /*::Home( cstr[0]->enc_offset,
          cstr[1]->enc_offset,
          cstr[2]->enc_offset,
          cstr[3]->enc_offset);*/
}


// void
// Vehicle::Add_Caster( double Kx, double Ky, double ang ) // EASY IMPLEMENTATION FOR NOW
// {
//   cstr[num_casters] = new Caster( Kx, Ky, ang );
//   num_casters++;
// }


void
Vehicle::Add_Caster( double _Kx, double _Ky, double _ang,double enc_offset_fine)
                     /*double  _b, double  _r,
                     double  _f, double _Mf, double _If,
                     double _Ih, double _Ii, double _Is, double _It, double _Ij,
                     double _Ns, double _Nt, double _Nw,
                     double _px, double _py, double _Mp, double _Ip) */
{
  cstr[num_casters] = new Caster( _Kx, _Ky, _ang,enc_offset_fine,params);
                                   /*_b,  _r,
                                  _f, _Mf, _If,
                                  _Ih, _Ii, _Is, _It, _Ij,
                                  _Ns, _Nt, _Nw,
                                  _px, _py, _Mp, _Ip);*/
  num_casters++;  
}


void
Vehicle::JointRad(PrVector &jRad, PrVector e)
{ 
  //STG_LONGBYTE e[8];
  //stg_->EncReadAll(e); // LATCHed by S2G on IRQ GENERATION

  for( int i=0; i<num_casters; i++ )
  {
    //Ni_2X2_MATRIX;
    /*#define Ni_2X2_MATRIX  \
    double Ni_00 = -1.0/cstr[i]->Ns; \
    double Ni_10 = -1.0/(cstr[i]->Ns * cstr[i]->Nw); \
    double Ni_11 =  1.0/(cstr[i]->Nt * cstr[i]->Nw);*/
    
    Float Ni_00 = -1.0/cstr[i]->Ns;
    Float Ni_10 = -1.0/(cstr[i]->Ns * cstr[i]->Nw);
    Float Ni_11 =  1.0/(cstr[i]->Nt * cstr[i]->Nw);
    
    int j = 2*i;

    //jRad[ j ] = -(Ni_00*e[j])*ENC_CNT2RAD - params.geometry.steering_offsets[i]*ENC_CNT2RAD;
    double es,et;
    es = e[j]+cstr[i]->enc_offset;
    et = e[j+1];
    jRad[ j ] = -(Ni_00*es           )*ENC_CNT2RAD;
    jRad[j+1] = -(Ni_10*es + Ni_11*et)*ENC_CNT2RAD;
    //printf("%d,%f, %f, %f, %f \n",i,ENC_CNT2RAD,cstr[i]->enc_offset,jRad[ j ],e[j] );
// NOTE: MINUS SIGN DUE TO DIRECTION OF ENCODERS
// NOTE: ENCODERS GIVE OPPOSITE SIGN OF MOTOR ROTATION

    cstr[i]->a = jRad[j];
    cstr[i]->c = cos(jRad[j]);
    cstr[i]->s = sin(jRad[j]);
  }
}

void
Vehicle::JointVel(PrVector &jnt_vel, PrVector motor_vel)
{ 
  //STG_LONGBYTE e[8];
  //stg_->EncReadAll(e); // LATCHed by S2G on IRQ GENERATION

  for( int i=0; i<num_casters; i++ )
  {
    //Ni_2X2_MATRIX;
    /*#define Ni_2X2_MATRIX  \
    double Ni_00 = -1.0/cstr[i]->Ns; \
    double Ni_10 = -1.0/(cstr[i]->Ns * cstr[i]->Nw); \
    double Ni_11 =  1.0/(cstr[i]->Nt * cstr[i]->Nw);*/
    
    Float Ni_00 = -1.0/cstr[i]->Ns;
    Float Ni_10 = -1.0/(cstr[i]->Ns * cstr[i]->Nw);
    Float Ni_11 =  1.0/(cstr[i]->Nt * cstr[i]->Nw);
    
    int j = 2*i;
    
    double es,et;
    es = motor_vel[j];
    et = motor_vel[j+1];
    jnt_vel[ j ] = -(Ni_00*es           );
    jnt_vel[j+1] = -(Ni_10*es + Ni_11*et);    
  }
}

void 
Vehicle::MotorTq(PrVector const &mtq, PrVector &dac_ticks)
/*** Each motor will produce the specified torque ***/
/*** tq should be a vector of 8 entries in Nm ***/
{
  long counts;
  
  for( unsigned char j=0; j<2*num_casters; j++ )
  { 
    dac_ticks[j] =( mtq[j] * (1.0/params.caster.Nm_PER_AMP) * (1.0/params.caster.AMPS_PER_VOLT) * params.caster.COUNTS_PER_VOLT );    
  }
}


void 
Vehicle::JointTq(PrVector const &jtq, PrVector &motor_tq)
/*** tq should be a vector of 8 entries in Nm ***/
{  
  for(int i=0; i<num_casters; i++ )
  { 
    //Ni_2X2_MATRIX;
    /*#define Ni_2X2_MATRIX  \
    double Ni_00 = -1.0/cstr[i]->Ns; \
    double Ni_10 = -1.0/(cstr[i]->Ns * cstr[i]->Nw); \
    double Ni_11 =  1.0/(cstr[i]->Nt * cstr[i]->Nw);*/
    
    Float Ni_00 = -1.0/cstr[i]->Ns;
    Float Ni_10 = -1.0/(cstr[i]->Ns * cstr[i]->Nw);
    Float Ni_11 =  1.0/(cstr[i]->Nt * cstr[i]->Nw);

    int j = 2*i;
    motor_tq[ j ] = Ni_00 * jtq[ j ] + Ni_10 * jtq[j+1];
    motor_tq[j+1] =                    Ni_11 * jtq[j+1];
  }
   
}


// CONSTRAINT MATRIX

void
Vehicle::Fill_C(PrMatrix &C)
{
  for(int i=0; i<num_casters ; i++ )
  { 
    double bi = cstr[i]->bi;
    double ri = cstr[i]->ri;
    double c  = cstr[i]->c;
    double s  = cstr[i]->s;
    double Kx = cstr[i]->Kx;
    double Ky = cstr[i]->Ky;

    int j = 2*i;

    C.elementAt( j ,0) =   bi*s;
    C.elementAt( j ,1) =  -bi*c;
    C.elementAt( j ,2) =  -bi*(Kx*c+Ky*s) - 1.0;

    C.elementAt(j+1,0) =   ri*c;
    C.elementAt(j+1,1) =   ri*s;
    C.elementAt(j+1,2) =   ri*(Kx*s-Ky*c);
  }
}


// JACOBIAN VIA C.P.'s to minimize slip in odometry 
// USE Jt_cp to minimize contact forces

void
Vehicle::Fill_Jcp(PrMatrix &J)
{
  int i;
  static PrMatrix  Cp(8,3),Cpt(3,8),CptCp(3,3),CptCp_i(3,3);
  static PrMatrix  CptCli(3,8);

  for( i=0; i<num_casters; i++ )
  {   // Cli IS 2x2 BLOCK DIAGONAL, SO MULTIPLY 
      // IN PIECES TO AVOID SLOW NxN MATRIX OPERATIONS 
    double b = cstr[i]->b;
    double r = cstr[i]->r;
    double c = cstr[i]->c;
    double s = cstr[i]->s;
    double Kx= cstr[i]->Kx;
    double Ky= cstr[i]->Ky;
        

    int j = 2*i;

    // Note: sign of p-dot is: wheel as viewed from ground

    CptCli.at(0, j ) =  b*s;
    CptCli.at(0,j+1) =  r*c;
    CptCli.at(1, j ) = -b*c;
    CptCli.at(1,j+1) =  r*s;
    CptCli.at(2, j ) = -b*((Kx*c+Ky*s)+b);
    CptCli.at(2,j+1) =  r* (Kx*s-Ky*c);

    Cp.at( j ,0) =  1.0;
    Cp.at( j ,1) =  0.0;
    Cp.at( j ,2) = -b*s - Ky;

    Cp.at(j+1,0) =  0.0;
    Cp.at(j+1,1) =  1.0;
    Cp.at(j+1,2) =  b*c + Kx;
  }

  Cp.transpose(Cpt);
  Cpt.multiply(Cp,CptCp);
  CptCp.inverseSPD(CptCp_i);
  CptCp_i.multiply(CptCli,J);
}



// JACOBIAN to Minimize Motor Torques 

void
Vehicle::Fill_Jt_gamma(PrMatrix &Jt)
{
  int i;
  static PrMatrix N(8,8), C(8,3), NC(8,3), NtNC(8,3);
  static PrMatrix NCt(3,8),NCtNC(3,3),NCtNC_i(3,3);

  Fill_C( C );

  for( i=0; i<num_casters ; i++ )
  {   // N IS 2x2 BLOCK DIAGONAL, SO MULTIPLY 
      // IN PIECES TO AVOID SLOW NxN MATRIX OPERATIONS 
    N_2X2_MATRIX;

    int j = 2*i;

    NC.at( j ,0)   = N_00* C.at(j,0);
    NC.at( j ,1)   = N_00* C.at(j,1);
    NC.at( j ,2)   = N_00* C.at(j,2);
    NC.at(j+1,0)   = N_10* C.at(j,0) + N_11* C.at(j+1,0);
    NC.at(j+1,1)   = N_10* C.at(j,1) + N_11* C.at(j+1,1);
    NC.at(j+1,2)   = N_10* C.at(j,2) + N_11* C.at(j+1,2);

    NtNC.at(0, j ) = N_00*NC.at(j,0) + N_10*NC.at(j+1,0);
    NtNC.at(1, j ) = N_00*NC.at(j,1) + N_10*NC.at(j+1,1);
    NtNC.at(2, j ) = N_00*NC.at(j,2) + N_10*NC.at(j+1,2);
    NtNC.at(0,j+1) =                   N_11*NC.at(j+1,0);
    NtNC.at(1,j+1) =                   N_11*NC.at(j+1,1);
    NtNC.at(2,j+1) =                   N_11*NC.at(j+1,2);
  }

  // wdot = Cn xdot ; wdot = N qdot ; qdot = C xdot
  // Jgamma = Cn# N   ; Cn = N C
  // J_gamma  = [(NC)t * NC]i (NC)t N qdot
  // Jt_gamma = Nt (NC) [(NC)t * NC]i
  NC.transpose(NCt);
  NCt.multiply(NC,NCtNC);
  NCtNC.inverseSPD(NCtNC_i);
  NtNC.multiply(NCtNC_i,Jt);
}


// JACOBIAN FOR VIRTUAL LINKAGE AT CONTACT POINTS


void
Vehicle::Fill_E_p(PrMatrix &E)
{
  int i,j;
  double ex, ey, mag_inv;
  double pix, pjx, piy, pjy;
 
  // DESCRIBE CONNECTIONS OF VIRTUAL TRUSS

//   int pi[NUM_TRUSS_LINKS]={0,1,2,3,1};
//   int pj[NUM_TRUSS_LINKS]={1,2,3,0,3};
  int pi[NUM_TRUSS_LINKS]={0,3,1,2,1,2};
  int pj[NUM_TRUSS_LINKS]={3,1,2,0,0,3};

  for( int k=0; k<NUM_TRUSS_LINKS; k++ )
  {
    i = pi[k];
    j = pj[k];

    pix = cstr[i]->Kx + cstr[i]->b*cstr[i]->c;
    pjx = cstr[j]->Kx + cstr[j]->b*cstr[j]->c;

    piy = cstr[i]->Ky + cstr[i]->b*cstr[i]->s;
    pjy = cstr[j]->Ky + cstr[j]->b*cstr[j]->s;

    ex = pix - pjx;
    ey = piy - pjy;
    mag_inv = 1.0/sqrt( SQ( ex ) + SQ ( ey ) );
    E.at(k, 2*i  ) = ex * mag_inv;
    E.at(k, 2*i+1) = ey * mag_inv;
    E.at(k, 2*j  ) =  -E.at(k, 2*i  );
    E.at(k, 2*j+1) =  -E.at(k, 2*i+1);
  }    
//Ep.display("Ep");

}


void
Vehicle::Fill_E_q(PrMatrix &E)
{
  int j = 2*num_casters;

  PrMatrix Ep(NUM_TRUSS_LINKS,j);
  PrMatrix Cli(j,j); 
  // q_dot = Cl * p_dot ==> p_dot = Cli * q_dot
  // eps_dot = Ep * p_dot
  // eps_dot = Ep * Cli * q_dot ==> E_q = Ep * Cli

  Fill_E_p( Ep );
  for(int i=0; i<num_casters; i++ )
  {     
    j = 2*i;
    Cli.at( j , j ) =  cstr[i]->b*cstr[i]->s;
    Cli.at( j ,j+1) =  cstr[i]->r*cstr[i]->c;
    
    Cli.at(j+1, j ) = -cstr[i]->b*cstr[i]->c;
    Cli.at(j+1,j+1) =  cstr[i]->r*cstr[i]->s;
  }
  
  Ep.multiply(Cli, E );
//Eq.display("Eq");
}


void
Vehicle::Add_Solid(double _x, double _y,
                   double _M, double _I)
{
  // ALL QUANTITIES zero ON OBJECT CREATION

  // COMPUTE NEW veh QUANTITIES w/ ARGS AND PREVIOUS M & I
  X_veh  = (M_veh * X_veh + _M * _x) / (M_veh + _M);
  Y_veh  = (M_veh * Y_veh + _M * _y) / (M_veh + _M);
  M_veh += _M;  // TOTAL veh MASS
  I_veh += _I + _M*(SQ(_x) + SQ(_y)); // 'I' AT veh CENTER

  // COMPUTE MASS MATRIX USING NEW veh QUANTITIES
  L_veh.at(0,0) =  M_veh;
  L_veh.at(0,1) =  0.0;
  L_veh.at(0,2) = -M_veh * Y_veh;

  L_veh.at(1,0) =  0.0;
  L_veh.at(1,1) =  M_veh;
  L_veh.at(1,2) =  M_veh * X_veh;

  L_veh.at(2,0) =  L_veh.at(0,2);
  L_veh.at(2,1) =  L_veh.at(1,2);
  L_veh.at(2,2) =  I_veh;
  
}


inline void
Vehicle::Fill_Mu_veh( double w )
{
  register double w2 = SQ(w);

  Mu_veh.at(0)  = -M_veh * X_veh * w2;
  Mu_veh.at(1)  = -M_veh * Y_veh * w2;
//Mu_veh.at(2)  =  0.0;           // ZERO AT CREATION
}


void
Vehicle::Dyn(PrVector qd, double w)
{
  // INITIALIZE TO VEHICLE PROPERTIES
  Lambda = L_veh;
  Fill_Mu_veh( w );
  Mu = Mu_veh;
  
  // ADD DYNAMICS FROM THE CASTERS
  for(int i=0; i<num_casters; i++)
  { int j = 2*i;
    cstr[i]->Fill_LM(qd[j],qd[j+1],w);
    Lambda += cstr[i]->Lambda;
    Mu += cstr[i]->Mu;    
  }
  
}


PrVector
Vehicle::Fill_tqS(PrVector const &qd, PrVector const &tq,
                  PrVector &tqS)
{
  int i,j;
  double m,b,frd,atq,tqX;
//   double bM[4] = {0.35, 0.57, 0.35, 0.57};
//   double mM[4] = {0.21, 0.17, 0.21, 0.17};
//   double tqM   = 0.35;
  double rdM   = 25.0;
  double bM[4] = {0.20, 0.50, 0.20, 0.50};
  double mM[4] = {0.10, 0.10, 0.10, 0.10};
  double tqM   = 0.60;

  double tqA = 0.0;
  static PrVector fS(3);

  for(i=0; i<4; i++)
  { j = 2*i;
    frd = 1.0 - fabs(qd[j+1])/rdM;
    b = bM[i]*frd;
    m = mM[i]*frd;
    atq = fabs(tq[j]);
    tqX = atq<tqM ? atq/tqM : 1.0;
    tqS[j] = (m*qd[j] + (qd[j]>0?b:-b)) * tqX;
    tqA += tqS[j];
  }

  fS[2] = tqA;
  return fS;
}

