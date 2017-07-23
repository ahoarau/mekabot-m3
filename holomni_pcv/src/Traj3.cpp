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

#include<stdio.h>
#include<math.h>
#include<assert.h>

#include "Traj3.h"

#define DECEL_WINDOW  0.003
#define POS_WINDOW    0.0002

Traj3::Traj3(double freq)
{ 
  assert( freq > 0.1 );
  period_=1.0/freq;
  curPos_  = new PrVector(3);
  curVel_  = new PrVector(3);
  destPos_ = new PrVector(3);
  destVel_ = new PrVector(3);
  acc_     = new PrVector(3);
  maxVel_  = new PrVector(3);
}

Traj3::~Traj3()
{ 
  delete curPos_;
  delete curVel_;
  delete destPos_;
  delete destVel_;
  delete acc_;
  delete maxVel_;
}

void 
Traj3::curPos(PrVector &_curPos)
{
  *curPos_ = _curPos;
}

void 
Traj3::curVel(PrVector &_curVel)
{
  *curVel_ = _curVel;
}

void
Traj3::maxVel(double v_lim, double w_lim)
{
 v_lim_ = fabs(v_lim);
 w_lim_ = fabs(w_lim);

 (*maxVel_)[0] = v_lim_;
 (*maxVel_)[1] = v_lim_;
 (*maxVel_)[2] = w_lim_;
}

void
Traj3::maxVel(PrVector &_maxVel)
{
  for(int i ; i<_maxVel.size() ; ++i)
  { (*maxVel_)[i] = fabs(_maxVel[i]);
  }
}


void
Traj3::accel(double lin_acc, double rot_acc)
{
  lin_acc_ = lin_acc;
  rot_acc_ = rot_acc;

  (*acc_)[0] = fabs(lin_acc_);
  (*acc_)[1] = fabs(lin_acc_);
  (*acc_)[2] = fabs(rot_acc_);
}


void
Traj3::accel(PrVector &acc)
{
  for(int i ; i<acc.size() ; ++i)
  { (*acc_)[i] = fabs(acc[i]);
  }
}


void
Traj3::dest2(PrVector &destPos)
{
  PrVector errPos(3);
  double   errLi;

  *destPos_=destPos;

  errPos = *destPos_ - *curPos_;
  errLi  = HYPOT(errPos[0],errPos[1]);
  if(1.0/errLi < POS_WINDOW )
    errLi = 1;        // SHOULD BE 1/POS_WINDOW ???
  errLi = 1.0/errLi;  // AT 1, IT GIVES A DEADBAND


  (*acc_)[0] = fabs(lin_acc_ * errPos[0]*errLi);
  (*acc_)[1] = fabs(lin_acc_ * errPos[1]*errLi);

  (*acc_)[2] = rot_acc_;


   (*maxVel_)[0] = fabs(v_lim_ * errPos[0]*errLi);
   (*maxVel_)[1] = fabs(v_lim_ * errPos[1]*errLi);

   (*maxVel_)[2] = w_lim_;
}


void
Traj3::get_uV(PrVector &destUvel, PrVector &trajPos,
              PrVector &trajVel,  PrVector &trajAcc)
{
  PrVector errVel(3);
  double   velL;          // LENGTH

  // LIMIT LIN COMMAND SPEEDS TO UNIT MAG LIN
  velL = HYPOT(destUvel[0],destUvel[1]);  // 'L' IS ALWAYS POSITIVE
  if( velL > 1.0 )  // WON'T DIVIDE BY zero BECAUSE THIS CHECK
  { destUvel[0] = destUvel[0]/velL;
    destUvel[1] = destUvel[1]/velL;
  }
  // LIMIT ROT COMMAND SPEED TO UNIT MAG ROT
  if( destUvel[2] >  1.0 ) destUvel[2] =  1.0;
  if( destUvel[2] < -1.0 ) destUvel[2] = -1.0;

  // SCALE UNIT INPUT TO V_LIMIT AND W_LIMIT
  errVel[0] = destUvel[0]*v_lim_ - (*curVel_)[0];
  errVel[1] = destUvel[1]*v_lim_ - (*curVel_)[1];
  errVel[2] = destUvel[2]*w_lim_ - (*curVel_)[2];

  double   accel;         // ACCELERATION TMP VAR
  double   errL,errLi;    // LENGTHS
  double   v_win =  lin_acc_ * period_; // MAX delta_V MAG.
  double   w_win =  rot_acc_ * period_; // MAX delta_W MAG.

  //LIN ACCEL AT MAX UNLESS ONE STEP TO ZERO ERROR
  errL  = HYPOT(errVel[0],errVel[1]); // 'L' IS ALWAYS POSITIVE
  if( errL < v_win*0.002 )
    errL = v_win*0.002;
  if( errL < v_win )
    accel = lin_acc_ * errL/v_win;
  else
    accel = lin_acc_;
  errLi = 1.0/errL;
  trajAcc[0] = accel * errVel[0]*errLi;  // x component
  trajAcc[1] = accel * errVel[1]*errLi;  // y component

  //ROT ACCEL AT MAX UNLESS ONE STEP TO ZERO ERROR
  if     ( errVel[2] >  w_win ) trajAcc[2] =  rot_acc_;
  else if( errVel[2] < -w_win ) trajAcc[2] = -rot_acc_;
  else    trajAcc[2] =  rot_acc_ * errVel[2]/w_win;

  // COMPUTE NEW VELOCITIES
  trajVel = *curVel_ + trajAcc*period_;

  // COMPUTE NEW POSITIONS
  trajPos = *curPos_ + trajVel*period_ + 
             trajAcc*(0.5*period_*period_);

  // AGE THE VALUES
  *curVel_ = trajVel;
  *curPos_ = trajPos;
}


void
Traj3::get_V_lin(PrVector &destVel, PrVector &trajPos,
                 PrVector &trajVel, PrVector &trajAcc)
{
  PrVector errVel(3);
  double   velL;             // LENGTH
  double   velR;             // RATIO

  // LIMIT COMMAND SPEEDS DUE TO LIN (SCALE ENTIRE VECTOR)
  velL = HYPOT(destVel[0],destVel[1]);  // 'L' IS ALWAYS POSITIVE
  velR = velL/v_lim_;
  if( velR > 1.0 )  // WON'T DIVIDE BY zero BECAUSE THIS CHECK
    destVel *= (1.0/velR);
  // LIMIT COMMAND SPEEDS DUE TO ROT (SCALE ENTIRE VECTOR)
  velL = fabs( destVel[2] );
  velR = velL/w_lim_;
  if( velR > 1.0 )  // WON'T DIVIDE BY zero BECAUSE THIS CHECK
     destVel *= (1.0/velR);

  errVel = destVel - *curVel_; 

  double   accel;      // ACCELERATION TMP VAR
  double   errL;       // LENGTHS
  double   v_win =  lin_acc_ * period_; // MAX delta_V MAG.

  // LIN ACCEL AT MAX UNLESS ONE STEP TO ZERO ERROR
  // ROT ACCEL IS PROPORTIONAL TO LIN ACCEL...
  // ROT ACCEL IS TO ROT V_ERR, AS LIN ACCEL IS TO LIN V_ERR
  // ASSUMES USE OF INPUT WITH LIMITED ROT SPEED
  errL  = HYPOT(errVel[0],errVel[1]); // 'L' IS ALWAYS POSITIVE
  if( errL < v_win*0.002 )  // AVOID DIV. BY ZERO
    errL = v_win*0.002;
  if( errL < v_win )        // ONE STEP TO ZERO ERROR
    accel = lin_acc_ * errL/v_win;
  else
    accel = lin_acc_;

  trajAcc = errVel * (accel/errL); // RATIO ALWAYS POSITIVE

 // COMPUTE NEW VELOCITIES
  trajVel = *curVel_ + trajAcc*period_;

  // COMPUTE NEW POSITIONS
  trajPos = *curPos_ + trajVel*period_ + 
             trajAcc*(0.5*period_*period_);

  // AGE THE VALUES
  *curVel_ = trajVel;
  *curPos_ = trajPos;
}


int 
Traj3::get(PrVector &trajPos, PrVector &trajVel, PrVector &trajAcc)
{
  int i;
  int goalcount = (*destPos_).size();
  double decelDist, decelError, decel, error;
  PrVector errPos(3);


  for (i=0; i < (*destPos_).size() ; ++i)
    {
      error = (*destPos_)[i] - (*curPos_)[i];
      if ( ((*acc_)[i] != 0) && (fabs(error) > POS_WINDOW) )
  {
    /* check to see if we can decelerate to *destPos_ */
    decelDist=(*curVel_)[i]*fabs((*curVel_)[i])/(*acc_)[i]/2.0; 
    decelError=decelDist - error;
    //printf("%i destPos:%f curPos:%f error:%f, decelDist:%f decelError:%f\n",i,(*destPos_)[i],(*curPos_)[i],error, decelDist,decelError);
    if ( ((error>0) && (decelError>0) && (decelError <  DECEL_WINDOW)) ||
         ((error<0) && (decelError<0) && (decelError > -DECEL_WINDOW)) )
      {
        /* calculate deceleration and apply it */
        decel = -(*curVel_)[i] * (*curVel_)[i]/error/2.0;
        (*curVel_)[i] += period_ * decel;
        trajAcc[i] = (decel>0 ? (*acc_)[i] : -(*acc_)[i]);
        if ( fabs(error) < POS_WINDOW) // NOT USED CURRENTLY ???
        { (*curVel_)[i] = 0.0;
          (*curPos_)[i] = (*destPos_)[i];
        }
      }
    else
      {
        /* check to see if we need to accelerate, because we don't
     need to decelerate */
        /* positive acceleration: */
        if ( ((decelError < 0.0) && ((*curVel_)[i] < (*maxVel_)[i])) ||
        ((*curVel_)[i] < -(*maxVel_)[i]) )
    {
      (*curVel_)[i] += period_ * (*acc_)[i];
      trajAcc[i] =  (*acc_)[i];
      if ((*curVel_)[i] > (*maxVel_)[i])
      { (*curVel_)[i] = (*maxVel_)[i];
      }
    }
        if ( ((decelError > 0.0) && ((*curVel_)[i] > -(*maxVel_)[i])) ||
        ((*curVel_)[i] > (*maxVel_)[i]) )
    {
      (*curVel_)[i] -= period_ * (*acc_)[i];
      trajAcc[i] = -(*acc_)[i];
      if ((*curVel_)[i] < -(*maxVel_)[i])
      { (*curVel_)[i] = -(*maxVel_)[i];
      }
    }
        /* else we don't need to do anything */
        // ie. stay at +/- maxVel
      }
    (*curPos_)[i] += period_ * (*curVel_)[i];
  }
      else
        {
          (*curVel_)[i] = 0.0;
          trajAcc[i] = 0.0;
//    (*curPos_)[i] = (*destPos_)[i];
          --goalcount;
        }

      trajVel[i] = (*curVel_)[i];
      trajPos[i] = (*curPos_)[i];

    }

  return goalcount;
}

