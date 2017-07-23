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

#ifndef _Traj3_h_
#define _Traj3_h_

#include "matrix/PrVector.h"

class
Traj3
{
public:
  Traj3(double freq);
 ~Traj3();  

  void curPos(PrVector &_curPos);
  void curVel(PrVector &_curVel);
  void maxVel(double v_lim=0.5, double w_lim=1.57);
  void maxVel(PrVector &_maxVel);
  void accel(double lin_acc=0.25, double rot_acc=1.0);
  void accel(PrVector &acc);
  void dest2(PrVector &destPos);

  // FOR VELOCITY CONTROL
  void get_uV(PrVector &destUvel, PrVector &trajPos,
              PrVector &trajVel,  PrVector &trajAcc);

  void get_V_lin(PrVector &destvel, PrVector &trajPos,
                 PrVector &trajVel, PrVector &trajAcc);

  int get(PrVector &trajPos, PrVector &trajVel, PrVector &trajAcc);

private:
  double period_;
  double v_lim_;
  double w_lim_;
  double lin_acc_;
  double rot_acc_;

  PrVector *curPos_;
  PrVector *curVel_;

  PrVector *destPos_;
  PrVector *destVel_;
  PrVector *acc_;
  PrVector *maxVel_;

};

#endif // _Traj3_h_
