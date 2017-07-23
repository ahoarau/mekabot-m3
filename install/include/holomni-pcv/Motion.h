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

#ifndef _Motion_h_
#define _Motion_h_

//#include "MyDeque.h"
#include "PrGlobalDefn.h"


typedef enum {
 TRAJ_M_CIRC,
 TRAJ_M_CVEL,
 TRAJ_M_HOME,
 TRAJ_M_NULL,
 TRAJ_M_PACE,
 TRAJ_M_RAND, 
 TRAJ_M_SQAR,
 TRAJ_M_SAVE,
 TRAJ_M_JOYP,
 TRAJ_M_JOYV,
 TRAJ_M_ACCEL,
 TRAJ_M_STEP,
 TRAJ_M_GOAL,
 TRAJ_M_ZERO,
 TRAJ_M_W,
 TRAJ_M_OFF,
 TRAJ_M_OP_FORCE,
 TRAJ_M_CART_LOCAL,
 TRAJ_M_CART_GLOB,
 TRAJ_M_JOINT_TORQUE
} TrajMode;

typedef struct
{ TrajMode mode;
  char name[12];
} TrajType;

typedef enum {
 CTRL_LM,
 CTRL_L,
 CTRL_L_CONST,
 CTRL_TEST,
 CTRL_CALI,
 CTRL_CF_CMD,
 CTRL_OFF
} CtrlMode;

typedef enum {
 CTRL_F_LOCAL,
 CTRL_F_GLOBAL
} CtrlFrame;

typedef enum {
 FRIC_NONE,
 FRIC_ANTI,
 FRIC_BOHR
//  FRIC_SIGD,
} FricMode;

typedef struct Param_struct
{ Float val;
  char  name[20];
} Param_struct;


#define TRAJ_END_EXIT  (-100)
#define TRAJ_END_NOW   (-1)
#define TRAJ_END_NONE  ( 1E7) // 115 DAYS

// MOTION LIST TYPES
/*typedef MyDeque<Motion *>         MotionList;
typedef MyDequeIterator<Motion *> MotionIter;

// SHORTCUT TO GET LOCAL COPY OF SINGLETON
#define GET_mq   MotionList &mq = Motion::ListGet(); // get Motion Queue
#define GET_mi   MotionIter &mi = Motion::IterGet(); // get Motion Itererator
#define GET_mx   MotionIter &mx = Motion::NextGet(); // get Motion Next
*/

#define REV2RAD_  (6.28318530718)
#define DEG2RAD_  (0.0174532925199)

// PROTOTYPES
TrajMode const  traj_name2mode( char *name );
char     const *traj_mode2name( TrajMode mode );

#endif // _Motion_h_
