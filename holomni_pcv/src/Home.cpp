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
#include <assert.h>

#include "Home.h"
using namespace std;

Home::Home( int step_freq, Pcv_Params params_in,vector<int> calibrated_in):params(params_in.home),calibrated(calibrated_in),
    des_tq(8,0.0),first_step(8,1),e_des(8,0),e_old(8,0),e(8,0),e_offset(8,0),e1(8,0),state(8,0)
{
    hw_freq = step_freq;
    hw_period = 1.0/hw_freq;
}

void Home::Step(vector<long> enc_ticks, vector<int> index_pulse)
{

    enum {INIT,STANDBY,READY,DONE};
    double dt;
    Float v_now;
    unsigned short axis;

    e=enc_ticks;

    for( axis=0; axis<8 ; axis+=2)
    {
        if (calibrated[axis])
            continue;
        if (first_step[axis])
        {
            first_step[axis]=0;
            e_des[axis]=e_old[axis]=e[axis];
            state[axis] = INIT;
        }

        if( state[axis]!=DONE )
        {
            e_now[axis]=e[axis];
            switch(state[axis])
            {
            case INIT:
                if( ! index_pulse[axis])
                { e1[axis]=e[axis];
                    state[axis] = STANDBY;
                }
                break;

            case STANDBY:
                if( ! index_pulse[axis])
                { if( e_now[axis] - e1[axis] > 800 )
                    { state[axis] = READY;
                    }
                }
                else
                    state[axis] = INIT;
                break;

            case READY:
                if(index_pulse[axis])
                {
                    calibrated[axis]=1;
                    e1[axis]=e[axis];
                    e_offset[axis] = params.offset[axis]+(e_now[axis]+e1[axis])/2L;
                    state[axis] = DONE;
                }
                break;

            case DONE:
                break;    // DO NOTHING

            } //case

            dt = hw_period;
            if ( dt != 0.0 )
            {
                e_des[axis] += (long)(params.v_des*dt);
                if(e_des[axis]-e_now[axis]>params.e_err)
                    e_des[axis] = e_now[axis]+params.e_err;
                v_now = (e_now[axis]-e_old[axis])/dt;
                tq = (long)(-params.KPz_*(e_now[axis]-e_des[axis]))-(long)(params.KVz_*v_now);
                des_tq[axis]=-tq;
                e_old[axis] = e_now[axis];
            }
        } // while
    } // for axis

    return;
} 
