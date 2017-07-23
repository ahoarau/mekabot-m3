/* 
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef  M3RT_COMP_SHM_H
#define  M3RT_COMP_SHM_H

#include "m3rt/base/component.h"
#include "m3rt/base/component_base.pb.h"
//#include <m3rt/toolbox/toolbox.h>
#ifdef __cplusplus
extern "C" {
#endif 
#include <rtai_registry.h>
#include <rtai.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sched.h>
#include <rtai_nam2num.h>
#include <rtai_sem.h>
#include <rtai_malloc.h> 
#ifdef __cplusplus
}  // extern "C"
#endif 
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/m3ec_def.h"
#include "m3rt/base/toolbox.h"


namespace m3rt
{
////////////////////////////////////////////////////////////////
 
/**
 * @brief
 *
 */
class M3CompShm: public M3Component{
	public:
		M3CompShm(int p=EC_PRIORITY):M3Component(p),shm(NULL),status_sem(NULL), command_sem(NULL){}
		
	protected:		
        /**
         * @brief
         *
         * @return size_t
         */
        virtual size_t GetStatusSdsSize()=0;
        /**
         * @brief
         *
         * @return size_t
         */
        virtual size_t GetCommandSdsSize()=0;
        /**
         * @brief
         *
         * @param data
         */
        virtual void SetCommandFromSds(unsigned char * data)=0;
        /**
         * @brief
         *
         * @param data
         */
        virtual void SetSdsFromStatus(unsigned char * data)=0;
        /**
         * @brief
         *
         * @param filename
         * @return bool
         */
        virtual bool ReadConfig(const char * filename);
        /**
         * @brief
         *
         */
        virtual void Startup();
        /**
         * @brief
         *
         */
        virtual void Shutdown();
        /**
         * @brief
         *
         */
        virtual void StepStatus();
        /**
         * @brief
         *
         */
        virtual void StepCommand();
        /**
         * @brief
         *
         * @param sds
         */
        virtual void ResetCommandSds(unsigned char * sds){memset(sds,0,MAX_SDS_SIZE_BYTES);}
        /**
         * @brief
         *
         */
        void request_status(){rt_sem_wait(status_sem);}
        /**
         * @brief
         *
         */
        void release_status(){rt_sem_signal(status_sem);}
        /**
         * @brief
         *
         */
        void request_command(){rt_sem_wait(command_sem);}
        /**
         * @brief
         *
         */
        void release_command(){rt_sem_signal(command_sem);}
	private:
        M3Sds * shm;				 
        SEM * status_sem; 
        SEM * command_sem;	 
        std::string shm_id; 
};

}
#endif

