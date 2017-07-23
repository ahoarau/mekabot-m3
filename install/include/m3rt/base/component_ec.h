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

#ifndef  M3RT_COMPONENT_EC_H
#define  M3RT_COMPONENT_EC_H

#include "m3rt/base/component.h"
#include "m3rt/base/component_base.pb.h"
#include "m3rt/base/m3ec_def.h"
#include "m3rt/base/toolbox.h"



namespace m3rt
{
////////////////////////////////////////////////////////////////

/**
 * @brief
 *
 */
class M3ComponentEc: public M3Component{
	public:
		M3ComponentEc(int p=EC_PRIORITY):M3Component(p),shm(NULL),pdo_id(0),virtual_mode(0){}
		friend class M3RtSystem;
	protected:
        /**
         * @brief
         *
         */
        void PrettyPrint();
		//Deprecated: DSP is 16bit word, Client is 32
		//So when doing a sizeof() on a PDO struct that contains
		//an array, can be off by 2 as the client will pad align to word siz
		//This functionality is no longer needed, but must be careful in the future
		//of this effect.
		//virtual size_t GetStatusPdoSize()=0;
		//virtual size_t GetCommandPdoSize()=0;
        /**
         * @brief
         *
         * @return M3EtherCATStatus
         */
        virtual M3EtherCATStatus * GetEcStatus()=0;
        /**
         * @brief
         *
         * @param data
         */
        virtual void SetStatusFromPdo(unsigned char * data)=0;
        /**
         * @brief
         *
         * @param data
         */
        virtual void SetPdoFromCommand(unsigned char * data)=0;
		
		//Override these to pump virtual hardware data through the system
        /**
         * @brief
         *
         */
        virtual void SetStatusFromPdoVirtual(){};
        /**
         * @brief
         *
         */
        virtual void SetPdoFromCommandVirtual(){};
		
        /**
         * @brief
         *
         * @param slaves
         * @param slaves_responding
         * @return bool
         */
        bool SetSlaveEcShm(M3EcSlaveShm * slaves, int slaves_responding);
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
         * @param name
         * @param id
         */
        void RegisterPdo(const char * name, int id){pdo_names.push_back(name);pdo_ids.push_back(id);}
        /**
         * @brief
         *
         * @param id
         * @return bool
         */
        bool IsPdoVersion(int id){return pdo_id==id;}
        /**
         * @brief
         *
         * @return bool
         */
        bool IsVirtualMode(){return virtual_mode;}
        /**
         * @brief
         *
         */
        virtual void Shutdown(){if (shm) ResetCommandPdo(shm->cmd);}
        /**
         * @brief
         *
         */
        virtual void Startup();
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
		//By convention, on M3 EtherCAT devices will go into a safe reset
		//state when given a PDO of zero. Override if this is not the case.
        /**
         * @brief
         *
         * @param pdo
         */
        virtual void ResetCommandPdo(unsigned char * pdo){memset(pdo,0,MAX_PDO_SIZE_BYTES);}
	private:
        M3EcSlaveShm * shm; 
        /**
         * @brief
         *
         * @return bool
         */
        bool IsEcError();
        int pdo_id; 
        std::vector<std::string> pdo_names; 
        std::vector<int> pdo_ids; 
        int tmp_cnt; 
        bool virtual_mode; 
};

}
#endif

