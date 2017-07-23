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
					
#ifndef M3RT_COMPONENT_H
#define  M3RT_COMPONENT_H

#include <string>
#include "m3rt/base/component_base.pb.h"
#include <google/protobuf/message.h>
#include <google/protobuf/descriptor.h>
#include <iostream>
#include "m3rt/base/toolbox.h"

namespace m3rt
{
class M3ComponentFactory;
class M3RtSystem;

/**
 * @brief
 *
 * A component can be in one of 4 states: \n
 *         M3COMP_STATE_INIT = 0; \n
 *         M3COMP_STATE_ERR = 1; \n
 *         M3COMP_STATE_SAFEOP = 2; \n
 *         M3COMP_STATE_OP = 3; \n
 *    \n
 *    State INIT: On creation, Startup() not yet called.\n
 *    State OP: Initialized, running normally. Status and Command data is modified. A module must be placed in OP by an external process.\n
 *    State SAFEOP: Only Status data is modified. SAFEOP is set externally in case of system error.\n
 *    State ERR: A non-recoverable error or safety exception has been triggered.\n
 *    \n
 *    Each component must implement  Startup, Shutdown, StepStateOp, StepStateSafeOp, StepStateError \n
 *    After successful Startup,     it should do a SetStateSafeOp else SetStateError\n
 *    After successful StepStateOp, it should do a SetStateOp else SetStateError\n
 *    After successful StepStateSafeOp, it should do a SetStateSafeOp else SetStateError\n
 *    After a StepStateError, 	  it should do a SetStateError\n
 *    \n
 *    The client process must manually place each SafeOp component in state Op.
 */
class M3Component{
	public:
		M3Component(int p=0):factory(NULL),priority(p),version_id(-1),doc_path(""){GOOGLE_PROTOBUF_VERIFY_VERSION;}
        /**
         * @brief
         *
         */
        virtual ~M3Component(){}
		friend class M3RtSystem;
		friend class M3RtLogService;
        /**
         * @brief
         *
         * @return std::string
         */
        std::string GetName(){return GetBaseStatus()->name();}
        /**
         * @brief
         *
         * @return int
         */
        int  GetState(){return (int)GetBaseStatus()->state();}
        /**
         * @brief
         *
         * @return int
         */
        int  GetPriority(){return priority;}
        /**
         * @brief
         *
         * @param p
         */
        void  SetPriority(int p){priority=p;}
        /**
         * @brief
         *
         * @param v
         */
        void SetVerbose(bool v){verbose_=v;}
        /**
         * @brief
         *
         * @return const YAML::Node
         */
        const YAML::Node& GetConfig(){return this->doc;}
        /**
         * @brief
         *
         */
        void SetStateError(){GetBaseStatus()->set_state(M3COMP_STATE_ERR);}
        /**
         * @brief
         *
         */
        void SetStateOp(){if (!IsStateError()) GetBaseStatus()->set_state(M3COMP_STATE_OP);}
        /**
         * @brief
         *
         */
        void SetStateSafeOp(){if (!IsStateError()) GetBaseStatus()->set_state(M3COMP_STATE_SAFEOP);}
        /**
         * @brief
         *
         * @return bool
         */
        bool IsStateError(){return GetBaseStatus()->state() == M3COMP_STATE_ERR;}
        /**
         * @brief
         *
         * @return bool
         */
        bool IsStateSafeOp(){return GetBaseStatus()->state() == M3COMP_STATE_SAFEOP;}
        /**
         * @brief
         *
         * @return bool
         */
        bool IsStateOp(){return GetBaseStatus()->state() == M3COMP_STATE_OP;}
        /**
         * @brief
         *
         * @return bool
         */
        bool IsRateFast(){return GetBaseStatus()->rate()=="fast";}
        /**
         * @brief
         *
         * @return bool
         */
        bool IsRateMedium(){return GetBaseStatus()->rate()=="medium";}
        /**
         * @brief
         *
         * @return bool
         */
        bool IsRateSlow(){return GetBaseStatus()->rate()=="slow";}
        /**
         * @brief
         *
         * @param id
         * @return bool
         */
        bool IsVersion(int id){return version_id==id;}
        /**
         * @brief
         *
         * @return const std::string
         */
        const std::string GetConfigPath(){ return doc_path;}
        /**
         * @brief
         *
         * @param name
         * @param id
         */
        void RegisterVersion(const char * name, int id){version_names.push_back(name);version_ids.push_back(id);}
        /**
         * @brief
         *
         */
        virtual void Startup()=0;
        /**
         * @brief
         *
         */
        virtual void Shutdown()=0;
        /**
         * @brief
         *
         */
        virtual void StepStatus()=0;
        /**
         * @brief
         *
         */
        virtual void StepCommand()=0;
		
        /**
         * @brief
         *
         * @param ts
         */
        void SetTimestamp(int64_t ts){GetBaseStatus()->set_timestamp(ts);}
        /**
         * @brief
         *
         * @param ts
         */
        void GetTimestamp(int64_t ts){GetBaseStatus()->timestamp();}
        /**
         * @brief
         *
         * @param f
         */
        void SetFactory(M3ComponentFactory * f){factory=f;}
		
        /**
         * @brief
         *
         */
        virtual void PrettyPrint();
        /**
         * @brief
         *
         * @return google::protobuf::Message
         */
        virtual google::protobuf::Message *  GetCommand()=0;
        /**
         * @brief
         *
         * @return google::protobuf::Message
         */
        virtual google::protobuf::Message *  GetStatus()=0;
        /**
         * @brief
         *
         * @return google::protobuf::Message
         */
        virtual google::protobuf::Message *  GetParam()=0;
        /**
         * @brief
         *
         * @param s
         */
        void ParseCommandTest(std::string & s){}
        /**
         * @brief
         *
         * @param s
         * @return bool
         */
        virtual bool SerializeStatus(std::string & s);
	protected:
        /**
         * @brief
         *
         * @param name
         * @param id
         */
        void RegisterVersionID(const char * name, int id);
        /**
         * @brief
         *
         * @param s
         * @return bool
         */
        virtual bool ParseCommand(std::string & s);
        /**
         * @brief
         *
         * @param s
         * @return bool
         */
        virtual bool ParseParam(std::string & s);
        /**
         * @brief
         *
         * @return bool
         */
        virtual bool LinkDependentComponents(){return true;}
        /**
         * @brief
         *
         * @return M3BaseStatus
         */
        virtual M3BaseStatus *  GetBaseStatus()=0;
	protected:
        /**
         * @brief
         *
         * @param filename
         * @return bool
         */
        virtual bool ReadConfig(const char * filename);
        m3rt::M3ComponentFactory * factory; 
        int priority; 
                bool verbose_; 
        std::vector<std::string> version_names; 
        std::vector<int> version_ids; 
        int version_id; 
        YAML::Node doc; 
        std::string doc_path; 
};

//Factory defn.
/**
 * @brief
 *
 */
typedef M3Component * create_comp_t();
/**
 * @brief
 *
 */
typedef void destroy_comp_t(M3Component *);
extern std::map< std::string, create_comp_t *, std::less<std::string> > creator_factory;	//global 
extern std::map< std::string, destroy_comp_t *, std::less<std::string> > destroyer_factory; //global 

}

#endif

