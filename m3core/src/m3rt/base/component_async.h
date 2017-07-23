/* 
M3 -- Meka Robotics Robot Components
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

#ifndef M3_COMPONENT_ASYNC_H
#define M3_COMPONENT_ASYNC_H

#include "m3rt/base/component.h"
#include <google/protobuf/message.h>
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"

//#include "inttypes.h"

#ifdef __RTAI__	
#ifdef __cplusplus
extern "C" {
#endif 
#include <rtai.h>
#include <rtai_sem.h>
#ifdef __cplusplus
}  // extern "C"
#endif 
#endif


// Class to inherit from for async communications within M3


namespace m3rt
{
    static int num_asyncs = 0; 
	
/**
 * @brief
 *
 */
class M3ComponentAsync : public M3Component
{
	public:
		M3ComponentAsync(int p=EC_PRIORITY):M3Component(p),initializing(true),rc(-1),stop_thread(false)
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Updated safety thresholds to use motor model.
			
		}
		
        /**
         * @brief
         *
         */
        virtual void StepAsync()=0;
        /**
         * @brief
         *
         * @return bool
         */
        bool IsStopping(){return stop_thread;}
        /**
         * @brief
         *
         */
        void SignalStop(){stop_thread = true;}
        /**
         * @brief
         *
         * @return bool
         */
        bool IsInitializing(){return initializing;}
		
#ifdef __RTAI__			
        SEM * cmd_mutex; 
            SEM * status_mutex; 
#endif

        /**
         * @brief
         *
         * @return google::protobuf::Message
         */
        virtual google::protobuf::Message *  GetCommandAsync()=0;
        /**
         * @brief
         *
         * @return google::protobuf::Message
         */
        virtual google::protobuf::Message *  GetStatusAsync()=0;
        /**
         * @brief
         *
         * @return google::protobuf::Message
         */
        virtual google::protobuf::Message *  GetParamAsync()=0;
		
        /**
         * @brief
         *
         * @return google::protobuf::Message
         */
        virtual google::protobuf::Message *  GetCommandShared()=0;
        /**
         * @brief
         *
         * @return google::protobuf::Message
         */
        virtual google::protobuf::Message *  GetStatusShared()=0;
        /**
         * @brief
         *
         * @return google::protobuf::Message
         */
        virtual google::protobuf::Message *  GetParamShared()=0;
		
        /**
         * @brief
         *
         * @return google::protobuf::Message
         */
        virtual google::protobuf::Message * GetStatusThread()=0;
        bool initializing; 
		
	protected:
        enum {DEFAULT, ISS};		 
		
		
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
         * @return bool
         */
        virtual bool LinkDependentComponents();
	private:	      
          long rc; 
          bool stop_thread; 
          int tmp; 

};


}

#endif


