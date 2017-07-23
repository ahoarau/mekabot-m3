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

#ifndef RT_SERVICE_H
#define RT_SERVICE_H

#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/toolbox.h"
#include "m3rt/base/component_factory.h"
#include "m3rt/rt_system/rt_data_service.h"
#include "m3rt/rt_system/rt_log_service.h"
#include "m3rt/rt_system/rt_system.h"

#ifdef __RTAI__
#ifdef __cplusplus
extern "C" {
#endif 
#include <rtai.h>
#include <rtai_sem.h>
#ifdef __cplusplus
}  // extern "C"
#endif 
#else
#include <pthread.h>
#include <semaphore.h>
#endif

//No m3rt namespace for swig-ability
	
/**
 * @brief
 *
 */
class M3RtService{
public:
	M3RtService():rt_system(NULL),log_service(NULL),svc_task(NULL),next_port(10000),num_rtsys_attach(0){
            log_components.reserve(50);
            data_services.reserve(20);
        }
    /**
     * @brief
     *
     */
    ~M3RtService();
    /**
     * @brief
     *
     * @return bool
     */
    bool Startup();
    /**
     * @brief
     *
     */
    void Shutdown();
    /**
     * @brief
     *
     * @return int
     */
    int AttachRtSystem(); //Return number attached: 0=error
     /**
      * @brief
      *
      * @return int
      */
    int RemoveRtSystem();//Return number attached: 0=all removed
    /**
     * @brief
     *
     * @return bool
     */
    bool IsRtSystemOperational(){if (rt_system==NULL) return false; return rt_system->IsRtSystemActive();}
    /**
     * @brief
     *
     * @param name
     * @return bool
     */
    bool SetComponentStateSafeOp(char * name);
    /**
     * @brief
     *
     * @param name
     * @return bool
     */
    bool SetComponentStateOp(char * name);
    /**
     * @brief
     *
     */
    void SetComponentStateOpAll(){if (rt_system==NULL) return; rt_system->SetComponentStateOpAll();}
    /**
     * @brief
     *
     */
    void SetComponentStateSafeOpAll(){if (rt_system==NULL) return; rt_system->SetComponentStateOpAll();}
    /**
     * @brief
     *
     * @return int
     */
    int AttachDataService();
    /**
     * @brief
     *
     * @return bool
     */
    bool AttachRosService();
    /**
     * @brief
     *
     * @return bool
     */
    bool RemoveRosService();
    /**
     * @brief
     *
     * @param port
     * @return bool
     */
    bool RemoveDataService(int port);
    /**
     * @brief
     *
     * @param name
     * @param path
     * @param freq
     * @param page_size
     * @param verbose
     * @return bool
     */
    bool AttachLogService(char * name, char * path, double freq,int page_size,int verbose);
	//bool AddRosComponent(const char * name);
    /**
     * @brief
     *
     * @param name
     * @return bool
     */
    bool AddLogComponent(char * name){log_components.push_back(name);}
    /**
     * @brief
     *
     * @return bool
     */
    bool RemoveLogService();
    /**
     * @brief
     *
     * @return bool
     */
    bool IsDataServiceRunning();
    /**
     * @brief
     *
     * @return bool
     */
    bool IsLogServiceRunning(){return log_service!=NULL;}
    /**
     * @brief
     *
     * @return bool
     */
    bool IsRosServiceRunning(){return false;}
    /**
     * @brief
     *
     * @return bool
     */
    bool IsRtSystemRunning(){return rt_system !=NULL;}
    /**
     * @brief
     *
     * @return bool
     */
    bool IsServiceThreadActive();
    /**
     * @brief
     *
     * @return int
     */
    int GetNumComponents();
    /**
     * @brief
     *
     * @param idx
     * @return const char
     */
    const char *  GetComponentName(int idx);
    /**
     * @brief
     *
     * @param idx
     * @return const char
     */
    const char *  GetComponentType(int idx);
    /**
     * @brief
     *
     * @param name
     * @return int
     */
    int GetComponentState(const char * name);
    /**
     * @brief
     *
     * @param name
     * @return int
     */
    int GetComponentIdx(const char * name);
    /**
     * @brief
     *
     * @param name
     * @return bool
     */
    bool PrettyPrintComponent(const char * name);
    /**
     * @brief
     *
     * @return bool
     */
    bool PrettyPrintRtSystem();
    /**
     * @brief
     *
     * @param name
     * @param port
     * @return bool
     */
    bool ClientSubscribeStatus(const char * name, int port);
    /**
     * @brief
     *
     * @return bool
     */
    bool IsDataServiceError();
private:
    int hlt; /**< The thread created at Startup()*/
    m3rt::M3RtSystem  * rt_system; 
    m3rt::M3ComponentFactory factory; //Can only create one instance of this. 
    std::vector<m3rt::M3RtDataService*> data_services; 
    m3rt::M3RtLogService *log_service; 
    std::vector<std::string> log_components; 
#ifdef __RTAI__
    RT_TASK *svc_task; 
#else
	int * svc_task; // to preserve initializer
#endif
    std::vector<int> ports; 
    int next_port; 
    int num_rtsys_attach; 
};


#endif
