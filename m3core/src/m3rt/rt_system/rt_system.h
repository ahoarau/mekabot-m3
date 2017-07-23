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

#ifndef RT_SYSTEM_H
#define RT_SYSTEM_H

#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/m3ec_def.h"
#include "m3rt/base/toolbox.h"
#include "m3rt/base/component.h"
#include "m3rt/base/component_ec.h"
#include "m3rt/base/component_factory.h"
#include "m3rt/base/component_base.pb.h" 
#include "m3rt/rt_system/rt_log_service.h"
//#include "m3rt/rt_system/rt_ros_service.h"
#include <string>
#include <vector>

#ifdef __RTAI__
#ifdef __cplusplus
extern "C" {
#endif 
#include <rtai.h>
#include "rtai_sem.h"
#ifdef __cplusplus
}  // extern "C"
#endif 
#endif
#include <semaphore.h>
#include <pthread.h>
#include <sys/time.h>
#include <algorithm>

#ifdef __cplusplus11__
#include <atomic>
#endif

namespace m3rt
{
/**
 * @brief
 *
 */
class M3RtSystem
{
public:
    /**
     * @brief The class that contains the realtime loop
     *
     * @param f
     */
    M3RtSystem(M3ComponentFactory * f):log_service(NULL),
        shm_ec(0),shm_sem(0),ext_sem(NULL),sync_sem(0),factory(f),logging(false),hard_realtime(true),ready_sem(NULL),
        safeop_required(false){GOOGLE_PROTOBUF_VERIFY_VERSION;}
    friend class M3RtDataService;
    /**
     * @brief
     *
     */
    ~M3RtSystem();
    /**
     * @brief
     *
     * @return bool
     */
    bool Startup();
    /**
     * @brief
     *
     * @return bool
     */
    bool StartupComponents();
    /**
     * @brief
     *
     * @return bool
     */
    bool Shutdown();
    /**
     * @brief
     *
     * @param safeop_only
     * @param dry_run
     * @return bool
     */
    bool Step(bool safeop_only,bool dry_run=false);
    /**
     * @brief
     *
     */
    void PrettyPrint();
    /**
     * @brief
     *
     */
    void PrettyPrintComponents();
    /**
     * @brief
     *
     * @param idx
     */
    void PrettyPrintComponent(int idx);
    /**
     * @brief
     *
     */
    void PrettyPrintComponentNames();
    /**
     * @brief
     *
     * @param name
     * @return M3Component
     */
    M3Component * 	GetComponent(std::string name){return factory->GetComponent(name);}
    /**
     * @brief
     *
     * @param idx
     * @return M3Component
     */
    M3Component *  	GetComponent(int idx){return factory->GetComponent(idx);}
    /**
     * @brief
     *
     * @param idx
     * @return std::string
     */
    std::string  	GetComponentName(int idx){return factory->GetComponentName(idx);}
    /**
     * @brief
     *
     * @param idx
     * @return std::string
     */
    std::string  	GetComponentType(int idx){return factory->GetComponentType(idx);}
    /**
     * @brief
     *
     * @return int
     */
    int 		GetNumComponents(){return factory->GetNumComponents();}
    /**
     * @brief
     *
     * @param name
     * @return int
     */
    int 		GetComponentIdx(std::string name){return factory->GetComponentIdx(name);}
    /**
     * @brief
     *
     * @param idx
     * @return int
     */
    int			GetComponentState(int idx);
    /**
     * @brief
     *
     * @param idx
     * @return bool
     */
    bool SetComponentStateOp(int idx);
    /**
     * @brief
     *
     * @param idx
     * @return bool
     */
    bool SetComponentStateSafeOp(int idx);
    /**
     * @brief
     *
     */
    void SetComponentStateSafeOpAll(void);
    /**
     * @brief
     *
     */
    void SetComponentStateOpAll(void);
    /**
     * @brief
     *
     * @return bool
     */
    bool IsOperational(){return !safeop_required;}
    /**
     * @brief
     *
     * @return bool
     */
    bool IsHardRealTime(){return hard_realtime;}
    /**
     * @brief
     *
     * @return M3ComponentFactory
     */
    bool IsRtSystemActive(){return sys_thread_active;}
    M3ComponentFactory * GetFactory()const{return factory;}
    /**
     * @brief
     *
     * @param timeout_ns
     * @return bool
     */
    bool WaitForEcComponents(mReal timeout_ns=3e9);
#ifdef __RTAI__
    /**
     * @brief
     *
     * @return int
     */
    int GetEcCounter(){return shm_ec->counter;}
    SEM * ready_sem; 
    SEM * sync_sem; 
    SEM * shm_sem; 
    SEM * ext_sem; 
#else
    sem_t * shm_sem;
    sem_t * sync_sem;
    sem_t * ext_sem;
    sem_t * ready_sem;
    int GetEcCounter(){return 0;}
#endif
    /**
     * @brief
     *
     * @param f
     */
    void SetFactory(M3ComponentFactory * f){factory=f;}
    /**
     * @brief
     *
     * @param l
     */
    void AttachLogService(M3RtLogService * l){log_service=l;}

    /**
     * @brief
     *
     */
    void RemoveLogService(){log_service=NULL;M3_DEBUG("Log service stopped at %d\n",log_service);}
    /**
     * @brief
     *
     * @param msg
     * @return bool
     */
    bool ParseCommandFromExt(M3CommandAll & msg);  //Must be thread safe
    /**
     * @brief
     *
     * @param msg
     * @param names
     * @return bool
     */
    bool SerializeStatusToExt(M3StatusAll & msg, std::vector<std::string>& names); //Must be thread safe
    int over_step_cnt;
#ifdef __cplusplus11__
    std::atomic<bool> logging; 
    std::atomic<bool> sys_thread_end;
    std::atomic<bool> sys_thread_active;
#else
    bool logging; 
    bool sys_thread_end;
    bool sys_thread_active;
#endif
private:
    /**
     * @brief
     *
     */
    void CheckComponentStates();
    M3ComponentFactory * factory; 
    M3EcSystemShm *  shm_ec; 
#ifdef __cplusplus11__
    std::atomic<bool> safeop_required;
#else
    bool safeop_required;
#endif
    bool hard_realtime; 
    std::vector<M3ComponentEc *>	m3ec_list; 
    std::vector<M3Component *>	m3rt_list; 
#ifdef __RTAI__
    RTIME last_cycle_time; 
#else
    long long last_cycle_time;
#endif
    M3RtLogService * log_service; 

    std::vector<int> idx_map_ec; 
    std::vector<int> idx_map_rt; 
    long hst; 
    double test; 
	
protected:
    template <class T>
    /**
     * @brief
     *
     * @param name
     * @param comp_list
     * @return bool
     */
    bool IsComponentInList(std::string& name,std::vector<T*>& comp_list){
	for(int i=0;i<comp_list.size();++i){
	      if( comp_list[i]->GetName() == name)
		return true;
	}
	return false;
    }
#ifdef __RTAI__
    /**
     * @brief
     *
     * @return SEM
     */
    SEM * GetExtSem(){return ext_sem;}
#else
    sem_t * GetExtSem(){return ext_sem;}
#endif
	// Here we read the config files in robot_config1:robot_config_add:robot_config_overlap
	template <class T>
    /**
     * @brief
     *
     * @param filename
     * @param component_type
     * @param comp_list
     * @param idx_map
     * @return bool
     */
    bool ReadConfig(const char* filename, const char* component_type, std::vector<T*>& comp_list, std::vector< int >& idx_map)
	{
		std::vector<std::string> vpath;
		GetFileConfigPath(filename,vpath);
		bool ret=false;
		// let's read first the last ones, and go back to the first one (so we can check if already exists)
		for(std::vector<std::string>::reverse_iterator it=vpath.rbegin();it!=vpath.rend();++it){
		      // Old notations (without the "-" is doesnt not guarranty order
			std::cout<<std::endl;
			M3_INFO("Reading %s for %s\n\n",(*it).c_str(),component_type);
			if( ret=this->ReadConfigUnordered(*it,component_type,comp_list,idx_map) && comp_list.size()>0){
			  M3_WARN("Old config file detected, please update your %s\n",(*it).c_str());
			  continue;
			}
#ifndef YAMLCPP_03

			try{
				ret = this->ReadConfigOrdered(*it,component_type,comp_list,idx_map);
			}catch(std::exception &e){
				M3_ERR("Error while reading %s checking for %s config: %s\n",(*it).c_str(),component_type,e.what());
			}
#endif
		}
		return ret;
	}
	template <class T>
    /**
     * @brief
     *
     * @param filename
     * @param component_type
     * @param comp_list
     * @param idx_map
     * @return bool
     */
    bool ReadConfigUnordered(const std::string& filename,const char * component_type,std::vector<T>& comp_list,std::vector<int>& idx_map)
	{
		try{
		YAML::Node doc;
#ifdef YAMLCPP_03
		std::ifstream fin(filename.c_str());
		YAML::Parser parser(fin);
		while(parser.GetNextDocument(doc)) {
#else
		doc = YAML::LoadFile(filename);
		if(doc.IsNull()){M3_ERR("%s not found, please update the robot's config files.\n",filename.c_str()); return false;}
#endif

#ifdef YAMLCPP_03
			if(!doc.FindValue(component_type)) {
#else
			if(!doc[component_type]){
#endif
				M3_INFO("No %s key in %s. Proceeding without it...\n",component_type,filename.c_str());
				return true;
			}
			
#ifdef YAMLCPP_03
			const YAML::Node& components = doc[component_type];
			for(YAML::Iterator it = components.begin(); it != components.end(); ++it) {
				std::string dir;
				it.first() >> dir;
#else
			YAML::Node components = doc[component_type];
			for(YAML::const_iterator it_rt = components.begin();it_rt != components.end(); ++it_rt) {
				std::string dir = it_rt->first.as<std::string>();
#endif
				
#ifdef YAMLCPP_03
				for(YAML::Iterator it_dir = components[dir.c_str()].begin();
					it_dir != components[dir.c_str()].end(); ++it_dir) {
					std::string  name;
					std::string  type;
					it_dir.first() >> name;
					it_dir.second() >> type;
#else
				YAML::Node dir_comp = components[dir.c_str()];
				for(YAML::const_iterator it_dir = dir_comp.begin();it_dir != dir_comp.end(); ++it_dir) {
					std::string name=it_dir->first.as<std::string>();
					std::string type=it_dir->second.as<std::string>();
#endif
					if(IsComponentInList(name,comp_list))
					{
					  M3_WARN("Component %s (of type %s) already loaded, please make sure your component's name is unique.\n",name.c_str(),type.c_str());
					  continue;
					}
					T m = reinterpret_cast<T>(factory->CreateComponent(type));
					if(m != NULL) {
						m->SetFactory(factory);
						std::string f = dir + "/" + name + ".yml";
						try {
							std::cout <<"------------------------------------------"<<std::endl;
							std::cout <<"Component " << name<<" of type "<<type<<std::endl;
							if(m->ReadConfig(f.c_str())) { //A.H: this should look first in local and to back to original if it exists
								comp_list.push_back(m);
								idx_map.push_back(GetNumComponents() - 1);
							} else {
								factory->ReleaseComponent(m);
								M3_ERR("Error reading config for %s\n", name.c_str());
							}
						} catch(...) {
							M3_WARN("Error while parsing config files for %s %s \n",component_type, name.c_str());
							factory->ReleaseComponent(m);
						}

					}
				}
			}
		return true;
#ifdef YAMLCPP_03
		}
#endif
		}catch(std::exception &e){
			//M3_ERR("(Unordered) Error while reading %s config (old config): %s\n",component_type,e.what());
			return false;
		}
		std::cout<<std::endl;
	}
#ifndef YAMLCPP_03
	template <class T>
    /**
     * @brief
     *
     * @param filename
     * @param component_type
     * @param comp_list
     * @param idx_map
     * @return bool
     */
    bool ReadConfigOrdered(const std::string& filename,const char * component_type,std::vector<T>& comp_list,std::vector<int>& idx_map)
	{
		// New version with -ma17: -actuator1:type1 etc
		YAML::Node doc = YAML::LoadFile(filename);
		if(doc.IsNull()){M3_ERR("%s not found, please update the robot's config files.\n",filename.c_str()); return false;}
		//for(std::vector<YAML::Node>::const_iterator it_doc=all_docs.begin(); it_doc!=all_docs.end();++it_doc){
			//doc = *it_doc;
			if(!doc[component_type]){
				M3_INFO("No %s keys in %s. Proceeding without it...\n",component_type,filename.c_str());
				return true;
			}
			const YAML::Node& components = doc[component_type];
			for(YAML::const_iterator it_rt = components.begin();it_rt != components.end(); ++it_rt) {
				const std::string dir =it_rt->begin()->first.as<std::string>();
				const YAML::Node& dir_comp = it_rt->begin()->second;
				for(YAML::const_iterator it_dir = dir_comp.begin();it_dir != dir_comp.end(); ++it_dir) {
					std::string name=it_dir->begin()->first.as<std::string>();
					std::string type=it_dir->begin()->second.as<std::string>();
					if(IsComponentInList(name,comp_list))
					{
					  M3_WARN("Component %s (of type %s) already loaded, please make sure your component's name is unique.\n",name.c_str(),type.c_str());
					  continue;
					}
					T m = reinterpret_cast<T>(factory->CreateComponent(type));
					if(m != NULL) {
						m->SetFactory(factory);
						std::string f = dir + "/" + name + ".yml";
						try {
							std::cout <<"------------------------------------------"<<std::endl;
							std::cout <<"Component " << name<<" of type "<<type<<std::endl;
							if(m->ReadConfig(f.c_str())) { //A.H: this should look first in local and to back to original if it exists
								comp_list.push_back(m);
								idx_map.push_back(GetNumComponents() - 1);
							} else {
								factory->ReleaseComponent(m);
								M3_ERR("Error reading config for %s\n", name.c_str());
							}
						} catch(...) {
							M3_WARN("Error while parsing config files for %s %s \n",component_type, name.c_str());
							factory->ReleaseComponent(m);
						}

					}
				}
				//std::cout <<"------------------------------------------"<<std::endl;
			}
		std::cout<<std::endl;
		return true;
	}
#endif
};


}
#endif


