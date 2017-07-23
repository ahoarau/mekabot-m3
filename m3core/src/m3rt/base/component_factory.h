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

#ifndef  M3RT_COMPONENT_FACTORY_H
#define  M3RT_COMPONENT_FACTORY_H

#include "m3rt/base/component.h"
#include <string>
#include <vector>
#include <iterator>

namespace m3rt
{

//Because of Protocol Buffers implementation
//Only one instance of libprotobuf can be loaded
//So make this once on system startup, close only on exit
/**
 * @brief
 *
 */
class M3ComponentFactory{
public:
    /**
     * @brief
     *
     */
    M3ComponentFactory(){}
    /**
     * @brief
     *
     */
    ~M3ComponentFactory(){}
    /**
     * @brief
     *
     * @return bool
     */
    bool Startup();												//Load libraries
    /**
     * @brief
     *
     */
    void Shutdown();											//Free libraries, release components
    /**
     * @brief
     *
     * @param type
     * @return M3Component
     */
    M3Component * CreateComponent(std::string type);			//Instantiate a component of this type
    /**
     * @brief
     *
     * @param c
     * @return bool
     */
    bool ReleaseComponent(M3Component * c);						//Safe delete of a component
    /**
     * @brief
     *
     */
    void ReleaseAllComponents();								//Safe delete of all components
    /**
     * @brief
     *
     * @param idx
     * @return M3Component
     */
    M3Component *  	GetComponent(int idx);
    /**
     * @brief
     *
     * @param idx
     * @return std::string
     */
    std::string  	GetComponentType(int idx);
    /**
     * @brief
     *
     * @param name
     * @return int
     */
    int 			GetComponentIdx(std::string name); 			//Returns -1 if not found
    /**
     * @brief
     *
     * @param name
     * @return M3Component
     */
    M3Component * 	GetComponent(std::string name);
    /**
     * @brief
     *
     * @param idx
     * @return std::string
     */
    std::string  	GetComponentName(int idx);
    /**
     * @brief
     *
     * @return int
     */
    int 			GetNumComponents();
    /**
     * @brief
     *
     * @return M3MonitorStatus
     */
    M3MonitorStatus * GetMonitorStatus(){return &monitor_status;}
private:
    /**
     * @brief
     *
     * @param filename
     * @return bool
     */
    bool ReadConfig(const char * filename);
    /**
     * @brief
     *
     * @param lib
     * @return bool
     */
    bool AddComponentLibrary(std::string lib);
    std::vector<M3Component *>	m3_list; 
    std::vector<std::string>	m3_types; 
    std::vector<void *> 		dl_list; 						//handles for dynamic libs 
    std::vector<std::string> 	dl_list_str; 
    std::vector<std::string> 	dl_types; 
    M3MonitorStatus  monitor_status; 					//Container for all component rt stats 
};

}

#endif

