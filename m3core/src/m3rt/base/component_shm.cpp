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



#include "m3rt/base/component_shm.h"

namespace m3rt
{

    void M3CompShm::request_status()
    {
        rt_sem_wait(status_sem);
    }

    void M3CompShm::release_status()
    {
        rt_sem_signal(status_sem);
    }

    void M3CompShm::request_command()
    {
        rt_sem_wait(command_sem);
    }

    void M3CompShm::release_command()
    {
        rt_sem_signal(command_sem);
    }

    bool M3CompShm::ReadConfig(const char * filename)
    {
        //YAML::Node doc;

        if (!M3Component::ReadConfig(filename)) return false;
        //GetYamlDoc(filename, doc);
        try{
            doc["shm_id"] >> shm_id;
        }catch(...){
            shm_id = "M3WAR";
            M3_WARN("shm_id key not found, please add it to %s\n",filename);
        }
        return true;
    }


    void  M3CompShm::StepStatus()
    {
        if (!shm)
        {
            SetStateSafeOp();
            return;
        }
        //if (!IsStateError()) //
        SetSdsFromStatus(shm->status);
    }

    void  M3CompShm::StepCommand()
    {

        if (!shm) return;
        if (!IsStateOp())
        ResetCommandSds(shm->cmd);
        else
        SetCommandFromSds(shm->cmd);

    }

    void  M3CompShm::Startup()
    {
        SetStateSafeOp();
        #ifdef __RTAI__
        command_sem = rt_typed_sem_init(nam2num((shm_id+"C").c_str()), 1, BIN_SEM | FIFO_Q );
        status_sem = rt_typed_sem_init(nam2num((shm_id+"S").c_str()), 1, BIN_SEM | FIFO_Q );
        shm = (M3Sds*)rt_shm_alloc(nam2num((shm_id+"M").c_str()),sizeof(M3Sds),USE_VMALLOC);
        #else
        command_sem = sem_open ((shm_id+"C").c_str(), O_CREAT, 0660, 0);
        status_sem = sem_open ((shm_id+"S").c_str(), O_CREAT, 0660, 0);

        if(command_sem == SEM_FAILED)
        {
            M3_ERR("Could not create command semaphore\n");
        }

        if(status_sem == SEM_FAILED)
        {
            M3_ERR("Could not create command semaphore\n");
        }

        int fd = shm_open((shm_id+"M").c_str(), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
        if(fd == -1)
        {
            M3_ERR("Could not create posix shared memory\n");
            return;
        }
        if(ftruncate(fd, sizeof(M3Sds)) == -1 )
        {
            M3_ERR("Could not ftruncate the shared memory\n");
        }
        shm = (M3Sds*)mmap(NULL, sizeof(M3Sds),PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if(shm == MAP_FAILED)
        {
            M3_ERR("mmap failed\n");
        }
        #endif
        memset(shm,0,sizeof(M3Sds));
    }

    void  M3CompShm::Shutdown()
    {
        #ifdef __RTAI__
        rt_shm_free(nam2num((shm_id+"M").c_str()));
        #endif
        rt_sem_delete(command_sem);
        rt_sem_delete(status_sem);
    }

}
