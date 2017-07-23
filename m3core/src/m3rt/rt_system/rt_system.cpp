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

#include "m3rt/rt_system/rt_system.h"
//#include "m3rt/base/m3ec_pdo_v1_def.h"
#include <unistd.h>
#include <string>

#if defined(__RTAI__) && defined(__cplusplus)
extern "C" {
#include <rtai.h>
#include <rtai_lxrt.h>
#include <rtai_sem.h>
#include <rtai_sched.h>
#include <rtai_nam2num.h>
#include <rtai_shm.h>
#include <rtai_malloc.h>	 
}
#endif

#include <ctime>

namespace m3rt
{
using namespace std;
static int step_cnt = 0;
#ifdef __RTAI__
static RT_TASK * main_task;
#endif
unsigned long long getNanoSec(void)
{
    struct timeval tp;
    struct timezone tzp;

    tzp.tz_minuteswest = 0;

    (void) gettimeofday(&tp, &tzp); //
    return 1000000000LL * (long long) tp.tv_sec +
            1000LL * (long long) tp.tv_usec;
}


void *rt_system_thread(void *arg)
{
    M3RtSystem *m3sys = (M3RtSystem *)arg;
    m3sys->sys_thread_end = false; //gonna be at true is startup fails=> no wait
    bool safeop_only = false;
    int tmp_cnt = 0;
    bool ready_sent=false;
    int sem_cnt=0;
    M3_INFO("Starting M3RtSystem real-time thread.\n");
#ifdef __RTAI__
    rt_allow_nonroot_hrt();
    RTIME print_dt=1e9;
    RT_TASK *task=NULL;
    RTIME start, end, dt,tick_period,dt_wait;
#ifdef ONESHOT_MODE
    M3_INFO("Oneshot mode activated.\n");
    rt_set_oneshot_mode();
#endif
    if ( !(rt_is_hard_timer_running() ))
    {
        M3_INFO("Starting the real-time timer.\n");
        tick_period = start_rt_timer(nano2count(RT_TIMER_TICKS_NS) );
    }else{
        M3_INFO("Real-time timer running.\n");
        tick_period = nano2count(RT_TIMER_TICKS_NS);
    }
    M3_INFO("Beginning RTAI Initialization.\n");
    if(!( task = rt_task_init_schmod(nam2num("M3SYS"), 0, 0, 0, SCHED_FIFO, 0xF))) {
        m3rt::M3_ERR("Failed to create RT-TASK M3SYS\n", 0);
        m3sys->sys_thread_active = false;
        return 0;
    }
    M3_INFO("RT Task Scheduled.\n");
    M3_INFO("Nonroot hrt initialized.\n");
    rt_task_use_fpu(task, 1);
    M3_INFO("Use fpu initialized.\n");
    mlockall(MCL_CURRENT | MCL_FUTURE);
    M3_INFO("Mem lock all initialized.\n");
    RTIME tick_period_orig = tick_period;

#endif
#ifdef __RTAI__
    RTIME print_start=rt_get_time_ns();
    RTIME diff=0;
    int dt_us,tick_period_us,overrun_us;
#endif
#if defined(__RTAI__)
#ifndef ONESHOT_MODE
    RTIME now = rt_get_time();
#ifndef __NO_KERNEL_SYNC__
    rt_sleep(nano2count((long long)1e9));
#endif
    if(rt_task_make_periodic(task, rt_get_time() + tick_period, tick_period)) {
        M3_ERR("Couldn't make rt_system task periodic.\n");
        return 0;
    }
    M3_INFO("Periodic task initialized.\n");
#endif
#endif

#ifndef __RTAI__
    usleep(1e6);
    M3_INFO("Using pthreads\n");
#endif

#if defined(__RTAI__)
    if(!m3sys->IsHardRealTime()){
        M3_INFO("Soft real time initialized.\n");
        rt_make_soft_real_time();
    }else{
        M3_INFO("Hard real time initialized.\n");
        rt_make_hard_real_time();
    }
#ifndef __NO_KERNEL_SYNC__
    M3_INFO("Dry running components...\n");
    for(int i = 0; i < m3sys->GetNumComponents(); i++){
        m3sys->GetComponent(i)->SetVerbose(false);

    }
    bool dry_run_ok=false;
    now=rt_get_time_ns();
    print_start = rt_get_time_ns();
    // RTIME print_dt = 1e9;
    int nerr = 0;
    int ntrialsmin=500;
    while(1){
        if(m3sys->sys_thread_end)
            return 0;
        nerr = 0;
        // Let's try to step
        dry_run_ok = m3sys->Step(false,true);
        // We count the num of error raised
        for(int i = 0; i < m3sys->GetNumComponents(); i++)
            if(m3sys->GetComponent(i)->IsStateError()) nerr++;
        // Print the status
        if (dry_run_ok==false && ((rt_get_time_ns() -print_start) > print_dt))
        {
            print_start = rt_get_time_ns();
            M3_INFO("Components ready %d/%d\n",m3sys->GetNumComponents()-nerr,m3sys->GetNumComponents());
        }
        // If step wasn't ok, we give it another chance
        if(!dry_run_ok){
            m3sys->SetComponentStateOpAll();
        }else if(ntrialsmin<=0){
            M3_INFO("All %d components successfully started.\n",m3sys->GetNumComponents());
            break;
        }
        // Timeout
        if ((rt_get_time_ns()- now) >=  (RTIME)4e9)
        {
            dry_run_ok=false;
            break;
        }
        ntrialsmin--;
        rt_task_wait_period();
    }
    if(!dry_run_ok){
        M3_INFO("Dry run failed, server should stop now. Please restart it.\n");
        return 0;
    }
    for(int i = 0; i < m3sys->GetNumComponents(); i++){
        m3sys->GetComponent(i)->SetVerbose(true);

    }
#endif
    M3_INFO("Entering realtime loop.\n");
#endif
#ifdef __NO_KERNEL_SYNC__
    M3_INFO("Kernel sync is disabled (virtual installation only)\n");
#endif
#ifndef __RTAI__
    long long start, end, dt;
#endif

    m3sys->over_step_cnt = 0;
    m3sys->sys_thread_end = false;
    m3sys->sys_thread_active = true;

    while(1) {
        if(m3sys->sys_thread_end) break;
#ifdef __RTAI__
        start = rt_get_cpu_time_ns();
#else
        start = getNanoSec();
#endif
        if(!m3sys->Step(safeop_only))  //This waits on m3ec.ko semaphore for timing
            break;
#ifdef __RTAI__
        end = rt_get_cpu_time_ns();
        dt = end - start;
        /*
        Check the time it takes to run components, and if it takes longer
        than our period, make us run slower. Otherwise this task locks
        up the CPU.*/
        if(dt > count2nano(tick_period) && step_cnt > 10) {
            m3sys->over_step_cnt++;
            dt_us = static_cast<int>(((dt) / 1000));
            tick_period_us = static_cast<int>(((count2nano(tick_period)) / 1000));
            overrun_us = dt_us - tick_period_us;
            rt_printk("Previous period: %d us overrun (dt: %d us, des_period: %d us)\n", overrun_us, dt_us, tick_period_us);
            if(m3sys->over_step_cnt > 5000) {
                M3_INFO("Step %d: Computation time of components is too long (dt:%d). Forcing all components to state SafeOp - switching to SAFE REALTIME.\n", step_cnt,(int)(dt/1000.0));
                M3_INFO("Previous period: %d. New period: %d\n", (int)(count2nano(tick_period)/1000), (int)(dt/1000));
                tick_period = nano2count(dt);
                rt_make_soft_real_time();
                rt_set_period(task,tick_period);
                safeop_only = true;
                m3sys->over_step_cnt = 0;
            }
        } else {
            if(m3sys->over_step_cnt > 0){
                m3sys->over_step_cnt--;
            }
        }
#ifndef ONESHOT_MODE
        rt_task_wait_period(); //No longer need as using sync semaphore of m3ec.ko // A.H : oneshot mode is too demanding => periodic mode is necessary !

#else
        diff = count2nano(tick_period)-(rt_get_cpu_time_ns()-start);
        rt_sleep(min((RTIME)0,nano2count(diff)));
#endif
        if (rt_get_time_ns() -print_start > print_dt)
        {
            rt_printk("M3System freq : %d (dt: %d us / des period: %d us)\n",tmp_cnt,(int)(dt/1000.0),(int)(count2nano(tick_period)/1000));
            if(!(rt_is_hard_real_time(task)))
                rt_printk("WARNING: M3System is running in SOFT real-time mode !\n");
            tmp_cnt = 0;
            print_start = rt_get_time_ns();

        }
#else
        if(!ready_sent){
            sem_post(m3sys->ready_sem);
            ready_sent=true;
        }
        end = getNanoSec();
        dt = end - start;
        if(tmp_cnt++==1000)
        {
            tmp_cnt=0;
            std::cout<<"Loop computation time : "<<dt/1000<<" us (sleeping "<<( RT_TIMER_TICKS_NS - ((unsigned int)dt)) / 1000000 <<" us)"<<endl;
        }
        usleep((RT_TIMER_TICKS_NS - ((unsigned int)dt)) / 1000);
#endif
        tmp_cnt++;
    }
#ifdef __RTAI__
    rt_make_soft_real_time();
    rt_task_delete(task);
#endif
    m3sys->sys_thread_active = false;
    return 0;
}

M3RtSystem::~M3RtSystem() {}

////////////////////////////////////////////////////////////////////////////////////////////

bool M3RtSystem::Startup()
{
    sys_thread_active = false;
    BannerPrint(60, "Startup of M3RtSystem");
    if(!this->StartupComponents()) {
        sys_thread_active = false;
        return false;
    }
    usleep(500000);
    long ret=0;//return for the thread
#ifdef __RTAI__
    hst = rt_thread_create((void *)rt_system_thread, (void *)this, 1000000);
    ret = (hst!=0 ? 0:-1);
#else
    ret = pthread_create((pthread_t *)&hst, NULL, (void * ( *)(void *))rt_system_thread, (void *)this);
#endif

    if(!(ret==0)){
        m3rt::M3_INFO("Startup of M3RtSystem thread failed (error code [%ld]).\n",ret);
        return false;
    }
    for(int i = 0; i < 10; i++) {
        if(sys_thread_active)
            break;
        usleep(1e6); //Wait until enters hard real-time and components loaded. Can take some time if alot of components.max wait = 1sec
        
    }
    if(!sys_thread_active) {
        m3rt::M3_INFO("Startup of M3RtSystem thread failed, thread still not active.\n");
        return false;
    }
    return true;
}

bool M3RtSystem::Shutdown()
{
    M3_INFO("Begin shutdown of M3RtSystem...\n");
    //Stop RtSystem thread
    sys_thread_end = true;

    usleep(500000);
    
    float timeout_s = 4;
    time_t start_time=time(0);
    while(sys_thread_active && (float)difftime(time(0),start_time) < timeout_s)
    {
        m3rt::M3_INFO("Waiting for RtSystem thread to shutdown... (%.2fs/%.2fs)\n",(float)difftime(time(0),start_time) ,timeout_s);
        usleep(500000);
    }

    if(sys_thread_active) {
        m3rt::M3_WARN("M3RtSystem thread did not shutdown correctly\n");
        //return false;
    }
#ifdef __RTAI__
    if(shm_ec != NULL)
#endif
    {
        //Send out final shutdown command to EC slaves
        int n_comp = GetNumComponents();
        for(int i = n_comp; i > 0; --i){
            //long int shutdown_thread;
            //int ret = pthread_create((pthread_t *)&shutdown_thread, NULL, (void * ( *)(void *))shutdown, (void *)GetComponent(i-1));
            M3_INFO("%s is shutting down...",GetComponentName(i-1).c_str());
            GetComponent(i-1)->Shutdown();
            printf("OK (%d/%d)\n",n_comp-i+1,n_comp);
        }
        //usleep(2e6);
#ifdef __RTAI__
        rt_shm_free(nam2num(SHMNAM_M3MKMD));
#endif
    }
    if(ext_sem != NULL) {
#ifdef __RTAI__
        rt_sem_delete(ext_sem);
#else
        delete ext_sem;
#endif
        ext_sem = NULL;
    }
    if(ready_sem != NULL) {
#ifdef __RTAI__
        rt_sem_delete(ready_sem);
#else
        delete ready_sem;
#endif
        ready_sem = NULL;
    }
#ifdef __RTAI__
    rt_task_delete(main_task);
    main_task=NULL;
#endif
    shm_ec = NULL;
    shm_sem = NULL;
    sync_sem = NULL;
    factory->ReleaseAllComponents();
    M3_INFO("Shutdown of M3RtSystem complete\n");
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////

bool M3RtSystem::StartupComponents()
{
    M3_INFO("Reading components config files ...\n");
    if(!ReadConfig(M3_CONFIG_FILENAME,"ec_components",this->m3ec_list,this->idx_map_ec))
        return false;
    if(!ReadConfig(M3_CONFIG_FILENAME,"rt_components",this->m3rt_list,this->idx_map_rt))
        return false;
    M3_INFO("Done reading components config files.\n");
#ifdef __RTAI__
    main_task = rt_task_init_schmod(nam2num("M3MAIN"),RT_TASK_PRIORITY,RT_STACK_SIZE,0,SCHED_FIFO,0xF);
    if(!main_task){
        M3_ERR("Unable to start M3RtSystem main RTAI task, abording.\n");
        return false;
    }
#endif
#ifdef __RTAI__
    sync_sem = (SEM *)rt_get_adr(nam2num(SEMNAM_M3SYNC));
    if(!sync_sem) {
        M3_ERR("Unable to find the SYNCSEM semaphore.\n", 0);
        return false;
    }
    M3_INFO("Getting Kernel EC components.\n");
#ifndef __NO_KERNEL_SYNC__
    if(!rt_sem_wait_timed(sync_sem,nano2count(1e9)))
        M3_WARN("Timeout for sync signal with kernel, all frames might not be processed.\n");
#endif
    shm_ec = (M3EcSystemShm *) rtai_malloc(nam2num(SHMNAM_M3MKMD), 1);
    if(shm_ec)
        M3_INFO("Found %d active M3 EtherCAT slaves\n", shm_ec->slaves_active);
    else {
        M3_ERR("Rtai_malloc failure for SHMNAM_M3KMOD\n", 0);
        return false;
    }
    shm_sem = (SEM *)rt_get_adr(nam2num(SEMNAM_M3LSHM));
    if(!shm_sem) {
        M3_ERR("Unable to find the SEMNAM_M3LSHM semaphore.\n", 0);
        return false;
    }

    ext_sem = rt_typed_sem_init(nam2num(SEMNAM_M3LEXT), 1, BIN_SEM);
#else
    ext_sem = new sem_t();
    sem_init(ext_sem, 1, 1);
#endif
    if(!ext_sem) {
        M3_ERR("Unable to find the M3LEXT semaphore (probably hasn't been cleared properly, reboot can solve this problem).\n");
        //return false;
    }

#ifdef __RTAI__
    ready_sem = rt_typed_sem_init(nam2num(SEMNAM_M3READY), 1, BIN_SEM);
#else
    ready_sem = new sem_t();
    sem_init(ready_sem, 1, 1);
#endif
    if(!ready_sem) {
        M3_ERR("Unable to find the M3READY semaphore.\n");
        //return false;
    }
    M3_INFO("Matching Kernel EC components with config file...\n");
    int rm_cnt=0;
    for(vector<M3ComponentEc *>::iterator it_ec=m3ec_list.begin();it_ec!=m3ec_list.end();/*++it_ec*/){
        if((*it_ec)->SetSlaveEcShm(shm_ec->slave, shm_ec->slaves_responding) == false){
            factory->ReleaseComponent((*it_ec));
            m3ec_list.erase(it_ec);
            rm_cnt++;
        }else{
            it_ec++;
        }
    }
    for(int i=0;i<rm_cnt;i++){
        idx_map_ec.pop_back();
    }
    // Hack to put back the indexes
    for(int i=0;i<idx_map_rt.size();i++)
        idx_map_rt[i]-=rm_cnt;
    //Link dependent components. Drop failures.
    //Keep dropping until no failures
    vector<M3Component *> bad_link;
    bool failure = true;
    M3_INFO("Linking components ...\n");
    while(GetNumComponents() > 0 && failure) {
        bad_link.clear();
        failure = false;
        for(int i = 0; i < GetNumComponents(); i++) {
            if(!GetComponent(i)->LinkDependentComponents()) {
                M3_WARN("Failure LinkDependentComponents for %s\n", GetComponent(i)->GetName().c_str());
                failure = true;
                bad_link.push_back(GetComponent(i));
            }
        }
        if(failure) {
            vector<M3Component *>::iterator ci;
            vector<M3ComponentEc *>::iterator eci;
            for(int i = 0; i < bad_link.size(); i++) {
                for(eci = m3ec_list.begin(); eci != m3ec_list.end(); ++eci)
                    if((*eci) == bad_link[i]) {
                        //(*eci)->Shutdown();
                        m3ec_list.erase(eci);
                        break;
                    }
                for(ci = m3rt_list.begin(); ci != m3rt_list.end(); ++ci)
                    if((*ci) == bad_link[i]) {
                        //(*ci)->Shutdown();
                        m3rt_list.erase(ci);
                        break;
                    }
                factory->ReleaseComponent(bad_link[i]);
            }
        }
    }
    
    if(GetNumComponents() == 0) {
        M3_WARN("No M3 Components could be loaded....\n", 0);
        return false;
    }
    M3_INFO("Done linking components.\n");
    M3_INFO("Starting up components ...\n");
    for(int i = 0; i < GetNumComponents(); i++) {
        GetComponent(i)->Startup();
    }
    M3_INFO("Done starting up components.\n");
    CheckComponentStates();
    PrettyPrintComponentNames();
    //Setup Monitor
    M3MonitorStatus *s = factory->GetMonitorStatus();
    for(int i = 0; i < GetNumComponents(); i++) {
        M3MonitorComponent *c = s->add_components();
        c->set_name(GetComponent(i)->GetName());
    }
    for(int i = 0; i < NUM_EC_DOMAIN; i++) {
        s->add_ec_domains();
    }
    return true;
}

bool M3RtSystem::ParseCommandFromExt(M3CommandAll &msg)
{
    int idx, i;
    string s;

    for(i = 0; i < msg.name_cmd_size(); i++) {
        idx = GetComponentIdx(msg.name_cmd(i));
        if(idx >= 0) {
            s = msg.datum_cmd(i);
            GetComponent(idx)->ParseCommand(s);
        } else {
            //M3_WARN("Invalid Command component name %s in ParseCommandFromExt\n",s.c_str());
            M3_WARN("Invalid Command component name %s in ParseCommandFromExt\n", msg.name_cmd(i).c_str());

        }
    }

    for(i = 0; i < msg.name_param_size(); i++) {
        idx = GetComponentIdx(msg.name_param(i));
        if(idx >= 0) {
            s = msg.datum_param(i);
            GetComponent(idx)->ParseParam(s);
        } else {
            M3_WARN("Invalid Param component name %s in ParseCommandFromExt\n", s.c_str());
        }
    }

    return true;
}

bool M3RtSystem::SerializeStatusToExt(M3StatusAll &msg, vector<string>& names)
{
    for(int i = 0; i < names.size(); i++) {
        M3Component *m = GetComponent(names[i]);
        if(m != NULL) {
            string datum;
            if(!m->SerializeStatus(datum)) {
                //Bug where SerializeToString fails for bad message size
                //Patched protobuf/message.cc to allow but return false
                //FixME!
                //datum.clear();
            }
            if(i >= msg.datum_size()) {
                //Grow message
                msg.add_datum(datum);
                msg.add_name(names[i]);
            } else {
                msg.set_datum(i, datum);
                msg.set_name(i, names[i]);
            }
        }
    }
    return true;
}



void M3RtSystem::CheckComponentStates()
{
    if(safeop_required)
        return;
    for(int i = 0; i < GetNumComponents(); i++) {
        if(GetComponent(i)->IsStateError()) { //All or none in OP
            M3_WARN("Component error detected for %s. Forcing to state SAFEOP\n", GetComponent(i)->GetName().c_str());

            safeop_required = true;
            GetComponent(i)->SetStateSafeOp();
            //return;
        }
    }
}

bool M3RtSystem::SetComponentStateOp(int idx)
{
    if(safeop_required)
        return false;
    if(idx < GetNumComponents() && idx >= 0)
        if(GetComponent(idx)->IsStateSafeOp()) {
            GetComponent(idx)->SetStateOp();
            return true;
        }
    return false;
}
void M3RtSystem::SetComponentStateOpAll(void)
{
    std::vector<M3Component* >::iterator it_rt;
    std::vector<M3ComponentEc* >::iterator it_ec;
    for(it_ec = m3ec_list.begin();it_ec!=m3ec_list.end();++it_ec)
        (*it_ec)->SetStateOp();
    for(it_rt = m3rt_list.begin();it_rt!=m3rt_list.end();++it_rt)
        (*it_rt)->SetStateOp();
}
void M3RtSystem::SetComponentStateSafeOpAll(void)
{
    std::vector<M3Component* >::iterator it_rt;
    std::vector<M3ComponentEc* >::iterator it_ec;
    for(it_ec = m3ec_list.begin();it_ec!=m3ec_list.end();++it_ec)
        (*it_ec)->SetStateSafeOp();
    for(it_rt = m3rt_list.begin();it_rt!=m3rt_list.end();++it_rt)
        (*it_rt)->SetStateSafeOp();
}

bool M3RtSystem::SetComponentStateSafeOp(int idx)
{
    if(idx < GetNumComponents() && idx >= 0) {
        GetComponent(idx)->SetStateSafeOp();
        return true;
    }
    return false;
}
int M3RtSystem::GetComponentState(int idx)
{
    if(idx < GetNumComponents() && idx >= 0)
        return GetComponent(idx)->GetState();
    return -1;
}

void M3RtSystem::PrettyPrint()
{
    int nece = 0, nrte = 0, necs = 0, nrts = 0, neco = 0, nrto = 0;
    vector<M3ComponentEc *>::iterator i;
    for(i = m3ec_list.begin(); i != m3ec_list.end(); ++i) {
        if((*i)->IsStateError()) nece++;
        if((*i)->IsStateSafeOp()) necs++;
        if((*i)->IsStateOp()) neco++;
    }
    vector<M3Component *>::iterator j;
    for(j = m3rt_list.begin(); j != m3rt_list.end(); ++j) {
        if((*j)->IsStateError()) nrte++;
        if((*j)->IsStateSafeOp()) nrts++;
        if((*j)->IsStateOp()) nrto++;
    }

    BannerPrint(80, "M3 SYSTEM");
    M3_PRINTF("Operational: %s\n", IsOperational() ? "yes" : "no");
    M3_PRINTF("Ec components: %d\n", (int)m3ec_list.size());
    M3_PRINTF("Rt components: %d\n", (int)m3rt_list.size());
    M3_PRINTF("Ec components in error: %d\n", nece);
    M3_PRINTF("Rt components in error: %d\n", nrte);
    M3_PRINTF("Ec components in safeop: %d\n", necs);
    M3_PRINTF("Rt components in safeop: %d\n", nrts);
    M3_PRINTF("Ec components in op: %d\n", neco);
    M3_PRINTF("Rt components in op: %d\n", nrto);
}



void M3RtSystem::PrettyPrintComponentNames()
{
    BannerPrint(60, "M3 SYSTEM COMPONENTS");
    for(int i = 0; i < GetNumComponents(); i++)
        M3_PRINTF("%s \n-- Config: %s\n", GetComponentName(i).c_str(),GetComponent(i)->GetConfigPath().c_str());
    BannerPrint(60, "");
}

void M3RtSystem::PrettyPrintComponents()
{
    PrettyPrint();
    for(int i = 0; i < GetNumComponents(); i++)
        GetComponent(i)->PrettyPrint();
    M3_PRINTF("\n\n\n");
}

void M3RtSystem::PrettyPrintComponent(int idx)
{
    if(idx < GetNumComponents() && idx >= 0)
        GetComponent(idx)->PrettyPrint();
}

bool M3RtSystem::Step(bool safeop_only,bool dry_run)
{
#ifdef __RTAI__
    RTIME start, end, dt, start_c, end_c, start_p, end_p;
#else
    long long start, end, dt, start_c, end_c, start_p, end_p;
#endif
    bool ret_step=true;
    vector<M3ComponentEc *>::iterator j;
    step_cnt++;

    /*
        1: Block External Data Service from chaging state
        2: Wait until EtherCAT mod signals finished a cycle (synchronize)
        3: Acquire lock on EtherCAT shared mem
        4: Get data from EtherCAT shared mem
        5: Step all components
        6: Transmit newly computed commands to EtherCAT shared mem
        7: Upload status to logger
        8: Release locks, allow External Data Service to update components if new data is available.
    */

    /*
        The order of components in m3ec_list and m3rt_list is important as they can overwrite eachother.
        Given components A, B, where A computes and sets value B.x .
        If we place A before B in the component list of m3_config.yml, then A.Step() will run before B.Step() within once cycle.
        Otherwise, B.Step() uses the A.x value from the previous cycle.

        If we have an External Data Service that sets B.x=e periodically, then A.Step() will overwrite value e.
        Therefore if we want to directly communicate with B.x from the outside world, we must not publish to component A.
    */
    
    //Do some bookkeeping
    M3MonitorStatus *s = factory->GetMonitorStatus();
    
    
#ifdef __RTAI__
    start_c = rt_get_cpu_time_ns();
    rt_sem_wait(ext_sem);
    end_c = rt_get_cpu_time_ns();
    s->set_t_ext_sem_wait(end_c - start_c);
#ifndef __NO_KERNEL_SYNC__
    start_c = rt_get_cpu_time_ns();
    rt_sem_wait(sync_sem); // AH: this guy is causing ALL the overrruns
    end_c = rt_get_cpu_time_ns();
    s->set_t_sync_sem_wait(end_c - start_c);
#endif
    start_c = rt_get_cpu_time_ns();
    rt_sem_wait(shm_sem);
    end_c = rt_get_cpu_time_ns();
    s->set_t_shm_sem_wait(end_c - start_c);
    
    start = rt_get_cpu_time_ns();
#else
    sem_wait(ext_sem);
    start = getNanoSec();
#endif
    if(safeop_only) { // in case we are too slow
        for(int i = 0; i < GetNumComponents(); i++)
            if(GetComponent(i)->IsStateError()) {
                if(!dry_run)
                    GetComponent(i)->SetStateSafeOp();
                else
                    GetComponent(i)->SetStateOp();
            }

    }

    
    int nop = 0, nsop = 0, nerr = 0;
    for(int i = 0; i < GetNumComponents(); i++) {
        if(GetComponent(i)->IsStateError()) nerr++;
        if(GetComponent(i)->IsStateSafeOp()) nsop++;
        if(GetComponent(i)->IsStateOp()) nop++;
        M3MonitorComponent *c = s->mutable_components(i);
        c->set_state((M3COMP_STATE)GetComponent(i)->GetState());
    }
    if(m3ec_list.size() != 0) {
        for(int i = 0; i < NUM_EC_DOMAIN; i++) {
            s->mutable_ec_domains(i)->set_t_ecat_wait_rx(shm_ec->monitor[i].t_ecat_wait_rx);
            s->mutable_ec_domains(i)->set_t_ecat_rx(shm_ec->monitor[i].t_ecat_rx);
            s->mutable_ec_domains(i)->set_t_ecat_wait_shm(shm_ec->monitor[i].t_ecat_wait_shm);
            s->mutable_ec_domains(i)->set_t_ecat_shm(shm_ec->monitor[i].t_ecat_shm);
            s->mutable_ec_domains(i)->set_t_ecat_wait_tx(shm_ec->monitor[i].t_ecat_wait_tx);
            s->mutable_ec_domains(i)->set_t_ecat_tx(shm_ec->monitor[i].t_ecat_tx);
        }
    }
    s->set_num_components_safeop(nsop);
    s->set_num_components_op(nop);
    s->set_num_components_err(nerr);
    s->set_num_components(GetNumComponents());
    s->set_num_components_ec(m3ec_list.size());
    s->set_num_components_rt(m3rt_list.size());
    s->set_operational(IsOperational());
    s->set_num_ethercat_cycles(GetEcCounter());
#ifdef __RTAI__
    //Set timestamp for all
    int64_t ts = shm_ec->timestamp_ns / 1000;
#else
    int64_t ts = getNanoSec() / 1000;
#endif

    for(int i = 0; i < GetNumComponents(); i++)
        GetComponent(i)->SetTimestamp(ts);

#ifdef __RTAI__
    start_p = rt_get_cpu_time_ns();
#else
    start_p = getNanoSec();
#endif

#ifdef __RTAI__
    //Get Status from EtherCAT
    for(int j = 0; j <= MAX_PRIORITY; j++) {
        for(int i = 0; i < m3ec_list.size(); i++) { //=m3ec_list.begin(); j!=m3ec_list.end(); ++j)
            if(m3ec_list[i]->GetPriority() == j) {
                start_c = rt_get_cpu_time_ns();
                m3ec_list[i]->StepStatus();
                end_c = rt_get_cpu_time_ns();
                M3MonitorComponent *c = s->mutable_components(idx_map_ec[i]);
                c->set_cycle_time_status_us((mReal)(end_c - start_c) / 1000);
            }
        }
    }
#endif

    //Set Status on non-EC components
    for(int j = 0; j <= MAX_PRIORITY; j++) {
        for(int i = 0; i < m3rt_list.size(); i++) {
            if(m3rt_list[i]->GetPriority() == j) {

#ifdef __RTAI__
                start_c = rt_get_cpu_time_ns();
#else
                start_c = getNanoSec();
#endif
                m3rt_list[i]->StepStatus();
#ifdef __RTAI__
                end_c = rt_get_cpu_time_ns();
#else
                end_c = getNanoSec();
#endif
                M3MonitorComponent *c = s->mutable_components(idx_map_rt[i]);
                c->set_cycle_time_status_us((mReal)(end_c - start_c) / 1000);
            }
        }
    }
#ifdef __RTAI__
    end_p = rt_get_cpu_time_ns();
#else
    end_p = getNanoSec();
#endif
    s->set_cycle_time_status_us((mReal)(end_p - start_p) / 1000);
    //Set Command on non-EC components
    //Step components in reverse order
#ifdef __RTAI__
    start_p = rt_get_cpu_time_ns();
#else
    start_p = getNanoSec();
#endif

    for(int j = MAX_PRIORITY; j >= 0; j--) {
        for(int i = 0; i < m3rt_list.size(); i++) {
            if(m3rt_list[i]->GetPriority() == j) {

#ifdef __RTAI__
                start_c = rt_get_cpu_time_ns();
#else
                start_c = getNanoSec();
#endif
                m3rt_list[i]->StepCommand();
#ifdef __RTAI__
                end_c = rt_get_cpu_time_ns();
#else
                end_c = getNanoSec();
#endif
                M3MonitorComponent *c = s->mutable_components(idx_map_rt[i]);
                c->set_cycle_time_command_us((mReal)(end_c - start_c) / 1000);
            }
        }
    }
#ifdef __RTAI__
    //Send Command to EtherCAT
    for(int j = MAX_PRIORITY; j >= 0; j--) {
        for(int i = 0; i < m3ec_list.size(); i++) {
            if(m3ec_list[i]->GetPriority() == j) {
                start_c = rt_get_cpu_time_ns();
                m3ec_list[i]->StepCommand();
                end_c = rt_get_cpu_time_ns();
                M3MonitorComponent *c = s->mutable_components(idx_map_ec[i]);
                c->set_cycle_time_command_us((mReal)(end_c - start_c) / 1000);
            }

        }
    }
    end_p = rt_get_cpu_time_ns();
#else
    end_p = getNanoSec();
#endif
    s->set_cycle_time_command_us((mReal)(end_p - start_p) / 1000);
    //Now see if any errors raised
    CheckComponentStates();
    if(dry_run&&safeop_required){// Let's give it another chance!
        safeop_required=false;
        ret_step=false;
    }else if(dry_run&&!safeop_required){
        ret_step=true;
    }

    if(log_service) {
        logging = true;
        if(!log_service->Step())
            M3_DEBUG("Step() of log service failed.\n");
        logging = false;
    }
#ifdef __RTAI__
    end = rt_get_cpu_time_ns();
#else
    end = getNanoSec();
#endif
    mReal elapsed = (mReal)(end - start) / 1000;
    if(elapsed > s->cycle_time_max_us() && step_cnt > 10)
        s->set_cycle_time_max_us(elapsed);
    s->set_cycle_time_us(elapsed);
    int64_t period = end - last_cycle_time;
    mReal rate = 1 / (mReal)period;
    s->set_cycle_frequency_hz((mReal)(rate * 1000000000.0));
    last_cycle_time = end;
#ifdef __RTAI__
    rt_sem_signal(shm_sem);
    rt_sem_signal(ext_sem);
#else
    sem_post(ext_sem);
#endif
    return ret_step;
}

}
