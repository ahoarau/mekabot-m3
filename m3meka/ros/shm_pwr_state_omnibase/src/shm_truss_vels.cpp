 /*************************************************************************
 * 
 * REDWOOD CONFIDENTIAL
 * Author: Aaron Edsinger
 * __________________
 * 
 *  [2012] - [+] Redwood Robotics Incorporated 
 *  All Rights Reserved.
 * 
 * All information contained herein is, and remains
 * the property of Redwood Robotics Incorporated and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to Redwood Robotics Incorporated
 * and its suppliers and may be covered by U.S. and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Redwood Robotics Incorporated.
 */
 
#include <rtai_sched.h>
#include <stdio.h>
#include <signal.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>
#include "m3rt/base/m3ec_def.h>
#include "m3rt/base/m3rt_def.h>
#include <rtai_nam2num.h>
#include <rtai_registry.h>
#include "m3/vehicles/omnibase_shm_sds.h"

// Needed for ROS
#include <ros/ros.h>
#include <shm_omnibase_controller/M3TrussVel.h>
#include <tf/transform_broadcaster.h>


#define RT_TASK_FREQUENCY_MEKA_OMNI_SHM 100
#define RT_TIMER_TICKS_NS_MEKA_OMNI_SHM (1000000000 / RT_TASK_FREQUENCY_MEKA_OMNI_SHM)		//Period of rt-timer 
#define MEKA_ODOM_SHM "OSHMM"
#define MEKA_ODOM_CMD_SEM "OSHMC"
#define MEKA_ODOM_STATUS_SEM "OSHMS"

////////////////////////////////////////////////////////////////////////////////////
static int sys_thread_active = 0;
static int sys_thread_end=0;
static int end=0;
static int hst;
static M3OmnibaseShmSdsCommand cmd;
static M3OmnibaseShmSdsStatus status;
static int sds_status_size;
static int sds_cmd_size;
static long step_cnt = 0;
static void endme(int dummy) { std::cout << "END\n"; end=1; }
static int64_t last_cmd_ts;
shm_omnibase_controller::M3TrussVel truss_vel_g;
ros::Publisher truss_vel_publisher_g;

////////////////////////////////////////////////////////////////////////////////////


///////  Periodic Control Loop:
void StepShm();


///////////////////////////////

void SetTimestamp(int64_t  timestamp)
{
  cmd.timestamp = timestamp;
    return; 
}

int64_t GetTimestamp()
{  
    return status.timestamp; 
}

////////////////////////// MAIN COMPUTATION METHOD /////////////////////////////

void StepShm(int cntr)
{   
    
    
    
    truss_vel_g.header.stamp = ros::Time::now();  

  
    for(int i = 0; i < 6; i++)      
	truss_vel_g.truss_vels[i] = status.truss_vels[i];
    
    truss_vel_g.truss_vel_products[0] = status.truss_vels[0] * status.truss_vels[4] * status.truss_vels[3];
    truss_vel_g.truss_vel_products[1] = status.truss_vels[4] * status.truss_vels[1] * status.truss_vels[2];
    truss_vel_g.truss_vel_products[2] = status.truss_vels[2] * status.truss_vels[3] * status.truss_vels[5];
    truss_vel_g.truss_vel_products[3] = status.truss_vels[0] * status.truss_vels[5] * status.truss_vels[1];
    
    truss_vel_publisher_g.publish(truss_vel_g);
    

    
    /*if (cntr % 100 == 0)
      {	
	if (1)
	{
	  printf("********************************\n");
	  printf("timestamp: %ld\n", (status.timestamp - last_cmd_ts)/1000000);
	  //printf("to: %ld\n", VEL_TIMEOUT_NS);	  
	  {	    
	    //printf("JOINT %d\n", i);
	    printf("------------------------------\n");
	    printf("X: %f\n",status.x);
	    printf("Y: %f\n", status.y);
	    printf("YAW: %f\n", status.yaw);
	    printf("Vx: %f\n", odom_g.twist.twist.linear.x);	  
	    printf("Vy: %f\n", odom_g.twist.twist.linear.y);
	    printf("Va: %f\n", odom_g.twist.twist.angular.z);
	     printf("------------------------------\n");
	    printf("\n");
	  }
	}
      }*/
    
      /*if (cntr % 100 == 0)
      {	
	if (1)
	{
	  printf("********************************\n");
	  printf("timestamp: %ld\n", status.timestamp);	  
	  {	    
	    //printf("JOINT %d\n", i);
	    printf("------------------------------\n");
	    printf("X: %f\n", odom_g.pose.pose.position.x);
	    printf("Y: %f\n", odom_g.pose.pose.position.y);
	    printf("YAW: %f\n", th);
	    printf("Vx: %f\n", odom_g.twist.twist.linear.x);	  
	    printf("Vy: %f\n", odom_g.twist.twist.linear.y);
	    printf("Va: %f\n", odom_g.twist.twist.angular.z);
	     printf("------------------------------\n");
	    printf("\n");
	  }
	}
      }*/
    
  
}



////////////////////////// RTAI PROCESS BOILERPLATE /////////////////////////////

static void* rt_system_thread(void * arg)
{	
	SEM * status_sem;
	SEM * command_sem;
	RT_TASK *task;
	int cntr=0;
	M3Sds * sds = (M3Sds *)arg;
	printf("Starting real-time thread\n");
		
	
	sds_status_size = sizeof(M3OmnibaseShmSdsStatus);	
	
	
	task = rt_task_init_schmod(nam2num("TVELP"), 0, 0, 0, SCHED_FIFO, 0xF);
	rt_allow_nonroot_hrt();
	if (task==NULL)
	{
		printf("Failed to create RT-TASK TSHMP\n");
		return 0;
	}
	status_sem=(SEM*)rt_get_adr(nam2num(MEKA_ODOM_STATUS_SEM));
	
	if (!status_sem)
	{
		printf("Unable to find the %s semaphore.\n",MEKA_ODOM_STATUS_SEM);
		rt_task_delete(task);
		return 0;
	}
	
	
	RTIME tick_period = nano2count(RT_TIMER_TICKS_NS_MEKA_OMNI_SHM); 
	RTIME now = rt_get_time();
	rt_task_make_periodic(task, now + tick_period, tick_period); 
	mlockall(MCL_CURRENT | MCL_FUTURE);
	rt_make_hard_real_time();
	long long start_time, end_time, dt;
	long long step_cnt = 0;
	sys_thread_active=1;
	
	while(!sys_thread_end)
	{
		start_time = nano2count(rt_get_cpu_time_ns());
		rt_sem_wait(status_sem);
		memcpy(&status, sds->status, sds_status_size);		
		rt_sem_signal(status_sem);
		
		StepShm(cntr);		
		
				
		end_time = nano2count(rt_get_cpu_time_ns());
		dt=end_time-start_time;
		/*
		Check the time it takes to run components, and if it takes longer
		than our period, make us run slower. Otherwise this task locks
		up the CPU.*/
		if (dt > tick_period && step_cnt>10) 
		{
			printf("Step %lld: Computation time of components is too long. Forcing all components to state SafeOp.\n",step_cnt);
			printf("Previous period: %f. New period: %f\n", (double)count2nano(tick_period),(double)count2nano(dt));
 			tick_period=dt;
			//rt_task_make_periodic(task, end + tick_period,tick_period);			
		}
		step_cnt++;
		
		rt_task_wait_period();
	}	
	printf("Exiting RealTime Thread...\n",0);	
	rt_make_soft_real_time();	
	rt_task_delete(task);	
	sys_thread_active=0;
	return 0;
}


////////////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{	
	RT_TASK *task;
	M3Sds * sys;
	int cntr=0;
	
	rt_allow_nonroot_hrt();
	
	/*ros::init(argc, argv, "base_controller"); // initialize ROS node
  	ros::AsyncSpinner spinner(1); // Use 1 thread - check if you actually need this for only publishing
  	spinner.start();
        ros::NodeHandle root_handle;*/
	
	ros::init(argc, argv, "base_truss_vel", ros::init_options::NoSigintHandler); // initialize ROS node
	ros::AsyncSpinner spinner(1); // Use 1 thread - check if you actually need this for only publishing
	spinner.start();
        ros::NodeHandle root_handle;
	ros::NodeHandle p_nh("~");	
	
	
	truss_vel_publisher_g = root_handle.advertise<shm_omnibase_controller::M3TrussVel>("omnibase_truss_vel", 1, true);
	
	truss_vel_g.header.stamp = ros::Time::now();
	truss_vel_g.header.frame_id = "truss_vel";
	truss_vel_g.truss_vels.resize(6, 0.0);
	truss_vel_g.truss_vel_products.resize(4, 0.0);
	
	signal(SIGINT, endme);

	if (sys = (M3Sds*)rt_shm_alloc(nam2num(MEKA_ODOM_SHM),sizeof(M3Sds),USE_VMALLOC))
		printf("Found shared memory starting shm_omnibase_controller.");
	else
	{
		printf("Rtai_malloc failure for %s\n",MEKA_ODOM_SHM);
		return 0;
	}

	rt_allow_nonroot_hrt();
	/*if (!(task = rt_task_init_schmod(nam2num("TSHM"), RT_TASK_PRIORITY, 0, 0, SCHED_FIFO, 0xF)))
	{
		rt_shm_free(nam2num(TORQUE_SHM));
		printf("Cannot init the RTAI task %s\n","TSHM");
		return 0;
	}*/
	hst=rt_thread_create((void*)rt_system_thread, sys, 10000);
	usleep(100000); //Let start up
	if (!sys_thread_active)
	{
		rt_task_delete(task);
		rt_shm_free(nam2num(MEKA_ODOM_SHM));
		printf("Startup of thread failed.\n",0);
		return 0;
	}
	while(!end)
	{		
		usleep(250000);
		
	}
	printf("Removing RT thread...\n",0);
	sys_thread_end=1;
	//rt_thread_join(hst);	
	usleep(1250000);	
	if (sys_thread_active)printf("Real-time thread did not shutdown correctly\n");	
	//rt_task_delete(task);	
	rt_shm_free(nam2num(MEKA_ODOM_SHM));	
	ros::shutdown();	
	return 0;
}


