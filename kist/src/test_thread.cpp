
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
//#include <iostream>
#include <controller.h>
#include "dynamixel_sdk.h"   
#include <sync_read_write2.h>                              // Uses Dynamixel SDK library
#include <linear_read_write.h>


#define PERIOD_NS 1000000  // 1Khz
#define SEC_IN_NSEC 1000000000
#define PROTOCOL_VERSION                2.0   
#define DEVICENAME                      "/dev/ttyUSB0" 

using namespace std;

double now_time , now_time2;

CController Control;
CRobot_Arm_TR RobotArm;
Clinear linear; 
int tmpcnt = 0;
int tmpcnt2 = 0;
int linear_goal = 0;
int old_linear_goal = 0;
bool linear_play_state = 0;

void *dynamixelthread(void *data)
{  
  while(1)
  {
    tmpcnt2++;
    RobotArm.RX();
    if(tmpcnt2 == 200)
    {
      linear.read_encoder();
      tmpcnt2 = 0;
    }
    if(linear_play_state == 1)
    { 
      linear.goalposition2Uchar(Control._x_goal[0]+1000000);
      linear_play_state = 0;
      RobotArm.TX();

    }
    usleep(10);
  }
  RobotArm.end();
}

int main()
{
  pthread_t thread1 ;
  struct timespec ts;
  struct timespec ts2;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  clock_gettime(CLOCK_MONOTONIC, &ts2);
  int start_time = ts2.tv_sec;

//////////////// initialize RS485
  RobotArm.start();
  RobotArm.RX();
  linear.Open_linear();
  linear.goalposition2Uchar(-8000000);
  int cnt_timer = 0;
 cout << "Start RS485 Communication!" << endl << "Wait for 7.5 sec" <<endl<<endl;
  while(1)
  {
      while(ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        ts.tv_nsec += 100000000;     
       
      if(cnt_timer >= 75) //5sec
      {
        cnt_timer =0;
       break;
      }
      cnt_timer++;
  }
  RobotArm.goalposition(RobotArm._dxl_present_position);
  linear.homing();

//////////////// RS485 communication
  pthread_create(&thread1, NULL, &dynamixelthread,  (void*) &ctime);
  while(1)
  {
    while(ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }

        while(ts2.tv_nsec >= SEC_IN_NSEC)
        {
            ts2.tv_sec++;
            ts2.tv_nsec -= SEC_IN_NSEC;
        }

        old_linear_goal = linear_goal;
        ts.tv_nsec +=PERIOD_NS;
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        now_time = ts2.tv_sec - start_time;
        now_time2 = now_time + ts2.tv_nsec / 1000000000.0;
        Control.get_present_position(RobotArm._dxl_present_position);
        Control.Finite_State_Machine(now_time2);
        RobotArm.goalposition(Control._x_goal);
        linear_goal = Control._x_goal[0];

        if(old_linear_goal == linear_goal)
        {       
        }

        else
        {
          linear_play_state = 1;
        }

        if(Control._new_mode == 1)
        {
          linear.homing();
          Control._new_mode = 0;
          std::cout<<Control._new_mode<<std::endl;
        }
  }
  pthread_join(thread1,NULL);
  return (0);
}

