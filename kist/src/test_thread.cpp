
#include <pthread.h>
#include <stdio.h>
//#include <iostream>
#include <controller.h>
#include "dynamixel_sdk.h"   
#include <sync_read_write2.h>                              // Uses Dynamixel SDK library


#define PERIOD_NS 10000000  // 50hz
#define SEC_IN_NSEC 1000000000
#define PROTOCOL_VERSION                2.0   
#define DEVICENAME                      "/dev/ttyUSB0" 

using namespace std;

double now_time , now_time2;

CController Control;
CRobot_Arm_TR RobotArm;

void *dynamixelthread(void *data)
{  
  //Arm2.start();
  while(1)
  {
    RobotArm.TXRX();
  }
  RobotArm.end();
}

int main()
{
  //pthread_attr_t attr ; // , attr2;
  pthread_t thread1 ; // , thread2;
  //pthread_attr_init(&attr);
  struct timespec ts;
  struct timespec ts2;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  clock_gettime(CLOCK_MONOTONIC, &ts2);
  int start_time = ts2.tv_sec;

//////////////// initialize RS485
  RobotArm.start();
  RobotArm.RX();
int cnt_timer = 0;
 cout << "Start RS485 Communication!" << endl << "Wait for 2 sec" <<endl<<endl;
  while(1)
  {
      while(ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        //cout << cnt_timer/10.0 << " s" << endl;
        ts.tv_nsec += 100000000;     
       
      if(cnt_timer >= 20) //5sec
      {
       break;
      }
      cnt_timer++;
  }
  RobotArm.goalposition(RobotArm._dxl_present_position);

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
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        ts.tv_nsec +=PERIOD_NS;
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        now_time = ts2.tv_sec - start_time;
        now_time2 = now_time + ts2.tv_nsec / 1000000000.0;
        
        std::cout << "time:" << now_time2 << "sec" << std::endl;

        Control.get_present_position(RobotArm._dxl_present_position);
        Control.Finite_State_Machine(now_time2);
        RobotArm.goalposition(Control._goal_position);
       
  }
  //pthread_create(&thread1, NULL, &dynamixelthread,  (void*) &ctime);
  pthread_join(thread1,NULL);
  return (0);
}
