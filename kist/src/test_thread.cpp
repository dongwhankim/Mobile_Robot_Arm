
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
  //Arm2.start();
  while(1)
  {
    tmpcnt++;
    RobotArm.TX();
    if(tmpcnt == 150)
    {
      linear.read_encoder();
      tmpcnt = 0;
      RobotArm.RX();
      //linear.goalposition2Uchar_rel(linear_goal);
    }
    if(linear_play_state == 1)
    { 
      linear.goalposition2Uchar_rel(linear_goal);
      //usleep(3000);
      //linear.read_encoder();
      //usleep(100000);
      //std::cout << "position : "<<linear._read_buf<<std::endl ;
      linear_play_state = 0;
      //std::cout<<Control._x_goal[0]<<std::endl;
    }
        //RobotArm.TXRX();
    //std::cout<<Control._x_goal[0]<<std::endl;
    usleep(10);
  }
  RobotArm.end();
}

// void *linearthread(void *data)
// {  
//   //Arm2.start();
//   while(1)
//   {
//     linear.goalposition2Uchar(linear_goal);
//   }
// }
int main()
{
  //pthread_attr_t attr ; // , attr2;
  pthread_t thread1 ;//, thread2;
  //pthread_attr_init(&attr);
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
        //cout << cnt_timer/10.0 << " s" << endl;
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
  //pthread_create(&thread2, NULL, &linearthread,  (void*) &ctime);

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
        //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        ts.tv_nsec +=PERIOD_NS;
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        now_time = ts2.tv_sec - start_time;
        now_time2 = now_time + ts2.tv_nsec / 1000000000.0;
        //std::cout << "time:" << now_time2 << "sec" << std::endl;
        //linear.read_encoder();
        Control.get_present_position(RobotArm._dxl_present_position);
        //Control.get_linear_present_position(linear._linear_present_position);
        Control.Finite_State_Machine(now_time2);
        RobotArm.goalposition(Control._goal_position);
        linear_goal = Control._x_goal[0];
        if(old_linear_goal == linear_goal)
        {       
        }
        else
        {
          linear_play_state = 1;
        }
        //std::cout<<linear_goal<<std::endl;
        //linear.Communication(linear_goal);
  }
    //     if(Control._new_mode == 1)
    //     {
    //       Control.check_RX_RobotArm(1);
    //     switch (Control.CurrentState)
	  //   {
 		// case 0: // 시작자세 및 박스번호 준비
  	// 	  linear.Communication(-3000000);
		//       break;
  
  	// 	case 1: // 해당 박스 위로 준비자세
   	// 		linear.Communication(0);
		// 			break;
					
		// case 2: // 박스 앞으로 이동
    // 		linear.Communication(-3000000);
		// 			break;
		// case 3: // 박스를 들고 다시 나오기
  	// 		linear.Communication(0);
		// 		break;
		// case 4: // 테이블 앞으로 이동
		//     linear.Communication(-3000000);
		// case 5: // 내려놓으러 가기 2초동안 이동 , 4초까지 내려놓는거 대기
   	// 		linear.Communication(0);
		// 		break;
		// case 6: // 초기상태로 재설정
   	// 		linear.Communication(-3000000);
		// 			break;
    //   }
    //     }
  //pthread_create(&thread1, NULL, &dynamixelthread,  (void*) &ctime);
  pthread_join(thread1,NULL);
  //pthread_join(thread2,NULL);
  return (0);
}

