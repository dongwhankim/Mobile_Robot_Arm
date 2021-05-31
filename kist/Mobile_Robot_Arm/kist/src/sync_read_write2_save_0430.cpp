/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Sync Read and Sync Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//
#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <trajectory.h>
#include <controller.h>
#include "dynamixel_sdk.h"   
#include "sync_read_write2.h"                               // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          512                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          564
#define ADDR_PRO_PRESENT_POSITION       580

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define DXL3_ID                         3                   // Dynamixel#2 ID: 3
#define DXL4_ID                         4                   // Dynamixel#2 ID: 3
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      0              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
#define PERIOD_NS 20000000  // 50hz
#define SEC_IN_NSEC 1000000000

#define PI 3.141592

struct timespec ts;
struct timespec ts2;

int fprintf(FILE* stream, const char* format, ...);
double target[5];
double x_goal[5];
double RANGE(double angle);
  double q[5],z,x_,y_, gamma_1, k1_1, k2_1;
  double cos_q2, sin_q2_1, sin_q2_2, q2_1, q2_2, k1_2, k2_2, gamma_2, q1_1, q1_2, q3_1, q3_2;
  bool rev = false;
  int state = 0;
  int mode = 0;
  int mode2 = 0;
  double now_time , now_time2, now_time3, now_time4;
  int dxl_goal_position[5];
sync::sync()
{
}
sync::~sync()
{
}
double RANGE(double angle) {
	while (angle > PI || angle <= -PI) {
		if (angle > PI) {
			angle = angle - 2 * PI;
		}
		else {
			angle = angle + 2 * PI;
		}
	}
	return angle;
}

int ret;


int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

void sync::goalposition(int* _dxl_goal_position)
{
  for(int i = 0 ; i<5 ; i++)
  {
     dxl_goal_position[i] = _dxl_goal_position[i];
  }
}
int getValue()
{
	// 도시락 고르기
	int Box_Number;
	std::cout << "\n 도시락 선택 4(↖)3(↗) \n             2(↙)1(↘) :";
	std::cin >> Box_Number;
	return Box_Number;
}

int getValue2()
{
	// 놓을 테이블 고르기
	int Table_Number;
	std::cout << "\n 놓을 테이블 선택 1(→),2(←):";
	std::cin >> Table_Number;
	return Table_Number;
}

int main()
{

  CController Control;

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  //int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result
  bool dxl_getdata_result = false;                  // GetParam result
  //int dxl_goal_position1[2] = {0, 0};  // Goal position
  // int dxl_goal_position2[2] = {0, 0};
  // int dxl_goal_position3[2] = {0, 0};

  uint8_t dxl_error = 0;                            // Dynamixel error
  //uint8_t param_goal_position[4];
  uint8_t param_goal_position1[4];
   uint8_t param_goal_position2[4];
   uint8_t param_goal_position3[4];
   uint8_t param_goal_position4[4];
  //int32_t dxl1_present_position = 0 , dxl2_present_position = 0 , dxl3_present_position = 0;                         // Present position
  int32_t dxl_present_position[5];
  

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
  }

   // Enable Dynamixel#2 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
   }
   else
   {
     printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
   }

    // Enable Dynamixel#3 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
   }
   else
   {
     printf("Dynamixel#%d has been successfully connected \n", DXL3_ID);
   }
    // Enable Dynamixel#4 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
   }
   else
   {
     printf("Dynamixel#%d has been successfully connected \n", DXL4_ID);
   }

  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
    return 0;
  }

  // // Add parameter storage for Dynamixel#2 present position value
   dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
   if (dxl_addparam_result != true)
   {
     fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
     return 0;
   }

   // Add parameter storage for Dynamixel#3 present position value
   dxl_addparam_result = groupSyncRead.addParam(DXL3_ID);
   if (dxl_addparam_result != true)
   {
     fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL3_ID);
     return 0;
   }

   // Add parameter storage for Dynamixel#4 present position value
   dxl_addparam_result = groupSyncRead.addParam(DXL4_ID);
   if (dxl_addparam_result != true)
   {
     fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL4_ID);
     return 0;
   }

    clock_gettime(CLOCK_MONOTONIC, &ts);
    clock_gettime(CLOCK_MONOTONIC, &ts2);
    int start_time = ts2.tv_sec;
    //int dxl_goal_position1 = 0 , dxl_goal_position2 = 0 , dxl_goal_position3 = 0 ;
    //FILE* fp = fopen("output.txt", "w");

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
      dxl_comm_result = groupSyncRead.txRxPacket(); // << long time
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (groupSyncRead.getError(DXL1_ID, &dxl_error))
      {
        printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
      }
       else if (groupSyncRead.getError(DXL2_ID, &dxl_error))
       {
         printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
       }
       else if (groupSyncRead.getError(DXL3_ID, &dxl_error))
       {
         printf("[ID:%03d] %s\n", DXL3_ID, packetHandler->getRxPacketError(dxl_error));
       }
        else if (groupSyncRead.getError(DXL4_ID, &dxl_error))
       {
         printf("[ID:%03d] %s\n", DXL4_ID, packetHandler->getRxPacketError(dxl_error));
       }
     dxl_present_position[1] = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
     dxl_present_position[2] = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
     dxl_present_position[3] = groupSyncRead.getData(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
     dxl_present_position[4] = groupSyncRead.getData(DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

    //   printf("Press any key to continue! (or press ESC to quit!)\n");
    //  if (getch() == ESC_ASCII_VALUE)
    //    break;

      Control.get_present_position(dxl_present_position);

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        ts.tv_nsec +=PERIOD_NS;
        clock_gettime(CLOCK_MONOTONIC, &ts2);
         now_time = ts2.tv_sec - start_time;
         now_time2 = now_time + ts2.tv_nsec / 1000000000.0;

       std::cout << "time : " << now_time2<< " (s)" << std::endl;

      Control.Finite_State_Machine(now_time2);

    // Allocate goal position value into byte array
     param_goal_position1[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[1]));
     param_goal_position1[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[1]));
     param_goal_position1[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[1]));
     param_goal_position1[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[1]));
    
     param_goal_position2[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[2]));
     param_goal_position2[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[2]));
     param_goal_position2[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[2]));
     param_goal_position2[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[2]));

     param_goal_position3[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[3]));
     param_goal_position3[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[3]));
     param_goal_position3[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[3]));
     param_goal_position3[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[3]));

     param_goal_position4[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[4]));
     param_goal_position4[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[4]));
     param_goal_position4[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[4]));
     param_goal_position4[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[4]));


    groupSyncWrite.clearParam();

    // Add Dynamixel#1 goal position value to the Syncwrite storage
     dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position1);
     if (dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
       return 0;
     }

     // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
     dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position2);
     if (dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
       return 0;
     }

     // Add Dynamixel#3 goal position value to the Syncwrite parameter storage
     dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position3);
     if (dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID);
       return 0;
     }

     // Add Dynamixel#4 goal position value to the Syncwrite parameter storage
     dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param_goal_position4);
     if (dxl_addparam_result != true)
     {
       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL4_ID);
       return 0;
     }

        // clock_gettime(CLOCK_MONOTONIC, &ts2);
        // now_time = ts2.tv_sec - start_time;
        // now_time2 = now_time + ts2.tv_nsec / 1000000000.0;
        // std::cout << "3 : " << now_time2 << std::endl;
     
    // Syncwrite goal position
     dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    
       //Check if groupsyncread data of Dynamixel#1 is available
       dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
       if (dxl_getdata_result != true)
       {
         fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL1_ID);
         return 0;
       }

       // Check if groupsyncread data of Dynamixel#2 is available
       dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
       if (dxl_getdata_result != true)
       {
         fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL2_ID);
         return 0;
       }

       // Check if groupsyncread data of Dynamixel#3 is available
       dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
       if (dxl_getdata_result != true)
       {
         fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL3_ID);
         return 0;
       }

        // Check if groupsyncread data of Dynamixel#4 is available
       dxl_getdata_result = groupSyncRead.isAvailable(DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
       if (dxl_getdata_result != true)
       {
         fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL4_ID);
         return 0;
       }


     printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d \t[ID:%03d] GoalPos:%03d  PresPos:%03d \n", DXL1_ID, dxl_goal_position[1], dxl_present_position[1], DXL2_ID, dxl_goal_position[2], dxl_present_position[2], DXL3_ID, dxl_goal_position[3], dxl_present_position[3], DXL4_ID, dxl_goal_position[4], dxl_present_position[4]);
     
    }

  // Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

   // Disable Dynamixel#2 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
   }

    // Disable Dynamixel#3 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
   }

    // Disable Dynamixel#4 Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
   }
  // Close port
  portHandler->closePort();

  return 0;
}
