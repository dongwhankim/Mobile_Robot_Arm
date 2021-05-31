#ifndef sync_read_write_h //?
#define sync_read_write_h //?

class CRobot_Arm_TR
{
    public:
	  CRobot_Arm_TR();
	  virtual ~CRobot_Arm_TR(); //	

    void TXRX();
    void TX();
    void RX();
    void start();
    void end();
    void goalposition(int* _dxl_goal_position);
    void present_position();
      
    int32_t _dxl_present_position[5];

   private:

  bool _dxl_addparam_result = false;                 // addParam result
  bool _dxl_getdata_result = false;                  // GetParam result

  uint8_t _dxl_error = 0;                            // Dynamixel error
  uint8_t _param_goal_position1[4];
  uint8_t _param_goal_position2[4];
  uint8_t _param_goal_position3[4];
  uint8_t _param_goal_position4[4];                     // Present position
  int _dxl_goal_position[5];
  
};
#endif