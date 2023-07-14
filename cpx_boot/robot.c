

#include "stdint.h"
#include "robot.h"
#include "cm_mode.h"
#include "cm_config.h"
#include "sc_servo_status.h"

#define ROBOT_XX        0
#define ROBOT_2AXIS     1

#if ROBOT_XX
#include "robot_xx.h"
#endif

#if ROBOT_2AXIS
#include "robot_axis_2.h"
#endif

master_robot_t* g_master_robot=MASTER_ROBOT;
robot_t trobot[MAX_AXIS_NUM];



/*
index 1-8
*/
uint32_t Master_Robot_Get_Control(uint8_t index)
{
  uint32_t control=0;
  switch(index)
  {
    case 1:{control = g_master_robot->contorl_1; break;}
    case 2:{control = g_master_robot->contorl_2; break;}
    case 3:{control = g_master_robot->contorl_3; break;}
    case 4:{control = g_master_robot->contorl_4; break;}
    case 5:{control = g_master_robot->contorl_5; break;}
    case 6:{control = g_master_robot->contorl_6; break;}
    case 7:{control = g_master_robot->contorl_7; break;}
    case 8:{control = g_master_robot->contorl_8; break;}
  }
  return control;
}

void Master_Robot_Set_Control(uint8_t index,uint32_t control)
{
  switch(index)
  {
    case 1:{g_master_robot->contorl_1 = control; break;}
    case 2:{g_master_robot->contorl_2 = control; break;}
    case 3:{g_master_robot->contorl_3 = control; break;}
    case 4:{g_master_robot->contorl_4 = control; break;}
    case 5:{g_master_robot->contorl_5 = control; break;}
    case 6:{g_master_robot->contorl_6 = control; break;}
    case 7:{g_master_robot->contorl_7 = control; break;}
    case 8:{g_master_robot->contorl_8 = control; break;}
  }
}





void Master_Robot_Set_Status(uint8_t index,uint32_t status)
{
  switch(index)
  {
    case 1:{g_master_robot->status_1 = status; break;}
    case 2:{g_master_robot->status_2 = status; break;}
    case 3:{g_master_robot->status_3 = status; break;}
    case 4:{g_master_robot->status_4 = status; break;}
    case 5:{g_master_robot->status_5 = status; break;}
    case 6:{g_master_robot->status_6 = status; break;}
    case 7:{g_master_robot->status_7 = status; break;}
    case 8:{g_master_robot->status_8 = status; break;}
  }
}

uint32_t Master_Robot_Get_Status(uint8_t index)
{
  
  uint32_t status=0;
  switch(index)
  {
    case 1:{status = g_master_robot->status_1; break;}
    case 2:{status = g_master_robot->status_2; break;}
    case 3:{status = g_master_robot->status_3; break;}
    case 4:{status = g_master_robot->status_4; break;}
    case 5:{status = g_master_robot->status_5; break;}
    case 6:{status = g_master_robot->status_6; break;}
    case 7:{status = g_master_robot->status_7; break;}
    case 8:{status = g_master_robot->status_8; break;}
  }
  return status;
}



void Master_Robot_Set_Status_Bit(uint8_t index,uint32_t status)
{
  uint32_t _status=Master_Robot_Get_Status(index);

  _status |= status;
  switch(index)
  {
    case 1:{g_master_robot->status_1 = _status; break;}
    case 2:{g_master_robot->status_2 = _status; break;}
    case 3:{g_master_robot->status_3 = _status; break;}
    case 4:{g_master_robot->status_4 = _status; break;}
    case 5:{g_master_robot->status_5 = _status; break;}
    case 6:{g_master_robot->status_6 = _status; break;}
    case 7:{g_master_robot->status_7 = _status; break;}
    case 8:{g_master_robot->status_8 = _status; break;}
  }
}









void Master_Robot_Set_Pos(uint8_t index,double pos)
{
  int32_t  s32pos_l=0;
  int32_t  s32pos_h=0;
  s32pos_l = (int32_t)(pos*3072.0);
  s32pos_h = s32pos_l&0xFFFFFF00;
  s32pos_l = s32pos_l<<24;

  
  switch(index)
  {
    case 1:{g_master_robot->axis1_pos_l = s32pos_l; //低8位
            g_master_robot->axis1_pos_h = s32pos_h;  break;}   //高24位
            
    case 2:{g_master_robot->axis2_pos_l = s32pos_l; //低8位
            g_master_robot->axis2_pos_h = s32pos_h;  break;}   //高24位
            
    case 3:{g_master_robot->axis3_pos_l = s32pos_l; //低8位
            g_master_robot->axis3_pos_h = s32pos_h;  break;}   //高24位
            
    case 4:{g_master_robot->axis4_pos_l = s32pos_l; //低8位
            g_master_robot->axis4_pos_h = s32pos_h;  break;}   //高24位
            
    case 5:{g_master_robot->axis5_pos_l = s32pos_l; //低8位
            g_master_robot->axis5_pos_h = s32pos_h;  break;}   //高24位
            
    case 6:{g_master_robot->axis6_pos_l = s32pos_l; //低8位
            g_master_robot->axis6_pos_h = s32pos_h;  break;}   //高24位
            
    case 7:{g_master_robot->axis7_pos_l = s32pos_l; //低8位
            g_master_robot->axis7_pos_h = s32pos_h;  break;}   //高24位
            
    case 8:{g_master_robot->axis8_pos_l = s32pos_l; //低8位
            g_master_robot->axis8_pos_h = s32pos_h;  break;}   //高24位

  }
}

/*上位机与CPX交互
1.模式   闭环或开环
2.电机报错，清除报错

*/
robot_cpx_t trobot_cpx;

void Robot_Update_CPX(void)
{
  // Robot_XX_Update();
  //Robot_Axis2_Update();
  uint8_t u8tmp=0;
  uint8_t u8index=0;
  uint32_t u32tmp=0;
  //控制字
  trobot_cpx.control_word = g_master_robot->contorl_cpx;
  trobot_cpx.status_word = g_master_robot->status_cpx;
  //模式控制
  for(uint8_t index=0;index<8;index++)
  {
    u32tmp = ServoStatus_Get_ControlWord(index);

    if(u32tmp&SERVOCONTROL_MASTER_ENALBE_CTRL)   //主站控制时才能改变控制模式
    {
      u32tmp = 1<<(index+8);
      if(trobot_cpx.control_word&u32tmp) //伺服索引对应
      {
         //Cm_Set_ServoMode(index,kServoMode_Postion);
         //trobot_cpx.control_word&=(~u32tmp); //控制字清除
      }
      else
      {
        //Cm_Set_ServoMode(index,kServoMode_OpenLoop);
      }
    }
  }

  //错误清除
  if(trobot_cpx.control_word&ROBOT_CONTROL_RESET_CPXFAULT_MASK)
  {
    for(uint8_t index=0;index<8;index++)
    {
      u32tmp = 1<<(index+24);
      if(trobot_cpx.control_word&u32tmp) //伺服索引对应
      {
         ServoStatus_Set_ControlWord(index,SERVOCONTROL_FAULT_CLEAR);
         trobot_cpx.control_word&=(~u32tmp); //控制字清除
      }
    }
  }

/****************************状态字*******************************************/
  
  
  //模式显示
  for(uint8_t index=0;index<8;index++)
  {
     u32tmp = 1<<(index+8);
     if(Cm_Get_ServoMode(index) == kServoMode_Postion )  //模式显示
     {
       trobot_cpx.status_word |= u32tmp;
     }
     else
     {
       trobot_cpx.status_word &= (~u32tmp);
     }
  }
  
  
  //CPX 报错显示
  for(uint8_t index=0;index<8;index++)
  {
     u32tmp = 1<<(index+24);
     if(ServoStatus_Get_Fault(index))  //报错
     {
       trobot_cpx.status_word |= u32tmp;
     }
     else
     {
       trobot_cpx.status_word &= (~u32tmp);
     }
  }
  
  g_master_robot->contorl_cpx = trobot_cpx.control_word;
  g_master_robot->status_cpx = trobot_cpx.status_word;
  
  //状态字更新
}










void Robot_Init(void)
{
  for(uint8_t index=0;index<MAX_AXIS_NUM;index++)
  {
    trobot[index].part_index = 0;
    trobot[index].axis_index = 0;
    trobot[index].axis_offset = 0;
    trobot[index].axis_degree = 0;
    trobot[index].axis_rr = 1;
  }
#if ROBOT_XX





   Robot_XX_Init();
#endif

#if ROBOT_2AXIS
  Robot_Axis2_Init();
#endif



}

void Robot_Config(void)
{


#if ROBOT_XX

   //trobot[0].part_index = 1;
   //trobot[0].axis_index = 1;
   //trobot[0].axis_offset = 0;
   //trobot[0].axis_degree = 0;
   //trobot[0].axis_rr = 121;

   //trobot[1].part_index = 2;
   //trobot[1].axis_index = 2;
   //trobot[1].axis_offset = 0;
   //trobot[1].axis_degree = 180;
   //trobot[1].axis_rr = 81;

   //trobot[2].part_index = 4;
   //trobot[2].axis_index = 3;
   //trobot[2].axis_offset = 0;
   //trobot[2].axis_degree = 0;
   //trobot[2].axis_rr = 81;

   Robot_XX_Config();
#endif

#if ROBOT_2AXIS
  //trobot[0].part_index = 1;
  //trobot[0].axis_index = 1;
  //trobot[0].axis_offset = 0;
  //trobot[0].axis_degree = 0;
  //trobot[0].axis_rr = 121;
  
  //trobot[1].part_index = 2;
  //trobot[1].axis_index = 2;
  //trobot[1].axis_offset = 0;
  //trobot[1].axis_degree = 180;
  //trobot[1].axis_rr = 81;









  Robot_Axis2_Config();
#endif
}


void Robot_Update(void)
{
  Robot_Update_CPX();

#if ROBOT_XX
  Robot_XX_Update();
#endif

#if ROBOT_2AXIS
  Robot_Axis2_Update();
#endif
}


void Robot_InvKin_Servo(void)
{
#if ROBOT_XX
  Robot_XX_InvKin_Servo();
#endif

#if ROBOT_2AXIS
  Robot_Axis2_InvKin_Servo();
#endif
}


void Robot_Set_Servo_Cmd(uint8_t index,operation_mode_t operation_mode)
{
#if ROBOT_XX
  Robot_XX_Set_Servo_Cmd( index, operation_mode);
#endif

#if ROBOT_2AXIS
  Robot_Axis2_Set_Servo_Cmd( index, operation_mode);
#endif
}


uint8_t Robot_Get_RunMode(void)
{
#if ROBOT_XX
  return Robot_XX_Get_RunMode();
#endif

#if ROBOT_2AXIS
  return Robot_Axis2_Get_RunMode();
#endif
}










