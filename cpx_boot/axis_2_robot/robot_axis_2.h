

#ifndef _ROBOT_AXIS_2_H
#define _ROBOT_AXIS_2_H

#define ROBOT_AXIS_NUM     2

#define ROBOT_MOROT_NUM    2


/*机器人实际位置更新索引*/

#define  ROBOT_AXIS2_DEG_INDEX   (uint8_t)3   //(uint8_t)1
#define  ROBOT_AXIS2_R_INDEX     (uint8_t)4   //(uint8_t)2

/*
机器人


工件




电机




*/





typedef struct
{
  /*机器人*/
  uint8_t robot_fault;
  uint8_t robot_fault_enable;    //机器人状态  使能 1:报警
  uint8_t robot_fault_index;     //机器人状态  索引 1:报警
  uint8_t robot_fault_cmd;       //机器人状态  指令 1:报警
  
  
  uint8_t robot_run_mode;      //Jog  Part 模式
  uint8_t robot_index;         //读写 状态字 控制字

  uint32_t robot_status_word;
  uint32_t robot_control_word;
  
  /*工件*/  
  uint8_t part_r_index;        //工件指令索引2
  uint8_t part_deg_index;      //工件指令索引1


  
  double part_r_max;  /*最大半径*/
  double part_r_min;  /*最小半径*/
  
  double cmd_part_r;                //工件指令
  double cmd_part_deg;              //工件指令
  
  double actual_part_r;             //工件实际
  double actual_part_deg;           //工件实际

  double actual_part_r1;             //工件实际
  double actual_part_deg1;           //工件实际



   uint8_t updata_partmode_flag;       //Jog 转到part 模式标志位       
   uint16_t updata_cnt;                //更新计数
  //float facter_part_r_cmd;       //指令乘以此值         指令因子
  //float facter_part_deg_cmd;     //指令乘以此值         指令因子
  
  //float facter_part_r_actual;    //反馈乘以此值         反馈因子
 // float facter_part_deg_actual;  //反馈乘以此值         反馈因子
  
  /*电机*/
  uint8_t motor_index[ROBOT_MOROT_NUM];
  double motor_offset[ROBOT_MOROT_NUM];
  uint16_t motor_deg[ROBOT_MOROT_NUM];
  float motor_rr[ROBOT_MOROT_NUM];


  
  uint8_t *motor_enable[ROBOT_MOROT_NUM];
  uint32_t axis_ppr[ROBOT_MOROT_NUM];//电机转一圈的cts数  负载转一圈的cts
  
  
  double cmd_motor_deg[ROBOT_MOROT_NUM];
  double cmd_motor_cnt[ROBOT_MOROT_NUM];
  
  double actual_motor_deg[ROBOT_MOROT_NUM];
  const double* pmotor_feedback[ROBOT_MOROT_NUM];   //指向电机位置反馈
  
  
  double facter_deg2motor[ROBOT_MOROT_NUM];                        //角度到电机指令因子
  double facter_motor2deg[ROBOT_MOROT_NUM];                        //电机指令到角度因子
  
  float L1;
  float L2;
  float L3;

  float D2L1;  //L1 导数方便计算   除转乘  0.5/L1
  float L1X2;  //L1 导数方便计算   除转乘  0.5/L1


}robot_axis2_t;
























void Robot_Axis2_Init(void);
void Robot_Axis2_Config(void);
void Robot_Axis2_Update(void);
void Robot_Axis2_Set_Servo_Cmd(uint8_t index,operation_mode_t operation_mode);
void Robot_Axis2_InvKin_Servo(void);
uint8_t Robot_Axis2_Get_RunMode(void);





#endif






















