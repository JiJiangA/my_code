

#ifndef _ROBOT_XX_H
#define _ROBOT_XX_H


#define ROBOT_AXIS_NUM     3
#define ROBOT_MOROT_NUM    5



#define  ROBOT_XX_Z_INDEX          (uint8_t)1   //轴长度
#define  ROBOT_XX_DEG_INDEX        (uint8_t)2   //轴角度偏移量
#define  ROBOT_XX_TWITS_INDEX          (uint8_t)3   //Twits 轴移动角度


typedef struct
{
/*机器人*/
  /*机器人*/
  uint8_t robot_fault;
  uint8_t robot_fault_enable;    //机器人状态  使能 1:报警
  uint8_t robot_fault_index;     //机器人状态  索引 1:报警
  uint8_t robot_fault_cmd;       //机器人状态  指令 1:报警
  
  
  uint8_t robot_run_mode;      //Jog  Part 模式
  uint8_t robot_index;         

  uint32_t robot_status_word;  //读写 状态字 控制字
  uint32_t robot_control_word;


  /*工件*/  
  uint8_t part_z_index;        //工件指令索引
  uint8_t part_deg_index;      //工件指令索引
  uint8_t part_b_index;
  
  double part_z_max;  /*最大半径*/
  double part_z_min;  /*最小半径*/

  double cmd_part_z;                //
  double cmd_part_deg;                //
  double cmd_part_b;

  double actual_part_z;
  double actual_part_deg;
  double actual_part_b;


  float facter_part_z_cmd;       //指令乘以此值
  float facter_part_deg_cmd;     //指令乘以此值
  float facter_part_b_cmd;     //指令乘以此值

  

  float facter_part_z_actual;    //反馈乘以此值
  float facter_part_deg_actual;  //反馈乘以此值
  float facter_part_b_actual;  //反馈乘以此值



/*电机*/
  uint8_t  motor_index[ROBOT_MOROT_NUM];
  double   motor_offset[ROBOT_MOROT_NUM];
  uint16_t motor_deg[ROBOT_MOROT_NUM];
  float    motor_rr[ROBOT_MOROT_NUM];

  uint8_t *motor_enable[ROBOT_MOROT_NUM];

  uint32_t axis_ppr[ROBOT_MOROT_NUM];    //轴转一圈




  double cmd_motor_deg[ROBOT_MOROT_NUM];
  double cmd_motor_cnt[ROBOT_MOROT_NUM];
  
  double actual_motor_deg[ROBOT_MOROT_NUM];
  const double* pmotor_feedback[ROBOT_MOROT_NUM];   //指向电机位置反馈


  double facter_deg2motor[ROBOT_MOROT_NUM];                        //电机指令到角度因子
  double facter_motor2deg[ROBOT_MOROT_NUM];                        //电机指令到角度因子

  float L1;
  float L2;
  float L3;

  float D_2L1;//1/2L

 // uint8_t  flag_pos_update;        //位置更新标志位
}robot_xx_t;





void Robot_XX_Init(void);
void Robot_XX_Config(void);
void Robot_XX_InvKin_Servo(void);
void Robot_XX_Update(void);
void Robot_XX_Set_Servo_Cmd(uint8_t index,operation_mode_t operation_mode);



uint8_t Robot_XX_Get_RunMode(void);






#endif



























