
/*
双轴 晶圆搬运机器人

机器人1 晶圆扫描           只能当作增量式编码器使用

J1 J2 J3

Twist  J4 J5
*/


#include "stdint.h"
#include "robot.h"
#include "slave_master.h"
#include "math.h"
#include "bsp_math.h"


#include "sc_inverse_kinematics.h"
#include "sc_servo_manager.h"
#include "sc_servo_status.h"
#include "sc_servo_cmd.h"

#include "tune.h"
#include "tune_pos_loop.h"

#include "cm_mode.h"
#include "cm_encoder.h"

#include "robot_xx.h"

#include "bsp_rt.h"



extern servo_status_t tservo_status[];
extern slave_master_t tslave_master[];


extern servo_cmd_t gt_servos_cmd;

extern servos_t gt_servos;
extern float Test[];

extern robot_t trobot[];
robot_xx_t trobot_xx;


double Robot_XX_Get_Motor_Feedback(uint8_t index);
void Robot_XX_ForKin(double motor1_feadback,double motor2_feadback,double motor3_feadback,double* part_z,double* part_deg);
void Robot_XX_Set_JogMode_CmdFeedback(void);
void Robot_XX_Set_PartMode_CmdFeedback(void);
void Robot_XX_Set_Cmd_PartDisable(uint8_t index);



void Robot_XX_Set_RobotFalut(uint8_t fault_flage);











void Robot_XX_Init(void)
{
  double axis_ppr=0;
/*工件 索引配置*/ 
  trobot[0].part_index = 1;  //工件指令
  trobot[1].part_index = 2;
  trobot[2].part_index = 4;
/*轴 索引配置*/
  trobot[0].axis_index = 1;//机器人轴与协处理器轴索引匹配
  trobot[1].axis_index = 2;
  trobot[2].axis_index = 3;
  trobot[3].axis_index = 4;
  trobot[4].axis_index = 5;

/*机器人状态*/
  trobot_xx.robot_fault_index  =0;
  trobot_xx.robot_fault_enable =0;
  trobot_xx.robot_fault_cmd    =0;

  /*机器人*/
  trobot_xx.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;      //Jog  Part 模式
  trobot_xx.robot_index    = 1;         //读写 状态字 控制字

   trobot_xx.robot_status_word = 0;
   trobot_xx.robot_control_word =0;;

   
  /*工件*/
    trobot_xx.part_z_max=0;
  
    trobot_xx.cmd_part_z=0;                
    trobot_xx.cmd_part_deg=0;
    trobot_xx.cmd_part_b=0;
    
    trobot_xx.actual_part_z=0;
    trobot_xx.actual_part_deg=0;
    trobot_xx.actual_part_b=0;
    
    trobot_xx.part_z_index = 1;
    trobot_xx.part_deg_index = 2;
    trobot_xx.part_b_index = 3;

  /*电机*/
    for(uint8_t index=0;index<ROBOT_MOROT_NUM;index++)
    {
      trobot_xx.cmd_motor_deg[index]=0;
      trobot_xx.actual_motor_deg[index]=0;
      trobot_xx.cmd_motor_cnt[index]=0;

      trobot_xx.motor_index[index] = index+1;
      trobot_xx.motor_offset[index] = 0;
      trobot_xx.motor_deg[index] = 0;
      trobot_xx.motor_rr[index] = 1;
      trobot_xx.pmotor_feedback[index] = &trobot_xx.cmd_motor_cnt[index];

      axis_ppr = Encoder_Get_PPR(index)*trobot_xx.motor_rr[index];//机械一转多少个脉冲
      
      trobot_xx.facter_deg2motor[index] = axis_ppr/360.0;
      trobot_xx.facter_motor2deg[index] = 360.0/(double)axis_ppr;
      trobot_xx.axis_ppr[index] = (uint32_t)axis_ppr;
    }

    trobot_xx.L1 = 300;
    trobot_xx.L2 = 300;
    trobot_xx.L3 = 300;
    trobot_xx.D_2L1 = 0.5/trobot_xx.L1;



    
    trobot_xx.part_z_max = 550;
    trobot_xx.part_z_min = -550;
}

void Robot_XX_Config(void)
{
  uint32_t motor1_encoder=0;
  uint32_t motor2_encoder=0;
  uint32_t motor3_encoder=0;
  uint8_t index_error=0;
  float offset=0;
  double axis_ppr=0;
  uint32_t encoder_ppr=0;

  trobot_xx.robot_control_word = Master_Robot_Get_Control(trobot_xx.robot_index);
  trobot_xx.robot_status_word = Master_Robot_Get_Status(trobot_xx.robot_index);


  
/*检查索引*/
  /*主站指令索引*/
  for(uint8_t index=0;index<ROBOT_AXIS_NUM;index++)
  {
    if(trobot[index].part_index==0||trobot[index].part_index>8)
    {
       index_error=1;
    }
  }

  for(uint8_t index=0;index<ROBOT_MOROT_NUM;index++)
  {
    if(trobot[index].axis_index==0||trobot[index].axis_index>8)
    {
       index_error=1;
    }
  }

  /*索引没有错误*/
  if(index_error == 0)
  {
    trobot_xx.part_z_index   = trobot[0].part_index-1;
    trobot_xx.part_deg_index = trobot[1].part_index-1;
    trobot_xx.part_b_index   = trobot[2].part_index-1;
    
    trobot_xx.motor_index[0] = trobot[0].axis_index-1;
    trobot_xx.motor_index[1] = trobot[1].axis_index-1;
    trobot_xx.motor_index[2] = trobot[2].axis_index-1;
    trobot_xx.motor_index[3] = trobot[3].axis_index-1;
    trobot_xx.motor_index[4] = trobot[4].axis_index-1;


    for(uint8_t index=0;index<ROBOT_MOROT_NUM;index++)
    {
      trobot_xx.motor_enable[index] = &tservo_status[trobot_xx.motor_index[index]].amp_enable;
      //trobot_xx.pmotor_feedback[index] = &tslave_master[trobot_xx.motor_index[index]].servo_posfeedback;
      trobot_xx.pmotor_feedback[index] = &gt_servos_cmd.motor_pos_cmd[trobot_xx.motor_index[index]];

      trobot_xx.motor_deg[index]    = trobot[trobot_xx.motor_index[index]].axis_degree;
      trobot_xx.motor_offset[index] = trobot[trobot_xx.motor_index[index]].axis_offset;
      trobot_xx.motor_rr[index]     = trobot[trobot_xx.motor_index[index]].axis_rr;

      encoder_ppr = Encoder_Get_PPR(index);
      axis_ppr = (double)encoder_ppr*trobot_xx.motor_rr[index];
      
      trobot_xx.facter_deg2motor[index] = axis_ppr/360.0;
      trobot_xx.facter_motor2deg[index] = 360.0/axis_ppr;

      trobot_xx.axis_ppr[index] = (uint32_t)axis_ppr;
      /*将角度的偏移量加到*/
      offset = (float)trobot_xx.motor_deg[index]*trobot_xx.facter_deg2motor[index]; //角度偏移量设置
      trobot_xx.motor_offset[index] = -(trobot_xx.motor_offset[index]-offset);

    }

    //trobot_xx.motor_ppr[3] = Encoder_Get_PPR(3)*116.4046;
    //trobot_xx.motor_ppr[4] = Encoder_Get_PPR(4)*116.4046;
    //motor_rr = Encoder_Get_PPR(3)*116.4046;

    
    //trobot_xx.facter_deg2motor[3] = motor_rr/360.0;
    //trobot_xx.facter_motor2deg[3] = 1.0/trobot_xx.facter_deg2motor[3];

    //trobot_xx.facter_deg2motor[4] = motor_rr/360.0;
    //trobot_xx.facter_motor2deg[4] = 1.0/trobot_xx.facter_deg2motor[4];
    
    
    
    
    Robot_XX_Set_JogMode_CmdFeedback();

  }
  else
  {
    trobot_xx.robot_fault_index = 1;
    trobot_xx.robot_status_word |= ROBOT_STASUS_FAULT_INDEX_MASK;
  }


/*设置控制字模式*/
  trobot_xx.robot_control_word |=ROBOT_CONTROL_RUN_MODE_MASK;
  Master_Robot_Set_Control(trobot_xx.robot_index,trobot_xx.robot_control_word);
  Master_Robot_Set_Status(trobot_xx.robot_index, trobot_xx.robot_status_word);

}




/*
主站更新实时报告
*/


void Robot_XX_Update(void)
{
  int32_t axis_z=0;
  int32_t axis_deg=0;
  uint8_t index=0;
  uint32_t u32tmp=0;
  double motor1_encoder=0;
  double motor2_encoder=0;
  double motor3_encoder=0;

/*读取控制字状态字，实现控制字和更改状态*/
  trobot_xx.robot_control_word= Master_Robot_Get_Control(trobot_xx.robot_index);
  trobot_xx.robot_status_word = Master_Robot_Get_Status(trobot_xx.robot_index);

/*主站位置指令更新*/
  if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART) /*只有在part模式下才能更改电机指令*/
  {
    if(trobot_xx.robot_control_word&ROBOT_CONTROL_CMD_UPDATA_MASK) //协处理器更新位置指令
    {
      /*位置更换禁止中断，防止指令和位置不一致*/      
      DISABLE_GLOBA_IRQ();
      Robot_XX_Set_PartMode_CmdFeedback();
      //Slave_Master_Set_Master_Poscmd(trobot_xx.part_deg_index, trobot_xx.actual_part_deg);
      //Slave_Master_Set_Master_Poscmd(trobot_xx.part_z_index, trobot_xx.actual_part_z);
      ENABLE_GLOBA_IRQ();
      /*清除控制字*/
      trobot_xx.robot_control_word = trobot_xx.robot_control_word&(~ROBOT_CONTROL_CMD_UPDATA_MASK);
      /*设置状态字*/
      trobot_xx.robot_status_word |= ROBOT_CONTROL_CMD_UPDATA_MASK;
      //trobot_xx.flag_pos_update = 1;
    }
  }

/*控制字      运行模式*/
  if(trobot_xx.robot_control_word&ROBOT_CONTROL_RUN_MODE_MASK)  
  {
    //JOG 模式
    if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART)
    {
      /*位置更换禁止中断*/
      DISABLE_GLOBA_IRQ();
      Robot_XX_Set_JogMode_CmdFeedback();
      trobot_xx.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;
      ENABLE_GLOBA_IRQ();
    }
  }
  else
  {
    //part 模式
    if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_JOG)
    {
      /*切换到part 模式下查看是否有致命错误*/
      if(trobot_xx.robot_fault_index!=0)       /*伺服索引致命错误, 在part 模式下是指令错误*/
      {
        trobot_xx.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;
        trobot_xx.robot_status_word |= ROBOT_CONTROL_RUN_MODE_MASK;
      }
      else
      {
        /*位置更换禁止中断*/
        DISABLE_GLOBA_IRQ();
        Robot_XX_Set_PartMode_CmdFeedback();
        trobot_xx.robot_run_mode = ROBOT_CONTROL_RUN_MODE_PART;
        ENABLE_GLOBA_IRQ();
      }
    }
  }



  if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART)
  {
    //错误检测
     //检测机器人模块下机器人   使能状态是否有不一致
     if(*trobot_xx.motor_enable[0] == *trobot_xx.motor_enable[1])
     {
       trobot_xx.robot_fault_enable=0;
     }
     else
     {
        trobot_xx.robot_fault_enable=1;
     }
    }
  else
  {
    trobot_xx.robot_fault_enable=0;
    trobot_xx.robot_fault_cmd = 0;
  }


  //机器人使能报错
  if(trobot_xx.robot_fault_enable == 0)
  {
    trobot_xx.robot_status_word &= (~ROBOT_STASUS_FAULT_NOTALL_MOROT_ENABLE_MASK);
  }
  else
  {
    trobot_xx.robot_status_word |= ROBOT_STASUS_FAULT_NOTALL_MOROT_ENABLE_MASK;
  }


  if(trobot_xx.robot_fault_cmd == 0) /*指令错误*/
  {
    trobot_xx.robot_status_word &= (~ROBOT_STASUS_FAULT_CMD_ERROE_MASK);
  }
  else
  {
    trobot_xx.robot_status_word |= ROBOT_STASUS_FAULT_CMD_ERROE_MASK;
  }


  //机器人索引报错
  if(trobot_xx.robot_fault_index==0)
  {
    trobot_xx.robot_status_word &= (~ROBOT_STASUS_FAULT_INDEX_MASK);
  }
  else
  {
    trobot_xx.robot_status_word |= ROBOT_STASUS_FAULT_INDEX_MASK;
  }



  //机器人总报错
  if(trobot_xx.robot_status_word &ROBOT_STATUS_FAULT_MASK)
  {    
    trobot_xx.robot_status_word |= ROBOT_STASUS_FAULT_MASK;
  }
  else
  {
    trobot_xx.robot_status_word &= (~ROBOT_STASUS_FAULT_MASK);
  }

  /*状态     运行模式更新*/
  if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_JOG)
  {
    trobot_xx.robot_status_word |=ROBOT_CONTROL_RUN_MODE_MASK;
  }
  else
  {
    trobot_xx.robot_status_word &=(~ROBOT_CONTROL_RUN_MODE_MASK);
  }

  motor1_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[0]);
  motor1_encoder = motor1_encoder + trobot_xx.motor_offset[0];
  
  motor2_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[1]);
  motor2_encoder = motor2_encoder + trobot_xx.motor_offset[1];

  motor3_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[2]);
  motor3_encoder = motor3_encoder + trobot_xx.motor_offset[2];


  Robot_XX_ForKin(motor1_encoder,motor2_encoder,motor3_encoder,&trobot_xx.actual_part_z,&trobot_xx.actual_part_deg);
  /*位置更新*/

  Master_Robot_Set_Pos(1,trobot_xx.actual_part_z);
  Master_Robot_Set_Pos(2,trobot_xx.actual_part_deg);


  Master_Robot_Set_Control(trobot_xx.robot_index,trobot_xx.robot_control_word);
  Master_Robot_Set_Status(trobot_xx.robot_index, trobot_xx.robot_status_word);

}















/*运动学计算
输入 电机1 脉冲指令
输入 电机2 脉冲指令
输入 电机3 脉冲指令

输出 工件坐标 z  高度
输出 工件坐标 deg 旋转角度
*/
void Robot_XX_ForKin(double motor1_feadback,double motor2_feadback,double motor3_feadback,double* part_z,double* part_deg)
{
  double tmp_rad=0;
/*反馈转换为角度*/
  trobot_xx.actual_motor_deg[0] = motor1_feadback*trobot_xx.facter_motor2deg[0];//
  trobot_xx.actual_motor_deg[1] = motor2_feadback*trobot_xx.facter_motor2deg[1];//
  trobot_xx.actual_motor_deg[2] = motor3_feadback*trobot_xx.facter_motor2deg[2];//


/*角度计算工件坐标*/
  tmp_rad = trobot_xx.actual_motor_deg[0]*DEG_TO_RAD;
  *part_z = 2*trobot_xx.L1*sinf(tmp_rad);
  *part_deg = trobot_xx.actual_motor_deg[2];
}

/*运动学计算
输入 工件坐标 z  高度
输入 工件坐标 deg 旋转角度

输出 电机1 脉冲指令
输出 电机2 脉冲指令
输出 电机3 脉冲指令
*/
void Robot_XX_InvKin(double master_part_z,double master_part_deg,double master_part_b,
double* motor1_cmd,double* motor2_cmd,double* motor3_cmd,double* motor4_cmd,double* motor5_cmd)
{
  double motor1_rad=0;
  double z_tmp=0;
  /*主站位置转换*/
  trobot_xx.cmd_part_z   = master_part_z;
  trobot_xx.cmd_part_deg = master_part_deg;
  trobot_xx.cmd_part_b   =  master_part_b;

  /*指令超过限制报错*/
//*如果cmd_part_z大于2倍的Z
   if(trobot_xx.cmd_part_z>trobot_xx.part_z_max||trobot_xx.cmd_part_z<trobot_xx.part_z_min)
   {
     trobot_xx.robot_fault_cmd =1;
     return;
   }
   else
   {
     trobot_xx.robot_fault_cmd =0;
   }

  motor1_rad = trobot_xx.cmd_part_z*trobot_xx.D_2L1;
  trobot_xx.cmd_motor_deg[0] = asinf(motor1_rad)*RAD_TO_DEG;             //J1 角度
  trobot_xx.cmd_motor_deg[1] = -2*trobot_xx.cmd_motor_deg[0];         //J2 角度
  trobot_xx.cmd_motor_deg[2] = trobot_xx.cmd_motor_deg[0] + trobot_xx.cmd_part_deg;//J3 角度

  trobot_xx.cmd_motor_deg[3] = trobot_xx.cmd_part_b;
  trobot_xx.cmd_motor_deg[4] = trobot_xx.cmd_part_b;



  *motor1_cmd = trobot_xx.cmd_motor_deg[0] * trobot_xx.facter_deg2motor[0] - trobot_xx.motor_offset[0];//
  *motor2_cmd = trobot_xx.cmd_motor_deg[1] * trobot_xx.facter_deg2motor[1] - trobot_xx.motor_offset[1];//
  *motor3_cmd = trobot_xx.cmd_motor_deg[2] * trobot_xx.facter_deg2motor[2] - trobot_xx.motor_offset[2];//

  *motor4_cmd = trobot_xx.cmd_motor_deg[3] * trobot_xx.facter_deg2motor[3] - trobot_xx.motor_offset[3];//
  *motor5_cmd = trobot_xx.cmd_motor_deg[4] * trobot_xx.facter_deg2motor[4] - trobot_xx.motor_offset[4];//

}







/*逆运动学计算*/
void Robot_XX_InvKin_Servo(void)
{
  /*只有在part 模式下，才进行逆运动学计算*/
  if(trobot_xx.robot_run_mode==ROBOT_CONTROL_RUN_MODE_PART)
  {
    if(trobot_xx.robot_fault !=0)return;
    /*逆运动学计算，根据主站指令计算出电机指令*/
    Robot_XX_InvKin(*gt_invkinematics.poscmd_master[trobot_xx.part_z_index],*gt_invkinematics.poscmd_master[trobot_xx.part_deg_index],*gt_invkinematics.poscmd_master[trobot_xx.part_b_index],
    &trobot_xx.cmd_motor_cnt[0],&trobot_xx.cmd_motor_cnt[1],&trobot_xx.cmd_motor_cnt[2],&trobot_xx.cmd_motor_cnt[3],&trobot_xx.cmd_motor_cnt[4]);

    for(uint8_t index=0;index<MAX_AXIS_NUM;index++)
    {
      if(index == trobot_xx.motor_index[0])
      {
        *gt_invkinematics.poscmd_motor[index] = trobot_xx.cmd_motor_cnt[0];
      }
      else if(index == trobot_xx.motor_index[1])
      {
        *gt_invkinematics.poscmd_motor[index] = trobot_xx.cmd_motor_cnt[1];
      }
      else if(index == trobot_xx.motor_index[2])
      {
        *gt_invkinematics.poscmd_motor[index] = trobot_xx.cmd_motor_cnt[2];
      }
      else if(index == trobot_xx.motor_index[3])
      {
        *gt_invkinematics.poscmd_motor[index] = trobot_xx.cmd_motor_cnt[3];
      }
      else if(index == trobot_xx.motor_index[4])
      {
        *gt_invkinematics.poscmd_motor[index] = trobot_xx.cmd_motor_cnt[4];
      }
      else{
        *gt_invkinematics.poscmd_motor[index] = *gt_invkinematics.poscmd_master[index];
      }
    }
  }
  else
  {
    for(uint8_t index=0;index<MAX_AXIS_NUM;index++)
    {
      *gt_invkinematics.poscmd_motor[index] = *gt_invkinematics.poscmd_master[index];
    }

  }
}



/*指令更新*/
void Robot_XX_Set_Servo_Cmd(uint8_t index,operation_mode_t operation_mode)
{
  uint8_t enable = ServoStatus_Get_AmpEnable(index);

  if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART) 
  {
    
    if(enable == 1)
    {
      if(gt_servos.run_mode[index] == SERVO_TUNE_MODE)
      {
        *gt_servos.pservo_poscmd[index] = Get_Tune_Pos(index); /*调试位置指令*/           
        Slave_Master_Set_Master_Poscmd(index,*gt_servos.pservo_poscmd[index]);
      }
    }
      /*工件指令模式*/
     if(enable == 0)
     {
       if(index == trobot_xx.motor_index[0]||index == trobot_xx.motor_index[1]||index == trobot_xx.motor_index[2]
         ||index == trobot_xx.motor_index[3]||index == trobot_xx.motor_index[4])
        {
           Robot_XX_Set_Cmd_PartDisable(index);  /*反馈等于工件指令*/
        }
        else
        {
          Slave_Master_Set_Master_Poscmd(index,*gt_servos.pservo_posfeedback[index]); /*指令等于反馈*/
        }
     }
  }
  else /*Jog 命令模式*/
  {
    if(enable == 1)
    {
      if(gt_servos.run_mode[index] == SERVO_TUNE_MODE)
      {
        *gt_servos.pservo_poscmd[index] = Get_Tune_Pos(index); /*调试位置指令*/           
        Slave_Master_Set_Master_Poscmd(index,*gt_servos.pservo_poscmd[index]);
      }
    }
    else
    {
      Slave_Master_Set_Master_Poscmd(index,*gt_servos.pservo_posfeedback[index]);
    }
  }
}




/*
工件模式下 根据反馈设置  主站工件指令，反馈值

根据电机反馈角度           算出工件坐标
*/
void Robot_XX_Set_PartMode_CmdFeedback(void)
{
  double motor1_encoder=0;
  double motor2_encoder=0;
  double motor3_encoder=0;

  motor1_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[0]);
  motor1_encoder = motor1_encoder + trobot_xx.motor_offset[0];
  
  motor2_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[1]);
  motor2_encoder = motor2_encoder + trobot_xx.motor_offset[1];
    
  motor3_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[2]);
  motor3_encoder = motor3_encoder + trobot_xx.motor_offset[2];


/*************************************************************************************/
  motor1_encoder=0;
  motor2_encoder=0;
  motor3_encoder=0;


/*************************************************************************************/
/*根据编码器反馈计算工件坐标*/
  Robot_XX_ForKin(motor1_encoder,motor2_encoder,motor3_encoder,&trobot_xx.actual_part_z,&trobot_xx.actual_part_deg);

  Slave_Master_Set_Master_Poscmd(trobot_xx.part_deg_index,trobot_xx.actual_part_deg);
  Slave_Master_Set_Master_Poscmd(trobot_xx.part_z_index,  trobot_xx.actual_part_z);

}

/*
Jog 模式

主站  指令 和 
*/
void Robot_XX_Set_JogMode_CmdFeedback(void)
{

  for(uint8_t index=0;index<ROBOT_MOROT_NUM;index++)
  {
    Slave_Master_Set_Master_Poscmd(trobot_xx.motor_index[index],*trobot_xx.pmotor_feedback[index]);
  }
}

uint32_t Global_Get_ServoCnt(void);
/*
Part 模式下 Disable应该表现的样子
*/
void Robot_XX_Set_Cmd_PartDisable(uint8_t index)
{
  //每个伺服周期执行一次，以伺服周期计数作为标志位
  static uint32_t servo_cnt_flage[2];
  double motor1_encoder=0;
  double motor2_encoder=0;
  double motor3_encoder=0;

  
  servo_cnt_flage[0] = Global_Get_ServoCnt();
  if(servo_cnt_flage[0]==servo_cnt_flage[1])
  {
    return;
  }
  else
  {
    servo_cnt_flage[1] = servo_cnt_flage[0];
  }
  /*全部使能才不更新指令*/

    motor1_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[0]);
    motor1_encoder = motor1_encoder + trobot_xx.motor_offset[0];
    
    motor2_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[1]);
    motor2_encoder = motor2_encoder + trobot_xx.motor_offset[1];
      
    motor3_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[2]);
    motor3_encoder = motor3_encoder + trobot_xx.motor_offset[2];
  
  /*根据编码器反馈计算工件坐标*/
    Robot_XX_ForKin(motor1_encoder,motor2_encoder,motor3_encoder,&trobot_xx.actual_part_z,&trobot_xx.actual_part_deg);
    Slave_Master_Set_Master_Poscmd(trobot_xx.part_deg_index,trobot_xx.actual_part_deg);
    Slave_Master_Set_Master_Poscmd(trobot_xx.part_z_index,  trobot_xx.actual_part_z);
}


/*
此处的索引时 轴的索引，不是协处理器的索引，是轴的索引
*/
double Robot_XX_Get_Motor_Feedback(uint8_t index)
{
  return *trobot_xx.pmotor_feedback[index];
}


uint8_t Robot_XX_Get_RunMode(void)
{
  return trobot_xx.robot_run_mode;
}


/*
安全保护
实际指令
*/
/*出现错误调用此函数*/
void Robot_XX_Set_RobotFalut(uint8_t fault_flage)
{
  if(trobot_xx.robot_fault_index==1)return;
  /*计算行程错误*/
 if(fault_flage==0)
 {
   ServoStatus_Set_StatusWord(trobot_xx.motor_index[0],SERVOSTATUS_FALULT_ROBOT_MOTOR_MASK);
   ServoStatus_Set_StatusWord(trobot_xx.motor_index[1],SERVOSTATUS_FALULT_ROBOT_MOTOR_MASK);
   ServoStatus_Set_StatusWord(trobot_xx.motor_index[2],SERVOSTATUS_FALULT_ROBOT_MOTOR_MASK);
 }
 else
 {
   ServoStatus_Reset_StatusWord(trobot_xx.motor_index[0],SERVOSTATUS_FALULT_ROBOT_MOTOR_MASK);
   ServoStatus_Reset_StatusWord(trobot_xx.motor_index[1],SERVOSTATUS_FALULT_ROBOT_MOTOR_MASK);
   ServoStatus_Reset_StatusWord(trobot_xx.motor_index[2],SERVOSTATUS_FALULT_ROBOT_MOTOR_MASK);
 }
}











