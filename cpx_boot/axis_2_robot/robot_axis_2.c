/*
双轴 晶圆搬运机器人
偏移和角度

绝对值编码器反馈
机器人的零点的位置，对应 正运动学和逆运动学的角度，和当前绝对值编码器的值
角度和正运动学和逆运动学公式有关，如果公式中没有，则角度为0即可




对于绝对值编码器的机器人，偏移量才有意义，才能记住机器人的当前位置。
偏移量是针对机器人的特定角度，可以说是与某个角度相关。


偏移量 = -(偏移量-角度对应的cts)

当运动到此位置时，为对应的角度

正运动学计算时         当前编码器值+偏移量
逆运动学计算时         当前编码器值-偏移量
给出工件指令，得到的电机指令，是没有偏移的，所以因


//确定初始位
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
#include "cm_encoder_abs.h"

#include "robot_axis_2.h"
  
#include "bsp_rt.h"
  
extern servo_status_t tservo_status[];
extern slave_master_t tslave_master[];
extern servos_t gt_servos;
extern robot_t trobot[];

extern float Test[];

robot_axis2_t trobot_axis2;

/*
机器人正运动学计算，根据反馈计算，工件当前的位置
*/
void Robot_Axis2_ForKin(double m1_deg,double m2_deg,double* axis_deg,double* axis_r);

/*
将机器人运动模式改为Part模式
需要根据反馈修改 主站的part指令
*/
void Robot_Axis2_Set_PartMode_CmdFeedback(void);


/*
改为JOG模式，将主站指令改为当前的反馈
*/
void Robot_Axis2_Set_JogMode_CmdFeedback(void);


void Robot_Axis2_Set_RobotFalut(uint8_t fault_flage);


/*
part 模式下 disable 时电机指令设置
*/
void Robot_Axis2_Set_Cmd_PartDisable(uint8_t index);

/*
电机反馈，累加的反馈，所以不能改变反馈
*/
int32_t Robot_Axis2_Get_Motor_Feedback(uint8_t index);

//主站位置反馈
void Robot_Updata_ActualPartPos_Master(void);



/*机器人初始化*/
void Robot_Axis2_Init(void)
{
  //double facter_cmd=4096;
  double axis_ppr=0;
  /*工件 索引配置*/
  trobot[0].part_index = 1;
  trobot[1].part_index = 2;
  /*轴 索引配置*/
  trobot[0].axis_index = 1;
  trobot[1].axis_index = 2;


  /*机器人*/
  /*机器人状态*/
  trobot_axis2.robot_fault_index  =0;
  trobot_axis2.robot_fault_enable =0;
  trobot_axis2.robot_fault_cmd    =0;

  /*机器人*/

  trobot_axis2.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;      //Jog  Part 模式
  trobot_axis2.robot_index    = 2;         //读写 状态字 控制字

  trobot_axis2.robot_status_word = 0;
  trobot_axis2.robot_control_word =0;;

  /*工件*/

  trobot_axis2.cmd_part_r=0;                //
  trobot_axis2.cmd_part_deg=0;

  trobot_axis2.actual_part_r=0;
  trobot_axis2.actual_part_deg=0;


  trobot_axis2.part_deg_index=0;
  trobot_axis2.part_r_index=0;


  trobot_axis2.updata_partmode_flag=0;       //Jog 转到part 模式标志位       
  trobot_axis2.updata_cnt=0;                //更新计数

  //trobot_axis2.facter_part_r_cmd=0.000244140625;       //指令乘以此值
  //trobot_axis2.facter_part_deg_cmd=0.000244140625;     //指令乘以此值

  //trobot_axis2.facter_part_r_actual=facter_cmd;    //反馈乘以此值
 // trobot_axis2.facter_part_deg_actual=facter_cmd;  //反馈乘以此值

  //trobot_axis2.facter_part_r_cmd=1.0/facter_cmd;       //指令乘以此值
  //trobot_axis2.facter_part_deg_cmd=1.0/facter_cmd;;     //指令乘以此值


  /*电机*/
    for(uint8_t index=0;index<ROBOT_MOROT_NUM;index++)
    {
      trobot_axis2.cmd_motor_deg[index]   = 0;
      trobot_axis2.actual_motor_deg[index]= 0;
      trobot_axis2.cmd_motor_cnt[index]   = 0;

      trobot_axis2.motor_index[index]  = index+1;
      trobot_axis2.motor_offset[index] = 0;
      trobot_axis2.motor_deg[index] = 0;
      trobot_axis2.motor_rr[index] = 1;
      trobot_axis2.pmotor_feedback[index] = &trobot_axis2.cmd_motor_cnt[index];

      
      axis_ppr = Encoder_Get_PPR(index)*trobot_axis2.motor_rr[index];
      
      trobot_axis2.facter_deg2motor[index] = axis_ppr/360.0;
      trobot_axis2.facter_motor2deg[index] = 360.0/(double)axis_ppr;
      trobot_axis2.axis_ppr[index] = axis_ppr;
    }
    trobot_axis2.L1 = 190.5;
    trobot_axis2.L2 = 190.5;
    trobot_axis2.L3 = 284.1;

    trobot_axis2.part_r_max=trobot_axis2.L3 + 2*trobot_axis2.L1*cosf(15*DEG_TO_RAD);
    trobot_axis2.part_r_min=trobot_axis2.L3 - 2*trobot_axis2.L1*cosf(15*DEG_TO_RAD);



    trobot_axis2.D2L1 = 0.5/ trobot_axis2.L1;
    trobot_axis2.L1X2 =  trobot_axis2.L1*2;

}



/*机器人配置*/
void Robot_Axis2_Config(void)
{
  uint32_t motor1_encoder=0;
  uint32_t motor2_encoder=0;
  uint32_t motor3_encoder=0;
  uint8_t index_error=0;
  float offset=0;
  double axis_ppr=0;
  uint32_t encoder_ppr=0;
  
  trobot_axis2.robot_control_word = Master_Robot_Get_Control(trobot_axis2.robot_index);
  trobot_axis2.robot_status_word = Master_Robot_Get_Status(trobot_axis2.robot_index);

  //检查索引是否错误
  for(uint8_t index=0;index<ROBOT_AXIS_NUM;index++)
  {
    if(trobot[index].part_index==0||trobot[index].part_index>2)
    {
      index_error=1;
    }
  }

  for(uint8_t index=0;index<ROBOT_MOROT_NUM;index++)
  {
    if(trobot[index].axis_index==0||trobot[index].axis_index>2)
    {
      index_error=1;
    }
  }

  /*索引没有错误*/
  if(index_error == 0)
  {

  
    trobot_axis2.part_deg_index = trobot[0].part_index-1;
    trobot_axis2.part_r_index   = trobot[1].part_index-1;
   
    
    trobot_axis2.motor_index[0] = trobot[0].axis_index-1;
    trobot_axis2.motor_index[1] = trobot[1].axis_index-1;



    for(uint8_t index=0;index<ROBOT_MOROT_NUM;index++)
    {
      trobot_axis2.motor_enable[index] = &tservo_status[trobot_axis2.motor_index[index]].amp_enable;

      trobot_axis2.motor_deg[index]    = trobot[trobot_axis2.motor_index[index]].axis_degree;
      trobot_axis2.motor_offset[index] = (double)trobot[trobot_axis2.motor_index[index]].axis_offset;
      trobot_axis2.motor_rr[index]     = trobot[trobot_axis2.motor_index[index]].axis_rr;
      
      trobot_axis2.pmotor_feedback[index] = &tslave_master[trobot_axis2.motor_index[index]].servo_posfeedback;
      
      encoder_ppr = Encoder_Get_PPR(trobot_axis2.motor_index[index]);

      axis_ppr = (double)encoder_ppr*trobot_axis2.motor_rr[index];

      trobot_axis2.facter_deg2motor[index] = axis_ppr/360.0;
      trobot_axis2.facter_motor2deg[index] = 360.0/axis_ppr;

      trobot_axis2.axis_ppr[index] = axis_ppr;

      /*将角度的偏移量加到*/
      offset = (float)trobot_axis2.motor_deg[index]*trobot_axis2.facter_deg2motor[index]; //角度偏移量设置
      trobot_axis2.motor_offset[index] = -(trobot_axis2.motor_offset[index]-offset);
    }


    
    /*直驱      初始化反馈和主站指令*/
   // motor1_encoder = Encoder_ABS_Get_Value(trobot_axis2.motor_index[0]);
    //motor1_encoder = (motor1_encoder + trobot_axis2.motor_offset[0]+trobot_axis2.axis_ppr[0])%trobot_axis2.axis_ppr[0];
    
   // motor2_encoder = Encoder_ABS_Get_Value(trobot_axis2.motor_index[1]);
    //motor2_encoder = (motor2_encoder + trobot_axis2.motor_offset[1]+trobot_axis2.axis_ppr[1])%trobot_axis2.axis_ppr[1];
    //假设 零点位置   q1 90 q2 0度

    //Jog 模式 设置反馈和指令

    //motor1_encoder = trobot_axis2.axis_ppr[0]/4;
    //motor2_encoder = 0;

    //Encoder_ABS_Set_Value(0,motor1_encoder);

    //Slave_Master_Set_FeedBack(trobot_axis2.motor_index[0],(double)motor1_encoder);
    //Slave_Master_Set_FeedBack(trobot_axis2.motor_index[1],(double)motor2_encoder);
    //默认时Jog模式
    Robot_Axis2_Set_JogMode_CmdFeedback();

    
    //Robot_Axis2_ForKin(motor1_encoder,motor2_encoder,&trobot_axis2.actual_part_deg,&trobot_axis2.actual_part_r);

    //part  指令设置
    //Slave_Master_Set_Master_Poscmd(trobot_axis2.part_deg_index,trobot_axis2.actual_part_deg);
    //Slave_Master_Set_Master_Poscmd(trobot_axis2.part_r_index,trobot_axis2.actual_part_r);

  }
  else
  {
    trobot_axis2.robot_fault_index = 1;
    trobot_axis2.robot_status_word |= ROBOT_STASUS_FAULT_INDEX_MASK;
  }

  
  if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_JOG)
  {
    trobot_axis2.robot_control_word |=ROBOT_CONTROL_RUN_MODE_MASK;
  }
  else
  {
    trobot_axis2.robot_control_word &=(~ROBOT_CONTROL_RUN_MODE_MASK);
  }
  Master_Robot_Set_Control(trobot_axis2.robot_index,trobot_axis2.robot_control_word);
  Master_Robot_Set_Status(trobot_axis2.robot_index, trobot_axis2.robot_status_word);

}





/*
机器人 状态更新
*/
void Robot_Axis2_Update(void)
{
  int32_t axis_r=0;
  int32_t axis_deg=0;
  
  double motor1_encoder=0;
  double motor2_encoder=0;

  uint32_t u32tmp=0;


/*读取控制字状态字，实现控制字和更改状态*/


  trobot_axis2.robot_control_word= Master_Robot_Get_Control(trobot_axis2.robot_index);
  trobot_axis2.robot_status_word = Master_Robot_Get_Status(trobot_axis2.robot_index);



/*控制字      运行模式*/
  if(trobot_axis2.robot_control_word&ROBOT_CONTROL_RUN_MODE_MASK)  //如果时Jog 模式
  {
    //Part 模式切换到 Jog模式   切换到jog模式可以将机器人的报错清除
    if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART)
    {      
      /*位置更换禁止中断*/
      DISABLE_GLOBA_IRQ();
      Robot_Axis2_Set_JogMode_CmdFeedback();
      trobot_axis2.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;
      trobot_axis2.robot_fault_enable =0;
      trobot_axis2.robot_fault_cmd    =0;
      ENABLE_GLOBA_IRQ();
    }
  }
  else
  {
    //Jog 模式切换到 Part模式
    if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_JOG)
    {
      /*切换到part 模式下查看是否有致命错误*/
      if(trobot_axis2.robot_fault==0)       /*伺服索引致命错误, 在part 模式下是指令错误*/
      {
        /*位置更换禁止中断*/
        DISABLE_GLOBA_IRQ();
        Robot_Axis2_Set_PartMode_CmdFeedback();
        trobot_axis2.robot_run_mode = ROBOT_CONTROL_RUN_MODE_PART;
        ENABLE_GLOBA_IRQ();
        trobot_axis2.updata_partmode_flag=1;
        trobot_axis2.updata_cnt=0;
      }
    }
  }



//错误检查
  /*使能检测*/
  if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART)
  {
    //错误检测
     //检测机器人模块下机器人   使能状态是否有不一致
     if(*trobot_axis2.motor_enable[0] == *trobot_axis2.motor_enable[1])
     {
       trobot_axis2.robot_fault_enable=0;
     }
     else
     {
        trobot_axis2.robot_fault_enable=1;
     }
  }
  else
  {
    trobot_axis2.robot_fault_enable=0;
    trobot_axis2.robot_fault_cmd = 0;
  }


  //机器人使能报错
  if(trobot_axis2.robot_fault_enable == 0)
  {
    trobot_axis2.robot_status_word &= (~ROBOT_STASUS_FAULT_NOTALL_MOROT_ENABLE_MASK);
  }
  else
  {
    trobot_axis2.robot_status_word |= ROBOT_STASUS_FAULT_NOTALL_MOROT_ENABLE_MASK;
  }
  
  
  //机器人指令报错
  if(trobot_axis2.robot_fault_cmd == 0) /*指令错误*/
  {
    trobot_axis2.robot_status_word &= (~ROBOT_STASUS_FAULT_CMD_ERROE_MASK);
  }
  else
  {
    trobot_axis2.robot_status_word |= ROBOT_STASUS_FAULT_CMD_ERROE_MASK;
  }


  //机器人索引报错
  if(trobot_axis2.robot_fault_index==0)
  {
    trobot_axis2.robot_status_word &= (~ROBOT_STASUS_FAULT_INDEX_MASK);
  }
  else
  {
    trobot_axis2.robot_status_word |= ROBOT_STASUS_FAULT_INDEX_MASK;
  }


  //机器人总报错
  if(trobot_axis2.robot_status_word &ROBOT_STATUS_FAULT_MASK)
  {    
    /*发生错误切换到Jog 模式控制字也改变*/
    trobot_axis2.robot_status_word |= ROBOT_STASUS_FAULT_MASK;
    //trobot_axis2.robot_control_word |= ROBOT_CONTROL_RUN_MODE_MASK;
    
    //Robot_Axis2_Set_RobotFalut(1);
    //trobot_axis2.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;
  }
  else
  {
    trobot_axis2.robot_status_word &= (~ROBOT_STASUS_FAULT_MASK);
  }

  /***************        状态 显示            *********/
  /*状态     运行模式更新*/
  if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_JOG)
  {
    trobot_axis2.robot_status_word |=ROBOT_CONTROL_RUN_MODE_MASK;
  }
  else
  {
    trobot_axis2.robot_status_word &=(~ROBOT_CONTROL_RUN_MODE_MASK);
  }

  //控制字状态字更新
  Master_Robot_Set_Control(trobot_axis2.robot_index,trobot_axis2.robot_control_word);
  Master_Robot_Set_Status(trobot_axis2.robot_index, trobot_axis2.robot_status_word);


  

  //part 实际位置反馈到主站
  if(trobot_axis2.updata_partmode_flag==0)  //如果切换到part模式，将等待一段时间再更新part 位置信息到主站
  {
    Robot_Updata_ActualPartPos_Master();
  }
  else
  {
    trobot_axis2.updata_cnt++;
    if(trobot_axis2.updata_cnt>50)
    {
      trobot_axis2.updata_cnt=0;
      trobot_axis2.updata_partmode_flag=0;
    }

  }
}



//计算当前的part  的实际坐标
void Robot_Cala_ActualPartPos(double* part_deg,double* part_r)
{
  double motor1_encoder=0;
  double motor2_encoder=0;

  motor1_encoder = Robot_Axis2_Get_Motor_Feedback(0);
  motor1_encoder = motor1_encoder + trobot_axis2.motor_offset[0];

  motor2_encoder = Robot_Axis2_Get_Motor_Feedback(1);
  motor2_encoder = motor2_encoder + trobot_axis2.motor_offset[1];

  Robot_Axis2_ForKin(motor1_encoder,motor2_encoder,part_deg,part_r);
}

//更新part 实际坐标到主站

void Robot_Updata_ActualPartPos_Master(void)
{
  Robot_Cala_ActualPartPos(&trobot_axis2.actual_part_deg,&trobot_axis2.actual_part_r);

  Master_Robot_Set_Pos(ROBOT_AXIS2_DEG_INDEX,trobot_axis2.actual_part_deg);
  Master_Robot_Set_Pos(ROBOT_AXIS2_R_INDEX,trobot_axis2.actual_part_r);
}

//更新part 实际坐标到位置指令
void Robot_Updata_ActualPartPos_MasterPosCmd(void)
{

  Robot_Cala_ActualPartPos(&trobot_axis2.actual_part_deg,&trobot_axis2.actual_part_r);

  Slave_Master_Set_Master_Poscmd(trobot_axis2.part_deg_index,trobot_axis2.actual_part_deg);
  Slave_Master_Set_Master_Poscmd(trobot_axis2.part_r_index,  trobot_axis2.actual_part_r);

}

/*运动学计算
输入 电机1角度 0-360
输入 电机2角度 0-360

输出 工件坐标 角度 0-360
输出 工件坐标 半径 0-R
*/
void Robot_Axis2_ForKin(double motor1_feadback,double motor2_feadback,double* part_deg,double* part_r)
{

  double tmp_rad=0;
  double deg2_deg1=0;
/*反馈转换为角度*/
  trobot_axis2.actual_motor_deg[0] = motor1_feadback*trobot_axis2.facter_motor2deg[0];
  trobot_axis2.actual_motor_deg[1] = motor2_feadback*trobot_axis2.facter_motor2deg[1];

  deg2_deg1 = trobot_axis2.actual_motor_deg[1]-trobot_axis2.actual_motor_deg[0];

  tmp_rad = deg2_deg1*DEG_TO_RAD*0.5;
  if(deg2_deg1 >165 || deg2_deg1<-165)
  {
    //报错 Jog 模式不报错
   // if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_JOG)
    //{
      trobot_axis2.robot_fault_cmd =1;
    //}
    return;
  }
  else
  {
    trobot_axis2.actual_part_deg1 = 0.5*(trobot_axis2.actual_motor_deg[1]+trobot_axis2.actual_motor_deg[0]);
    trobot_axis2.actual_part_r1 = trobot_axis2.L1X2*sinf(tmp_rad) + trobot_axis2.L3;

    *part_deg = trobot_axis2.actual_part_deg1;
    *part_r   = trobot_axis2.actual_part_r1  ;
  }
}





/*逆运动学计算

输出 工件坐标 角度 0-360
输出 工件坐标 半径 0-R

输出 电机1角度 0-360
输出 电机2角度 0-360
*/
void Robot_Axis2_InvKin(double master_part_deg,double master_part_r,double* motor1_cmd,double* motor2_cmd)
{
  double dtmp=0;
  double deg0=0;

  /*主站位置转换*/

  trobot_axis2.cmd_part_r   = master_part_r   ;
  trobot_axis2.cmd_part_deg = master_part_deg ;

  if(trobot_axis2.cmd_part_r>trobot_axis2.part_r_max || trobot_axis2.cmd_part_r<trobot_axis2.part_r_min)
  {
    trobot_axis2.robot_fault_cmd =1;
    return;
    //报错
  }
  else
  {
    dtmp = (trobot_axis2.cmd_part_r - trobot_axis2.L3)*trobot_axis2.D2L1;   //弧度计算
    dtmp = asinf(dtmp)*RAD_TO_DEG;//转换为弧度转换为角度
    
    trobot_axis2.cmd_motor_deg[0] = trobot_axis2.cmd_part_deg - dtmp;
    trobot_axis2.cmd_motor_deg[1] = trobot_axis2.cmd_part_deg + dtmp;

    *motor1_cmd = trobot_axis2.cmd_motor_deg[0] * trobot_axis2.facter_deg2motor[0] - trobot_axis2.motor_offset[0];//20bit
    *motor2_cmd = trobot_axis2.cmd_motor_deg[1] * trobot_axis2.facter_deg2motor[1] - trobot_axis2.motor_offset[1];//20bit
  }
}



/*伺服机器人逆运动学计算*/
void Robot_Axis2_InvKin_Servo(void)
{
  if(trobot_axis2.robot_run_mode==ROBOT_CONTROL_RUN_MODE_PART)
  {
    if(trobot_axis2.robot_fault !=0)return;
    Robot_Axis2_InvKin(*gt_invkinematics.poscmd_master[trobot_axis2.part_deg_index],*gt_invkinematics.poscmd_master[trobot_axis2.part_r_index],&trobot_axis2.cmd_motor_cnt[0],&trobot_axis2.cmd_motor_cnt[1]);

    /*判断电机是否使能*/
    for(uint8_t index=0;index<MAX_AXIS_NUM;index++)
    {
      if(index == trobot_axis2.motor_index[0])
      {
        *gt_invkinematics.poscmd_motor[index] = trobot_axis2.cmd_motor_cnt[0];
      }
      else if(index == trobot_axis2.motor_index[1])
      {
        *gt_invkinematics.poscmd_motor[index] = trobot_axis2.cmd_motor_cnt[1];
      }
      else
      {
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


/*机器人伺服指令更新
part 模式下如果 机器人下电机使能没有同时开启，则指令不变


*/
void Robot_Axis2_Set_Servo_Cmd(uint8_t index,operation_mode_t operation_mode)
{
  uint8_t enable = ServoStatus_Get_AmpEnable(index);
 
  if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART) 
  {
    if(enable == 1)
    {
      if(gt_servos.run_mode[index] == SERVO_TUNE_MODE)
      {
        double tune_pos_cmd=0;
        tune_pos_cmd = Get_Tune_Pos(index); /*调试位置指令*/           
        Slave_Master_Set_Master_Poscmd(index,tune_pos_cmd);
      }
    }
      /*工件指令模式*/
     if(enable == 0)
     {
       if(index == trobot_axis2.motor_index[0]||index == trobot_axis2.motor_index[1])
        {
            Robot_Axis2_Set_Cmd_PartDisable(index);  /*反馈等于工件指令*/
        }
        else
        {
          Slave_Master_Set_Master_Poscmd(index,tslave_master[index].servo_posfeedback); /*指令等于反馈*/
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
      Slave_Master_Set_Master_Poscmd(index,tslave_master[index].servo_posfeedback);
    }
  }
}

/*
Jog 模式切换到part 模式进行指令变换，位置指令更新需要变换
电机不是能时，指令需要跟着反馈变换

part 模式切换到Jog模式也需要变换
*/


/*切换到工件模式时            指令和反馈处理
Jog 模式切换到part 模式进行指令变换，位置指令更新需要变换
*/
void Robot_Axis2_Set_PartMode_CmdFeedback(void)
{

  Robot_Cala_ActualPartPos(&trobot_axis2.actual_part_deg,&trobot_axis2.actual_part_r);

  //更新 part 实际位置到主站
  Master_Robot_Set_Pos(ROBOT_AXIS2_DEG_INDEX,trobot_axis2.actual_part_deg);
  Master_Robot_Set_Pos(ROBOT_AXIS2_R_INDEX,trobot_axis2.actual_part_r);

  //更新位置指令
  Slave_Master_Set_Master_Poscmd(trobot_axis2.part_deg_index,trobot_axis2.actual_part_deg);
  Slave_Master_Set_Master_Poscmd(trobot_axis2.part_r_index,  trobot_axis2.actual_part_r);
}


/*切换到Jog模式时            指令和反馈处理
part 模式切换到Jog模式也需要变换
*/
void Robot_Axis2_Set_JogMode_CmdFeedback(void)
{
  //更新位置指令
  Slave_Master_Set_Master_Poscmd(trobot_axis2.motor_index[0],*trobot_axis2.pmotor_feedback[0]);
  Slave_Master_Set_Master_Poscmd(trobot_axis2.motor_index[1],*trobot_axis2.pmotor_feedback[1]);
}


uint32_t Global_Get_ServoCnt(void);
void Robot_Axis2_Set_Cmd_PartDisable(uint8_t index)
{
  //每个伺服周期执行一次，以伺服周期计数作为标志位
  static uint32_t servo_cnt_flage[2];
  double motor1_encoder=0;
  double motor2_encoder=0;
  
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


/*实时检测位置反馈，更新位置指令*/
  motor1_encoder = Robot_Axis2_Get_Motor_Feedback(0);
  motor1_encoder = motor1_encoder + trobot_axis2.motor_offset[0];
  
  motor2_encoder = Robot_Axis2_Get_Motor_Feedback(1);
  motor2_encoder = motor2_encoder + trobot_axis2.motor_offset[1];

  //Slave_Master_Set_FeedBack(trobot_axis2.motor_index[0],motor1_encoder);
  //Slave_Master_Set_FeedBack(trobot_axis2.motor_index[1],motor2_encoder);


  Robot_Axis2_ForKin(motor1_encoder,motor2_encoder,&trobot_axis2.actual_part_deg,&trobot_axis2.actual_part_r);


  Slave_Master_Set_Master_Poscmd(trobot_axis2.part_deg_index,trobot_axis2.actual_part_deg);
  Slave_Master_Set_Master_Poscmd(trobot_axis2.part_r_index,  trobot_axis2.actual_part_r);

}




/*
机器人反馈类型不同
使用不同的处理方法
兴芯 使用单圈绝对值编码器反馈，
将电机位置反馈作为其绝对位置
*/
int32_t Robot_Axis2_Get_Motor_Feedback(uint8_t index)
{
  return (int32_t)*trobot_axis2.pmotor_feedback[index];
}


uint8_t Robot_Axis2_Get_RunMode(void)
{
  return trobot_axis2.robot_run_mode;
}


/*
安全保护
实际指令
*/

/*出现错误调用此函数*/
void Robot_Axis2_Set_RobotFalut(uint8_t fault_flage)
{
  /*计算行程错误*/
 if(fault_flage==0)
 {
   ServoStatus_Reset_StatusWord(trobot_axis2.motor_index[0],SERVOSTATUS_FALULT_ROBOT_MOTOR_MASK);
   ServoStatus_Reset_StatusWord(trobot_axis2.motor_index[1],SERVOSTATUS_FALULT_ROBOT_MOTOR_MASK);
 }
 else
 {
   ServoStatus_Set_StatusWord(trobot_axis2.motor_index[0],SERVOSTATUS_FALULT_ROBOT_MOTOR_MASK);
   ServoStatus_Set_StatusWord(trobot_axis2.motor_index[1],SERVOSTATUS_FALULT_ROBOT_MOTOR_MASK);
 }
}










