
/*
˫�� ��Բ���˻�����

������1 ��Բɨ��           ֻ�ܵ�������ʽ������ʹ��

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
/*���� ��������*/ 
  trobot[0].part_index = 1;  //����ָ��
  trobot[1].part_index = 2;
  trobot[2].part_index = 4;
/*�� ��������*/
  trobot[0].axis_index = 1;//����������Э������������ƥ��
  trobot[1].axis_index = 2;
  trobot[2].axis_index = 3;
  trobot[3].axis_index = 4;
  trobot[4].axis_index = 5;

/*������״̬*/
  trobot_xx.robot_fault_index  =0;
  trobot_xx.robot_fault_enable =0;
  trobot_xx.robot_fault_cmd    =0;

  /*������*/
  trobot_xx.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;      //Jog  Part ģʽ
  trobot_xx.robot_index    = 1;         //��д ״̬�� ������

   trobot_xx.robot_status_word = 0;
   trobot_xx.robot_control_word =0;;

   
  /*����*/
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

  /*���*/
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

      axis_ppr = Encoder_Get_PPR(index)*trobot_xx.motor_rr[index];//��еһת���ٸ�����
      
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


  
/*�������*/
  /*��վָ������*/
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

  /*����û�д���*/
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
      /*���Ƕȵ�ƫ�����ӵ�*/
      offset = (float)trobot_xx.motor_deg[index]*trobot_xx.facter_deg2motor[index]; //�Ƕ�ƫ��������
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


/*���ÿ�����ģʽ*/
  trobot_xx.robot_control_word |=ROBOT_CONTROL_RUN_MODE_MASK;
  Master_Robot_Set_Control(trobot_xx.robot_index,trobot_xx.robot_control_word);
  Master_Robot_Set_Status(trobot_xx.robot_index, trobot_xx.robot_status_word);

}




/*
��վ����ʵʱ����
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

/*��ȡ������״̬�֣�ʵ�ֿ����ֺ͸���״̬*/
  trobot_xx.robot_control_word= Master_Robot_Get_Control(trobot_xx.robot_index);
  trobot_xx.robot_status_word = Master_Robot_Get_Status(trobot_xx.robot_index);

/*��վλ��ָ�����*/
  if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART) /*ֻ����partģʽ�²��ܸ��ĵ��ָ��*/
  {
    if(trobot_xx.robot_control_word&ROBOT_CONTROL_CMD_UPDATA_MASK) //Э����������λ��ָ��
    {
      /*λ�ø�����ֹ�жϣ���ָֹ���λ�ò�һ��*/      
      DISABLE_GLOBA_IRQ();
      Robot_XX_Set_PartMode_CmdFeedback();
      //Slave_Master_Set_Master_Poscmd(trobot_xx.part_deg_index, trobot_xx.actual_part_deg);
      //Slave_Master_Set_Master_Poscmd(trobot_xx.part_z_index, trobot_xx.actual_part_z);
      ENABLE_GLOBA_IRQ();
      /*���������*/
      trobot_xx.robot_control_word = trobot_xx.robot_control_word&(~ROBOT_CONTROL_CMD_UPDATA_MASK);
      /*����״̬��*/
      trobot_xx.robot_status_word |= ROBOT_CONTROL_CMD_UPDATA_MASK;
      //trobot_xx.flag_pos_update = 1;
    }
  }

/*������      ����ģʽ*/
  if(trobot_xx.robot_control_word&ROBOT_CONTROL_RUN_MODE_MASK)  
  {
    //JOG ģʽ
    if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART)
    {
      /*λ�ø�����ֹ�ж�*/
      DISABLE_GLOBA_IRQ();
      Robot_XX_Set_JogMode_CmdFeedback();
      trobot_xx.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;
      ENABLE_GLOBA_IRQ();
    }
  }
  else
  {
    //part ģʽ
    if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_JOG)
    {
      /*�л���part ģʽ�²鿴�Ƿ�����������*/
      if(trobot_xx.robot_fault_index!=0)       /*�ŷ�������������, ��part ģʽ����ָ�����*/
      {
        trobot_xx.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;
        trobot_xx.robot_status_word |= ROBOT_CONTROL_RUN_MODE_MASK;
      }
      else
      {
        /*λ�ø�����ֹ�ж�*/
        DISABLE_GLOBA_IRQ();
        Robot_XX_Set_PartMode_CmdFeedback();
        trobot_xx.robot_run_mode = ROBOT_CONTROL_RUN_MODE_PART;
        ENABLE_GLOBA_IRQ();
      }
    }
  }



  if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART)
  {
    //������
     //��������ģ���»�����   ʹ��״̬�Ƿ��в�һ��
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


  //������ʹ�ܱ���
  if(trobot_xx.robot_fault_enable == 0)
  {
    trobot_xx.robot_status_word &= (~ROBOT_STASUS_FAULT_NOTALL_MOROT_ENABLE_MASK);
  }
  else
  {
    trobot_xx.robot_status_word |= ROBOT_STASUS_FAULT_NOTALL_MOROT_ENABLE_MASK;
  }


  if(trobot_xx.robot_fault_cmd == 0) /*ָ�����*/
  {
    trobot_xx.robot_status_word &= (~ROBOT_STASUS_FAULT_CMD_ERROE_MASK);
  }
  else
  {
    trobot_xx.robot_status_word |= ROBOT_STASUS_FAULT_CMD_ERROE_MASK;
  }


  //��������������
  if(trobot_xx.robot_fault_index==0)
  {
    trobot_xx.robot_status_word &= (~ROBOT_STASUS_FAULT_INDEX_MASK);
  }
  else
  {
    trobot_xx.robot_status_word |= ROBOT_STASUS_FAULT_INDEX_MASK;
  }



  //�������ܱ���
  if(trobot_xx.robot_status_word &ROBOT_STATUS_FAULT_MASK)
  {    
    trobot_xx.robot_status_word |= ROBOT_STASUS_FAULT_MASK;
  }
  else
  {
    trobot_xx.robot_status_word &= (~ROBOT_STASUS_FAULT_MASK);
  }

  /*״̬     ����ģʽ����*/
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
  /*λ�ø���*/

  Master_Robot_Set_Pos(1,trobot_xx.actual_part_z);
  Master_Robot_Set_Pos(2,trobot_xx.actual_part_deg);


  Master_Robot_Set_Control(trobot_xx.robot_index,trobot_xx.robot_control_word);
  Master_Robot_Set_Status(trobot_xx.robot_index, trobot_xx.robot_status_word);

}















/*�˶�ѧ����
���� ���1 ����ָ��
���� ���2 ����ָ��
���� ���3 ����ָ��

��� �������� z  �߶�
��� �������� deg ��ת�Ƕ�
*/
void Robot_XX_ForKin(double motor1_feadback,double motor2_feadback,double motor3_feadback,double* part_z,double* part_deg)
{
  double tmp_rad=0;
/*����ת��Ϊ�Ƕ�*/
  trobot_xx.actual_motor_deg[0] = motor1_feadback*trobot_xx.facter_motor2deg[0];//
  trobot_xx.actual_motor_deg[1] = motor2_feadback*trobot_xx.facter_motor2deg[1];//
  trobot_xx.actual_motor_deg[2] = motor3_feadback*trobot_xx.facter_motor2deg[2];//


/*�Ƕȼ��㹤������*/
  tmp_rad = trobot_xx.actual_motor_deg[0]*DEG_TO_RAD;
  *part_z = 2*trobot_xx.L1*sinf(tmp_rad);
  *part_deg = trobot_xx.actual_motor_deg[2];
}

/*�˶�ѧ����
���� �������� z  �߶�
���� �������� deg ��ת�Ƕ�

��� ���1 ����ָ��
��� ���2 ����ָ��
��� ���3 ����ָ��
*/
void Robot_XX_InvKin(double master_part_z,double master_part_deg,double master_part_b,
double* motor1_cmd,double* motor2_cmd,double* motor3_cmd,double* motor4_cmd,double* motor5_cmd)
{
  double motor1_rad=0;
  double z_tmp=0;
  /*��վλ��ת��*/
  trobot_xx.cmd_part_z   = master_part_z;
  trobot_xx.cmd_part_deg = master_part_deg;
  trobot_xx.cmd_part_b   =  master_part_b;

  /*ָ������Ʊ���*/
//*���cmd_part_z����2����Z
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
  trobot_xx.cmd_motor_deg[0] = asinf(motor1_rad)*RAD_TO_DEG;             //J1 �Ƕ�
  trobot_xx.cmd_motor_deg[1] = -2*trobot_xx.cmd_motor_deg[0];         //J2 �Ƕ�
  trobot_xx.cmd_motor_deg[2] = trobot_xx.cmd_motor_deg[0] + trobot_xx.cmd_part_deg;//J3 �Ƕ�

  trobot_xx.cmd_motor_deg[3] = trobot_xx.cmd_part_b;
  trobot_xx.cmd_motor_deg[4] = trobot_xx.cmd_part_b;



  *motor1_cmd = trobot_xx.cmd_motor_deg[0] * trobot_xx.facter_deg2motor[0] - trobot_xx.motor_offset[0];//
  *motor2_cmd = trobot_xx.cmd_motor_deg[1] * trobot_xx.facter_deg2motor[1] - trobot_xx.motor_offset[1];//
  *motor3_cmd = trobot_xx.cmd_motor_deg[2] * trobot_xx.facter_deg2motor[2] - trobot_xx.motor_offset[2];//

  *motor4_cmd = trobot_xx.cmd_motor_deg[3] * trobot_xx.facter_deg2motor[3] - trobot_xx.motor_offset[3];//
  *motor5_cmd = trobot_xx.cmd_motor_deg[4] * trobot_xx.facter_deg2motor[4] - trobot_xx.motor_offset[4];//

}







/*���˶�ѧ����*/
void Robot_XX_InvKin_Servo(void)
{
  /*ֻ����part ģʽ�£��Ž������˶�ѧ����*/
  if(trobot_xx.robot_run_mode==ROBOT_CONTROL_RUN_MODE_PART)
  {
    if(trobot_xx.robot_fault !=0)return;
    /*���˶�ѧ���㣬������վָ���������ָ��*/
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



/*ָ�����*/
void Robot_XX_Set_Servo_Cmd(uint8_t index,operation_mode_t operation_mode)
{
  uint8_t enable = ServoStatus_Get_AmpEnable(index);

  if(trobot_xx.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART) 
  {
    
    if(enable == 1)
    {
      if(gt_servos.run_mode[index] == SERVO_TUNE_MODE)
      {
        *gt_servos.pservo_poscmd[index] = Get_Tune_Pos(index); /*����λ��ָ��*/           
        Slave_Master_Set_Master_Poscmd(index,*gt_servos.pservo_poscmd[index]);
      }
    }
      /*����ָ��ģʽ*/
     if(enable == 0)
     {
       if(index == trobot_xx.motor_index[0]||index == trobot_xx.motor_index[1]||index == trobot_xx.motor_index[2]
         ||index == trobot_xx.motor_index[3]||index == trobot_xx.motor_index[4])
        {
           Robot_XX_Set_Cmd_PartDisable(index);  /*�������ڹ���ָ��*/
        }
        else
        {
          Slave_Master_Set_Master_Poscmd(index,*gt_servos.pservo_posfeedback[index]); /*ָ����ڷ���*/
        }
     }
  }
  else /*Jog ����ģʽ*/
  {
    if(enable == 1)
    {
      if(gt_servos.run_mode[index] == SERVO_TUNE_MODE)
      {
        *gt_servos.pservo_poscmd[index] = Get_Tune_Pos(index); /*����λ��ָ��*/           
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
����ģʽ�� ���ݷ�������  ��վ����ָ�����ֵ

���ݵ�������Ƕ�           �����������
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
/*���ݱ������������㹤������*/
  Robot_XX_ForKin(motor1_encoder,motor2_encoder,motor3_encoder,&trobot_xx.actual_part_z,&trobot_xx.actual_part_deg);

  Slave_Master_Set_Master_Poscmd(trobot_xx.part_deg_index,trobot_xx.actual_part_deg);
  Slave_Master_Set_Master_Poscmd(trobot_xx.part_z_index,  trobot_xx.actual_part_z);

}

/*
Jog ģʽ

��վ  ָ�� �� 
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
Part ģʽ�� DisableӦ�ñ��ֵ�����
*/
void Robot_XX_Set_Cmd_PartDisable(uint8_t index)
{
  //ÿ���ŷ�����ִ��һ�Σ����ŷ����ڼ�����Ϊ��־λ
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
  /*ȫ��ʹ�ܲŲ�����ָ��*/

    motor1_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[0]);
    motor1_encoder = motor1_encoder + trobot_xx.motor_offset[0];
    
    motor2_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[1]);
    motor2_encoder = motor2_encoder + trobot_xx.motor_offset[1];
      
    motor3_encoder = Robot_XX_Get_Motor_Feedback(trobot_xx.motor_index[2]);
    motor3_encoder = motor3_encoder + trobot_xx.motor_offset[2];
  
  /*���ݱ������������㹤������*/
    Robot_XX_ForKin(motor1_encoder,motor2_encoder,motor3_encoder,&trobot_xx.actual_part_z,&trobot_xx.actual_part_deg);
    Slave_Master_Set_Master_Poscmd(trobot_xx.part_deg_index,trobot_xx.actual_part_deg);
    Slave_Master_Set_Master_Poscmd(trobot_xx.part_z_index,  trobot_xx.actual_part_z);
}


/*
�˴�������ʱ �������������Э�����������������������
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
��ȫ����
ʵ��ָ��
*/
/*���ִ�����ô˺���*/
void Robot_XX_Set_RobotFalut(uint8_t fault_flage)
{
  if(trobot_xx.robot_fault_index==1)return;
  /*�����г̴���*/
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











