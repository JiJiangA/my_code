/*
˫�� ��Բ���˻�����
ƫ�ƺͽǶ�

����ֵ����������
�����˵�����λ�ã���Ӧ ���˶�ѧ�����˶�ѧ�ĽǶȣ��͵�ǰ����ֵ��������ֵ
�ǶȺ����˶�ѧ�����˶�ѧ��ʽ�йأ������ʽ��û�У���Ƕ�Ϊ0����




���ھ���ֵ�������Ļ����ˣ�ƫ�����������壬���ܼ�ס�����˵ĵ�ǰλ�á�
ƫ��������Ի����˵��ض��Ƕȣ�����˵����ĳ���Ƕ���ء�


ƫ���� = -(ƫ����-�Ƕȶ�Ӧ��cts)

���˶�����λ��ʱ��Ϊ��Ӧ�ĽǶ�

���˶�ѧ����ʱ         ��ǰ������ֵ+ƫ����
���˶�ѧ����ʱ         ��ǰ������ֵ-ƫ����
��������ָ��õ��ĵ��ָ���û��ƫ�Ƶģ�������


//ȷ����ʼλ
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
���������˶�ѧ���㣬���ݷ������㣬������ǰ��λ��
*/
void Robot_Axis2_ForKin(double m1_deg,double m2_deg,double* axis_deg,double* axis_r);

/*
���������˶�ģʽ��ΪPartģʽ
��Ҫ���ݷ����޸� ��վ��partָ��
*/
void Robot_Axis2_Set_PartMode_CmdFeedback(void);


/*
��ΪJOGģʽ������վָ���Ϊ��ǰ�ķ���
*/
void Robot_Axis2_Set_JogMode_CmdFeedback(void);


void Robot_Axis2_Set_RobotFalut(uint8_t fault_flage);


/*
part ģʽ�� disable ʱ���ָ������
*/
void Robot_Axis2_Set_Cmd_PartDisable(uint8_t index);

/*
����������ۼӵķ��������Բ��ܸı䷴��
*/
int32_t Robot_Axis2_Get_Motor_Feedback(uint8_t index);

//��վλ�÷���
void Robot_Updata_ActualPartPos_Master(void);



/*�����˳�ʼ��*/
void Robot_Axis2_Init(void)
{
  //double facter_cmd=4096;
  double axis_ppr=0;
  /*���� ��������*/
  trobot[0].part_index = 1;
  trobot[1].part_index = 2;
  /*�� ��������*/
  trobot[0].axis_index = 1;
  trobot[1].axis_index = 2;


  /*������*/
  /*������״̬*/
  trobot_axis2.robot_fault_index  =0;
  trobot_axis2.robot_fault_enable =0;
  trobot_axis2.robot_fault_cmd    =0;

  /*������*/

  trobot_axis2.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;      //Jog  Part ģʽ
  trobot_axis2.robot_index    = 2;         //��д ״̬�� ������

  trobot_axis2.robot_status_word = 0;
  trobot_axis2.robot_control_word =0;;

  /*����*/

  trobot_axis2.cmd_part_r=0;                //
  trobot_axis2.cmd_part_deg=0;

  trobot_axis2.actual_part_r=0;
  trobot_axis2.actual_part_deg=0;


  trobot_axis2.part_deg_index=0;
  trobot_axis2.part_r_index=0;


  trobot_axis2.updata_partmode_flag=0;       //Jog ת��part ģʽ��־λ       
  trobot_axis2.updata_cnt=0;                //���¼���

  //trobot_axis2.facter_part_r_cmd=0.000244140625;       //ָ����Դ�ֵ
  //trobot_axis2.facter_part_deg_cmd=0.000244140625;     //ָ����Դ�ֵ

  //trobot_axis2.facter_part_r_actual=facter_cmd;    //�������Դ�ֵ
 // trobot_axis2.facter_part_deg_actual=facter_cmd;  //�������Դ�ֵ

  //trobot_axis2.facter_part_r_cmd=1.0/facter_cmd;       //ָ����Դ�ֵ
  //trobot_axis2.facter_part_deg_cmd=1.0/facter_cmd;;     //ָ����Դ�ֵ


  /*���*/
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



/*����������*/
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

  //��������Ƿ����
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

  /*����û�д���*/
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

      /*���Ƕȵ�ƫ�����ӵ�*/
      offset = (float)trobot_axis2.motor_deg[index]*trobot_axis2.facter_deg2motor[index]; //�Ƕ�ƫ��������
      trobot_axis2.motor_offset[index] = -(trobot_axis2.motor_offset[index]-offset);
    }


    
    /*ֱ��      ��ʼ����������վָ��*/
   // motor1_encoder = Encoder_ABS_Get_Value(trobot_axis2.motor_index[0]);
    //motor1_encoder = (motor1_encoder + trobot_axis2.motor_offset[0]+trobot_axis2.axis_ppr[0])%trobot_axis2.axis_ppr[0];
    
   // motor2_encoder = Encoder_ABS_Get_Value(trobot_axis2.motor_index[1]);
    //motor2_encoder = (motor2_encoder + trobot_axis2.motor_offset[1]+trobot_axis2.axis_ppr[1])%trobot_axis2.axis_ppr[1];
    //���� ���λ��   q1 90 q2 0��

    //Jog ģʽ ���÷�����ָ��

    //motor1_encoder = trobot_axis2.axis_ppr[0]/4;
    //motor2_encoder = 0;

    //Encoder_ABS_Set_Value(0,motor1_encoder);

    //Slave_Master_Set_FeedBack(trobot_axis2.motor_index[0],(double)motor1_encoder);
    //Slave_Master_Set_FeedBack(trobot_axis2.motor_index[1],(double)motor2_encoder);
    //Ĭ��ʱJogģʽ
    Robot_Axis2_Set_JogMode_CmdFeedback();

    
    //Robot_Axis2_ForKin(motor1_encoder,motor2_encoder,&trobot_axis2.actual_part_deg,&trobot_axis2.actual_part_r);

    //part  ָ������
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
������ ״̬����
*/
void Robot_Axis2_Update(void)
{
  int32_t axis_r=0;
  int32_t axis_deg=0;
  
  double motor1_encoder=0;
  double motor2_encoder=0;

  uint32_t u32tmp=0;


/*��ȡ������״̬�֣�ʵ�ֿ����ֺ͸���״̬*/


  trobot_axis2.robot_control_word= Master_Robot_Get_Control(trobot_axis2.robot_index);
  trobot_axis2.robot_status_word = Master_Robot_Get_Status(trobot_axis2.robot_index);



/*������      ����ģʽ*/
  if(trobot_axis2.robot_control_word&ROBOT_CONTROL_RUN_MODE_MASK)  //���ʱJog ģʽ
  {
    //Part ģʽ�л��� Jogģʽ   �л���jogģʽ���Խ������˵ı������
    if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART)
    {      
      /*λ�ø�����ֹ�ж�*/
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
    //Jog ģʽ�л��� Partģʽ
    if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_JOG)
    {
      /*�л���part ģʽ�²鿴�Ƿ�����������*/
      if(trobot_axis2.robot_fault==0)       /*�ŷ�������������, ��part ģʽ����ָ�����*/
      {
        /*λ�ø�����ֹ�ж�*/
        DISABLE_GLOBA_IRQ();
        Robot_Axis2_Set_PartMode_CmdFeedback();
        trobot_axis2.robot_run_mode = ROBOT_CONTROL_RUN_MODE_PART;
        ENABLE_GLOBA_IRQ();
        trobot_axis2.updata_partmode_flag=1;
        trobot_axis2.updata_cnt=0;
      }
    }
  }



//������
  /*ʹ�ܼ��*/
  if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_PART)
  {
    //������
     //��������ģ���»�����   ʹ��״̬�Ƿ��в�һ��
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


  //������ʹ�ܱ���
  if(trobot_axis2.robot_fault_enable == 0)
  {
    trobot_axis2.robot_status_word &= (~ROBOT_STASUS_FAULT_NOTALL_MOROT_ENABLE_MASK);
  }
  else
  {
    trobot_axis2.robot_status_word |= ROBOT_STASUS_FAULT_NOTALL_MOROT_ENABLE_MASK;
  }
  
  
  //������ָ���
  if(trobot_axis2.robot_fault_cmd == 0) /*ָ�����*/
  {
    trobot_axis2.robot_status_word &= (~ROBOT_STASUS_FAULT_CMD_ERROE_MASK);
  }
  else
  {
    trobot_axis2.robot_status_word |= ROBOT_STASUS_FAULT_CMD_ERROE_MASK;
  }


  //��������������
  if(trobot_axis2.robot_fault_index==0)
  {
    trobot_axis2.robot_status_word &= (~ROBOT_STASUS_FAULT_INDEX_MASK);
  }
  else
  {
    trobot_axis2.robot_status_word |= ROBOT_STASUS_FAULT_INDEX_MASK;
  }


  //�������ܱ���
  if(trobot_axis2.robot_status_word &ROBOT_STATUS_FAULT_MASK)
  {    
    /*���������л���Jog ģʽ������Ҳ�ı�*/
    trobot_axis2.robot_status_word |= ROBOT_STASUS_FAULT_MASK;
    //trobot_axis2.robot_control_word |= ROBOT_CONTROL_RUN_MODE_MASK;
    
    //Robot_Axis2_Set_RobotFalut(1);
    //trobot_axis2.robot_run_mode = ROBOT_CONTROL_RUN_MODE_JOG;
  }
  else
  {
    trobot_axis2.robot_status_word &= (~ROBOT_STASUS_FAULT_MASK);
  }

  /***************        ״̬ ��ʾ            *********/
  /*״̬     ����ģʽ����*/
  if(trobot_axis2.robot_run_mode == ROBOT_CONTROL_RUN_MODE_JOG)
  {
    trobot_axis2.robot_status_word |=ROBOT_CONTROL_RUN_MODE_MASK;
  }
  else
  {
    trobot_axis2.robot_status_word &=(~ROBOT_CONTROL_RUN_MODE_MASK);
  }

  //������״̬�ָ���
  Master_Robot_Set_Control(trobot_axis2.robot_index,trobot_axis2.robot_control_word);
  Master_Robot_Set_Status(trobot_axis2.robot_index, trobot_axis2.robot_status_word);


  

  //part ʵ��λ�÷�������վ
  if(trobot_axis2.updata_partmode_flag==0)  //����л���partģʽ�����ȴ�һ��ʱ���ٸ���part λ����Ϣ����վ
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



//���㵱ǰ��part  ��ʵ������
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

//����part ʵ�����굽��վ

void Robot_Updata_ActualPartPos_Master(void)
{
  Robot_Cala_ActualPartPos(&trobot_axis2.actual_part_deg,&trobot_axis2.actual_part_r);

  Master_Robot_Set_Pos(ROBOT_AXIS2_DEG_INDEX,trobot_axis2.actual_part_deg);
  Master_Robot_Set_Pos(ROBOT_AXIS2_R_INDEX,trobot_axis2.actual_part_r);
}

//����part ʵ�����굽λ��ָ��
void Robot_Updata_ActualPartPos_MasterPosCmd(void)
{

  Robot_Cala_ActualPartPos(&trobot_axis2.actual_part_deg,&trobot_axis2.actual_part_r);

  Slave_Master_Set_Master_Poscmd(trobot_axis2.part_deg_index,trobot_axis2.actual_part_deg);
  Slave_Master_Set_Master_Poscmd(trobot_axis2.part_r_index,  trobot_axis2.actual_part_r);

}

/*�˶�ѧ����
���� ���1�Ƕ� 0-360
���� ���2�Ƕ� 0-360

��� �������� �Ƕ� 0-360
��� �������� �뾶 0-R
*/
void Robot_Axis2_ForKin(double motor1_feadback,double motor2_feadback,double* part_deg,double* part_r)
{

  double tmp_rad=0;
  double deg2_deg1=0;
/*����ת��Ϊ�Ƕ�*/
  trobot_axis2.actual_motor_deg[0] = motor1_feadback*trobot_axis2.facter_motor2deg[0];
  trobot_axis2.actual_motor_deg[1] = motor2_feadback*trobot_axis2.facter_motor2deg[1];

  deg2_deg1 = trobot_axis2.actual_motor_deg[1]-trobot_axis2.actual_motor_deg[0];

  tmp_rad = deg2_deg1*DEG_TO_RAD*0.5;
  if(deg2_deg1 >165 || deg2_deg1<-165)
  {
    //���� Jog ģʽ������
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





/*���˶�ѧ����

��� �������� �Ƕ� 0-360
��� �������� �뾶 0-R

��� ���1�Ƕ� 0-360
��� ���2�Ƕ� 0-360
*/
void Robot_Axis2_InvKin(double master_part_deg,double master_part_r,double* motor1_cmd,double* motor2_cmd)
{
  double dtmp=0;
  double deg0=0;

  /*��վλ��ת��*/

  trobot_axis2.cmd_part_r   = master_part_r   ;
  trobot_axis2.cmd_part_deg = master_part_deg ;

  if(trobot_axis2.cmd_part_r>trobot_axis2.part_r_max || trobot_axis2.cmd_part_r<trobot_axis2.part_r_min)
  {
    trobot_axis2.robot_fault_cmd =1;
    return;
    //����
  }
  else
  {
    dtmp = (trobot_axis2.cmd_part_r - trobot_axis2.L3)*trobot_axis2.D2L1;   //���ȼ���
    dtmp = asinf(dtmp)*RAD_TO_DEG;//ת��Ϊ����ת��Ϊ�Ƕ�
    
    trobot_axis2.cmd_motor_deg[0] = trobot_axis2.cmd_part_deg - dtmp;
    trobot_axis2.cmd_motor_deg[1] = trobot_axis2.cmd_part_deg + dtmp;

    *motor1_cmd = trobot_axis2.cmd_motor_deg[0] * trobot_axis2.facter_deg2motor[0] - trobot_axis2.motor_offset[0];//20bit
    *motor2_cmd = trobot_axis2.cmd_motor_deg[1] * trobot_axis2.facter_deg2motor[1] - trobot_axis2.motor_offset[1];//20bit
  }
}



/*�ŷ����������˶�ѧ����*/
void Robot_Axis2_InvKin_Servo(void)
{
  if(trobot_axis2.robot_run_mode==ROBOT_CONTROL_RUN_MODE_PART)
  {
    if(trobot_axis2.robot_fault !=0)return;
    Robot_Axis2_InvKin(*gt_invkinematics.poscmd_master[trobot_axis2.part_deg_index],*gt_invkinematics.poscmd_master[trobot_axis2.part_r_index],&trobot_axis2.cmd_motor_cnt[0],&trobot_axis2.cmd_motor_cnt[1]);

    /*�жϵ���Ƿ�ʹ��*/
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


/*�������ŷ�ָ�����
part ģʽ����� �������µ��ʹ��û��ͬʱ��������ָ���


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
        tune_pos_cmd = Get_Tune_Pos(index); /*����λ��ָ��*/           
        Slave_Master_Set_Master_Poscmd(index,tune_pos_cmd);
      }
    }
      /*����ָ��ģʽ*/
     if(enable == 0)
     {
       if(index == trobot_axis2.motor_index[0]||index == trobot_axis2.motor_index[1])
        {
            Robot_Axis2_Set_Cmd_PartDisable(index);  /*�������ڹ���ָ��*/
        }
        else
        {
          Slave_Master_Set_Master_Poscmd(index,tslave_master[index].servo_posfeedback); /*ָ����ڷ���*/
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
      Slave_Master_Set_Master_Poscmd(index,tslave_master[index].servo_posfeedback);
    }
  }
}

/*
Jog ģʽ�л���part ģʽ����ָ��任��λ��ָ�������Ҫ�任
���������ʱ��ָ����Ҫ���ŷ����任

part ģʽ�л���JogģʽҲ��Ҫ�任
*/


/*�л�������ģʽʱ            ָ��ͷ�������
Jog ģʽ�л���part ģʽ����ָ��任��λ��ָ�������Ҫ�任
*/
void Robot_Axis2_Set_PartMode_CmdFeedback(void)
{

  Robot_Cala_ActualPartPos(&trobot_axis2.actual_part_deg,&trobot_axis2.actual_part_r);

  //���� part ʵ��λ�õ���վ
  Master_Robot_Set_Pos(ROBOT_AXIS2_DEG_INDEX,trobot_axis2.actual_part_deg);
  Master_Robot_Set_Pos(ROBOT_AXIS2_R_INDEX,trobot_axis2.actual_part_r);

  //����λ��ָ��
  Slave_Master_Set_Master_Poscmd(trobot_axis2.part_deg_index,trobot_axis2.actual_part_deg);
  Slave_Master_Set_Master_Poscmd(trobot_axis2.part_r_index,  trobot_axis2.actual_part_r);
}


/*�л���Jogģʽʱ            ָ��ͷ�������
part ģʽ�л���JogģʽҲ��Ҫ�任
*/
void Robot_Axis2_Set_JogMode_CmdFeedback(void)
{
  //����λ��ָ��
  Slave_Master_Set_Master_Poscmd(trobot_axis2.motor_index[0],*trobot_axis2.pmotor_feedback[0]);
  Slave_Master_Set_Master_Poscmd(trobot_axis2.motor_index[1],*trobot_axis2.pmotor_feedback[1]);
}


uint32_t Global_Get_ServoCnt(void);
void Robot_Axis2_Set_Cmd_PartDisable(uint8_t index)
{
  //ÿ���ŷ�����ִ��һ�Σ����ŷ����ڼ�����Ϊ��־λ
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
  /*ȫ��ʹ�ܲŲ�����ָ��*/


/*ʵʱ���λ�÷���������λ��ָ��*/
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
�����˷������Ͳ�ͬ
ʹ�ò�ͬ�Ĵ�����
��о ʹ�õ�Ȧ����ֵ������������
�����λ�÷�����Ϊ�����λ��
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
��ȫ����
ʵ��ָ��
*/

/*���ִ�����ô˺���*/
void Robot_Axis2_Set_RobotFalut(uint8_t fault_flage)
{
  /*�����г̴���*/
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










