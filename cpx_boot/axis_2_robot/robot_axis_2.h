

#ifndef _ROBOT_AXIS_2_H
#define _ROBOT_AXIS_2_H

#define ROBOT_AXIS_NUM     2

#define ROBOT_MOROT_NUM    2


/*������ʵ��λ�ø�������*/

#define  ROBOT_AXIS2_DEG_INDEX   (uint8_t)3   //(uint8_t)1
#define  ROBOT_AXIS2_R_INDEX     (uint8_t)4   //(uint8_t)2

/*
������


����




���




*/





typedef struct
{
  /*������*/
  uint8_t robot_fault;
  uint8_t robot_fault_enable;    //������״̬  ʹ�� 1:����
  uint8_t robot_fault_index;     //������״̬  ���� 1:����
  uint8_t robot_fault_cmd;       //������״̬  ָ�� 1:����
  
  
  uint8_t robot_run_mode;      //Jog  Part ģʽ
  uint8_t robot_index;         //��д ״̬�� ������

  uint32_t robot_status_word;
  uint32_t robot_control_word;
  
  /*����*/  
  uint8_t part_r_index;        //����ָ������2
  uint8_t part_deg_index;      //����ָ������1


  
  double part_r_max;  /*���뾶*/
  double part_r_min;  /*��С�뾶*/
  
  double cmd_part_r;                //����ָ��
  double cmd_part_deg;              //����ָ��
  
  double actual_part_r;             //����ʵ��
  double actual_part_deg;           //����ʵ��

  double actual_part_r1;             //����ʵ��
  double actual_part_deg1;           //����ʵ��



   uint8_t updata_partmode_flag;       //Jog ת��part ģʽ��־λ       
   uint16_t updata_cnt;                //���¼���
  //float facter_part_r_cmd;       //ָ����Դ�ֵ         ָ������
  //float facter_part_deg_cmd;     //ָ����Դ�ֵ         ָ������
  
  //float facter_part_r_actual;    //�������Դ�ֵ         ��������
 // float facter_part_deg_actual;  //�������Դ�ֵ         ��������
  
  /*���*/
  uint8_t motor_index[ROBOT_MOROT_NUM];
  double motor_offset[ROBOT_MOROT_NUM];
  uint16_t motor_deg[ROBOT_MOROT_NUM];
  float motor_rr[ROBOT_MOROT_NUM];


  
  uint8_t *motor_enable[ROBOT_MOROT_NUM];
  uint32_t axis_ppr[ROBOT_MOROT_NUM];//���תһȦ��cts��  ����תһȦ��cts
  
  
  double cmd_motor_deg[ROBOT_MOROT_NUM];
  double cmd_motor_cnt[ROBOT_MOROT_NUM];
  
  double actual_motor_deg[ROBOT_MOROT_NUM];
  const double* pmotor_feedback[ROBOT_MOROT_NUM];   //ָ����λ�÷���
  
  
  double facter_deg2motor[ROBOT_MOROT_NUM];                        //�Ƕȵ����ָ������
  double facter_motor2deg[ROBOT_MOROT_NUM];                        //���ָ��Ƕ�����
  
  float L1;
  float L2;
  float L3;

  float D2L1;  //L1 �����������   ��ת��  0.5/L1
  float L1X2;  //L1 �����������   ��ת��  0.5/L1


}robot_axis2_t;
























void Robot_Axis2_Init(void);
void Robot_Axis2_Config(void);
void Robot_Axis2_Update(void);
void Robot_Axis2_Set_Servo_Cmd(uint8_t index,operation_mode_t operation_mode);
void Robot_Axis2_InvKin_Servo(void);
uint8_t Robot_Axis2_Get_RunMode(void);





#endif






















