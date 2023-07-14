

#ifndef _ROBOT_XX_H
#define _ROBOT_XX_H


#define ROBOT_AXIS_NUM     3
#define ROBOT_MOROT_NUM    5



#define  ROBOT_XX_Z_INDEX          (uint8_t)1   //�᳤��
#define  ROBOT_XX_DEG_INDEX        (uint8_t)2   //��Ƕ�ƫ����
#define  ROBOT_XX_TWITS_INDEX          (uint8_t)3   //Twits ���ƶ��Ƕ�


typedef struct
{
/*������*/
  /*������*/
  uint8_t robot_fault;
  uint8_t robot_fault_enable;    //������״̬  ʹ�� 1:����
  uint8_t robot_fault_index;     //������״̬  ���� 1:����
  uint8_t robot_fault_cmd;       //������״̬  ָ�� 1:����
  
  
  uint8_t robot_run_mode;      //Jog  Part ģʽ
  uint8_t robot_index;         

  uint32_t robot_status_word;  //��д ״̬�� ������
  uint32_t robot_control_word;


  /*����*/  
  uint8_t part_z_index;        //����ָ������
  uint8_t part_deg_index;      //����ָ������
  uint8_t part_b_index;
  
  double part_z_max;  /*���뾶*/
  double part_z_min;  /*��С�뾶*/

  double cmd_part_z;                //
  double cmd_part_deg;                //
  double cmd_part_b;

  double actual_part_z;
  double actual_part_deg;
  double actual_part_b;


  float facter_part_z_cmd;       //ָ����Դ�ֵ
  float facter_part_deg_cmd;     //ָ����Դ�ֵ
  float facter_part_b_cmd;     //ָ����Դ�ֵ

  

  float facter_part_z_actual;    //�������Դ�ֵ
  float facter_part_deg_actual;  //�������Դ�ֵ
  float facter_part_b_actual;  //�������Դ�ֵ



/*���*/
  uint8_t  motor_index[ROBOT_MOROT_NUM];
  double   motor_offset[ROBOT_MOROT_NUM];
  uint16_t motor_deg[ROBOT_MOROT_NUM];
  float    motor_rr[ROBOT_MOROT_NUM];

  uint8_t *motor_enable[ROBOT_MOROT_NUM];

  uint32_t axis_ppr[ROBOT_MOROT_NUM];    //��תһȦ




  double cmd_motor_deg[ROBOT_MOROT_NUM];
  double cmd_motor_cnt[ROBOT_MOROT_NUM];
  
  double actual_motor_deg[ROBOT_MOROT_NUM];
  const double* pmotor_feedback[ROBOT_MOROT_NUM];   //ָ����λ�÷���


  double facter_deg2motor[ROBOT_MOROT_NUM];                        //���ָ��Ƕ�����
  double facter_motor2deg[ROBOT_MOROT_NUM];                        //���ָ��Ƕ�����

  float L1;
  float L2;
  float L3;

  float D_2L1;//1/2L

 // uint8_t  flag_pos_update;        //λ�ø��±�־λ
}robot_xx_t;





void Robot_XX_Init(void);
void Robot_XX_Config(void);
void Robot_XX_InvKin_Servo(void);
void Robot_XX_Update(void);
void Robot_XX_Set_Servo_Cmd(uint8_t index,operation_mode_t operation_mode);



uint8_t Robot_XX_Get_RunMode(void);






#endif



























