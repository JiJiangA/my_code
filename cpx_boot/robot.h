
#ifndef _ROBOT_H
#define _ROBOT_H

#include "slave.h"




/*常用的单位换算*/
//3.14/180
#define  RAD_TO_DEG             57.295779513082321    //180/3.14
#define  DEG_TO_RAD             0.0174532925199433   //3.14/180


#define PMAC_78600               (uint32_t)(slave_base_addr | ((0x600<<1)|addr_y)<<2)


/*运行模式*/
#define ROBOT_CONTROL_RUN_MODE_MASK      (uint32_t)0x000000100 //bit0

#define ROBOT_CONTROL_RUN_MODE_JOG       (uint8_t)1
#define ROBOT_CONTROL_RUN_MODE_PART      (uint8_t)0



#define ROBOT_CONTROL_CMD_UPDATA_MASK      (uint32_t)0x000000200 //bit1

#define ROBOT_CONTROL_CMD_UPDATA_COMPLETE    (uint8_t)1
#define ROBOT_CONTROL_CMD_UPDATA_NCOMPLETE   (uint8_t)0



#define ROBOT_CONTROL_RESET_CPXFAULT_MASK    (uint32_t)0xFF000000
#define ROBOT_CONTROL_CPX_MODE_MASK          (uint32_t)0x0000FF00





#define ROBOT_STASUS_FAULT        (uint8_t)1
#define ROBOT_STASUS_NOFAULT      (uint8_t)0


#define ROBOT_STASUS_FAULT_MASK                          (uint32_t)0x000000400   //bit2
#define ROBOT_STASUS_FAULT_NOTALL_MOROT_ENABLE_MASK      (uint32_t)0x000001000   //bit4
#define ROBOT_STASUS_FAULT_CMD_ERROE_MASK                (uint32_t)0x000002000   //bit5
#define ROBOT_STASUS_FAULT_INDEX_MASK                    (uint32_t)0x000004000   //bit6






#define ROBOT_STATUS_FAULT_MASK    (ROBOT_STASUS_FAULT_CMD_ERROE_MASK|ROBOT_STASUS_FAULT_INDEX_MASK)



/*从站伺服轴*/
typedef struct{

__IO uint32_t status_cpx;      /*Y:78800*/
__IO uint32_t contorl_cpx;     /*X:78800*/

__IO uint32_t status_1;      /*Y:78801*/
__IO uint32_t contorl_1;     /*X:78801*/

__IO uint32_t status_2;      /*Y:78802*/
__IO uint32_t contorl_2;     /*X:78802*/

__IO uint32_t status_3;      /*Y:78803*/
__IO uint32_t contorl_3;     /*X:78803*/

__IO uint32_t status_4;      /*Y:78804*/
__IO uint32_t contorl_4;     /*X:78804*/

__IO uint32_t status_5;      /*Y:78805*/
__IO uint32_t contorl_5;     /*X:78805*/

__IO uint32_t status_6;      /*Y:78806*/
__IO uint32_t contorl_6;     /*X:78806*/

__IO uint32_t status_7;      /*Y:78807*/
__IO uint32_t contorl_7;     /*X:78807*/

__IO uint32_t status_8;      /*Y:78808*/
__IO uint32_t contorl_8;     /*X:78808*/

__IO uint32_t axis1_pos_l;    /*Y:78809*/
__IO uint32_t axis1_pos_h;    /*X:78809*/

__IO uint32_t axis2_pos_l;     /*Y:7880A*/
__IO uint32_t axis2_pos_h;    /*X:7880A*/

__IO uint32_t axis3_pos_l;     /*Y:7880B*/
__IO uint32_t axis3_pos_h;    /*X:7880B*/

__IO uint32_t axis4_pos_l;     /*Y:7880C*/
__IO uint32_t axis4_pos_h;    /*X:7880C*/

__IO uint32_t axis5_pos_l;     /*Y:7880D*/
__IO uint32_t axis5_pos_h;    /*X:7880D*/

__IO uint32_t axis6_pos_l;     /*Y:7880E*/
__IO uint32_t axis6_pos_h;    /*X:7880E*/

__IO uint32_t axis7_pos_l;     /*Y:7880F*/
__IO uint32_t axis7_pos_h;    /*X:7880F*/

__IO uint32_t axis8_pos_l;     /*Y:78810*/
__IO uint32_t axis8_pos_h;    /*X:78810*/

} master_robot_t;


#define MASTER_ROBOT                                            ((master_robot_t *)PMAC_78600)







/*机器人通用参数*/



typedef struct 
{
   uint8_t part_index;
   uint8_t axis_index;
 
   uint32_t axis_offset;
   uint16_t axis_degree;
   float axis_rr;
}robot_t;





typedef struct 
{
  uint32_t status_word;
  uint32_t control_word;



  
}robot_cpx_t;











uint32_t Master_Robot_Get_Control(uint8_t index);
uint32_t Master_Robot_Get_Status(uint8_t index);


void Master_Robot_Set_Status(uint8_t index,uint32_t status);
void Master_Robot_Set_Control(uint8_t index,uint32_t control);
void Master_Robot_Set_Pos(uint8_t index,double pos);


void Master_Robot_Set_Status_Bit(uint8_t index,uint32_t status);




void Robot_Init(void);
void Robot_Config(void);
void Robot_Update(void);
void Robot_InvKin_Servo(void);
uint8_t Robot_Get_RunMode(void);


#endif














