#ifndef _reset_motor
#define _reset_motor

#include "delay.h"
#include "sys.h"  

#include "stdlib.h"

#define CAN_BAUD_NUM    18		//可用配置波特率个数



//对象字典
#define  SET_VECLOCITY_OB    	0X2002  	//设置速度对象索引   
#define  SET_ACCELERATE_OB   	0X2006  	//设置加速度
#define  SET_DECELERATE_OB   	0X2007
#define  EMEGENCY_SHUTDOMN_OB   0x200C      //紧急关闭
#define  RELEASE_SHUTDOWN_OB    0X200D

#define  READ_MOTOR_CURRENT_OB  0X2100      //读电机输出电流
#define  FEED_BACK              0x2110      //读取电机速度，增量编码器或霍尔编码器都可以 单位RPM
#define  READ_MCU_TEMP_OB       0X210F      //读驱动器温度
#define  READ_ERROR_FLAG_OB     0X2112      //读错误标志
#define  REDA_FAULT_FLAGS       0x2112      //读错误标志



//命令
#define SET_VECLOCITY_CMD       0X2C        //设置速度命令


//SDO 发送 接收
#define SDO_SEND_ID            0X601        //SDO发送CAN_ID
#define SDO_RECEIVE_ID         0X581        //SDO接收CAN_ID

#define  DLC_DATA              0X08         //


__packed typedef struct{

u8 low_index;//索引低字节
u8 high_index;//索引高字节


}FIRST_INDEX;


__packed typedef union{

	__packed  struct{
		u8 cmd;

		FIRST_INDEX fir_index;

		u8 sub_index;

		u8 data[4];//SDO的

		}TPYE_CANOPEN;

	u8 payload_data[8];
	

	u32 speed_robotq;//保存速度
	
}TPYE_CANOPEN;

 
extern u16 rx_can2_id;

extern u8 can2_rx_data[8];

extern TPYE_CANOPEN thrrot_type;

void SET_SDO_HAL(u8 cmd,u16 index,u8 sub_index,u32 data );



void CAN2__ROBOTQ_Configuration(void);

void CAN2_SDO_WriteData(u8 cmd, u16 index, u8 sub_index, int data, u8 id);

void CAN2_RPDO_Set(u16 index,u8 sub_index,u32 data);

void CAN2_TPDO_Set(u16 robot_cmd,u16 index,u8 sub_index,u32 data);


void CAN2_RPDO1_Speed(s32 loop_speed, s32 high_data,int can_id, int numb, u8 send_len);//通过RPDO1下发速度
void CAN2_RPDO_WriteData(s32 RPDO_data_low,s32 RPDO_data_high,u8 id,u8 RPDO_num, u8 send_len);//RPDO下发数据到驱动器


void CAN2_RPDO_Speed(s32 get_speed, int can_id, u8 pdo_numb, u8 send_len);//发送速度给PC
void CAN2_RPDO_Positon(s32 get_positon, int can_id, u8 pdo_numb, u8 send_len);//发送位置给PC工控机
void CAN2_RPDO_Control_Word(u16 get_word, int can_id, u8 pdo_numb, u8 send_len);



void start_pdo(int get_cob_id, int data,u8 send_len);
void CAN_Baud_Process(unsigned int Baud,CAN_InitTypeDef *CAN_InitStructure);

void point_reset_hal(void);//原点复位操作，驱动器零点和要求的零点重合

void driver_init_hal(void);//驱动器初始化操作
#define RIGHT_TURN  1   //正转
#define LEFT_TURN   2   //反转

void vel_mode_run(u8 dir_turn);

















#define PER_CIR   2000     //驱动器细分    
                                                                                                                                                                                                      //驱动器细分数   25600pul/r
#define MOTOR_CIR  200   //电机实际转一圈需要的脉冲数200个脉冲
//#define MOTOR_CIR  2000   //电机实际转一圈需要的脉冲数200个脉冲   PYQ 11.06

#define CIR_ANG    360  //一圈360度

#define DEG_STP     1.8  //步距角1.8度  1个脉冲是1.8度

#define FOR_DIR      0//正转
#define CON_DIR      1//反转

#define KEY1         GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_11)    //板子上的S1按键，按下时输出低电平

#define PB15          GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)//人体感应







//#define PB13          GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)
#define PA12          GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)//右光电

//#define PD9          GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)
#define PA4          GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)//中间光电

//#define PD11          GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11)
#define PA3          GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)//左光电，以驾驶员视角






//#define PER_FRE       50   //分20次增加 减少频率    将要达到的频率分20次增加到  和 减速到零
 
//#define PER_FRE       50  //分20次增加 减少频率    将要达到的频率分20次增加到  和 减速到零   11.06
#define PER_FRE       10   //11.16   20HZ单位的增加频率 直达1000HZ

#define FRE_CONSTANT   1000 // 500HZ  正常运动频率为1000HZ   复位是1000HZ

#define UN_TRIGGER_FLAG    0//未触发中间光电
#define FIR_TRIGGER_FLAG   1//第一次触发中间光电
#define SEC_TRIGGER_FLAG   2//第二次触发中间光电
#define STOP_TRIGGER_FLAG  3//复位结束

extern int trigger;



//int abs(int);

typedef struct MOTOR_CON{

u8 Pul_Cnt;

u8 tim3_end_flag;//计时结束标志

u8 reset_flag;//复位标志   0开始  1结束

u8 ori_flag;//方向标志  0正转   1反转

int pul_num;//用于保存实际绝对脉冲数

int ab_angle;//用于记录电机实际的角度 
	
u32 Frequecy;	//电机频率

u16 acc_num;  //加速次数
	
u16 dec_num;  //减速次数

u8 acc_flag_star;//加速标志   为1时开始加速

u8 dec_flag_star;//减速标志

u8 constant_flag;//匀速开始标志

u8 end_move_flag;//结束运动标志

u8 reset_active_flag;//通过上位机发送指令开始复位标志

u8 Limit_switch_forward_flag;//正转限位触发标志   PD11中断

u8 Limit_switch_reverse_flag;//反转限位触发标志   PB13中断

u8 start_check_switch_flag;//上电检测电平标志，如果检测到了置1 ，就不进入后面的复位函数了


u8 second_reset_flag;//二次复位标志




}motor_type;

extern motor_type MOTOR_PARA;

void motor_init(void);
void pwm_init(u16 fre,u16 psc);

void TIM2_init(u32 arr,u32 psc);

void motor_hardl(u16 angle,u8 dir,u32 fre);

void motor_stop(u8 stop_flag);

void Dir_gpio_init(void);
void KEY_init(void);

void swag_hardl(void);

void Swtich_Init(void);


void Reset_hal(void);



void Swtich_Move_Hal(void);


void Pul_Cal(void);//计算实时脉冲数

void Motor_start(u16 rx_angle ,u8 rx_direction);//根据下发的角度和方向运动


void Start_ACCE_Dec(void);

void End_Acc_Dec(void);


void Trap_Hal(u16 fre);//使用梯形公式加减速运动参数

void Check_Switch_hanle(void);//上电先检查每一个开关电平，电机才运动



void second_reset_handle(void);



;


extern motor_type MOTOR_PARA;
















#endif

