#ifndef _reset_motor
#define _reset_motor

#include "delay.h"
#include "sys.h"  

#include "stdlib.h"

#define CAN_BAUD_NUM    18		//�������ò����ʸ���



//�����ֵ�
#define  SET_VECLOCITY_OB    	0X2002  	//�����ٶȶ�������   
#define  SET_ACCELERATE_OB   	0X2006  	//���ü��ٶ�
#define  SET_DECELERATE_OB   	0X2007
#define  EMEGENCY_SHUTDOMN_OB   0x200C      //�����ر�
#define  RELEASE_SHUTDOWN_OB    0X200D

#define  READ_MOTOR_CURRENT_OB  0X2100      //������������
#define  FEED_BACK              0x2110      //��ȡ����ٶȣ���������������������������� ��λRPM
#define  READ_MCU_TEMP_OB       0X210F      //���������¶�
#define  READ_ERROR_FLAG_OB     0X2112      //�������־
#define  REDA_FAULT_FLAGS       0x2112      //�������־



//����
#define SET_VECLOCITY_CMD       0X2C        //�����ٶ�����


//SDO ���� ����
#define SDO_SEND_ID            0X601        //SDO����CAN_ID
#define SDO_RECEIVE_ID         0X581        //SDO����CAN_ID

#define  DLC_DATA              0X08         //


__packed typedef struct{

u8 low_index;//�������ֽ�
u8 high_index;//�������ֽ�


}FIRST_INDEX;


__packed typedef union{

	__packed  struct{
		u8 cmd;

		FIRST_INDEX fir_index;

		u8 sub_index;

		u8 data[4];//SDO��

		}TPYE_CANOPEN;

	u8 payload_data[8];
	

	u32 speed_robotq;//�����ٶ�
	
}TPYE_CANOPEN;

 
extern u16 rx_can2_id;

extern u8 can2_rx_data[8];

extern TPYE_CANOPEN thrrot_type;

void SET_SDO_HAL(u8 cmd,u16 index,u8 sub_index,u32 data );



void CAN2__ROBOTQ_Configuration(void);

void CAN2_SDO_WriteData(u8 cmd, u16 index, u8 sub_index, int data, u8 id);

void CAN2_RPDO_Set(u16 index,u8 sub_index,u32 data);

void CAN2_TPDO_Set(u16 robot_cmd,u16 index,u8 sub_index,u32 data);


void CAN2_RPDO1_Speed(s32 loop_speed, s32 high_data,int can_id, int numb, u8 send_len);//ͨ��RPDO1�·��ٶ�
void CAN2_RPDO_WriteData(s32 RPDO_data_low,s32 RPDO_data_high,u8 id,u8 RPDO_num, u8 send_len);//RPDO�·����ݵ�������


void CAN2_RPDO_Speed(s32 get_speed, int can_id, u8 pdo_numb, u8 send_len);//�����ٶȸ�PC
void CAN2_RPDO_Positon(s32 get_positon, int can_id, u8 pdo_numb, u8 send_len);//����λ�ø�PC���ػ�
void CAN2_RPDO_Control_Word(u16 get_word, int can_id, u8 pdo_numb, u8 send_len);



void start_pdo(int get_cob_id, int data,u8 send_len);
void CAN_Baud_Process(unsigned int Baud,CAN_InitTypeDef *CAN_InitStructure);

void point_reset_hal(void);//ԭ�㸴λ����������������Ҫ�������غ�

void driver_init_hal(void);//��������ʼ������
#define RIGHT_TURN  1   //��ת
#define LEFT_TURN   2   //��ת

void vel_mode_run(u8 dir_turn);

















#define PER_CIR   2000     //������ϸ��    
                                                                                                                                                                                                      //������ϸ����   25600pul/r
#define MOTOR_CIR  200   //���ʵ��תһȦ��Ҫ��������200������
//#define MOTOR_CIR  2000   //���ʵ��תһȦ��Ҫ��������200������   PYQ 11.06

#define CIR_ANG    360  //һȦ360��

#define DEG_STP     1.8  //�����1.8��  1��������1.8��

#define FOR_DIR      0//��ת
#define CON_DIR      1//��ת

#define KEY1         GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_11)    //�����ϵ�S1����������ʱ����͵�ƽ

#define PB15          GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)//�����Ӧ







//#define PB13          GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)
#define PA12          GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)//�ҹ��

//#define PD9          GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)
#define PA4          GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)//�м���

//#define PD11          GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11)
#define PA3          GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)//���磬�Լ�ʻԱ�ӽ�






//#define PER_FRE       50   //��20������ ����Ƶ��    ��Ҫ�ﵽ��Ƶ�ʷ�20�����ӵ�  �� ���ٵ���
 
//#define PER_FRE       50  //��20������ ����Ƶ��    ��Ҫ�ﵽ��Ƶ�ʷ�20�����ӵ�  �� ���ٵ���   11.06
#define PER_FRE       10   //11.16   20HZ��λ������Ƶ�� ֱ��1000HZ

#define FRE_CONSTANT   1000 // 500HZ  �����˶�Ƶ��Ϊ1000HZ   ��λ��1000HZ

#define UN_TRIGGER_FLAG    0//δ�����м���
#define FIR_TRIGGER_FLAG   1//��һ�δ����м���
#define SEC_TRIGGER_FLAG   2//�ڶ��δ����м���
#define STOP_TRIGGER_FLAG  3//��λ����

extern int trigger;



//int abs(int);

typedef struct MOTOR_CON{

u8 Pul_Cnt;

u8 tim3_end_flag;//��ʱ������־

u8 reset_flag;//��λ��־   0��ʼ  1����

u8 ori_flag;//�����־  0��ת   1��ת

int pul_num;//���ڱ���ʵ�ʾ���������

int ab_angle;//���ڼ�¼���ʵ�ʵĽǶ� 
	
u32 Frequecy;	//���Ƶ��

u16 acc_num;  //���ٴ���
	
u16 dec_num;  //���ٴ���

u8 acc_flag_star;//���ٱ�־   Ϊ1ʱ��ʼ����

u8 dec_flag_star;//���ٱ�־

u8 constant_flag;//���ٿ�ʼ��־

u8 end_move_flag;//�����˶���־

u8 reset_active_flag;//ͨ����λ������ָ�ʼ��λ��־

u8 Limit_switch_forward_flag;//��ת��λ������־   PD11�ж�

u8 Limit_switch_reverse_flag;//��ת��λ������־   PB13�ж�

u8 start_check_switch_flag;//�ϵ����ƽ��־�������⵽����1 ���Ͳ��������ĸ�λ������


u8 second_reset_flag;//���θ�λ��־




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


void Pul_Cal(void);//����ʵʱ������

void Motor_start(u16 rx_angle ,u8 rx_direction);//�����·��ĽǶȺͷ����˶�


void Start_ACCE_Dec(void);

void End_Acc_Dec(void);


void Trap_Hal(u16 fre);//ʹ�����ι�ʽ�Ӽ����˶�����

void Check_Switch_hanle(void);//�ϵ��ȼ��ÿһ�����ص�ƽ��������˶�



void second_reset_handle(void);



;


extern motor_type MOTOR_PARA;
















#endif

