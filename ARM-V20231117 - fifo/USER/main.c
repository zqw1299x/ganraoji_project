//��Ŀ���ƣ�������Ż���Դ���Ƽ����ݼ�¼��
//��Ŀʱ�䣺2019/03/03
//���ܣ�˫·����ͨ��@115200 @8400000��FATFS��SD�������ݼ�¼
//Ӳ��ƽ̨��STM32F429 without SRAM
//���ߣ�yf
//ά����huzj  20190701
//��˾���ƣ���̨�������Ƽ����޹�˾

#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "sys.h"
#include "led.h"
#include "key.h"   
#include "sdio_sdcard.h"       
#include "ff.h"  
#include "stdlib.h"
#include "string.h"
#include "exti.h"
#include "rng.h"
#include "stmflash.h"

#include "PWMCatch.h"
#include "adc.h"
#include "OnOffControl.h"

//#define ISNEW 0		//=1�����Ƿ����°�ĳ���20190703Ӳ���汾,=0�������ϰ�ĳ���;
//#define		MYADDR	4					// ID Ψһ��һ���豸һ���ţ������ظ���������

extern int irqCount;
#define USR_DBUG 0
FATFS fs;					//�ļ�ϵͳ���������߼�������
FIL file;					//�ļ�������1
FILINFO fileinfo;	        //�ļ���Ϣ
DIR dir;  				    //Ŀ¼	


extern u32 cnt_t;
extern u32 cnt_t_old;
extern u32 cnt_t_record_old;
extern u32 cnt_UAV_Run;
extern u32 cnt_UAV_Run_old;
#define 	AUTO_MODE 	1

#define 	POWER_GF_fu12V_ON		    GPIO_SetBits(GPIOD,GPIO_Pin_12)
#define 	POWER_GF_fu12V_OFF			GPIO_ResetBits(GPIOD,GPIO_Pin_12)

#define 	POWER_GF_24V_OFF			GPIO_ResetBits(GPIOD,GPIO_Pin_13)
#define 	POWER_GF_24V_ON		        GPIO_SetBits(GPIOD,GPIO_Pin_13)

#define 	POWER_WB_12V_ON			    GPIO_SetBits(GPIOD,GPIO_Pin_14)
#define 	POWER_WB_12V_OFF		    GPIO_ResetBits(GPIOD,GPIO_Pin_14)

#define 	POWER_SZ_12V_ON			    GPIO_SetBits(GPIOD,GPIO_Pin_15)
#define 	POWER_SZ_12V_OFF		    GPIO_ResetBits(GPIOD,GPIO_Pin_15)

#define 	POWER_GF_ZHITONG_ON			GPIO_SetBits(GPIOE,GPIO_Pin_12)
#define 	POWER_GF_ZHITONG_OFF		GPIO_ResetBits(GPIOE,GPIO_Pin_12)

#define 	POWER_FENGSHAN_1_ON			GPIO_SetBits(GPIOA,GPIO_Pin_11)
#define 	POWER_FENGSHAN_1_OFF		GPIO_ResetBits(GPIOA,GPIO_Pin_11)
#define 	POWER_FENGSHAN_2_ON			GPIO_SetBits(GPIOD,GPIO_Pin_6)
#define 	POWER_FENGSHAN_2_OFF		GPIO_ResetBits(GPIOD,GPIO_Pin_6)

#define		START_SIG		            GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) //PA0
//#if ISNEW == 0
//	#define		RESTART_WB_SIG		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) //PE0
//	#define		ARM_STARTED_H		GPIO_SetBits(GPIOE,GPIO_Pin_1)
//	#define		ARM_STARTED_L		GPIO_ResetBits(GPIOE,GPIO_Pin_1)
//#else   //�°��IO;
//	#define		RESTART_WB_SIG		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) //PB0
//	#define		ARM_STARTED_H		GPIO_SetBits(GPIOB,GPIO_Pin_11)
//	#define		ARM_STARTED_L		GPIO_ResetBits(GPIOB,GPIO_Pin_11)
//#endif

extern void TIM2_Int_Init(u32 arr,u16 psc);
extern void TIM1_PWM_Init(u32 arr,u32 psc);
u32 bw;
u8 file_record_flag=0;
u8 storage_status=0;	//0xff��ʾδ��ȷʶ��SD�� 0x01�ļ�����/��ʧ�� 0x02�ռ䲻�� ����0�쳣���رռ�¼ģʽ��0x00�������Լ�¼��������¼ģʽ
u8 record_enable=1;
u8 power_ctrl_auto=0;

u8 MYADDR = 1;//������ص��豸����Ϣ;
u8 command_MYADDR[4]={0};//������ص��豸����Ϣ;FLASH_SAVE_ADDR-1024��Ϊ�洢�豸�ŵĵ�ַ,λ������10;

u8 selfcheckStep = 0;//�Լ���е��ڼ�����;
u8 selfCheckState = 0;//bit0Ϊ�Ƿ���;bit1ΪVT1���״̬;bit2ΪVT2���״̬;bit3ΪS1���״̬;bit4ΪZJP1���״̬;bit5ΪZJP2���״̬;
u16 adcx;//�Լ�ض�����;
double adc_double=0;

u8 check_status=0x07;//���š����֡�΢�����ػ�״̬;
u16 cnt_num;
u8 checksum;
u8 checksum_num;
u8 P_rec_num = 0;

u32 gps_lon_tgt;//�����㾭��;
u32 gps_lat_tgt;//������γ��;
u32 gps_lon_now;//����;
u32 gps_lat_now;//γ��;
u16 gps_hight_now;//�߶�;
u8 time_hour;
u8 time_min;
u16 time_s;

u16 uart2_rec_num=0;//����Ժ�д����յ������ò���ָ�����//�����˻����д���������;

float float_gps_lon_now;
float float_gps_lat_now;
float float_gps_hight_now;
u32   temp_int = 0;
extern u8 ganRaojPower;
extern u8 ganRaojFlag;
extern u8 fashejiPwer;

#if OUTINTERFACE == SANYUAN  //����Ժ���õ�������
float float_time_FX;
u16 time_FX;//FXʱ��;
u8 uart2_rec_parameter = 0;//�յ��������õ�ָ��õ�����;

u8 unreal_target_num;//������Ŀ������;
u8 same_unreal_target_num;//��ͬ��Ŀ������;
u32 first_target_len;//��һ��Ŀ�����;
u16 second_first_delay;//��2������1��Ŀ�����ʱ;
u16 third_second_delay;//��3������2��Ŀ�����ʱ;
u16 fourth_third_delay;//��4������3��Ŀ�����ʱ;
u16 pwm;//�������;
u8 target_bind;//Ŀ��װ��;
u8 priority;//���ȼ�;
u8 grj_run_state;//���Ż��������;
u8 jam_type;//������ʽ;
u8 noise_FM;//������Ƶ;

#elif OUTINTERFACE == _8511
u8 self_check_state = 0xFF;//�Լ�״̬;
u8 command_style_back = 0x55;//ָ��ر�;
u8 work_state;//״̬λ;
u16 rec_laser_fre;//�����״�Ƶ��;
u16 repeat_cycle;//�ظ�����;
u16 pulse_width;//����;

#elif OUTINTERFACE == _70debug
u8 self_check_state = 0xFF;//�Լ�״̬;
u8 command_style_back = 0x55;//ָ��ر�;
u8 work_state;//״̬λ;
double double_rec_laser_fre;
u16 rec_laser_fre;//�����״�Ƶ��;
double double_repeat_cycle;
u16 repeat_cycle;//�ظ�����;
double double_pulse_width;
u16 pulse_width;//����;
double double_machine_temperature;
u8 machine_temperature;//�豸�¶�;
extern u16 byte_read_u4,byte_read_u6;

#endif

u8 work_time=0;//����ʱ��;
#define MARK_TYPE 2
#define MARK_TARGET_BIND 3
#define MARK_JAM_TYPE 4
#define MARK_FIRST_TARGET_LEN 5
#define MARK_SECOND_FIRST_DELAY 8
#define MARK_THIRD_SECOND_DELAY 11
#define MARK_FOURTH_THIRD_DELAY 14
#define MARK_SENSITIVITY 17				//1B
#define MARK_DOPPLER_MOD_0 18			//3B
#define MARK_DOPPLER_MOD_1 22			//3B
#define MARK_DOPPLER_MOD_2 26			//3B
#define MARK_DOPPLER_MOD_3 30			//3B
#define MARK_RECIEVE_WEAKEN 34 		//1B
#define MARK_UNREAL_TARGET_NUM 35
#define MARK_PWM 36
#define MARK_NOISE_FM 38
#define MARK_NOISE_AM 41 				//1B
#define MARK_TRANSMIT_TIME 42 	//2B
#define MARK_STOP_TIME 44 			//2B
#define MARK_SEND_WEAKEN 46 		//1B
#define MARK_PRIORITY 47
#define MARK_SAME_UNREAL_TARGET_NUM 48
#define MARK_SAME_UNREAL_TARGET_RELAY 49
const u8 command[TX1_BUFF_SIZE]={0xEB,0x90,0x01/*2�����*/,0x2F/*3װ������Ŀ��*/,0x21/*4������ʽ*/,\
			0xE2,0x04,00/*5-7�������0*/,0xEE,0x02,0x00/*8-10�������1*/,0xE2,0x04,0x00/*11-13�������2*/,0xDC,0x05,0x00/*14-16�������3*/,0x00/*17������*/,\
			0x16,0x9F,0x02,0x00/*18-21�����յ���0*/,0xC4,0x09,0x00,0x00/*22-25�����յ���1*/,0x44,0xDD,0x07,0x00/*26-29�����յ���2*/,0x5A,0x7C,0x0A,0x00/*30-33�����յ���3*/,0x00/*34����˥��*/,\
			0x03/*35��Ŀ�����*/,0xFF,0x3F/*36-37�������*/,0x76,0x62,0x00/*38-40������Ƶ*/,0xFF/*41��������*/,\
			0xFA,0x00/*42-43ת��ʱ��*/,0x00,0x00/*44-45ֹͣʱ��*/,0x40/*46����˥��*/,0x00/*47���ȸ���Ŀ��*/,0x00/*48��ͬ��Ŀ������*/,\
			/*49-69��������Ϊ����*/
			};
//const u8 command[TX1_BUFF_SIZE]={0xEB,0x90,0x01/*2�����*/,0x2F/*3װ������Ŀ��*/,0x21/*4������ʽ*/,\
//			0xFA,0x00,0xF0/*5-7�������0*/,0xA6,0x0E,0x00/*8-10�������1*/,0x4C,0x1D,0x00/*11-13�������2*/,0xF2,0x2B,0x00/*14-16�������3*/,0x00/*17������*/,\
//			0xA3,0x70,0x3D,0xFA/*18-21�����յ���0*/,0xA3,0x70,0x3D,0x0A/*22-25�����յ���1*/,0x47,0xE1,0x7A,0x14/*26-29�����յ���2*/,0xEB,0x51,0xB8,0x1E/*30-33�����յ���3*/,0x00/*34����˥��*/,\
//			0x03/*35��Ŀ�����*/,0xC4,0x09/*36-37�������*/,0x20,0x4E,0x00/*38-40������Ƶ*/,0xFF/*41��������*/,\
//			0xFA,0x00/*42-43ת��ʱ��*/,0x00,0x00/*44-45ֹͣʱ��*/,0x40/*46����˥��*/,0x00/*47���ȸ���Ŀ��*/,0x00/*48��ͬ��Ŀ������*/,\
//			/*49-69��������Ϊ����*/
//			};
u8 command_flash[TX1_BUFF_SIZE]={0};
char send_enable=0;
char set_flag=0;
char reset_wb_flag = 0;
			
//Ҫд�뵽STM32 FLASH���ַ�������
#define TEXT_LENTH sizeof(command)	 		  	//���鳤��	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)
#define FLASH_SAVE_ADDR  0x080E0000 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.
										//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.

__align(4) u8 datatemp[TX1_BUFF_SIZE];
//STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)TEXT_Buffer,SIZE);
//STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);	
			
void gpio_init()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOFʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOFʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOFʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��

    //START flag pin �¸��Ż��µĽӿ�
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //��ͨ����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);              //��ʼ��

    //	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;//AF0��������;
    //	GPIO_Init(GPIOE, &GPIO_InitStructure);          //��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4|GPIO_Pin_12;//AF0�������;|GPIO_Pin_12ΪNTCֱͨ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      //OD����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);              //��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//??? 
    POWER_FENGSHAN_1_OFF;//                      ����1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOD, &GPIO_InitStructure);//??? 
    POWER_FENGSHAN_2_OFF;                               //����2

    // run status led PA4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //��ͨ����ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      //OD����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);              //��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;				
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //��ͨ����ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      //OD����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOB, &GPIO_InitStructure);              //��ʼ��
    RUN_LED = 1;

    //PD 13 14 15 ��Դ����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOD, &GPIO_InitStructure);              //��ʼ��

    //PB 3 4 5 14 ��ֹSPI����SD
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOB, &GPIO_InitStructure);              //��ʼ��

    //ʹ�� 422 ���� PE13 PE14
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //��ͨ���ģʽ
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);              //��ʼ��
    GPIO_SetBits(GPIOE,GPIO_Pin_13 | GPIO_Pin_14);	
    GPIO_ResetBits(GPIOE,GPIO_Pin_0 | GPIO_Pin_1);
    //	GPIO_ResetBits(GPIOE,GPIO_Pin_12);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);              //��ʼ��
}

void uart1_send_u1_tx_buff()
{
    while(u1_tx_idle!=1);																				//�鿴����1�ϴη����Ƿ������ֻҪӲ��������һ���������ˣ�����1 8.4m ����2 115200�����۴���2�ն�죬������֮�䴮��1һ�������ˣ���					
    u1_tx_idle=0;
    DMA_Cmd(DMA2_Stream7,ENABLE);
}

void uart2_send_u2_tx_buff()
{
    while(u2_tx_idle!=1);
    u2_tx_idle=0;
    DMA_Cmd(DMA1_Stream6,ENABLE);		
}

void uart2_send_tx_buff_len(int len)
{
    while(u2_tx_idle!=1);
    DMA_Cmd(DMA1_Stream6, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {}
    DMA1_Stream6->NDTR = len;   //����dma���ݴ�����

    u2_tx_idle=0;
    DMA_Cmd(DMA1_Stream6,ENABLE);
}

void uart3_send_u1_tx_buff()
{
    while(u3_tx_idle!=1);
    u3_tx_idle=0;
    DMA_Cmd(DMA1_Stream3,ENABLE);
}

void uart4_send_u2_tx_buff()
{
    while(u4_tx_idle!=1);
    u4_tx_idle=0;
    DMA_Cmd(DMA1_Stream4,ENABLE);
}

void uart4_send_tx_buff_len(int len)
{
    while(u4_tx_idle!=1);
    DMA_Cmd(DMA1_Stream4, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}
    DMA1_Stream4->NDTR = len;   //����dma���ݴ�����

    u4_tx_idle=0;
    DMA_Cmd(DMA1_Stream4,ENABLE);
}

void uart6_send_tx_buff()
{
    while(u6_tx_idle!=1);
    u6_tx_idle=0;
    DMA_Cmd(DMA2_Stream6,ENABLE);
}

void uart6_send_tx_buff_len(int len)
{
    while(u6_tx_idle!=1);
    DMA_Cmd(DMA2_Stream6, DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE) {}
    DMA2_Stream6->NDTR = len;   //����dma���ݴ�����

    u6_tx_idle=0;
    DMA_Cmd(DMA2_Stream6,ENABLE);
}

//�õ�����ʣ������
//drv:���̱��("0:"/"1:")
//total:������	 ����λKB��
//free:ʣ������	 ����λKB��
//����ֵ:0,����.����,�������
u8 exf_getfree(u8 *drv,u32 *total,u32 *free)
{
	FATFS *fs1;
	u8 res;
    u32 fre_clust=0, fre_sect=0, tot_sect=0;
    //�õ�������Ϣ�����д�����
    res =(u32)f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
    if(res==0)
	{
	    tot_sect=(fs1->n_fatent-2)*fs1->csize;	    //�õ���������
	    fre_sect=fre_clust*fs1->csize;			    //�õ�����������	   
#if _MAX_SS!=_MIN_SS				  				//������С����512�ֽ�,��ת��Ϊ512�ֽ�
		tot_sect*=fs1->ssize/_MAX_SS;
		fre_sect*=fs1->ssize/_MAX_SS;
#endif	  
		*total=tot_sect>>1;	//��λΪKB
		*free=fre_sect>>1;	//��λΪKB 
 	}
	return res;
}	

char usr_init_sdcard()
{
	char err=0;
	char cnt=0;
	u8 res = 0;
	u32 total,free;
	
	while(SD_Init())            //��ⲻ��SD��
	{
		delay_ms(500);
		LED0=!LED0;
		err=0xff;
		cnt++;
		if(cnt>2)
			return err;
	}
	
	res=f_mount(&fs,"0:",1); 	//����SD�� 
	if(res != 0)
	{
		err=0xff;
		return err;
	}
	if(exf_getfree("0",&total,&free)==0)
	{
		free=free>>10;
		if(free<2048)
		{
			err = 0x02;
			return err;
		}
	}
	return err;
}

//CONFIG FILE ARCHITECTURE
//FILENUM...Do not delete it if you do not understand this system!
//00001
//00002
//.....
//99999 (MAX)
int get_file_num()
{
    char err = 0;
    char buffer[128]={0};
    int num=0;
    err = f_open(&file,"0:/config.txt",FA_OPEN_ALWAYS|FA_WRITE|FA_READ);
    if(err != 0)
    {
        return -1;
    }
    else
    {
        if(f_gets(buffer,128,&file) == 0)
        {
            f_puts("FILENUM...Do not modify it if you do not understand this system!\r\n",&file);
            f_close(&file);
            return 1;
        }
        while(f_gets(buffer,128,&file)!=0);				
        if(buffer[0]!='F')
        {
            if(buffer[0]>0x30)
                num=(buffer[0]-0x30)*10000;
            if(buffer[1]>0x30)
                num=num+(buffer[1]-0x30)*1000;
            if(buffer[2]>0x30)
                num=num+(buffer[2]-0x30)*100;
            if(buffer[3]>0x30)
                num=num+(buffer[3]-0x30)*10;
            if(buffer[4]>0x30)
                num=num+buffer[4]-0x30;
            f_close(&file);
            return num+1;
        }
        else
        {
            f_close(&file);
            return 1;
        }
    }
}

void test_disk_speed()
{
	u32 j;
	int num=0;
	char buffer[32];
	char err=0;
	TIM2_Int_Init(10,8400-1);			//1 = 0.1ms
	for(j=0;j<RX1_BUFF_SIZE;j++)
    {
        if((j%16)<10)
            uart1_rx_buffer0[j]=(j%16)+0x30;
        else
            uart1_rx_buffer0[j]=(j%16)+0x37;
    }
	start:			
		num=get_file_num();
		if(num>0)
		{
			f_open(&file,"0:/config.txt",FA_WRITE);			    //������ļ�
			sprintf(buffer,"%05d\r\n",num);							    //����ǰ�ļ����Ϊ�ַ���
			f_lseek(&file,file.fsize);											//�ҵ��ļ���β
			//f_puts(buffer,&file);													//д���ļ����
			f_write(&file,buffer,7,&bw);
			f_close(&file);
			
			sprintf(buffer,"0:/%05d.dat",num);		
			err = f_open(&file,buffer,FA_OPEN_ALWAYS|FA_WRITE);
			delay_ms(5);
			if(err==0)
			while(1)
            {	
                TIM_Cmd(TIM3,ENABLE);
                for(j=0;j<512;j++)
                {
                    f_lseek(&file,file.fsize);													//�ҵ��ļ���β
                    f_write(&file,uart1_rx_buffer0,RX1_BUFF_SIZE,&bw);
                }
                TIM_Cmd(TIM3,DISABLE);
                TIM_SetCounter(TIM3, 0);
                sprintf(uart2_tx_buffer,"%.3f/s , %.2f MB/s\n",(float)(cnt_t)/1000.0,(float)(8000)/(float)(cnt_t));	//����ǰ�ļ����Ϊ�ַ���			
                uart2_send_u2_tx_buff();
                USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);          //��������ж�;
                cnt_t=0;
                f_close(&file);
                goto start;
            }
		}
}

int record_new()
{
    char err = 0;
    char buffer[128]={0};
    int num=0;
    num=get_file_num();
    if(num>0)
    {
        f_open(&file,"0:/config.txt",FA_WRITE);			//������ļ�
        sprintf(buffer,"%05d\r\n",num);					//����ǰ�ļ����Ϊ�ַ���
        f_lseek(&file,file.fsize);						//�ҵ��ļ���β
        f_puts(buffer,&file);							//д���ļ����
        f_close(&file);									//�ر��ļ�
        
        //׼���½����ݼ�¼�ļ�
        //sprintf(buffer,"0:/%05d.dat",num);
        sprintf(buffer,"0:/%05d.dat",num);
        err = f_open(&file,buffer,FA_OPEN_ALWAYS|FA_WRITE);
        if (err==0)
            file_record_flag=1;
        else
            return -1;
    }
    else
    {
        return -11;
    }
    return 0;
}

void write_file()
{
    if(uart1_rec_flag==1)	//1�������Ѿ�׼���ã����ڲ���2
    {
        uart1_rec_flag=0;
        f_lseek(&file,file.fsize);									//�ҵ��ļ���β
        f_write(&file,uart1_rx_buffer0,RX1_BUFF_SIZE,&bw);				
    }
    else if(uart1_rec_flag==2)
    {
        uart1_rec_flag=0;
        f_lseek(&file,file.fsize);									//�ҵ��ļ���β
        f_write(&file,uart1_rx_buffer1,RX1_BUFF_SIZE,&bw);				
    }					
}

int finish_record()
{
	u8 res;
	res=f_close(&file);
	file_record_flag=0;
	if(res!=0)
		return -1;
	else
		return 0;
}

void Uart2CycleSend()
{
	//double double_rec_laser_fre;
	//u16 rec_laser_fre;//�����״�Ƶ��;
	//double double_repeat_cycle;
	//u16 repeat_cycle;//�ظ�����;
	//double double_pulse_width;
	//u16 pulse_width;//����;
	//double double_machine_temperature;

	double xishu = 312.5 / 2 / 3.14159265359 / 8192;
	int int16_temp ,temp;
	if((cnt_t - cnt_t_old)>1)
	{
		cnt_t_old = cnt_t;
	
#if OUTINTERFACE == SANYUAN  ////////////////////////////////////////////////////
		uart2_tx_buffer[3] = (MYADDR<<4)|0x02;
		uart2_tx_buffer[4] = check_status;//�齨�ػ�״̬;
		//��ӷ���ս��������Ϣ;
		gps_lon_now = (int)(float_gps_lon_now*10000000);//����4bytes;
		uart2_tx_buffer[5] = gps_lon_now;
		uart2_tx_buffer[6] = gps_lon_now >> 8;
		uart2_tx_buffer[7] = gps_lon_now >> 16;
		uart2_tx_buffer[8] = gps_lon_now >> 24;
		gps_lat_now = (int)(float_gps_lat_now*10000000);//γ��4bytes;
		uart2_tx_buffer[9] = gps_lat_now;
		uart2_tx_buffer[10] = gps_lat_now >> 8;
		uart2_tx_buffer[11] = gps_lat_now >> 16;
		uart2_tx_buffer[12] = gps_lat_now >> 24;
		uart2_tx_buffer[13] = uart2_rx_buffer[17];//FXʱ��s��λ;
		uart2_tx_buffer[14] = uart2_rx_buffer[18];//FXʱ��s��λ;
		uart2_tx_buffer[15] = uart2_rec_num;//���յ�֡����;
		uart2_tx_buffer[16] = uart2_rec_num >> 8;//���յ�֡����;
		uart2_tx_buffer[17] = target_bind;
		uart2_tx_buffer[18] = priority;
		if(uart2_tx_buffer[18] != 0x00)
			gps_lon_now++;
		uart2_tx_buffer[19] = jam_type;
		
		uart2_tx_buffer[21] = 0x31;//��������汾��,��4bitΪFPGA�汾�ţ���4bitΪǶ��ʽ�汾��;
		uart2_tx_buffer[22] = cnt_t>>1;
		uart2_tx_buffer[23] = (cnt_t>>1) >> 8;//ָ������ò��ŵ���0x00;
		checksum = 0;
		for(checksum_num=3;checksum_num<24;checksum_num++)
		{
			checksum = checksum + uart2_tx_buffer[checksum_num];
		}
		uart2_tx_buffer[24] = checksum;
		uart2_send_u2_tx_buff();
#elif OUTINTERFACE==_119////////////////////////////////////////////////////
		uart2_tx_buffer[3] = (MYADDR<<4)|0x02;
		uart2_tx_buffer[4] = check_status;//�齨�ػ�״̬;
		//��ӷ���ս��������Ϣ;
		gps_lon_now = (int)(float_gps_lon_now*10000000);//����4bytes;
		uart2_tx_buffer[5] = gps_lon_now;
		uart2_tx_buffer[6] = gps_lon_now >> 8;
		uart2_tx_buffer[7] = gps_lon_now >> 16;
		uart2_tx_buffer[8] = gps_lon_now >> 24;
		gps_lat_now = (int)(float_gps_lat_now*10000000);//γ��4bytes;
		uart2_tx_buffer[9] = gps_lat_now;
		uart2_tx_buffer[10] = gps_lat_now >> 8;
		uart2_tx_buffer[11] = gps_lat_now >> 16;
		uart2_tx_buffer[12] = gps_lat_now >> 24;
		uart2_tx_buffer[13] = cnt_t;//ʱ��;
		uart2_tx_buffer[14] = cnt_t >> 8;//ʱ��;
		uart2_tx_buffer[15] = cnt_t >> 16;//ʱ��;
		uart2_tx_buffer[16] = cnt_t >> 24;//ʱ��;
		uart2_tx_buffer[17] = 0;		
		uart2_tx_buffer[18] = 0;	
		uart2_tx_buffer[19] = 0;
		uart2_send_u2_tx_buff();
#elif OUTINTERFACE==PLANE////////////////////////////////////////////////////
		memcpy(uart2_tx_buffer+6,uart2_temp_buffer+6,60);
		uart2_tx_buffer[2] = 64;
		uart2_tx_buffer[3] = 0x05;
		uart2_tx_buffer[4] = MYADDR;
		uart2_tx_buffer[5] = check_status;
		uart2_tx_buffer[6] = cnt_UAV_Run>>1;
		uart2_tx_buffer[7] = (cnt_UAV_Run>>1) >> 8;
		uart2_tx_buffer[8] = (cnt_UAV_Run>>1) >> 16;
		uart2_tx_buffer[9] = uart2_rec_num;
		checksum = 0;
		for(cnt_num = 2;cnt_num < TX2_BUFF_SIZE-1;cnt_num++)
		{
			checksum += uart2_tx_buffer[cnt_num];
		}
		uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;//У���;
		uart2_send_u2_tx_buff();
#elif OUTINTERFACE==_8511		////////////////////////////////////////////////////
		uart2_tx_buffer[2] = cnt_UAV_Run>>1;
		uart2_tx_buffer[3] = cnt_UAV_Run>>9;        //֡����;
		uart2_tx_buffer[4] = self_check_state;      //�豸�Լ�״̬;
		uart2_tx_buffer[5] = 0xFA;                  //�¶�;
		uart2_tx_buffer[6] = command_style_back;
		uart2_tx_buffer[7] = work_state;
		uart2_tx_buffer[8] = rec_laser_fre;         //�����״�Ƶ��;
		uart2_tx_buffer[9] = rec_laser_fre>>8;
		uart2_tx_buffer[10] = repeat_cycle;         //�ظ�����;
		uart2_tx_buffer[11] = repeat_cycle>>8;
		uart2_tx_buffer[12] = pulse_width;          //����;
		uart2_tx_buffer[13] = pulse_width>>8;

		checksum = 0;
		for(cnt_num = 0;cnt_num < TX2_BUFF_SIZE-2;cnt_num++)
		{
			checksum += uart2_tx_buffer[cnt_num];
		}
		uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;//У���;��Ӧ60���ķɻ�Э��;
//		uart2_tx_buffer[TX2_BUFF_SIZE-1] = (~checksum)+1;//У���;
		uart2_send_u2_tx_buff();
#elif OUTINTERFACE == _70debug////////////////////////////////////////////////////
		memcpy(uart2_jiexi_buffer,uart1_rx_buffer0,63);
		double_machine_temperature = (uart2_jiexi_buffer[15] | (uart2_jiexi_buffer[16] << 8))*503.975/4096 - 273.15;
		if(uart2_jiexi_buffer[15]==0 && uart2_jiexi_buffer[16] == 0)
		double_machine_temperature = 0;
		double_repeat_cycle = ((uart2_jiexi_buffer[23] | (uart2_jiexi_buffer[24] << 8) | (uart2_jiexi_buffer[25] << 16) | (uart2_jiexi_buffer[26] << 24)) - 
							  (uart2_jiexi_buffer[3] | (uart2_jiexi_buffer[4] << 8) | (uart2_jiexi_buffer[5] << 16) | (uart2_jiexi_buffer[6] << 24)))*3.2;//�ظ�����;

		int16_temp = (uart2_jiexi_buffer[7] | (uart2_jiexi_buffer[8] << 8));
		if((int16_temp & 0x8000) == 0x8000)
		{
			int16_temp = ~(int16_temp & 0x7FFF);
			int16_temp = ((int16_temp) & 0x7FFF) + 1;
			xishu = -1 * xishu;
		}
		rec_laser_fre = uart2_jiexi_buffer[12] & 0x07;
		double_rec_laser_fre = ((rec_laser_fre*2+1)*78.125+int16_temp * xishu)*16;//�����״�Ƶ��;
		if(uart2_jiexi_buffer[7]==0 && uart2_jiexi_buffer[8] == 0)
        double_rec_laser_fre = 0;
		
		double_pulse_width = (uart2_jiexi_buffer[9] | (uart2_jiexi_buffer[10] << 8) | (uart2_jiexi_buffer[11] << 16))*3.2;//����;
		uart2_tx_buffer[0] = 0xDD;
		uart2_tx_buffer[1] = 0x77;
		uart2_tx_buffer[2] = cnt_UAV_Run>>1;            //֡����;
		uart2_tx_buffer[3] = cnt_UAV_Run>>9;
		uart2_tx_buffer[4] = 0x00;                      //��Ϣ�����;
		uart2_tx_buffer[5] = command_style_back;        //ָ��ر�
		uart2_tx_buffer[6] = MYADDR;                    //���Ż��豸ID;
		uart2_tx_buffer[7] = 0x00;
		uart2_tx_buffer[8] = 0x00;
		uart2_tx_buffer[9] = 0x00;
		uart2_tx_buffer[10] = uart2_jiexi_buffer[13];   //GPS����;
		uart2_tx_buffer[11] = uart2_jiexi_buffer[14];   //GPS����;
		temp = (int)(double_machine_temperature);
		uart2_tx_buffer[12] = temp;                     //�¶�;
		if(uart2_jiexi_buffer[18] == 0x00)              //��ƭ;
			uart2_tx_buffer[13] = 0x04;
		else if(uart2_jiexi_buffer[18] == 0x01)         //ѹ��;
			uart2_tx_buffer[13] = 0x08;
		else if(uart2_jiexi_buffer[18] == 0x02)         //���;
			uart2_tx_buffer[13] = 0x0C;
		uart2_tx_buffer[13] |= (((~uart2_jiexi_buffer[21])&0xC0) >> 6);//uart2_tx_buffer[13]d��bit1������ͨ��1;bit2��ͨ��2������;
		
		rec_laser_fre = (int)(double_rec_laser_fre);
		if(double_rec_laser_fre-rec_laser_fre>=0.5)
		rec_laser_fre+=1;
		uart2_tx_buffer[14] = rec_laser_fre;
		uart2_tx_buffer[15] = rec_laser_fre >> 8;       //�����״�Ƶ��;
		repeat_cycle = (int)(double_repeat_cycle/50);
		uart2_tx_buffer[16] = repeat_cycle;
		uart2_tx_buffer[17] = repeat_cycle >> 8;        //�ظ�����;
		
		pulse_width = (int)(double_pulse_width/50);
		uart2_tx_buffer[18] = pulse_width;
		uart2_tx_buffer[19] = pulse_width >> 8;         //����;
		
		uart2_tx_buffer[20] = uart2_jiexi_buffer[17];
		uart2_tx_buffer[25] = (uart2_jiexi_buffer[21] & 0x1F);
		uart2_tx_buffer[26] = uart2_jiexi_buffer[20];
		uart2_tx_buffer[27] = uart2_jiexi_buffer[19];
		checksum = 0;
		for(cnt_num = 0;cnt_num < 29;cnt_num++)
		{
			checksum += uart2_tx_buffer[cnt_num];
		}
		uart2_tx_buffer[29] = checksum;//У���;
        memcpy(uart4_tx_buffer,uart2_tx_buffer,30);
        uart4_send_tx_buff_len(30);
		//uart2_send_u2_tx_buff();
        uart2_send_tx_buff_len(30);

#endif
	}
}

void check_sanyuan_start()
{
    //��FPGA����ָ�����������
    if(START_SIG==0&&set_flag==0)
    {
        set_flag=1;	
        POWER_SZ_12V_ON;
        check_status |= 0x10;
        delay_ms(3000);
        POWER_GF_fu12V_ON;
        POWER_GF_24V_ON;
        delay_ms(200);
        POWER_GF_ZHITONG_ON;
        check_status |= 0x08;
        POWER_WB_12V_ON;
        check_status |= 0x20;
        check_status|=0x80;					//״̬��Ϊ����
        delay_ms(500);				//��Ժ���������źź������š�΢��500ms�ڹ���
        STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
        memcpy(uart1_tx_buffer,datatemp,70);
        uart1_send_u1_tx_buff();
        TIM_CtrlPWMOutputs(TIM1, ENABLE);//����������;
//			ARM_STARTED_H;	
    }
}
//u8 unreal_target_num;//������Ŀ������;
//u8 same_unreal_target_num;//��ͬ��Ŀ������;
//u32 first_target_len;//��һ��Ŀ�����;
//u8 second_first_delay;//��2������1��Ŀ�����ʱ;
//u8 third_second_delay;//��3������2��Ŀ�����ʱ;
//u8 fourth_third_delay;//��4������3��Ŀ�����ʱ;
//u16 pwm;//�������;
//u8 target_bind;//Ŀ��װ��;
//u8 priority;//���ȼ�;
//u8 grj_run_state;//���Ż��������;
//u8 jam_type;//������ʽ;
//u8 noise_FM;//������Ƶ;
void Uart2RecieveAnalyse()
{
		u16 weizhi =0;
#if OUTINTERFACE == SANYUAN
		check_sanyuan_start();
		//�����2��������
		if(uart2_rec_flag=='F')
		{
			uart2_rec_flag=0;
			RUN_LED=!RUN_LED;
			uart2_rec_num ++;//���յ�ָ�����;
			//��������������򣬽����GPS��ʱ����Ϣ���߶�
			time_hour = uart2_rx_buffer[3];
			time_min = uart2_rx_buffer[4];
			time_s = (uart2_rx_buffer[6] << 8) | uart2_rx_buffer[5];
			gps_lon_now = (uart2_rx_buffer[10] << 24) | (uart2_rx_buffer[9] << 16) | (uart2_rx_buffer[8] << 8) | uart2_rx_buffer[7];
			gps_lat_now = (uart2_rx_buffer[14] << 24) | (uart2_rx_buffer[13] << 16) | (uart2_rx_buffer[12] << 8) | uart2_rx_buffer[11];
			gps_hight_now = (uart2_rx_buffer[16] << 8) | uart2_rx_buffer[15];
			time_FX = (uart2_rx_buffer[18] << 8) | uart2_rx_buffer[17];

			float_gps_lon_now = ((float)(gps_lon_now)) * 0.00000084;
			float_gps_lat_now = ((float)(gps_lat_now)) * 0.00000042;
			float_gps_hight_now = ((float)(gps_hight_now)) * 0.1;
			float_time_FX = (float)(time_FX * 0.01);
			
			if(uart2_rx_buffer[35]==0x0e)
			{
				STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)command,SIZE);
				memcpy(command_flash,datatemp,70);
				uart2_rec_parameter = 0;
			}
			else if(uart2_rx_buffer[35]!=0x0f)
			{
				unreal_target_num = uart2_rx_buffer[19];
				same_unreal_target_num = uart2_rx_buffer[20];
				first_target_len = (uart2_rx_buffer[23] << 16) | (uart2_rx_buffer[22] << 8) | uart2_rx_buffer[21];
				second_first_delay = (uart2_rx_buffer[25] << 8) | uart2_rx_buffer[24];
				third_second_delay = (uart2_rx_buffer[27] << 8) | uart2_rx_buffer[26];
				fourth_third_delay = (uart2_rx_buffer[29] << 8) | uart2_rx_buffer[28];
				pwm = (uart2_rx_buffer[31] << 8) | uart2_rx_buffer[30];
				target_bind = uart2_rx_buffer[32] & 0xEF;
				priority = uart2_rx_buffer[33];
				grj_run_state = uart2_rx_buffer[34]&0x01;
				jam_type = (uart2_rx_buffer[34]>>1) & 0x0f;
				noise_FM = (uart2_rx_buffer[34]>>5) & 0x07;
				
				uart2_rec_parameter = 0;
				STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
				if(memcmp(datatemp,command,SIZE))
				{
					jam_type |= 0x80;
				}
				else
				{
					jam_type &= 0x7F;
				}
				datatemp[MARK_UNREAL_TARGET_NUM] = unreal_target_num - 1;
				datatemp[MARK_SAME_UNREAL_TARGET_NUM] = same_unreal_target_num;
				temp_int = first_target_len*10/3;
				if(first_target_len==0)
				{
					temp_int = 0x800000;
				}
				datatemp[MARK_FIRST_TARGET_LEN]=temp_int;
				datatemp[MARK_FIRST_TARGET_LEN+1]=temp_int >> 8;
				datatemp[MARK_FIRST_TARGET_LEN+2]=temp_int >> 16;//��һ��Ŀ������������ֽ�;
				
//				temp_int = (first_target_len+second_first_delay)*10/3;//��ֹ;
//				datatemp[MARK_SECOND_FIRST_DELAY]=temp_int;
//				datatemp[MARK_SECOND_FIRST_DELAY+1]=temp_int >> 8;
//				datatemp[MARK_SECOND_FIRST_DELAY+2]=temp_int >> 16;//�ڶ���Ŀ������������ֽ�;
//				
//				temp_int = (first_target_len+second_first_delay + third_second_delay)*10/3;
//				datatemp[MARK_THIRD_SECOND_DELAY]=temp_int;
//				datatemp[MARK_THIRD_SECOND_DELAY+1]=temp_int >> 8;
//				datatemp[MARK_THIRD_SECOND_DELAY+2]=temp_int >> 16;//������Ŀ������������ֽ�;
//				
//				temp_int = (first_target_len+second_first_delay + third_second_delay + fourth_third_delay)*10/3;
//				datatemp[MARK_FOURTH_THIRD_DELAY]=temp_int;
//				datatemp[MARK_FOURTH_THIRD_DELAY+1]=temp_int >> 8;
//				datatemp[MARK_FOURTH_THIRD_DELAY+2]=temp_int >> 16;//���ĸ�Ŀ������������ֽ�;
				
				temp_int = pwm*250;
				if(pwm==0)
				{
					temp_int = 0x8000;
				}
				datatemp[MARK_PWM]=temp_int;
				datatemp[MARK_PWM+1]=temp_int >> 8;
				
				datatemp[MARK_TARGET_BIND]=target_bind;
				if(priority != 0)
					datatemp[MARK_PRIORITY]=priority;
				else
					datatemp[MARK_PRIORITY]=0;
				
				if(jam_type == 0)
					temp_int=0x00;
				else if(jam_type == 1)
					temp_int=0x10;
				else
					temp_int=0x21;
				datatemp[MARK_JAM_TYPE]=temp_int;
				if(noise_FM == 0)
					temp_int=(u32)(20000*2.5205769);
				else if(noise_FM == 1)
					temp_int=(u32)(10000*2.5205769);
				else
					temp_int=(u32)(1000*2.5205769);

				datatemp[MARK_NOISE_FM]=temp_int;
				datatemp[MARK_NOISE_FM+1]=temp_int >> 8;
				datatemp[MARK_NOISE_FM+2]=temp_int >> 16;
				if(memcmp(datatemp,command_flash,SIZE))
				{
					STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
					memcpy(command_flash,datatemp,70);
				}
				
				USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
				uart2_rec_parameter = 1;
				
			}
			else
			{
				uart2_tx_buffer[3]=(MYADDR<<4)|0x0f;
				for(checksum_num=4;checksum_num<TX2_BUFF_SIZE-1;checksum_num++)
				{
					uart2_tx_buffer[checksum_num]=0;
				}
				uart2_tx_buffer[TX2_BUFF_SIZE-1] = 0x16;
				USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
				uart2_send_u2_tx_buff();
				
			}
		}//�����2��������---------------END		
#elif OUTINTERFACE == _119
		//�����2��������
		if(uart2_rec_flag=='F')				
		{
			
			USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
		}//�����2��������---------------END
#elif OUTINTERFACE==PLANE
		//�����2-���˻�;
		if(cnt_t >= 300 && (check_status&0x80)== 0x80)//���˻����泬��150s�Զ��ػ�;
		{
//			send_enable = 0;
//			check_status&=0x7F;
//			record_enable=0;
//			POWER_GF_24V_OFF;
			//POWER_GF_24V_OFF
//			check_status &= 0xF7;
//			POWER_SZ_12V_OFF;
//			check_status &= 0xEF;
//			POWER_WB_12V_OFF;
//			check_status &= 0xDF;	
//			cnt_t = 0;
		}
		if(uart2_rec_flag=='F')
		{
			//���ͻظ���Ϣ,��Ҫ���һ��ָ����;
			uart2_rec_flag=0;	
			STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
			switch(uart2_rx_buffer[3])
			{
				case 0x01:
					
					break;
				case 0x03:
					TIM_CtrlPWMOutputs(TIM1, DISABLE);//����������;
//					TIM_Cmd(TIM1, DISABLE);  //ʹ��TIM14
				
					check_status&=0x7F;
					power_ctrl_auto = 1;
					record_enable=0;
				
					POWER_GF_24V_OFF;
					POWER_GF_ZHITONG_OFF;
					check_status &= 0xF7;

					POWER_SZ_12V_OFF;
					check_status &= 0xEF;
					
					POWER_WB_12V_OFF;
					check_status &= 0xDF;
					send_enable = 0;
					cnt_t = 0;
					break;
				case 0x04:
					memcpy(uart2_tx_buffer+4, datatemp+2, 62);
					uart2_tx_buffer[0] = 0xEB;
					uart2_tx_buffer[1] = 0x90;
					uart2_tx_buffer[2] = 0x40;
					uart2_tx_buffer[3] = 0xF4;
					checksum = 0;
					for(cnt_num = 2;cnt_num < TX2_BUFF_SIZE-1;cnt_num++)
					{
						checksum += uart2_tx_buffer[cnt_num];
					}
					uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;//У���;
					USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
					uart2_send_u2_tx_buff();//���ͻظ�ָ��;
					//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�;

					break;
				case 0x65:
					memcpy(datatemp+MARK_TYPE, uart2_rx_buffer+6, uart2_rx_buffer[5]);
					break;
				case 0x66:
					memcpy(datatemp+MARK_TARGET_BIND, uart2_rx_buffer+6, uart2_rx_buffer[5]);
					break;
				case 0x67:
					if(uart2_rx_buffer[6] == 0)
						datatemp[MARK_JAM_TYPE] = 0x00;
					else if(uart2_rx_buffer[6] == 1)
						datatemp[MARK_JAM_TYPE] = 0x10;
					else if(uart2_rx_buffer[6] == 2)
						datatemp[MARK_JAM_TYPE] = 0x21;
					else if(uart2_rx_buffer[6] == 3)
						datatemp[MARK_JAM_TYPE] = 0x22;
					else if(uart2_rx_buffer[6] == 4)
						datatemp[MARK_JAM_TYPE] = 0x23;
					else if(uart2_rx_buffer[6] == 5)
						datatemp[MARK_JAM_TYPE] = 0x24;
					break;
				case 0x68:
					temp_int = (((uart2_rx_buffer[8]&0x7F)<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*25/6;
					temp_int |= ((uart2_rx_buffer[8] & 0x80)<<16);
					datatemp[MARK_FIRST_TARGET_LEN]=temp_int;
					datatemp[MARK_FIRST_TARGET_LEN+1]=temp_int >> 8;
					datatemp[MARK_FIRST_TARGET_LEN+2]=temp_int >> 16;//��һ��Ŀ������������ֽ�;
					break;
				case 0x69:
					temp_int = (((uart2_rx_buffer[8]&0x7F)<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*25/6;
					temp_int |= ((uart2_rx_buffer[8] & 0x80)<<16);
					datatemp[MARK_SECOND_FIRST_DELAY]=temp_int;
					datatemp[MARK_SECOND_FIRST_DELAY+1]=temp_int >> 8;
					datatemp[MARK_SECOND_FIRST_DELAY+2]=temp_int >> 16;//�ڶ���Ŀ������������ֽ�;
					break;
				case 0x6A:
					temp_int = (((uart2_rx_buffer[8]&0x7F)<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*10/3;
					temp_int |= ((uart2_rx_buffer[8] & 0x80)<<16);
					datatemp[MARK_THIRD_SECOND_DELAY]=temp_int;
					datatemp[MARK_THIRD_SECOND_DELAY+1]=temp_int >> 8;
					datatemp[MARK_THIRD_SECOND_DELAY+2]=temp_int >> 16;//������Ŀ������������ֽ�;
					break;
				case 0x6B:
					temp_int = (((uart2_rx_buffer[8]&0x7F)<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*10/3;
					temp_int |= ((uart2_rx_buffer[8] & 0x80)<<16);
					datatemp[MARK_FOURTH_THIRD_DELAY]=temp_int;
					datatemp[MARK_FOURTH_THIRD_DELAY+1]=temp_int >> 8;
					datatemp[MARK_FOURTH_THIRD_DELAY+2]=temp_int >> 16;//���ĸ�Ŀ������������ֽ�;
					break;
				case 0x6C:
					memcpy(datatemp+MARK_SENSITIVITY, uart2_rx_buffer+6, uart2_rx_buffer[5]);
					break;
				case 0x6D:
					temp_int = (((uart2_rx_buffer[9]&0x7F)<<24) | (uart2_rx_buffer[8]<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*13743.8953472;
					temp_int |= ((uart2_rx_buffer[9] & 0x80)<<24);
					datatemp[MARK_DOPPLER_MOD_0]=temp_int;
					datatemp[MARK_DOPPLER_MOD_0+1]=temp_int >> 8;
					datatemp[MARK_DOPPLER_MOD_0+2]=temp_int >> 16;//
					datatemp[MARK_DOPPLER_MOD_0+3]=temp_int >> 24;//
					break;
				case 0x6E:
					temp_int = (((uart2_rx_buffer[9]&0x7F)<<24) | (uart2_rx_buffer[8]<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*1000/3.2;
					temp_int |= ((uart2_rx_buffer[9] & 0x80)<<24);
					datatemp[MARK_DOPPLER_MOD_1]=temp_int;
					datatemp[MARK_DOPPLER_MOD_1+1]=temp_int >> 8;
					datatemp[MARK_DOPPLER_MOD_1+2]=temp_int >> 16;//
					datatemp[MARK_DOPPLER_MOD_1+3]=temp_int >> 24;//
					break;
				case 0x6F:
					temp_int = (((uart2_rx_buffer[9]&0x7F)<<24) | (uart2_rx_buffer[8]<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*17179.869184;
					temp_int |= ((uart2_rx_buffer[9] & 0x80)<<24);
					datatemp[MARK_DOPPLER_MOD_2]=temp_int;
					datatemp[MARK_DOPPLER_MOD_2+1]=temp_int >> 8;
					datatemp[MARK_DOPPLER_MOD_2+2]=temp_int >> 16;//
					datatemp[MARK_DOPPLER_MOD_2+3]=temp_int >> 24;//
					break;
				case 0x70:
					temp_int = (((uart2_rx_buffer[9]&0x7F)<<24) | (uart2_rx_buffer[8]<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*17179.869184;
					temp_int |= ((uart2_rx_buffer[9] & 0x80)<<24);
					datatemp[MARK_DOPPLER_MOD_3]=temp_int;
					datatemp[MARK_DOPPLER_MOD_3+1]=temp_int >> 8;
					datatemp[MARK_DOPPLER_MOD_3+2]=temp_int >> 16;//
					datatemp[MARK_DOPPLER_MOD_3+3]=temp_int >> 24;//
					break;
				case 0x71:
					memcpy(datatemp+MARK_RECIEVE_WEAKEN, uart2_rx_buffer+6, uart2_rx_buffer[5]);
					break;
				case 0x72:
					datatemp[MARK_UNREAL_TARGET_NUM] = uart2_rx_buffer[6]-1;
					break;
				case 0x73:
					temp_int = (uart2_rx_buffer[7] & 0x7F)*25 + (uart2_rx_buffer[6])*1000/3.2;
					temp_int |= ((uart2_rx_buffer[7] & 0x80)<<8);
					datatemp[MARK_PWM]=temp_int;
					datatemp[MARK_PWM+1]=temp_int >> 8;
					break;
				case 0x74:
					temp_int = (((uart2_rx_buffer[8]&0x7F)<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*2.5205769;
					temp_int |= ((uart2_rx_buffer[8] & 0x80)<<16);
					datatemp[MARK_NOISE_FM]=temp_int;
					datatemp[MARK_NOISE_FM+1]=temp_int >> 8;
					datatemp[MARK_NOISE_FM+2]=temp_int >> 16;//
					break;
				case 0x75:
					memcpy(datatemp+MARK_NOISE_AM, uart2_rx_buffer+6, uart2_rx_buffer[5]);
					break;
				case 0x76:
					temp_int = (uart2_rx_buffer[7]<<8 | uart2_rx_buffer[6])/3.2;
					datatemp[MARK_TRANSMIT_TIME]=temp_int;
					datatemp[MARK_TRANSMIT_TIME+1]=temp_int >> 8;
					break;
				case 0x77:
					temp_int = (uart2_rx_buffer[7]<<8 | uart2_rx_buffer[6])/3.2;
					datatemp[MARK_STOP_TIME]=temp_int;
					datatemp[MARK_STOP_TIME+1]=temp_int >> 8;
					break;
				case 0x78:
					memcpy(datatemp+MARK_SEND_WEAKEN, uart2_rx_buffer+6, uart2_rx_buffer[5]);
					datatemp[MARK_SEND_WEAKEN] |= 0x40; 
					break;
				case 0x79:
					memcpy(datatemp+MARK_PRIORITY, uart2_rx_buffer+6, uart2_rx_buffer[5]);
					break;
				case 0x7A:
					memcpy(datatemp+MARK_SAME_UNREAL_TARGET_NUM, uart2_rx_buffer+6, uart2_rx_buffer[5]);
					break;
				case 0x7B:
					temp_int = (((uart2_rx_buffer[9]&0x7F)<<24) | (uart2_rx_buffer[8]<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))/3.2;
					datatemp[MARK_SAME_UNREAL_TARGET_RELAY]=temp_int;
					datatemp[MARK_SAME_UNREAL_TARGET_RELAY+1]=temp_int >> 8;
					datatemp[MARK_SAME_UNREAL_TARGET_RELAY+2]=temp_int >> 16;//
					datatemp[MARK_SAME_UNREAL_TARGET_RELAY+3]=temp_int >> 24;//
					break;
				default:
					break;
			}
			if(uart2_rx_buffer[3] >= 0x65 && uart2_rx_buffer[3] <= 0x7B)
			{
				STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
				memcpy(uart1_tx_buffer,datatemp,70);
				uart1_send_u1_tx_buff();
			}
			if(uart2_rx_buffer[3]!=0x04)
			{
				uart2_tx_buffer[2] = 64;
				uart2_tx_buffer[3] = 0x00;
				uart2_tx_buffer[4] = MYADDR;
				uart2_tx_buffer[5] = uart2_rx_buffer[3];
				checksum = 0;
				for(cnt_num = 2;cnt_num < TX2_BUFF_SIZE-1;cnt_num++)
				{
					checksum += uart2_tx_buffer[cnt_num];
				}
				uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;//У���;
				USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
				uart2_send_u2_tx_buff();    //���ͻظ�ָ��;
				//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�;
			}
			
			if(uart2_rx_buffer[3] == 0x02)
			{
				POWER_SZ_12V_ON;
				check_status |= 0x10;
				delay_ms(3000);
				
				POWER_GF_fu12V_ON;
				check_status|=0x80;					//״̬��Ϊ����
				POWER_GF_24V_ON;
				delay_ms(200);
				POWER_GF_ZHITONG_ON;
				delay_ms(200);
				check_status |= 0x08;
				POWER_WB_12V_ON;
				check_status |= 0x20;
				delay_ms(1000);
				STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
				memcpy(uart1_tx_buffer,datatemp,70);
				uart1_send_u1_tx_buff();
				cnt_t=0;
				TIM_CtrlPWMOutputs(TIM1, ENABLE);//����������;
				send_enable=1;
				record_enable=1;
				uart2_rec_num++;
			}
		}
#elif OUTINTERFACE == _8511
//u8 self_check_state = 0xFF;   //�Լ�״̬;
//u8 command_style_back = 0x55; //ָ��ر�;
//u8 work_state;                //״̬λ;
//u16 rec_laser_fre;            //�����״�Ƶ��;
//u16 repeat_cycle;             //�ظ�����;
//u16 pulse_width;              //����;
    
		if(uart2_rec_flag=='F')
		{
			uart2_rec_flag=0;	
			TEST_LED = !TEST_LED;
			if((uart2_rx_buffer[4] != 0x01) && (uart2_rx_buffer[2] != 0x13))//����1�ŷɻ���ִ��ָ��;
			{
				USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
				return;
			}
				
			command_style_back = uart2_rx_buffer[5];
			switch(uart2_rx_buffer[5])
			{
				case 0x33://������;
					RUN_LED = 0;
					POWER_GF_fu12V_ON;
					POWER_SZ_12V_ON;
					check_status |= 0x10;
					delay_ms(500);
					POWER_GF_24V_ON;
					delay_ms(200);
					POWER_GF_ZHITONG_ON;
					check_status |= 0x08;
					POWER_WB_12V_ON;
					check_status |= 0x20;
					check_status|=0x80;		//״̬��Ϊ����;
					break;
				case 0x35://�ظ���;
					RUN_LED = 1;
					POWER_GF_24V_OFF;
                    POWER_GF_ZHITONG_OFF;
					check_status &= 0x08;
					POWER_WB_12V_OFF;
					check_status &= 0xDF;
					POWER_SZ_12V_OFF;
					check_status &= 0xEF;
					check_status&=0x7F;		//״̬��Ϊ�ػ�;
					break;
				case 0x41://����1;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_0,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x43://����2;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_1,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x45://����3;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_2,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x47://����4;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_3,(u32*)datatemp,SIZE-1);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x49://����5;
					
					break;
				case 0x4B://����6;
					
					break;
				case 0x4D://����7;
					
					break;
				case 0x4F://����8;
					
					break;
				case 0x53://�Լ�;
					
					break;
				case 0x55://����;
					//���Ϳ��عر�;
					break;
				case 0x57://ж������;
					STMFLASH_Erase(FLASH_SAVE_ADDR,SIZE*1000);
					STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE-1);
					checksum = 0;
					for(cnt_num = 0;cnt_num < SIZE*4;cnt_num++)
					{
						if(datatemp[cnt_num] != 0xFF)
						{
							checksum = 0x01;
							break;
						}
					}
					if(checksum == 0x00)
					{
						command_style_back = 0x58;
					}
					else
					{
						command_style_back = 0x59;
					}
					break;
				case 0x61://����;
					
					break;
				case 0x63://����;
					
					break;
				default:
					break;
			}
			if(uart2_rx_buffer[2] != 0x13)
			{
//				checksum = 0;
//				for(cnt_num = 0;cnt_num < TX2_BUFF_SIZE-1;cnt_num++)
//				{
//					checksum += uart2_tx_buffer[cnt_num];
//				}
//				uart2_tx_buffer[TX2_BUFF_SIZE-1] = (~checksum) + 1;//У���;
//				USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
//				uart2_send_u2_tx_buff();//���ͻظ�ָ��;
			}
			else
			{
				uart1_tx_buffer[2] = 0x02;
				memcpy(uart1_tx_buffer+3,uart2_rx_buffer+3,10);
				uart1_send_u1_tx_buff();
			}
			USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2

		}
#elif OUTINTERFACE == _70debug          //�˲��ֵ�Э���Ӧ�ڲ�ͨ�ŵ��ֽ��������͵Ļ��ǰ��ս���WURENJI���ֽ�������
		if(uart2_rec_flag=='F')
		{
			uart2_rec_flag=0;	
			TEST_LED = !TEST_LED;
			if(uart2_rx_buffer[2] == 0xCE)
			{
				command_MYADDR[0] = 0x00;
				command_MYADDR[1] = uart2_rx_buffer[6];
				command_MYADDR[2] = 0x00;
				command_MYADDR[3] = 0x00;
				STMFLASH_Write(FLASH_SAVE_ADDR-1024,(u32*)command_MYADDR,1);
				command_MYADDR[1] = 0x00;
				STMFLASH_Read(FLASH_SAVE_ADDR-1024,(u32*)command_MYADDR,1);
				if(command_MYADDR[1] == uart2_rx_buffer[6])
					MYADDR= uart2_rx_buffer[6];
				else
					MYADDR = 1;
				USART_Cmd(USART2, ENABLE);      //ʹ�ܴ���2
				return;
			}

			if((uart2_rx_buffer[4] != MYADDR))  //�ɻ��Ų�����ִ��ָ��;
			{
				USART_Cmd(USART2, ENABLE);      //ʹ�ܴ���2
				return;
			}

			if(uart2_rx_buffer[2] == 0xCD)
			{
				uart3_tx_buffer[0] = 0x85;
				uart3_tx_buffer[1] = 0x11;
				uart3_tx_buffer[2] = uart2_rx_buffer[6];
				uart3_tx_buffer[3] = uart2_rx_buffer[7];
				uart3_send_u1_tx_buff();
				USART_Cmd(USART2, ENABLE);      //ʹ�ܴ���2
				return;
			}

			command_style_back = uart2_rx_buffer[5];
			switch(uart2_rx_buffer[5])
			{
				case 0x10://��ǰ��������;
					
					break;
				case 0x11://����1����;
					
					break;
				case 0x13://����2����;
					
					break;
				case 0x15://����3����;
					
					break;
				case 0x17://����4����;
					
					break;
				case 0x19://����5����;
					
					break;				
				case 0x1B://����6����;
					
					break;
				case 0x1D://����7����;
					
					break;
				case 0x1F://����8����;
					
					break;
				case 0x33://������;
					RUN_LED = 0;
					POWER_GF_fu12V_ON;
					delay_ms(2000);
					POWER_WB_12V_ON;
					check_status |= 0x20;
					delay_ms(2000);
					POWER_SZ_12V_ON;
					check_status |= 0x10;
					delay_ms(2000);
					POWER_GF_24V_ON;
					delay_ms(200);
					POWER_GF_ZHITONG_ON;
					check_status |= 0x08;
					check_status|=0x80;				//״̬��Ϊ����;
					break;
				case 0x35://�ظ���;
					RUN_LED = 1;
					POWER_GF_24V_OFF;
					POWER_GF_ZHITONG_OFF;
					check_status &= 0x08;
					break;
				case 0x41://����1;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_0,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_0+220,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_0+440,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_0+660,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x43://����2;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_1,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_1+220,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_1+440,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_1+660,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x45://����3;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_2,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_2+220,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_2+440,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_2+660,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x47://����4;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_3,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_3+220,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_3+440,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_3+660,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x49://����5;
										STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_4,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_4+220,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_4+440,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_4+660,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x4B://����6;
										STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_5,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_5+220,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_5+440,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_5+660,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x4D://����7;
										STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_6,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_6+220,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_6+440,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_6+660,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x4F://����8;
										STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_7,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_7+220,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_7+440,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
				delay_ms(50);
									STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_7+660,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x53://�Լ�;
					
					break;
				case 0x55://����;
					RUN_LED = 1;
					POWER_GF_24V_OFF;
					POWER_GF_ZHITONG_OFF;
					check_status &= 0x08;
					POWER_WB_12V_OFF;
					check_status &= 0xDF;
					POWER_SZ_12V_OFF;
					check_status &= 0xEF;
					check_status&=0x7F;				//״̬��Ϊ�ػ�;
					POWER_GF_fu12V_OFF;
				
					break;
				case 0x57://ж������;
					STMFLASH_Erase(FLASH_SAVE_ADDR,SIZE*2000);
					STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE-1);
					checksum = 0;
					for(cnt_num = 0;cnt_num < SIZE*4;cnt_num++)
					{
						if(datatemp[cnt_num] != 0xFF)
						{
							checksum = 0x01;
							break;
						}
					}
					if(checksum == 0x00)
					{
						command_style_back = 0x58;
					}
					else
					{
						command_style_back = 0x59;
					}
					break;
				case 0x61://����;
					
					break;
				case 0x63://����;
					
					break;
				default:
					break;
			}
			USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
		}
		else if(uart2_rec_flag == 'P')  //�������õ�����;
		{
			if(parameters[2] == 1)
			{
				STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)ALL_parameters,2500);
				weizhi = parameters[10];
				if(parameters[10] == 0x0A)
					weizhi=3;
				memcpy(ALL_parameters+(parameters[3] * OFFSETS_ADDR_1 +  weizhi* Param1_OFFSET),parameters+8,220);
				STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)ALL_parameters,2500);
				STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)ALL_parameters_yanzheng,2500);
				if(memcmp(ALL_parameters,ALL_parameters_yanzheng,2500))
				{
					BACK_LED = !BACK_LED;
				}
			}
			else
			{
                memcpy(uart1_tx_buffer,parameters+8,TX1_BUFF_SIZE);
                uart1_tx_buffer[0] = 0xEB;
                uart1_tx_buffer[1] = 0x90;
                uart1_send_u1_tx_buff();
			}
			
			uart2_rec_flag=0;
			TEST_LED = !TEST_LED;
			USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
		}
        
		else if(uart2_rec_flag == 'U')
		{
				int i = 0;
				uart2_rec_flag=0;
				//����2����ת��
				byte_read_u4 = ring_buffer_read(&usart2_rx_fifo, (uint8_t *)uart4_tx_buffer, 1024);
				if(byte_read_u4 > 0)
				{
						//uart4_send_tx_buff_len(byte_read_u4);
						logPrintU4(byte_read_u4);
				} else  uart2_rec_flag = 0;

				byte_read_u6 = ring_buffer_read(&usart2_rx_fifo1, (uint8_t *)uart6_tx_buffer, 1024);
				if(byte_read_u6 > 0)
				{
						uart6_send_tx_buff_len(byte_read_u6);
				} else  uart2_rec_flag = 0;          
		}

		if(ganRaojFlag == 1)
		{
				int i = 0;
				ganRaojFlag=0;
				if(ganRaojPower == 0x03)
				{
						POWER_GF_fu12V_ON;		    
						POWER_WB_12V_ON;		    
						POWER_SZ_12V_ON;
						printf("value:0x03\r\n");
				}
				else if(ganRaojPower == 0x02)
				{
						POWER_GF_fu12V_ON;		    
						POWER_WB_12V_ON;		    
						POWER_SZ_12V_ON;        
						printf("value:0x02\r\n");
				}
				else if(ganRaojPower == 0x01)
				{  
						POWER_SZ_12V_ON;  
						printf("value:0x01\r\n");
				}
				else if(ganRaojPower == 0x00)
				{
						POWER_GF_fu12V_OFF;		    
						POWER_GF_24V_OFF;	         
						POWER_WB_12V_OFF;		    
						POWER_SZ_12V_OFF;
						printf("value:0x00\r\n");
				}         
				if(fashejiPwer == 1){
						POWER_GF_fu12V_ON;		    
						POWER_GF_24V_ON;	        
						POWER_WB_12V_ON;		    
						POWER_SZ_12V_ON; 
						printf("fasheji:0x01\r\n");
				}
				else if(fashejiPwer == 0)
				{
						POWER_GF_24V_OFF;
						printf("fasheji:0x00\r\n");                
				}
			}
			if(uart6_rec_flag == 'W')
			{
					uart6_rec_flag = 0;
					uart2_send_tx_buff_len(uart6To2Len);
				  logPrintU4(uart6To2Len);
					//uart4_send_tx_buff_len(146);
			}
    
#endif
}

void selfcheck()
{
	WB_ZJPSW_ON;
	WB_ZJMSW_ON;
	WB_ZJSW_ON;
	WB_AF1_OFF;
	WB_AF0_ON;
	selfcheckStep |= 0x01;              //�Լ�����һ��
	delay_ms(500);
	printf("\r\nͨ����*********************************\n");
	//���ʹ��ڳ���,��FPGA�������ռ����������;
	selfCheckState |= (Read_VT1_SIG << 1);
	if(selfCheckState&0x02)
	{
		printf("\r\nVT1����-----------------\n");
	}
	else
	{
		printf("\r\nVT1�쳣-----------------\n");
	}
	WB_AF1_ON;
	WB_AF0_OFF;
	selfcheckStep |= 0x02;              //�Լ����ڶ���
	delay_ms(500);                      //�������ȴ�FPGA���з��ͼ���Լ�����͹���1���ļ��;
	selfCheckState |= (Read_VT2_SIG << 2);
	if(selfCheckState&0x04)
	{
		printf("\r\nVT2����-----------------\n");
	}
	else
	{
		printf("\r\nVT2�쳣-----------------\n");
	}
	adcx = Get_Adc_Average(ADC_Channel_13,50);//S1
	adc_double = (adcx*3.29/4096);
	printf("����1��ѹֵ=%f\n", adc_double);
	adcx = Get_Adc_Average(ADC_Channel_14,50);//ZJP1
	adc_double = (adcx*3.29/4096);
	printf("����1��ZJP1ֵ=%f\n", adc_double);

	WB_AF1_ON;
	WB_AF0_ON;
	selfcheckStep |= 0x04;  //�Լ���������
	delay_ms(500);          //���������з�����͹���2���ļ��;
	adcx = Get_Adc_Average(ADC_Channel_13,50);//s1
	adc_double = (adcx*3.29/4096);
	printf("����1��ѹֵ=%f\n", adc_double);
	adcx = Get_Adc_Average(ADC_Channel_15,50);//ZJP1
	adc_double = (adcx*3.29/4096);
	printf("����1��ZJP2ֵ=%f\n", adc_double);
	WB_ZJPSW_OFF;
	WB_ZJMSW_OFF;
	WB_ZJSW_OFF;
	WB_AF1_OFF;
	WB_AF0_OFF;
	selfcheckStep |= 0x08;  //�Լ����
	if((selfcheckStep & 0x0F) ==0x0F )
	{
		//�Ե�̨���ͺϸ���Լ���Ϣ;
		selfCheckState = 0x00;
	}
}
void main_process()
{
	short temp_TC1047 = -1;
  int i = 0;
	while(1)
	{
		if((selfCheckState & 0x01))
		{
			selfcheck();
			continue;
		}
		//�ϵ�򿪻����ѯ��¼ʹ�ܣ���״̬���Ƿ����ڼ�¼��Ĭ��״̬��ʹ������״̬������δ�ڼ�¼�У��򴴽����ļ���ʼ�ȴ�����׼��д��
		if(record_enable==1&&storage_status==0&&file_record_flag==0)
		{
			if(record_new()==0)
			{
				file_record_flag=1;							//��������λ�ļ����ڼ�¼��־
			}
		}
		else if(file_record_flag==1)
		{
			if(record_enable!=1)
			{
				finish_record();
				file_record_flag=0;	
			}
		}
 
//		if((cnt_UAV_Run - cnt_UAV_Run_old)>3)
//		{
//			cnt_UAV_Run_old = cnt_UAV_Run;
//			temp_TC1047 = Get_TC1047_Temprate();
//			if(temp_TC1047>20)
//			{
//				POWER_FENGSHAN_1_ON;
//				POWER_FENGSHAN_2_ON;
//			}
//			if(temp_TC1047<10)
//			{
//				POWER_FENGSHAN_1_OFF;
//				POWER_FENGSHAN_2_OFF;
//			}
//		}

//		if (send_enable==1)
//			Uart2CycleSend();   //ѭ������;

//        for(i=0;i<32;i++)uart1_rx_buffer0[i] = i;
//        
//        for(i=0;i<10;i++){
//            f_lseek(&file,file.fsize);
//            f_write(&file,uart1_rx_buffer0,32,&bw);
//        }

		Uart2RecieveAnalyse();  //����2���յ����ݺ�Ľ���;
		if(uart1_rec_flag==1)	//���ݴ���1��������------1�������Ѿ�׼���ã����ڲ���2
		{
				uart1_rec_flag=0;
		//if(uart1_rx_buffer0[0]==(char)(HEAD_INTERNAL_1)&&uart1_rx_buffer0[1]==(char)(HEAD_INTERNAL_2)&&uart1_rx_buffer0[2]==(char)(HEAD_INTERNAL_3)&&uart1_rx_buffer0[3]==(char)(HEAD_INTERNAL_4))				//������ȷ
		//{
				printf("file uart1_rec_flag:1-%d\r\n",file_record_flag);
				//if(uart1_rx_buffer0[2]==0x05&&file_record_flag==1)
				if(file_record_flag==1)																			//��������������Ѿ������ļ������Ѿ����������ļ��Ѿ������򿪣�													
				{
						f_lseek(&file,file.fsize);
						f_write(&file,uart1_rx_buffer0,RX1_BUFF_SIZE,&bw);
						//f_write(&file,uart1_rx_buffer0,RX1_BUFF_SIZE,&bw);
						RUN_LED=!RUN_LED;
				}
				if(send_enable==1)
				{
						#if OUTINTERFACE == SANYUAN
								memcpy(uart2_tx_buffer+3,uart1_rx_buffer0+2,21);
								uart2_tx_buffer[3] = (MYADDR<<4)|0x05;
								checksum = 0;
								for(cnt_num = 2;cnt_num < TX2_BUFF_SIZE-1;cnt_num++)
								{
										checksum += uart2_tx_buffer[cnt_num];
								}
								uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;        //У���;
								uart2_send_u2_tx_buff();

						//��������һ���ֽ�У��
						#elif OUTINTERFACE == _119
								uart2_tx_buffer[4]=0x03;	                        //������ʽ
								memcpy(uart2_tx_buffer+5,uart1_rx_buffer0+3,250);   //�ϴ�������ݣ�FPGA���� eb 90 05 + data���ϴ�eb 90 fc addr|05 xx + data��250 = 25������������
								uart2_tx_buffer[3] = (MYADDR<<4)|0x05;
								checksum = 0;
								for(cnt_num = 2;cnt_num < TX2_BUFF_SIZE-1;cnt_num++)
								{
										checksum += uart2_tx_buffer[cnt_num];
								}
								uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;//У���;
								uart2_send_u2_tx_buff();

						#elif OUTINTERFACE==PLANE
								//����������Ҫ������ظ��Ų�����ָ������;
								memcpy(uart2_temp_buffer+6,uart1_rx_buffer0+3,60);
								
						#elif OUTINTERFACE == _8511
								//����������Ҫ������ظ��Ų�����ָ������;
								memcpy(uart2_temp_buffer+6,uart1_rx_buffer0+3,60);
						#elif OUTINTERFACE == _70debug
								//memcpy(uart2_temp_buffer+3,uart1_rx_buffer0+3,60);
								
						#endif
				}	
		//}
		}
		else if(uart1_rec_flag==2)
		{
			uart1_rec_flag=0;
			//if(uart1_rx_buffer1[0]==(char)(HEAD_INTERNAL_1)&&uart1_rx_buffer1[1]==(char)(HEAD_INTERNAL_2)&&uart1_rx_buffer1[2]==(char)(HEAD_INTERNAL_3)&&uart1_rx_buffer1[3]==(char)(HEAD_INTERNAL_4))				//������ȷ
			//{
					printf("file uart1_rec_flag:2-%d\r\n",file_record_flag);
					//if(uart1_rx_buffer1[2]==0x05&&file_record_flag==1)
					if(file_record_flag==1)																			//��������������Ѿ������ļ������Ѿ����������ļ��Ѿ������򿪣�		
					{
							f_lseek(&file,file.fsize);
							f_write(&file,uart1_rx_buffer1,RX1_BUFF_SIZE,&bw);
							RUN_LED=!RUN_LED;
					}
					if(send_enable==1)
					{
							#if OUTINTERFACE == SANYUAN
									memcpy(uart2_tx_buffer+3,uart1_rx_buffer1+2,21);
									uart2_tx_buffer[3] = (MYADDR<<4)|0x05;
									uart2_send_u2_tx_buff();

							#elif OUTINTERFACE == _119
									uart2_tx_buffer[4]=0x03;	//������ʽ
									memcpy(uart2_tx_buffer+5,uart1_rx_buffer1+3,250); //�ϴ�������ݣ�FPGA���� eb 90 05 + data���ϴ�eb 90 fc addr|05 xx + data��250 = 25������������
									uart2_tx_buffer[3] = (MYADDR<<4)|0x05;
									uart2_send_u2_tx_buff();

							#elif OUTINTERFACE == PLANE
									memcpy(uart2_temp_buffer+6,uart1_rx_buffer0+3,60);
							#elif OUTINTERFACE == _8511
									//����������Ҫ������ظ��Ų�����ָ������;
									memcpy(uart2_temp_buffer+6,uart1_rx_buffer0+3,60);
							#elif OUTINTERFACE == _70debug
									//memcpy(uart2_temp_buffer+3,uart1_rx_buffer0+3,60);
							#endif
					}
			//}
		}
		if(cnt_t-cnt_t_record_old>0 && file_record_flag==1)		//����ļ����ڼ�¼����ÿ500ms����һ���ļ�
		{
				f_sync(&file);
		}
		if(cnt_t>cnt_t_record_old){
				cnt_t_record_old = cnt_t;
				for(i= 0;i<220;i++)uart1_tx_buffer[i] = i;
				uart1_send_u1_tx_buff();
		}
	}
}

/*�������������ʼ����*/
u8 rng_start()
{
	u8 err=0;
	u8 i = 0;
	while(1)
	{
		err=RNG_Init();
		if(err==0||i==3)
			break;
		else
			i++;
	}
	return err;
}

void check_start_sig()
{
	while(START_SIG!=0);
	delay_ms(100);//����;
	while(START_SIG!=0);//ȷʵΪ����;
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     //����ϵͳ�ж����ȼ�����2
	delay_init(168);									//��ʱ��ʼ��
	gpio_init();
	OnOffControl_Init();
	exti_func_init();
	storage_status=usr_init_sdcard();

	//  test_disk_speed();
	//	STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
	//	memcpy(command_flash,datatemp,70);
	//	if(datatemp[0]!=0xEB && datatemp[1]!=0x90)
	//	{
	//		STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)command,SIZE);
	//	}

	STMFLASH_Read(FLASH_SAVE_ADDR-1024,(u32*)command_MYADDR,1);
	if(command_MYADDR[1] == 0xFF)
		MYADDR= 1;
	else
		MYADDR = command_MYADDR[1];

#if OUTINTERFACE == SANYUAN
	uart2_init(614400);									//��ʼ������1������
	
	target_bind = datatemp[3];
	priority = datatemp[47];
	if(datatemp[4] == 0x00)
		jam_type = 0;
	else if(datatemp[4] == 0x10)
		jam_type = 1;
	else
		jam_type = 3;
	if(memcmp(datatemp,command,SIZE))
	{
		jam_type |= 0x80;
	}
	else
	{
		jam_type &= 0x7F;
	}
#elif OUTINTERFACE == _8511
	uart2_init(115200);
#else
	uart2_init(9600);									//�����������ͨ������Ϊ115200;
	uart3_init(115200);
	uart6_init(9600);                                   //��FPGA֮��ı��ô���;
	uart4_init(9600);
#endif
	uart1_init(5000000);								//��ʼ������1������Ϊ8.4M
//	uart1_init(115200);
	TIM2_Int_Init(5000-1,8400-1);			            //��ʼ���ڲ�500ms��ʱ����1 = 0.1ms   Tout = ((arr+1)*(psc+1))/Tclk;  
	TIM1_PWM_Init(20000-1,8400-1);				        //��ʼ��������
//	TIM3_CH1_Cap_Init(0XFFFF,12-1);
//	TIM4_CH1_Cap_Init(0XFFFF,12-1);                     //��������ʱ����;
	Adc_Init();
	check_status |= 0x40;								//�Զ�����״̬;
	RUN_LED=0;								            //LED����
	TEST_LED      = 1;
	BACK_LED      = 1;
	send_enable   = 1;
	if(OUTINTERFACE!=PLANE)		                        //�����˻�ģʽ�£��ϵ��ʹ�����ݷ���
        send_enable = 1;
//	else
//		ARM_STARTED_H;					                //���˻�ģʽ�£�����Ҫ0�������źţ�����ϵ���ʼ����Ϳ��Թ���-�˴���Ϊ����ϲ����豸ʹ�ã�ARM_STARTED_H �����豸�ϸ��Ż������Ƶ��� ��ʾ������ɿ��Ծ��в���
//	
	//119ģʽ�´������ȡ�����źŲ��ӳ��������·����Ż���������ָ��
#if OUTINTERFACE ==_119 
	check_start_sig();
	POWER_GF_fu12V_ON;
	POWER_SZ_12V_ON;
	delay_ms(300);
	//119�յ������źź��� GF��WB ��ֹǰ�ڲ���Ҫ����
	POWER_GF_24V_ON;
	delay_ms(200);
	POWER_GF_ZHITONG_ON;
	check_status |= 0x08;
	POWER_WB_12V_ON;
	check_status |= 0x20;
	check_status|=0x80;					                //״̬��Ϊ����
	delay_ms(1000);										//����12.5s����ǰ��2s�������ӳ�һ��15s
	STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
	memcpy(uart1_tx_buffer,datatemp,70);
	uart1_send_u1_tx_buff();
	TIM_CtrlPWMOutputs(TIM1, ENABLE);                   //����������;
	ARM_STARTED_H;
#endif
//WB_ZJMSW_ON;
	TIM_CtrlPWMOutputs(TIM1, ENABLE);                   //����������;
	delay_ms(50);
	POWER_SZ_12V_ON;
	POWER_FENGSHAN_1_ON;
	POWER_FENGSHAN_2_ON;
//	delay_ms(200);
//	POWER_GF_fu12V_ON;
//	POWER_GF_24V_ON;
//					delay_ms(200);
//					POWER_GF_ZHITONG_ON;
//	check_status |= 0x08;
//	POWER_WB_12V_ON;
//	check_status |= 0x20;
//	check_status|=0x80;					                //״̬��Ϊ����
	//rng_start();
	uart2_rec_num = 0;
	cnt_UAV_Run = 0;
	main_process();
}
