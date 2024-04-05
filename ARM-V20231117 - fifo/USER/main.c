//项目名称：弘武干扰机电源控制及数据记录卡
//项目时间：2019/03/03
//功能：双路串口通信@115200 @8400000及FATFS下SD卡内数据记录
//硬件平台：STM32F429 without SRAM
//作者：yf
//维护：huzj  20190701
//公司名称：烟台弘武机电科技有限公司

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

//#define ISNEW 0		//=1代表是否是新板的程序20190703硬件版本,=0代表是老板的程序;
//#define		MYADDR	4					// ID 唯一，一个设备一个号，不能重复！！！！

extern int irqCount;
#define USR_DBUG 0
FATFS fs;					//文件系统描述符，逻辑工作区
FIL file;					//文件描述符1
FILINFO fileinfo;	        //文件信息
DIR dir;  				    //目录	


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
//#else   //新板的IO;
//	#define		RESTART_WB_SIG		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) //PB0
//	#define		ARM_STARTED_H		GPIO_SetBits(GPIOB,GPIO_Pin_11)
//	#define		ARM_STARTED_L		GPIO_ResetBits(GPIOB,GPIO_Pin_11)
//#endif

extern void TIM2_Int_Init(u32 arr,u16 psc);
extern void TIM1_PWM_Init(u32 arr,u32 psc);
u32 bw;
u8 file_record_flag=0;
u8 storage_status=0;	//0xff表示未正确识别SD卡 0x01文件建立/打开失败 0x02空间不足 大于0异常，关闭记录模式。0x00正常可以记录，开启记录模式
u8 record_enable=1;
u8 power_ctrl_auto=0;

u8 MYADDR = 1;//保存相关的设备号信息;
u8 command_MYADDR[4]={0};//保存相关的设备号信息;FLASH_SAVE_ADDR-1024作为存储设备号的地址,位于扇区10;

u8 selfcheckStep = 0;//自检进行到第几部了;
u8 selfCheckState = 0;//bit0为是否检测;bit1为VT1检测状态;bit2为VT2检测状态;bit3为S1检测状态;bit4为ZJP1检测状态;bit5为ZJP2检测状态;
u16 adcx;//自检回读数据;
double adc_double=0;

u8 check_status=0x07;//功放、数字、微波开关机状态;
u16 cnt_num;
u8 checksum;
u8 checksum_num;
u8 P_rec_num = 0;

u32 gps_lon_tgt;//开机点经度;
u32 gps_lat_tgt;//开机点纬度;
u32 gps_lon_now;//经度;
u32 gps_lat_now;//纬度;
u16 gps_hight_now;//高度;
u8 time_hour;
u8 time_min;
u16 time_s;

u16 uart2_rec_num=0;//在三院中代表收到的配置参数指令个数//在无人机当中代表开机次数;

float float_gps_lon_now;
float float_gps_lat_now;
float float_gps_hight_now;
u32   temp_int = 0;
extern u8 ganRaojPower;
extern u8 ganRaojFlag;
extern u8 fashejiPwer;

#if OUTINTERFACE == SANYUAN  //仅三院会用到的数据
float float_time_FX;
u16 time_FX;//FX时间;
u8 uart2_rec_parameter = 0;//收到参数设置的指令并得到数据;

u8 unreal_target_num;//独立假目标数量;
u8 same_unreal_target_num;//相同假目标数量;
u32 first_target_len;//第一个目标距离;
u16 second_first_delay;//第2个到第1个目标的延时;
u16 third_second_delay;//第3个到第2个目标的延时;
u16 fourth_third_delay;//第4个到第3个目标的延时;
u16 pwm;//脉宽调制;
u8 target_bind;//目标装订;
u8 priority;//优先级;
u8 grj_run_state;//干扰机运行与否;
u8 jam_type;//干扰样式;
u8 noise_FM;//噪声调频;

#elif OUTINTERFACE == _8511
u8 self_check_state = 0xFF;//自检状态;
u8 command_style_back = 0x55;//指令回报;
u8 work_state;//状态位;
u16 rec_laser_fre;//侦收雷达频率;
u16 repeat_cycle;//重复周期;
u16 pulse_width;//脉宽;

#elif OUTINTERFACE == _70debug
u8 self_check_state = 0xFF;//自检状态;
u8 command_style_back = 0x55;//指令回报;
u8 work_state;//状态位;
double double_rec_laser_fre;
u16 rec_laser_fre;//侦收雷达频率;
double double_repeat_cycle;
u16 repeat_cycle;//重复周期;
double double_pulse_width;
u16 pulse_width;//脉宽;
double double_machine_temperature;
u8 machine_temperature;//设备温度;
extern u16 byte_read_u4,byte_read_u6;

#endif

u8 work_time=0;//工作时长;
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
const u8 command[TX1_BUFF_SIZE]={0xEB,0x90,0x01/*2类别码*/,0x2F/*3装订干扰目标*/,0x21/*4干扰样式*/,\
			0xE2,0x04,00/*5-7距离调制0*/,0xEE,0x02,0x00/*8-10距离调制1*/,0xE2,0x04,0x00/*11-13距离调制2*/,0xDC,0x05,0x00/*14-16距离调制3*/,0x00/*17灵敏度*/,\
			0x16,0x9F,0x02,0x00/*18-21多普勒调制0*/,0xC4,0x09,0x00,0x00/*22-25多普勒调制1*/,0x44,0xDD,0x07,0x00/*26-29多普勒调制2*/,0x5A,0x7C,0x0A,0x00/*30-33多普勒调制3*/,0x00/*34接收衰减*/,\
			0x03/*35假目标个数*/,0xFF,0x3F/*36-37脉宽调制*/,0x76,0x62,0x00/*38-40噪声调频*/,0xFF/*41噪声调幅*/,\
			0xFA,0x00/*42-43转发时间*/,0x00,0x00/*44-45停止时间*/,0x40/*46发送衰减*/,0x00/*47优先干扰目标*/,0x00/*48相同假目标数量*/,\
			/*49-69后续的作为备用*/
			};
//const u8 command[TX1_BUFF_SIZE]={0xEB,0x90,0x01/*2类别码*/,0x2F/*3装订干扰目标*/,0x21/*4干扰样式*/,\
//			0xFA,0x00,0xF0/*5-7距离调制0*/,0xA6,0x0E,0x00/*8-10距离调制1*/,0x4C,0x1D,0x00/*11-13距离调制2*/,0xF2,0x2B,0x00/*14-16距离调制3*/,0x00/*17灵敏度*/,\
//			0xA3,0x70,0x3D,0xFA/*18-21多普勒调制0*/,0xA3,0x70,0x3D,0x0A/*22-25多普勒调制1*/,0x47,0xE1,0x7A,0x14/*26-29多普勒调制2*/,0xEB,0x51,0xB8,0x1E/*30-33多普勒调制3*/,0x00/*34接收衰减*/,\
//			0x03/*35假目标个数*/,0xC4,0x09/*36-37脉宽调制*/,0x20,0x4E,0x00/*38-40噪声调频*/,0xFF/*41噪声调幅*/,\
//			0xFA,0x00/*42-43转发时间*/,0x00,0x00/*44-45停止时间*/,0x40/*46发送衰减*/,0x00/*47优先干扰目标*/,0x00/*48相同假目标数量*/,\
//			/*49-69后续的作为备用*/
//			};
u8 command_flash[TX1_BUFF_SIZE]={0};
char send_enable=0;
char set_flag=0;
char reset_wb_flag = 0;
			
//要写入到STM32 FLASH的字符串数组
#define TEXT_LENTH sizeof(command)	 		  	//数组长度	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)
#define FLASH_SAVE_ADDR  0x080E0000 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.
										//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.

__align(4) u8 datatemp[TX1_BUFF_SIZE];
//STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)TEXT_Buffer,SIZE);
//STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);	
			
void gpio_init()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOF时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOF时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOF时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟

    //START flag pin 新干扰机下的接口
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //普通输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);              //初始化

    //	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;//AF0测试输入;
    //	GPIO_Init(GPIOE, &GPIO_InitStructure);          //初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4|GPIO_Pin_12;//AF0测试输出;|GPIO_Pin_12为NTC直通;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      //OD上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);              //初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//??? 
    POWER_FENGSHAN_1_OFF;//                      风扇1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);//??? 
    POWER_FENGSHAN_2_OFF;                               //风扇2

    // run status led PA4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //普通输入模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      //OD上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);              //初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;				
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //普通输入模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      //OD上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);              //初始化
    RUN_LED = 1;

    //PD 13 14 15 电源控制
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);              //初始化

    //PB 3 4 5 14 防止SPI干扰SD
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);              //初始化

    //使能 422 发送 PE13 PE14
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //普通输出模式
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);              //初始化
    GPIO_SetBits(GPIOE,GPIO_Pin_13 | GPIO_Pin_14);	
    GPIO_ResetBits(GPIOE,GPIO_Pin_0 | GPIO_Pin_1);
    //	GPIO_ResetBits(GPIOE,GPIO_Pin_12);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);              //初始化
}

void uart1_send_u1_tx_buff()
{
    while(u1_tx_idle!=1);																				//查看串口1上次发送是否结束，只要硬件正常，一定发送完了（串口1 8.4m 串口2 115200，无论串口2收多快，两次收之间串口1一定发完了）。					
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
    DMA1_Stream6->NDTR = len;   //重设dma数据传输量

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
    DMA1_Stream4->NDTR = len;   //重设dma数据传输量

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
    DMA2_Stream6->NDTR = len;   //重设dma数据传输量

    u6_tx_idle=0;
    DMA_Cmd(DMA2_Stream6,ENABLE);
}

//得到磁盘剩余容量
//drv:磁盘编号("0:"/"1:")
//total:总容量	 （单位KB）
//free:剩余容量	 （单位KB）
//返回值:0,正常.其他,错误代码
u8 exf_getfree(u8 *drv,u32 *total,u32 *free)
{
	FATFS *fs1;
	u8 res;
    u32 fre_clust=0, fre_sect=0, tot_sect=0;
    //得到磁盘信息及空闲簇数量
    res =(u32)f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
    if(res==0)
	{
	    tot_sect=(fs1->n_fatent-2)*fs1->csize;	    //得到总扇区数
	    fre_sect=fre_clust*fs1->csize;			    //得到空闲扇区数	   
#if _MAX_SS!=_MIN_SS				  				//扇区大小不是512字节,则转换为512字节
		tot_sect*=fs1->ssize/_MAX_SS;
		fre_sect*=fs1->ssize/_MAX_SS;
#endif	  
		*total=tot_sect>>1;	//单位为KB
		*free=fre_sect>>1;	//单位为KB 
 	}
	return res;
}	

char usr_init_sdcard()
{
	char err=0;
	char cnt=0;
	u8 res = 0;
	u32 total,free;
	
	while(SD_Init())            //检测不到SD卡
	{
		delay_ms(500);
		LED0=!LED0;
		err=0xff;
		cnt++;
		if(cnt>2)
			return err;
	}
	
	res=f_mount(&fs,"0:",1); 	//挂载SD卡 
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
			f_open(&file,"0:/config.txt",FA_WRITE);			    //打开序号文件
			sprintf(buffer,"%05d\r\n",num);							    //整理当前文件序号为字符串
			f_lseek(&file,file.fsize);											//找到文件结尾
			//f_puts(buffer,&file);													//写入文件序号
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
                    f_lseek(&file,file.fsize);													//找到文件结尾
                    f_write(&file,uart1_rx_buffer0,RX1_BUFF_SIZE,&bw);
                }
                TIM_Cmd(TIM3,DISABLE);
                TIM_SetCounter(TIM3, 0);
                sprintf(uart2_tx_buffer,"%.3f/s , %.2f MB/s\n",(float)(cnt_t)/1000.0,(float)(8000)/(float)(cnt_t));	//整理当前文件序号为字符串			
                uart2_send_u2_tx_buff();
                USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);          //开启相关中断;
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
        f_open(&file,"0:/config.txt",FA_WRITE);			//打开序号文件
        sprintf(buffer,"%05d\r\n",num);					//整理当前文件序号为字符串
        f_lseek(&file,file.fsize);						//找到文件结尾
        f_puts(buffer,&file);							//写入文件序号
        f_close(&file);									//关闭文件
        
        //准备新建数据记录文件
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
    if(uart1_rec_flag==1)	//1中数据已经准备好，正在操作2
    {
        uart1_rec_flag=0;
        f_lseek(&file,file.fsize);									//找到文件结尾
        f_write(&file,uart1_rx_buffer0,RX1_BUFF_SIZE,&bw);				
    }
    else if(uart1_rec_flag==2)
    {
        uart1_rec_flag=0;
        f_lseek(&file,file.fsize);									//找到文件结尾
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
	//u16 rec_laser_fre;//侦收雷达频率;
	//double double_repeat_cycle;
	//u16 repeat_cycle;//重复周期;
	//double double_pulse_width;
	//u16 pulse_width;//脉宽;
	//double double_machine_temperature;

	double xishu = 312.5 / 2 / 3.14159265359 / 8192;
	int int16_temp ,temp;
	if((cnt_t - cnt_t_old)>1)
	{
		cnt_t_old = cnt_t;
	
#if OUTINTERFACE == SANYUAN  ////////////////////////////////////////////////////
		uart2_tx_buffer[3] = (MYADDR<<4)|0x02;
		uart2_tx_buffer[4] = check_status;//组建关机状态;
		//添加放置战术反馈信息;
		gps_lon_now = (int)(float_gps_lon_now*10000000);//经度4bytes;
		uart2_tx_buffer[5] = gps_lon_now;
		uart2_tx_buffer[6] = gps_lon_now >> 8;
		uart2_tx_buffer[7] = gps_lon_now >> 16;
		uart2_tx_buffer[8] = gps_lon_now >> 24;
		gps_lat_now = (int)(float_gps_lat_now*10000000);//纬度4bytes;
		uart2_tx_buffer[9] = gps_lat_now;
		uart2_tx_buffer[10] = gps_lat_now >> 8;
		uart2_tx_buffer[11] = gps_lat_now >> 16;
		uart2_tx_buffer[12] = gps_lat_now >> 24;
		uart2_tx_buffer[13] = uart2_rx_buffer[17];//FX时间s低位;
		uart2_tx_buffer[14] = uart2_rx_buffer[18];//FX时间s高位;
		uart2_tx_buffer[15] = uart2_rec_num;//接收的帧计数;
		uart2_tx_buffer[16] = uart2_rec_num >> 8;//接收的帧计数;
		uart2_tx_buffer[17] = target_bind;
		uart2_tx_buffer[18] = priority;
		if(uart2_tx_buffer[18] != 0x00)
			gps_lon_now++;
		uart2_tx_buffer[19] = jam_type;
		
		uart2_tx_buffer[21] = 0x31;//代表软件版本号,高4bit为FPGA版本号，低4bit为嵌入式版本号;
		uart2_tx_buffer[22] = cnt_t>>1;
		uart2_tx_buffer[23] = (cnt_t>>1) >> 8;//指令多余用不着的填0x00;
		checksum = 0;
		for(checksum_num=3;checksum_num<24;checksum_num++)
		{
			checksum = checksum + uart2_tx_buffer[checksum_num];
		}
		uart2_tx_buffer[24] = checksum;
		uart2_send_u2_tx_buff();
#elif OUTINTERFACE==_119////////////////////////////////////////////////////
		uart2_tx_buffer[3] = (MYADDR<<4)|0x02;
		uart2_tx_buffer[4] = check_status;//组建关机状态;
		//添加放置战术反馈信息;
		gps_lon_now = (int)(float_gps_lon_now*10000000);//经度4bytes;
		uart2_tx_buffer[5] = gps_lon_now;
		uart2_tx_buffer[6] = gps_lon_now >> 8;
		uart2_tx_buffer[7] = gps_lon_now >> 16;
		uart2_tx_buffer[8] = gps_lon_now >> 24;
		gps_lat_now = (int)(float_gps_lat_now*10000000);//纬度4bytes;
		uart2_tx_buffer[9] = gps_lat_now;
		uart2_tx_buffer[10] = gps_lat_now >> 8;
		uart2_tx_buffer[11] = gps_lat_now >> 16;
		uart2_tx_buffer[12] = gps_lat_now >> 24;
		uart2_tx_buffer[13] = cnt_t;//时间;
		uart2_tx_buffer[14] = cnt_t >> 8;//时间;
		uart2_tx_buffer[15] = cnt_t >> 16;//时间;
		uart2_tx_buffer[16] = cnt_t >> 24;//时间;
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
		uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;//校验和;
		uart2_send_u2_tx_buff();
#elif OUTINTERFACE==_8511		////////////////////////////////////////////////////
		uart2_tx_buffer[2] = cnt_UAV_Run>>1;
		uart2_tx_buffer[3] = cnt_UAV_Run>>9;        //帧计数;
		uart2_tx_buffer[4] = self_check_state;      //设备自检状态;
		uart2_tx_buffer[5] = 0xFA;                  //温度;
		uart2_tx_buffer[6] = command_style_back;
		uart2_tx_buffer[7] = work_state;
		uart2_tx_buffer[8] = rec_laser_fre;         //侦收雷达频率;
		uart2_tx_buffer[9] = rec_laser_fre>>8;
		uart2_tx_buffer[10] = repeat_cycle;         //重复周期;
		uart2_tx_buffer[11] = repeat_cycle>>8;
		uart2_tx_buffer[12] = pulse_width;          //脉宽;
		uart2_tx_buffer[13] = pulse_width>>8;

		checksum = 0;
		for(cnt_num = 0;cnt_num < TX2_BUFF_SIZE-2;cnt_num++)
		{
			checksum += uart2_tx_buffer[cnt_num];
		}
		uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;//校验和;适应60所的飞机协议;
//		uart2_tx_buffer[TX2_BUFF_SIZE-1] = (~checksum)+1;//校验和;
		uart2_send_u2_tx_buff();
#elif OUTINTERFACE == _70debug////////////////////////////////////////////////////
		memcpy(uart2_jiexi_buffer,uart1_rx_buffer0,63);
		double_machine_temperature = (uart2_jiexi_buffer[15] | (uart2_jiexi_buffer[16] << 8))*503.975/4096 - 273.15;
		if(uart2_jiexi_buffer[15]==0 && uart2_jiexi_buffer[16] == 0)
		double_machine_temperature = 0;
		double_repeat_cycle = ((uart2_jiexi_buffer[23] | (uart2_jiexi_buffer[24] << 8) | (uart2_jiexi_buffer[25] << 16) | (uart2_jiexi_buffer[26] << 24)) - 
							  (uart2_jiexi_buffer[3] | (uart2_jiexi_buffer[4] << 8) | (uart2_jiexi_buffer[5] << 16) | (uart2_jiexi_buffer[6] << 24)))*3.2;//重复周期;

		int16_temp = (uart2_jiexi_buffer[7] | (uart2_jiexi_buffer[8] << 8));
		if((int16_temp & 0x8000) == 0x8000)
		{
			int16_temp = ~(int16_temp & 0x7FFF);
			int16_temp = ((int16_temp) & 0x7FFF) + 1;
			xishu = -1 * xishu;
		}
		rec_laser_fre = uart2_jiexi_buffer[12] & 0x07;
		double_rec_laser_fre = ((rec_laser_fre*2+1)*78.125+int16_temp * xishu)*16;//侦收雷达频率;
		if(uart2_jiexi_buffer[7]==0 && uart2_jiexi_buffer[8] == 0)
        double_rec_laser_fre = 0;
		
		double_pulse_width = (uart2_jiexi_buffer[9] | (uart2_jiexi_buffer[10] << 8) | (uart2_jiexi_buffer[11] << 16))*3.2;//脉宽;
		uart2_tx_buffer[0] = 0xDD;
		uart2_tx_buffer[1] = 0x77;
		uart2_tx_buffer[2] = cnt_UAV_Run>>1;            //帧计数;
		uart2_tx_buffer[3] = cnt_UAV_Run>>9;
		uart2_tx_buffer[4] = 0x00;                      //信息类别码;
		uart2_tx_buffer[5] = command_style_back;        //指令回报
		uart2_tx_buffer[6] = MYADDR;                    //干扰机设备ID;
		uart2_tx_buffer[7] = 0x00;
		uart2_tx_buffer[8] = 0x00;
		uart2_tx_buffer[9] = 0x00;
		uart2_tx_buffer[10] = uart2_jiexi_buffer[13];   //GPS计数;
		uart2_tx_buffer[11] = uart2_jiexi_buffer[14];   //GPS计数;
		temp = (int)(double_machine_temperature);
		uart2_tx_buffer[12] = temp;                     //温度;
		if(uart2_jiexi_buffer[18] == 0x00)              //欺骗;
			uart2_tx_buffer[13] = 0x04;
		else if(uart2_jiexi_buffer[18] == 0x01)         //压制;
			uart2_tx_buffer[13] = 0x08;
		else if(uart2_jiexi_buffer[18] == 0x02)         //组合;
			uart2_tx_buffer[13] = 0x0C;
		uart2_tx_buffer[13] |= (((~uart2_jiexi_buffer[21])&0xC0) >> 6);//uart2_tx_buffer[13]d的bit1灵敏度通道1;bit2是通道2灵敏度;
		
		rec_laser_fre = (int)(double_rec_laser_fre);
		if(double_rec_laser_fre-rec_laser_fre>=0.5)
		rec_laser_fre+=1;
		uart2_tx_buffer[14] = rec_laser_fre;
		uart2_tx_buffer[15] = rec_laser_fre >> 8;       //侦收雷达频率;
		repeat_cycle = (int)(double_repeat_cycle/50);
		uart2_tx_buffer[16] = repeat_cycle;
		uart2_tx_buffer[17] = repeat_cycle >> 8;        //重复周期;
		
		pulse_width = (int)(double_pulse_width/50);
		uart2_tx_buffer[18] = pulse_width;
		uart2_tx_buffer[19] = pulse_width >> 8;         //脉宽;
		
		uart2_tx_buffer[20] = uart2_jiexi_buffer[17];
		uart2_tx_buffer[25] = (uart2_jiexi_buffer[21] & 0x1F);
		uart2_tx_buffer[26] = uart2_jiexi_buffer[20];
		uart2_tx_buffer[27] = uart2_jiexi_buffer[19];
		checksum = 0;
		for(cnt_num = 0;cnt_num < 29;cnt_num++)
		{
			checksum += uart2_tx_buffer[cnt_num];
		}
		uart2_tx_buffer[29] = checksum;//校验和;
        memcpy(uart4_tx_buffer,uart2_tx_buffer,30);
        uart4_send_tx_buff_len(30);
		//uart2_send_u2_tx_buff();
        uart2_send_tx_buff_len(30);

#endif
	}
}

void check_sanyuan_start()
{
    //向FPGA发送指令在这里加入
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
        check_status|=0x80;					//状态置为开机
        delay_ms(500);				//三院接收启动信号后开启功放、微波500ms在工作
        STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
        memcpy(uart1_tx_buffer,datatemp,70);
        uart1_send_u1_tx_buff();
        TIM_CtrlPWMOutputs(TIM1, ENABLE);//启动秒脉冲;
//			ARM_STARTED_H;	
    }
}
//u8 unreal_target_num;//独立假目标数量;
//u8 same_unreal_target_num;//相同假目标数量;
//u32 first_target_len;//第一个目标距离;
//u8 second_first_delay;//第2个到第1个目标的延时;
//u8 third_second_delay;//第3个到第2个目标的延时;
//u8 fourth_third_delay;//第4个到第3个目标的延时;
//u16 pwm;//脉冲调制;
//u8 target_bind;//目标装订;
//u8 priority;//优先级;
//u8 grj_run_state;//干扰机运行与否;
//u8 jam_type;//干扰样式;
//u8 noise_FM;//噪声调频;
void Uart2RecieveAnalyse()
{
		u16 weizhi =0;
#if OUTINTERFACE == SANYUAN
		check_sanyuan_start();
		//命令串口2读到数据
		if(uart2_rec_flag=='F')
		{
			uart2_rec_flag=0;
			RUN_LED=!RUN_LED;
			uart2_rec_num ++;//接收的指令计数;
			//在这里加入解算程序，解算出GPS，时间信息，高度
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
				datatemp[MARK_FIRST_TARGET_LEN+2]=temp_int >> 16;//第一个目标距离有三个字节;
				
//				temp_int = (first_target_len+second_first_delay)*10/3;//废止;
//				datatemp[MARK_SECOND_FIRST_DELAY]=temp_int;
//				datatemp[MARK_SECOND_FIRST_DELAY+1]=temp_int >> 8;
//				datatemp[MARK_SECOND_FIRST_DELAY+2]=temp_int >> 16;//第二个目标距离有三个字节;
//				
//				temp_int = (first_target_len+second_first_delay + third_second_delay)*10/3;
//				datatemp[MARK_THIRD_SECOND_DELAY]=temp_int;
//				datatemp[MARK_THIRD_SECOND_DELAY+1]=temp_int >> 8;
//				datatemp[MARK_THIRD_SECOND_DELAY+2]=temp_int >> 16;//第三个目标距离有三个字节;
//				
//				temp_int = (first_target_len+second_first_delay + third_second_delay + fourth_third_delay)*10/3;
//				datatemp[MARK_FOURTH_THIRD_DELAY]=temp_int;
//				datatemp[MARK_FOURTH_THIRD_DELAY+1]=temp_int >> 8;
//				datatemp[MARK_FOURTH_THIRD_DELAY+2]=temp_int >> 16;//第四个目标距离有三个字节;
				
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
				
				USART_Cmd(USART2, ENABLE);  //使能串口2
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
				USART_Cmd(USART2, ENABLE);  //使能串口2
				uart2_send_u2_tx_buff();
				
			}
		}//命令串口2读到数据---------------END		
#elif OUTINTERFACE == _119
		//命令串口2读到数据
		if(uart2_rec_flag=='F')				
		{
			
			USART_Cmd(USART2, ENABLE);  //使能串口2
		}//命令串口2读到数据---------------END
#elif OUTINTERFACE==PLANE
		//命令串口2-无人机;
		if(cnt_t >= 300 && (check_status&0x80)== 0x80)//无人机方面超过150s自动关机;
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
			//发送回复信息,需要填充一个指令码;
			uart2_rec_flag=0;	
			STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
			switch(uart2_rx_buffer[3])
			{
				case 0x01:
					
					break;
				case 0x03:
					TIM_CtrlPWMOutputs(TIM1, DISABLE);//启动秒脉冲;
//					TIM_Cmd(TIM1, DISABLE);  //使能TIM14
				
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
					uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;//校验和;
					USART_Cmd(USART2, ENABLE);  //使能串口2
					uart2_send_u2_tx_buff();//发送回复指令;
					//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断;

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
					datatemp[MARK_FIRST_TARGET_LEN+2]=temp_int >> 16;//第一个目标距离有三个字节;
					break;
				case 0x69:
					temp_int = (((uart2_rx_buffer[8]&0x7F)<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*25/6;
					temp_int |= ((uart2_rx_buffer[8] & 0x80)<<16);
					datatemp[MARK_SECOND_FIRST_DELAY]=temp_int;
					datatemp[MARK_SECOND_FIRST_DELAY+1]=temp_int >> 8;
					datatemp[MARK_SECOND_FIRST_DELAY+2]=temp_int >> 16;//第二个目标距离有三个字节;
					break;
				case 0x6A:
					temp_int = (((uart2_rx_buffer[8]&0x7F)<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*10/3;
					temp_int |= ((uart2_rx_buffer[8] & 0x80)<<16);
					datatemp[MARK_THIRD_SECOND_DELAY]=temp_int;
					datatemp[MARK_THIRD_SECOND_DELAY+1]=temp_int >> 8;
					datatemp[MARK_THIRD_SECOND_DELAY+2]=temp_int >> 16;//第三个目标距离有三个字节;
					break;
				case 0x6B:
					temp_int = (((uart2_rx_buffer[8]&0x7F)<<16) | (uart2_rx_buffer[7]<<8) | (uart2_rx_buffer[6]))*10/3;
					temp_int |= ((uart2_rx_buffer[8] & 0x80)<<16);
					datatemp[MARK_FOURTH_THIRD_DELAY]=temp_int;
					datatemp[MARK_FOURTH_THIRD_DELAY+1]=temp_int >> 8;
					datatemp[MARK_FOURTH_THIRD_DELAY+2]=temp_int >> 16;//第四个目标距离有三个字节;
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
				uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;//校验和;
				USART_Cmd(USART2, ENABLE);  //使能串口2
				uart2_send_u2_tx_buff();    //发送回复指令;
				//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断;
			}
			
			if(uart2_rx_buffer[3] == 0x02)
			{
				POWER_SZ_12V_ON;
				check_status |= 0x10;
				delay_ms(3000);
				
				POWER_GF_fu12V_ON;
				check_status|=0x80;					//状态置为开机
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
				TIM_CtrlPWMOutputs(TIM1, ENABLE);//启动秒脉冲;
				send_enable=1;
				record_enable=1;
				uart2_rec_num++;
			}
		}
#elif OUTINTERFACE == _8511
//u8 self_check_state = 0xFF;   //自检状态;
//u8 command_style_back = 0x55; //指令回报;
//u8 work_state;                //状态位;
//u16 rec_laser_fre;            //侦收雷达频率;
//u16 repeat_cycle;             //重复周期;
//u16 pulse_width;              //脉宽;
    
		if(uart2_rec_flag=='F')
		{
			uart2_rec_flag=0;	
			TEST_LED = !TEST_LED;
			if((uart2_rx_buffer[4] != 0x01) && (uart2_rx_buffer[2] != 0x13))//不是1号飞机则不执行指令;
			{
				USART_Cmd(USART2, ENABLE);  //使能串口2
				return;
			}
				
			command_style_back = uart2_rx_buffer[5];
			switch(uart2_rx_buffer[5])
			{
				case 0x33://开干扰;
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
					check_status|=0x80;		//状态置为开机;
					break;
				case 0x35://关干扰;
					RUN_LED = 1;
					POWER_GF_24V_OFF;
                    POWER_GF_ZHITONG_OFF;
					check_status &= 0x08;
					POWER_WB_12V_OFF;
					check_status &= 0xDF;
					POWER_SZ_12V_OFF;
					check_status &= 0xEF;
					check_status&=0x7F;		//状态置为关机;
					break;
				case 0x41://任务1;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_0,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x43://任务2;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_1,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x45://任务3;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_2,(u32*)datatemp,SIZE);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x47://任务4;
					STMFLASH_Read(FLASH_SAVE_ADDR + OFFSETS_ADDR_3,(u32*)datatemp,SIZE-1);
					memcpy(uart1_tx_buffer,datatemp,TX1_BUFF_SIZE);
					uart1_send_u1_tx_buff();
					break;
				case 0x49://任务5;
					
					break;
				case 0x4B://任务6;
					
					break;
				case 0x4D://任务7;
					
					break;
				case 0x4F://任务8;
					
					break;
				case 0x53://自检;
					
					break;
				case 0x55://待机;
					//发送开关关闭;
					break;
				case 0x57://卸载数据;
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
				case 0x61://备份;
					
					break;
				case 0x63://备份;
					
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
//				uart2_tx_buffer[TX2_BUFF_SIZE-1] = (~checksum) + 1;//校验和;
//				USART_Cmd(USART2, ENABLE);  //使能串口2
//				uart2_send_u2_tx_buff();//发送回复指令;
			}
			else
			{
				uart1_tx_buffer[2] = 0x02;
				memcpy(uart1_tx_buffer+3,uart2_rx_buffer+3,10);
				uart1_send_u1_tx_buff();
			}
			USART_Cmd(USART2, ENABLE);  //使能串口2

		}
#elif OUTINTERFACE == _70debug          //此部分的协议对应内部通信的字节数，发送的还是按照界面WURENJI的字节数发送
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
				USART_Cmd(USART2, ENABLE);      //使能串口2
				return;
			}

			if((uart2_rx_buffer[4] != MYADDR))  //飞机号不对则不执行指令;
			{
				USART_Cmd(USART2, ENABLE);      //使能串口2
				return;
			}

			if(uart2_rx_buffer[2] == 0xCD)
			{
				uart3_tx_buffer[0] = 0x85;
				uart3_tx_buffer[1] = 0x11;
				uart3_tx_buffer[2] = uart2_rx_buffer[6];
				uart3_tx_buffer[3] = uart2_rx_buffer[7];
				uart3_send_u1_tx_buff();
				USART_Cmd(USART2, ENABLE);      //使能串口2
				return;
			}

			command_style_back = uart2_rx_buffer[5];
			switch(uart2_rx_buffer[5])
			{
				case 0x10://当前工作参数;
					
					break;
				case 0x11://任务1参数;
					
					break;
				case 0x13://任务2参数;
					
					break;
				case 0x15://任务3参数;
					
					break;
				case 0x17://任务4参数;
					
					break;
				case 0x19://任务5参数;
					
					break;				
				case 0x1B://任务6参数;
					
					break;
				case 0x1D://任务7参数;
					
					break;
				case 0x1F://任务8参数;
					
					break;
				case 0x33://开干扰;
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
					check_status|=0x80;				//状态置为开机;
					break;
				case 0x35://关干扰;
					RUN_LED = 1;
					POWER_GF_24V_OFF;
					POWER_GF_ZHITONG_OFF;
					check_status &= 0x08;
					break;
				case 0x41://任务1;
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
				case 0x43://任务2;
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
				case 0x45://任务3;
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
				case 0x47://任务4;
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
				case 0x49://任务5;
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
				case 0x4B://任务6;
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
				case 0x4D://任务7;
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
				case 0x4F://任务8;
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
				case 0x53://自检;
					
					break;
				case 0x55://待机;
					RUN_LED = 1;
					POWER_GF_24V_OFF;
					POWER_GF_ZHITONG_OFF;
					check_status &= 0x08;
					POWER_WB_12V_OFF;
					check_status &= 0xDF;
					POWER_SZ_12V_OFF;
					check_status &= 0xEF;
					check_status&=0x7F;				//状态置为关机;
					POWER_GF_fu12V_OFF;
				
					break;
				case 0x57://卸载数据;
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
				case 0x61://备份;
					
					break;
				case 0x63://备份;
					
					break;
				default:
					break;
			}
			USART_Cmd(USART2, ENABLE);  //使能串口2
		}
		else if(uart2_rec_flag == 'P')  //参数配置的设置;
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
			USART_Cmd(USART2, ENABLE);  //使能串口2
		}
        
		else if(uart2_rec_flag == 'U')
		{
				int i = 0;
				uart2_rec_flag=0;
				//串口2数据转发
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
	selfcheckStep |= 0x01;              //自检进入第一步
	delay_ms(500);
	printf("\r\n通道打开*********************************\n");
	//发送串口程序,让FPGA开启接收检测的相关引脚;
	selfCheckState |= (Read_VT1_SIG << 1);
	if(selfCheckState&0x02)
	{
		printf("\r\nVT1正常-----------------\n");
	}
	else
	{
		printf("\r\nVT1异常-----------------\n");
	}
	WB_AF1_ON;
	WB_AF0_OFF;
	selfcheckStep |= 0x02;              //自检进入第二步
	delay_ms(500);                      //接下来等待FPGA进行发送检测以及增益和功放1检测的检测;
	selfCheckState |= (Read_VT2_SIG << 2);
	if(selfCheckState&0x04)
	{
		printf("\r\nVT2正常-----------------\n");
	}
	else
	{
		printf("\r\nVT2异常-----------------\n");
	}
	adcx = Get_Adc_Average(ADC_Channel_13,50);//S1
	adc_double = (adcx*3.29/4096);
	printf("功放1电压值=%f\n", adc_double);
	adcx = Get_Adc_Average(ADC_Channel_14,50);//ZJP1
	adc_double = (adcx*3.29/4096);
	printf("功放1的ZJP1值=%f\n", adc_double);

	WB_AF1_ON;
	WB_AF0_ON;
	selfcheckStep |= 0x04;  //自检进入第三步
	delay_ms(500);          //接下来进行发增益和功放2检测的检测;
	adcx = Get_Adc_Average(ADC_Channel_13,50);//s1
	adc_double = (adcx*3.29/4096);
	printf("功放1电压值=%f\n", adc_double);
	adcx = Get_Adc_Average(ADC_Channel_15,50);//ZJP1
	adc_double = (adcx*3.29/4096);
	printf("功放1的ZJP2值=%f\n", adc_double);
	WB_ZJPSW_OFF;
	WB_ZJMSW_OFF;
	WB_ZJSW_OFF;
	WB_AF1_OFF;
	WB_AF0_OFF;
	selfcheckStep |= 0x08;  //自检结束
	if((selfcheckStep & 0x0F) ==0x0F )
	{
		//对电台发送合格的自检信息;
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
		//上电或开机后查询记录使能，卡状态，是否正在记录，默认状态下使能若卡状态正常，未在记录中，则创建新文件开始等待数据准备写入
		if(record_enable==1&&storage_status==0&&file_record_flag==0)
		{
			if(record_new()==0)
			{
				file_record_flag=1;							//正常，置位文件正在记录标志
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
//			Uart2CycleSend();   //循环发送;

//        for(i=0;i<32;i++)uart1_rx_buffer0[i] = i;
//        
//        for(i=0;i<10;i++){
//            f_lseek(&file,file.fsize);
//            f_write(&file,uart1_rx_buffer0,32,&bw);
//        }

		Uart2RecieveAnalyse();  //串口2接收到数据后的解析;
		if(uart1_rec_flag==1)	//数据串口1读到数据------1中数据已经准备好，正在操作2
		{
				uart1_rec_flag=0;
		//if(uart1_rx_buffer0[0]==(char)(HEAD_INTERNAL_1)&&uart1_rx_buffer0[1]==(char)(HEAD_INTERNAL_2)&&uart1_rx_buffer0[2]==(char)(HEAD_INTERNAL_3)&&uart1_rx_buffer0[3]==(char)(HEAD_INTERNAL_4))				//数据正确
		//{
				printf("file uart1_rec_flag:1-%d\r\n",file_record_flag);
				//if(uart1_rx_buffer0[2]==0x05&&file_record_flag==1)
				if(file_record_flag==1)																			//读到侦查数据且已经开启文件（既已经开机，且文件已经建立打开）													
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
								uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;        //校验和;
								uart2_send_u2_tx_buff();

						//在这加最后一个字节校验
						#elif OUTINTERFACE == _119
								uart2_tx_buffer[4]=0x03;	                        //干扰样式
								memcpy(uart2_tx_buffer+5,uart1_rx_buffer0+3,250);   //上传侦查数据，FPGA返回 eb 90 05 + data；上传eb 90 fc addr|05 xx + data，250 = 25个脉冲描述字
								uart2_tx_buffer[3] = (MYADDR<<4)|0x05;
								checksum = 0;
								for(cnt_num = 2;cnt_num < TX2_BUFF_SIZE-1;cnt_num++)
								{
										checksum += uart2_tx_buffer[cnt_num];
								}
								uart2_tx_buffer[TX2_BUFF_SIZE-1] = checksum;//校验和;
								uart2_send_u2_tx_buff();

						#elif OUTINTERFACE==PLANE
								//读到数据需要更新相关干扰参数到指定发送;
								memcpy(uart2_temp_buffer+6,uart1_rx_buffer0+3,60);
								
						#elif OUTINTERFACE == _8511
								//读到数据需要更新相关干扰参数到指定发送;
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
			//if(uart1_rx_buffer1[0]==(char)(HEAD_INTERNAL_1)&&uart1_rx_buffer1[1]==(char)(HEAD_INTERNAL_2)&&uart1_rx_buffer1[2]==(char)(HEAD_INTERNAL_3)&&uart1_rx_buffer1[3]==(char)(HEAD_INTERNAL_4))				//数据正确
			//{
					printf("file uart1_rec_flag:2-%d\r\n",file_record_flag);
					//if(uart1_rx_buffer1[2]==0x05&&file_record_flag==1)
					if(file_record_flag==1)																			//读到侦查数据且已经开启文件（既已经开机，且文件已经建立打开）		
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
									uart2_tx_buffer[4]=0x03;	//干扰样式
									memcpy(uart2_tx_buffer+5,uart1_rx_buffer1+3,250); //上传侦查数据，FPGA返回 eb 90 05 + data；上传eb 90 fc addr|05 xx + data，250 = 25个脉冲描述字
									uart2_tx_buffer[3] = (MYADDR<<4)|0x05;
									uart2_send_u2_tx_buff();

							#elif OUTINTERFACE == PLANE
									memcpy(uart2_temp_buffer+6,uart1_rx_buffer0+3,60);
							#elif OUTINTERFACE == _8511
									//读到数据需要更新相关干扰参数到指定发送;
									memcpy(uart2_temp_buffer+6,uart1_rx_buffer0+3,60);
							#elif OUTINTERFACE == _70debug
									//memcpy(uart2_temp_buffer+3,uart1_rx_buffer0+3,60);
							#endif
					}
			//}
		}
		if(cnt_t-cnt_t_record_old>0 && file_record_flag==1)		//如果文件正在记录，则每500ms保存一次文件
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

/*随机数发生器开始启动*/
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
	delay_ms(100);//消抖;
	while(START_SIG!=0);//确实为低了;
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     //设置系统中断优先级分组2
	delay_init(168);									//延时初始化
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
	uart2_init(614400);									//初始化串口1波特率
	
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
	uart2_init(9600);									//北京星网宇达通信速率为115200;
	uart3_init(115200);
	uart6_init(9600);                                   //与FPGA之间的备用串口;
	uart4_init(9600);
#endif
	uart1_init(5000000);								//初始化串口1波特率为8.4M
//	uart1_init(115200);
	TIM2_Int_Init(5000-1,8400-1);			            //初始化内部500ms定时器。1 = 0.1ms   Tout = ((arr+1)*(psc+1))/Tclk;  
	TIM1_PWM_Init(20000-1,8400-1);				        //初始化秒脉冲
//	TIM3_CH1_Cap_Init(0XFFFF,12-1);
//	TIM4_CH1_Cap_Init(0XFFFF,12-1);                     //脉宽检测暂时不用;
	Adc_Init();
	check_status |= 0x40;								//自动控制状态;
	RUN_LED=0;								            //LED点亮
	TEST_LED      = 1;
	BACK_LED      = 1;
	send_enable   = 1;
	if(OUTINTERFACE!=PLANE)		                        //非无人机模式下，上电就使能数据发送
        send_enable = 1;
//	else
//		ARM_STARTED_H;					                //无人机模式下，不需要0点启动信号，因此上电后初始化完就可以工作-此处是为了配合测试设备使用，ARM_STARTED_H 测试设备上干扰机就绪灯点亮 表示启动完成可以经行测试
//	
	//119模式下从这里读取启动信号并延迟启动，下发干扰机启动工作指令
#if OUTINTERFACE ==_119 
	check_start_sig();
	POWER_GF_fu12V_ON;
	POWER_SZ_12V_ON;
	delay_ms(300);
	//119收到启动信号后开启 GF和WB 防止前期不必要发热
	POWER_GF_24V_ON;
	delay_ms(200);
	POWER_GF_ZHITONG_ON;
	check_status |= 0x08;
	POWER_WB_12V_ON;
	check_status |= 0x20;
	check_status|=0x80;					                //状态置为开机
	delay_ms(1000);										//延期12.5s，加前面2s及其他延迟一共15s
	STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
	memcpy(uart1_tx_buffer,datatemp,70);
	uart1_send_u1_tx_buff();
	TIM_CtrlPWMOutputs(TIM1, ENABLE);                   //启动秒脉冲;
	ARM_STARTED_H;
#endif
//WB_ZJMSW_ON;
	TIM_CtrlPWMOutputs(TIM1, ENABLE);                   //启动秒脉冲;
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
//	check_status|=0x80;					                //状态置为开机
	//rng_start();
	uart2_rec_num = 0;
	cnt_UAV_Run = 0;
	main_process();
}
