#ifndef __ONOFFCONTROL_H
#define __ONOFFCONTROL_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//自检控制端口定义
#define 	WB_ZJSW_ON		GPIO_SetBits(GPIOE,GPIO_Pin_5)
#define 	WB_ZJSW_OFF		GPIO_ResetBits(GPIOE,GPIO_Pin_5)
#define 	WB_ZJMSW_OFF	GPIO_SetBits(GPIOE,GPIO_Pin_6)
#define 	WB_ZJMSW_ON		GPIO_ResetBits(GPIOE,GPIO_Pin_6)
#define 	WB_ZJPSW_OFF	GPIO_SetBits(GPIOE,GPIO_Pin_7)
#define 	WB_ZJPSW_ON		GPIO_ResetBits(GPIOE,GPIO_Pin_7)

#define		RUN_LED			PAout(4)
#define		TEST_LED		PBout(12)
#define		BACK_LED		PBout(13)

#define 	WB_AF0_ON		GPIO_SetBits(GPIOE,GPIO_Pin_0)
#define 	WB_AF0_OFF		GPIO_ResetBits(GPIOE,GPIO_Pin_0)
#define 	WB_AF1_ON		GPIO_SetBits(GPIOE,GPIO_Pin_1)
#define 	WB_AF1_OFF		GPIO_ResetBits(GPIOE,GPIO_Pin_1)

#define		Read_fa0_SIG	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) //PE2
#define		Read_fa1_SIG	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3) //PE3

#define		Read_VT1_SIG	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) //PB4
#define		Read_VT2_SIG	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6) //PB6
//#define		Read_ZJP1_SIG		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4) //PC4
//#define		Read_ZJP2_SIG		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5) //PC5

void OnOffControl_Init(void);//初始化		 		

#endif
