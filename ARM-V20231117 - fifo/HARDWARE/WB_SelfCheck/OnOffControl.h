#ifndef __ONOFFCONTROL_H
#define __ONOFFCONTROL_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//�Լ���ƶ˿ڶ���
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

void OnOffControl_Init(void);//��ʼ��		 		

#endif
