#include "PWMCatch.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


//TIM14 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
//void TIM14_PWM_Init(u32 arr,u32 psc)
//{		 					 
//	//�˲������ֶ��޸�IO������
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);  	//TIM14ʱ��ʹ��    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//ʹ��PORTFʱ��	
//	
//	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14); //GPIOF9����λ��ʱ��14
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //GPIOA9 
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
//	GPIO_Init(GPIOF,&GPIO_InitStructure); //��ʼ��PF9
//	
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
//	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseStructure);
//	
//	//��ʼ��TIM14 Channel1 PWMģʽ	 
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
//	TIM_OCInitStructure.TIM_Pulse=0;
//	TIM_OC1Init(TIM14, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2

//	TIM_OC2PreloadConfig(TIM14, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
// 
//  TIM_ARRPreloadConfig(TIM14,ENABLE);
//	
//	TIM_Cmd(TIM14, ENABLE);  //ʹ��TIM14		
//}  

u8 ICPolarity_state = 0;//���嶨ʱ�����벶�������;0x00-�����ش���;0x01-�½��ش���;0x10-����;0x11-�����½�������;
TIM_ICInitTypeDef  TIM3_ICInitStructure;
//��ʱ��3ͨ��1���벶������
//arr���Զ���װֵ(TIM2,TIM3��32λ��!!)
//psc��ʱ��Ԥ��Ƶ��
void TIM3_CH1_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTAʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PA0

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3); //PA0����λ��ʱ��5
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM3���벶�����
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
		
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	
	
  TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��5
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}


//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û����������;1,�Ѿ�������������.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM3CH1_CAPTURE_STA=0;	//���벶��״̬	
u32	TIM3CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM3��32λ)
u32	TIM3CH1_CAPTURE_VAL1;	//���벶��ֵ1(TIM2/TIM3��32λ)
u32	TIM3CH1_CAPTURE_VAL2;	//���벶��ֵ2(TIM2/TIM3��32λ)
u8 TIM3CH1_One_cycle=0;// �Ƿ��������ֵ;
u32 TIM3CH1_nums = 0;//��������;
//��ʱ��5�жϷ������
void TIM3_IRQHandler(void)
{
 	if((TIM3CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if((TIM3CH1_One_cycle&0x01)==0)
		{
			if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//����1������
			{
				if(TIM3CH1_nums == 0)
					TIM3CH1_CAPTURE_VAL1=TIM_GetCapture1(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
				else if(TIM3CH1_nums == 1)
				{
					TIM3CH1_CAPTURE_VAL2=TIM_GetCapture1(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
					TIM3CH1_CAPTURE_STA |= 0x80;
				}
				TIM3CH1_nums++;
			}
		}
		else
		{
			if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//����1������
			{
				if(TIM3CH1_nums == 0)
				{
					TIM3CH1_CAPTURE_VAL1=TIM_GetCapture1(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
					TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				}
				else if(TIM3CH1_nums == 1)
				{
					TIM3CH1_CAPTURE_VAL2=TIM_GetCapture1(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
					TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising);		//CC1P=1 ����Ϊ�½��ز���
					TIM3CH1_CAPTURE_STA |= 0x80;
				}
				TIM3CH1_nums++;
			}
		}
		
		
		if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//���
		{
			if(TIM3CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM3CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM3CH1_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
					TIM3CH1_CAPTURE_VAL2=0XFFFF;
				}
				else
					TIM3CH1_CAPTURE_STA++;
			}
		}
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_Update); //����жϱ�־λ
	
// 	if((TIM3CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
//	{
//		if((TIM3CH1_One_cycle&0x01)==0)
//		{
//			if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//����1���������¼�
//			{
//				if(TIM3CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
//				{
//					TIM3CH1_CAPTURE_VAL=TIM_GetCapture1(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
//					TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//					TIM3CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
//				}else  								//��δ��ʼ,��һ�β���������
//				{
//					TIM_SetCounter(TIM3,0);
//					TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
//					TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��5
//					TIM3CH1_CAPTURE_STA=0;			//���
//					TIM3CH1_CAPTURE_VAL=0;
//					TIM3CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
//				}
//			}
//		}
//		else
//		{
//			TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//			if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//����1���������¼�
//			{
//				if(TIM3CH1_CAPTURE_STA&0X40)		//������һ���Ͻ��� 		
//				{
//					TIM3CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
//					TIM3CH1_CAPTURE_VAL=TIM_GetCapture1(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
//				}else  								//��δ��ʼ,��һ�β���������
//				{
//					TIM_SetCounter(TIM3,0);
//					TIM3CH1_CAPTURE_STA=0;			//���
//					TIM3CH1_CAPTURE_VAL=0;
//					TIM3CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
//				}
//			}
//		}
//		if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//���
//		{
//			if(TIM3CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
//			{
//				if((TIM3CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
//				{
//					TIM3CH1_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
//					TIM3CH1_CAPTURE_VAL=0XFFFF;
//				}
//				else
//					TIM3CH1_CAPTURE_STA++;
//			}
//		}
// 	}
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_Update); //����жϱ�־λ
}

TIM_ICInitTypeDef  TIM4_ICInitStructure;
//��ʱ��4ͨ��1���벶������
//arr���Զ���װֵ(TIM2,TIM5��32λ��!!)
//psc��ʱ��Ԥ��Ƶ��
void TIM4_CH1_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTBʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //GPIOD12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PA0

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); //PA0����λ��ʱ��5
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM5���벶�����
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
		
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	
	
  TIM_Cmd(TIM4,ENABLE ); 	//ʹ�ܶ�ʱ��4
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û����������;1,�Ѿ�������������.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM4CH1_CAPTURE_STA=0;	//���벶��״̬	
u32	TIM4CH1_CAPTURE_VAL1;	//���벶��ֵ1(TIM2/TIM5��32λ)
u32	TIM4CH1_CAPTURE_VAL2;	//���벶��ֵ2(TIM2/TIM5��32λ)
u8 TIM4CH1_One_cycle=0;// �Ƿ��������ֵ;
u32 TIM4CH1_nums = 0;//��������;
//��ʱ��5�жϷ������
void TIM4_IRQHandler(void)
{
 	if((TIM4CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if((TIM4CH1_One_cycle&0x01)==0)
		{
			if(TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)//����1������
			{
				if(TIM4CH1_nums == 0)
					TIM4CH1_CAPTURE_VAL1=TIM_GetCapture1(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
				else if(TIM4CH1_nums == 1)
				{
					TIM4CH1_CAPTURE_VAL2=TIM_GetCapture1(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
					TIM4CH1_CAPTURE_STA |= 0x80;
				}
				TIM4CH1_nums++;
			}
		}
		else
		{
			if(TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)//����1������
			{
				if(TIM4CH1_nums == 0)
				{
					TIM4CH1_CAPTURE_VAL1=TIM_GetCapture1(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
					TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				}
				else if(TIM4CH1_nums == 1)
				{
					TIM4CH1_CAPTURE_VAL2=TIM_GetCapture1(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
					TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising);		//CC1P=1 ����Ϊ�½��ز���
					TIM4CH1_CAPTURE_STA |= 0x80;
				}
				TIM4CH1_nums++;
			}
		}
		
		if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//���
		{
			if(TIM4CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM4CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM4CH1_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
					TIM4CH1_CAPTURE_VAL2=0XFFFFFFFF;
				}
				else
					TIM4CH1_CAPTURE_STA++;
			}
		}
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC1|TIM_IT_Update); //����жϱ�־λ
}

