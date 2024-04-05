#include "PWMCatch.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


//TIM14 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
//void TIM14_PWM_Init(u32 arr,u32 psc)
//{		 					 
//	//此部分需手动修改IO口设置
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);  	//TIM14时钟使能    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//使能PORTF时钟	
//	
//	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14); //GPIOF9复用位定时器14
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //GPIOA9 
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//	GPIO_Init(GPIOF,&GPIO_InitStructure); //初始化PF9
//	
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseStructure);
//	
//	//初始化TIM14 Channel1 PWM模式	 
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
//	TIM_OCInitStructure.TIM_Pulse=0;
//	TIM_OC1Init(TIM14, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2

//	TIM_OC2PreloadConfig(TIM14, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
// 
//  TIM_ARRPreloadConfig(TIM14,ENABLE);
//	
//	TIM_Cmd(TIM14, ENABLE);  //使能TIM14		
//}  

u8 ICPolarity_state = 0;//定义定时器输入捕获的类型;0x00-上升沿触发;0x01-下降沿触发;0x10-保留;0x11-上升下降均触发;
TIM_ICInitTypeDef  TIM3_ICInitStructure;
//定时器3通道1输入捕获配置
//arr：自动重装值(TIM2,TIM3是32位的!!)
//psc：时钟预分频数
void TIM3_CH1_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTA时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA0

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3); //PA0复用位定时器5
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	

	//初始化TIM3输入捕获参数
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
		
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	
  TIM_Cmd(TIM3,ENABLE ); 	//使能定时器5
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}


//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到上升沿;1,已经捕获到上升沿了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM3CH1_CAPTURE_STA=0;	//输入捕获状态	
u32	TIM3CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM3是32位)
u32	TIM3CH1_CAPTURE_VAL1;	//输入捕获值1(TIM2/TIM3是32位)
u32	TIM3CH1_CAPTURE_VAL2;	//输入捕获值2(TIM2/TIM3是32位)
u8 TIM3CH1_One_cycle=0;// 是否测试周期值;
u32 TIM3CH1_nums = 0;//定义数量;
//定时器5中断服务程序
void TIM3_IRQHandler(void)
{
 	if((TIM3CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if((TIM3CH1_One_cycle&0x01)==0)
		{
			if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//捕获1测周期
			{
				if(TIM3CH1_nums == 0)
					TIM3CH1_CAPTURE_VAL1=TIM_GetCapture1(TIM3);//获取当前的捕获值.
				else if(TIM3CH1_nums == 1)
				{
					TIM3CH1_CAPTURE_VAL2=TIM_GetCapture1(TIM3);//获取当前的捕获值.
					TIM3CH1_CAPTURE_STA |= 0x80;
				}
				TIM3CH1_nums++;
			}
		}
		else
		{
			if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//捕获1测脉宽
			{
				if(TIM3CH1_nums == 0)
				{
					TIM3CH1_CAPTURE_VAL1=TIM_GetCapture1(TIM3);//获取当前的捕获值.
					TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				}
				else if(TIM3CH1_nums == 1)
				{
					TIM3CH1_CAPTURE_VAL2=TIM_GetCapture1(TIM3);//获取当前的捕获值.
					TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising);		//CC1P=1 设置为下降沿捕获
					TIM3CH1_CAPTURE_STA |= 0x80;
				}
				TIM3CH1_nums++;
			}
		}
		
		
		if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//溢出
		{
			if(TIM3CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM3CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM3CH1_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					TIM3CH1_CAPTURE_VAL2=0XFFFF;
				}
				else
					TIM3CH1_CAPTURE_STA++;
			}
		}
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位
	
// 	if((TIM3CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
//	{
//		if((TIM3CH1_One_cycle&0x01)==0)
//		{
//			if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
//			{
//				if(TIM3CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
//				{
//					TIM3CH1_CAPTURE_VAL=TIM_GetCapture1(TIM3);//获取当前的捕获值.
//					TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//					TIM3CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
//				}else  								//还未开始,第一次捕获上升沿
//				{
//					TIM_SetCounter(TIM3,0);
//					TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
//					TIM_Cmd(TIM3,ENABLE ); 	//使能定时器5
//					TIM3CH1_CAPTURE_STA=0;			//清空
//					TIM3CH1_CAPTURE_VAL=0;
//					TIM3CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
//				}
//			}
//		}
//		else
//		{
//			TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//			if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
//			{
//				if(TIM3CH1_CAPTURE_STA&0X40)		//捕获到又一个上降沿 		
//				{
//					TIM3CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
//					TIM3CH1_CAPTURE_VAL=TIM_GetCapture1(TIM3);//获取当前的捕获值.
//				}else  								//还未开始,第一次捕获上升沿
//				{
//					TIM_SetCounter(TIM3,0);
//					TIM3CH1_CAPTURE_STA=0;			//清空
//					TIM3CH1_CAPTURE_VAL=0;
//					TIM3CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
//				}
//			}
//		}
//		if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//溢出
//		{
//			if(TIM3CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
//			{
//				if((TIM3CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
//				{
//					TIM3CH1_CAPTURE_STA|=0X80;		//标记成功捕获了一次
//					TIM3CH1_CAPTURE_VAL=0XFFFF;
//				}
//				else
//					TIM3CH1_CAPTURE_STA++;
//			}
//		}
// 	}
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位
}

TIM_ICInitTypeDef  TIM4_ICInitStructure;
//定时器4通道1输入捕获配置
//arr：自动重装值(TIM2,TIM5是32位的!!)
//psc：时钟预分频数
void TIM4_CH1_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTB时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //GPIOD12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA0

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); //PA0复用位定时器5
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	

	//初始化TIM5输入捕获参数
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
		
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	
  TIM_Cmd(TIM4,ENABLE ); 	//使能定时器4
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到上升沿;1,已经捕获到上升沿了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM4CH1_CAPTURE_STA=0;	//输入捕获状态	
u32	TIM4CH1_CAPTURE_VAL1;	//输入捕获值1(TIM2/TIM5是32位)
u32	TIM4CH1_CAPTURE_VAL2;	//输入捕获值2(TIM2/TIM5是32位)
u8 TIM4CH1_One_cycle=0;// 是否测试周期值;
u32 TIM4CH1_nums = 0;//定义数量;
//定时器5中断服务程序
void TIM4_IRQHandler(void)
{
 	if((TIM4CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if((TIM4CH1_One_cycle&0x01)==0)
		{
			if(TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)//捕获1测周期
			{
				if(TIM4CH1_nums == 0)
					TIM4CH1_CAPTURE_VAL1=TIM_GetCapture1(TIM4);//获取当前的捕获值.
				else if(TIM4CH1_nums == 1)
				{
					TIM4CH1_CAPTURE_VAL2=TIM_GetCapture1(TIM4);//获取当前的捕获值.
					TIM4CH1_CAPTURE_STA |= 0x80;
				}
				TIM4CH1_nums++;
			}
		}
		else
		{
			if(TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)//捕获1测脉宽
			{
				if(TIM4CH1_nums == 0)
				{
					TIM4CH1_CAPTURE_VAL1=TIM_GetCapture1(TIM4);//获取当前的捕获值.
					TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				}
				else if(TIM4CH1_nums == 1)
				{
					TIM4CH1_CAPTURE_VAL2=TIM_GetCapture1(TIM4);//获取当前的捕获值.
					TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising);		//CC1P=1 设置为下降沿捕获
					TIM4CH1_CAPTURE_STA |= 0x80;
				}
				TIM4CH1_nums++;
			}
		}
		
		if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//溢出
		{
			if(TIM4CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM4CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM4CH1_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					TIM4CH1_CAPTURE_VAL2=0XFFFFFFFF;
				}
				else
					TIM4CH1_CAPTURE_STA++;
			}
		}
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位
}

