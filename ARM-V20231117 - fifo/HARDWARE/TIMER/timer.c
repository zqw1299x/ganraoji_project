#include "sys.h"
#include "usart.h"

extern int irqCount;
u32 cnt_t=0;
u32 cnt_t_old = 0;
u32 cnt_t_record_old = 0;
u32 cnt_UAV_Run = 0;//开机时长，针对无人机做的一个变量;
u32 cnt_UAV_Run_old = 0;
#define		S_PULSE_H						GPIO_SetBits(GPIOD,GPIO_Pin_12)				
#define		S_PULSE_L						GPIO_ResetBits(GPIOD,GPIO_Pin_12)	


void TIM2_Int_Init(u32 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);             ///使能TIM时钟

    TIM_TimeBaseInitStructure.TIM_Period = arr; 	                //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler=psc;                    //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;   //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);                        //允许定时器更新中断

    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);              //初始化TIM

    NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;                   //定时器3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;      //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03;             //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM2, ENABLE);  //使能TIMER
}

//定时器2中断服务函数
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) 
	{
		cnt_t++;
		cnt_UAV_Run++;
        //if(cnt_t%2 == 0)printf("irqCount:%d\r\n",irqCount);
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  
}

void TIM1_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTF时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PE9	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOF9复用为定时器14
	


	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器1
	
	//初始化TIM1 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_Pulse = 19999;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	

	TIM_Cmd(TIM1, ENABLE);  //使能TIM14
//  TIM_CtrlPWMOutputs(TIM1, ENABLE);							  
}  
