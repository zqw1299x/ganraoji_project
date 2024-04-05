#include "exti.h"
#include "led.h"
	   
//外部中断初始化程序
//初始化PE15,PA0为中断输入.
void exti_func_init(void)
{
    NVIC_InitTypeDef   NVIC_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;

    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);   //使能GPIOE时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;              //PE15
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;            //普通输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);                  //初始化GPIOE15

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  //使能SYSCFG时钟

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource15);//PE2 连接到中断线2
    /* 配置EXTI_Line0 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;//LINE15
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE15
    EXTI_Init(&EXTI_InitStructure);//配置

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//外部中断0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
}

void EXTI15_10_IRQHandler(void)				// PE15下降沿是高速口同步接收DMA触发信号，约25ms触发一次
{
	EXTI_ClearITPendingBit(EXTI_Line15);    //清除LINE15上的中断标志位
	//DMA_Cmd(DMA2_Stream5,ENABLE);
	USART1->CR1=USART1->CR1|0x04;	        //同步信号到达，开启接收
}










