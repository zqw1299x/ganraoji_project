#include "exti.h"
#include "led.h"
	   
//�ⲿ�жϳ�ʼ������
//��ʼ��PE15,PA0Ϊ�ж�����.
void exti_func_init(void)
{
    NVIC_InitTypeDef   NVIC_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;

    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);   //ʹ��GPIOEʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;              //PE15
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;            //��ͨ����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);                  //��ʼ��GPIOE15

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  //ʹ��SYSCFGʱ��

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource15);//PE2 ���ӵ��ж���2
    /* ����EXTI_Line0 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;//LINE15
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش��� 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE15
    EXTI_Init(&EXTI_InitStructure);//����

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//�ⲿ�ж�0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;//�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����
}

void EXTI15_10_IRQHandler(void)				// PE15�½����Ǹ��ٿ�ͬ������DMA�����źţ�Լ25ms����һ��
{
	EXTI_ClearITPendingBit(EXTI_Line15);    //���LINE15�ϵ��жϱ�־λ
	//DMA_Cmd(DMA2_Stream5,ENABLE);
	USART1->CR1=USART1->CR1|0x04;	        //ͬ���źŵ����������
}










