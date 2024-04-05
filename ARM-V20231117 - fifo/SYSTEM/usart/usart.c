#include "sys.h"
#include "usart.h"	
#include "string.h"
#include "led.h"
//////////////////////////
#include "delay.h"
//////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4̽���߿�����
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/6/10
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  

__align(4) char uart1_rx_buffer0[RX1_BUFF_SIZE];
__align(4) char uart1_rx_buffer1[RX1_BUFF_SIZE];
__align(4) char uart2_rx_buffer[RX2_BUFF_SIZE];
__align(4) char uart2_rx_buffer1[RX2_BUFF_SIZE];
__align(4) char uart3_rx_buffer[RX3_BUFF_SIZE];
__align(4) char uart4_rx_buffer[RX4_BUFF_SIZE];
__align(4) char uart3_tx_buffer[TX3_BUFF_SIZE]={0x85,0x11};
__align(4) char uart4_tx_buffer[TX4_BUFF_SIZE]={0xEB,0x90,0xfc};

__align(4) unsigned char uart6_rx_buffer[RX6_BUFF_SIZE];
__align(4) unsigned char uart6_temp_rx_buff[RX6_BUFF_SIZE];
__align(4) unsigned char uart6_tx_buffer[TX6_BUFF_SIZE];

int irqCount = 0;

#if OUTINTERFACE == SANYUAN
__align(4) char uart2_tx_buffer[TX2_BUFF_SIZE]={0x96,0x96,0x15};//�����ϴ���������
#elif OUTINTERFACE == PLANE
__align(4) char uart2_tx_buffer[TX2_BUFF_SIZE]={0xEB,0x90,0x40};	//�����ϴ���������
#elif OUTINTERFACE == _8511
__align(4) char uart2_tx_buffer[TX2_BUFF_SIZE]={HEAD_BYTE_1,HEAD_BYTE_2};	//�����ϴ���������
#else
__align(4) char uart2_tx_buffer[TX2_BUFF_SIZE]={0xEB,0x90,0xfc};
#endif
__align(4) char uart2_temp_buffer[TX2_BUFF_SIZE]={0xEB,0x90,0x40};	//�����ϴ���������
__align(4) char uart2_jiexi_buffer[100]={0xEB,0x90,0x40};	//�����ϴ���������
//����;
//__align(4) char uart1_tx_buffer[TX1_BUFF_SIZE]={'$','G',0,0,14};
__align(4) char uart1_tx_buffer[TX1_BUFF_SIZE]={0xEB,0x90,0x01};        //��FPGA �·�ָ������
__align(4) char parameters[RX2_BUFF_SIZE];
__align(4) char ALL_parameters[10000];
__align(4) char ALL_parameters_yanzheng[10000];

char u1_tx_idle=1;
char u2_tx_idle=1;
char u3_tx_idle=1;
char u4_tx_idle=1;
char u6_tx_idle=1;
u16  uart2To6Len = 0;
u16  uart6To2Len = 0;
u16  byte_read_u4 = 0,byte_read_u6 = 0;
u8   ganRaojPower = 0,ganRaojFlag = 0,fashejiPwer = 0;
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{
	x = x; 
}
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((UART4->SR&0X40)==0);//ѭ������,ֱ���������   
	UART4->DR = (u8) ch;      
	return ch;
}
#endif

//logPrint
void logPrintU4(int len){

	int i = 0;
	printf("longPrint:");
	for(i=0;i<len;i++)
	printf("%02x ",uart4_tx_buffer[i]);
	printf("\r\n");
}

/*********************���ڽ���fifo���**********************/

RingBuffer usart2_rx_fifo;
RingBuffer usart2_rx_fifo1;

//������д�뻺����
uint32_t ring_buffer_write(RingBuffer *rb, const uint8_t *data, uint32_t count)
{
    uint32_t i;
    for(i = 0; i < count; i++)
    {
        uint32_t next_tail = (rb->tail + 1) % FIFO_BUFFER_SIZE;
        if(next_tail == rb->head)
        {
            //����������
            return i;
        }
        rb->buffer[rb->tail] = data[i];             //д������
        rb->tail = next_tail;                       //����βָ��
    }
    return count;
}

// �����ݴӻ�������ȡ
uint32_t ring_buffer_read(RingBuffer *rb, uint8_t *data, uint32_t count)
{
    uint32_t i;
    for(i = 0; i < count; i++)
    {
        if(rb->head == rb->tail)
        {
            //������Ϊ��
            return i;
        }
        data[i] = rb->buffer[rb->head];                  //������
        rb->head = (rb->head + 1) % FIFO_BUFFER_SIZE;    //����ָ��
    }
    return count;
}
/************************************************************/

//RX���赽�ڴ�DMAx�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMAͨ��ѡ��,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:�����ַ
//mar0:�洢��0��ַ
//mar1:�洢��1��ַ
//ndtr:���ݴ�����  
void MY_RX_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0,u32 mar1, u16 ndtr)
{ 
    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)                          //�õ���ǰstream������DMA2����DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   //DMA2ʱ��ʹ�� 
        
    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);   //DMA1ʱ��ʹ�� 
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}      //�ȴ�DMA������ 

    /* ���� DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                                    //ͨ��ѡ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;                         //DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = mar0;                           //DMA �洢��0��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //���赽�洢��ģʽ
    DMA_InitStructure.DMA_BufferSize = ndtr;                                //���ݴ����� 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //���������ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�洢������ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݳ���:8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�洢�����ݳ���:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //ʹ����ͨģʽ 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //����ͻ�����δ���  
    DMA_DoubleBufferModeConfig(DMA_Streamx,mar1, DMA_Memory_0);
    DMA_DoubleBufferModeCmd(DMA_Streamx, ENABLE);
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                              //��ʼ��DMA Stream
} 


void MY_RX2_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0, u16 ndtr)
{ 
    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)                              //�õ���ǰstream������DMA2����DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);       //DMA2ʱ��ʹ�� 
        
    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);       //DMA1ʱ��ʹ�� 
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}          //�ȴ�DMA������ 

    /* ���� DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                        //ͨ��ѡ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;             //DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = mar0;               //DMA �洢��0��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;     //���赽�洢��ģʽ
    DMA_InitStructure.DMA_BufferSize = ndtr;                    //���ݴ����� 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //���������ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�洢������ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݳ���:8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�洢�����ݳ���:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //ʹ����ͨģʽ 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //����ͻ�����δ���  
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                              //��ʼ��DMA Stream	
} 

//TX�ڴ浽����DMAx�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMAͨ��ѡ��,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:�����ַ
//mar0:�洢����ַ
//ndtr:���ݴ�����  
void MY_TX_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0, u16 ndtr)
{ 
    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)                          //�õ���ǰstream������DMA2����DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   //DMA2ʱ��ʹ�� 
        
    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);   //DMA1ʱ��ʹ�� 
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}      //�ȴ�DMA������ 

    /* ���� DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                    //ͨ��ѡ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;         //DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = mar0;           //DMA �洢��0��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; //���赽�洢��ģʽ
    DMA_InitStructure.DMA_BufferSize = ndtr;                //���ݴ����� 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //���������ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�洢������ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݳ���:8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�洢�����ݳ���:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //ʹ����ͨģʽ 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //����ͻ�����δ���  
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                              //��ʼ��DMA Stream	
} 
  	

//����111111111111111111111111111111111111111111111111111111
//����111111111111111111111111111111111111111111111111111111
//����111111111111111111111111111111111111111111111111111111
//����111111111111111111111111111111111111111111111111111111
//����111111111111111111111111111111111111111111111111111111
//����111111111111111111111111111111111111111111111111111111
//����111111111111111111111111111111111111111111111111111111
//����111111111111111111111111111111111111111111111111111111
//bound:������
void uart1_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);                //ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);               //ʹ��USART1ʱ��

    //����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);             //GPIOA9����ΪUSART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);            //GPIOA10����ΪUSART1

    //USART1�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;             //GPIOA9��GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	                //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //����
    GPIO_Init(GPIOA,&GPIO_InitStructure);                               //��ʼ��PA9��PA10

    //USART1 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;                                     //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode =  USART_Mode_Tx;	                            //�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure);                                       //��ʼ������1

    USART_ClearFlag(USART1, USART_FLAG_TC);

    //DMA����ͨ���ж�����
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);    

    //DMA����ͨ���ж�����
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);    

    MY_RX_DMA_Config(DMA2_Stream5,DMA_Channel_4,(u32)&USART1->DR,(u32)uart1_rx_buffer0,(u32)uart1_rx_buffer1,RX1_BUFF_SIZE);
    MY_TX_DMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)uart1_tx_buffer,TX1_BUFF_SIZE);

    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);

    DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
    DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA2_Stream5,ENABLE);			//��ʹ��DMA���պ�ʹ�ܴ��ڣ���ֹ���ֽ�������
    USART_Cmd(USART1, ENABLE);              //ʹ�ܴ���1 
}

// ����222222222222222222222222222222222222222222222222222222
// ����222222222222222222222222222222222222222222222222222222
// ����222222222222222222222222222222222222222222222222222222
// ����222222222222222222222222222222222222222222222222222222
// ����222222222222222222222222222222222222222222222222222222
// ����222222222222222222222222222222222222222222222222222222
// ����222222222222222222222222222222222222222222222222222222
// ��ʼ��----------------------115200��DMA�շ� //
void uart2_init(u32 bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);                //ʹ��GPIOAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);               //ʹ��USART2ʱ��

    //����2��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);             //GPIOA9����ΪUSART2
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);             //GPIOA10����ΪUSART2

    //USART2�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;              //GPIOA2��GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	                //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //����
    GPIO_Init(GPIOA,&GPIO_InitStructure);                               //��ʼ��PA9��PA10

    //USART2 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;                         //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;         //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;              //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                 //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure);                                       //��ʼ������2

    USART_ClearFlag(USART2, USART_FLAG_TC);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                      //��������ж�

    //USART2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                   //����1�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;             //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		            //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			            //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	                                    //����ָ���Ĳ�����ʼ��VIC�Ĵ���

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //����ʹ��DMA
    //MY_RX_DMA_Config(DMA1_Stream5,DMA_Channel_4,(u32)&USART1->DR,(u32)uart2_rx_buffer,(u32)uart2_rx_buffer1,RX2_BUFF_SIZE);
    MY_TX_DMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)&uart2_tx_buffer[0],TX2_BUFF_SIZE);
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
    DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);

    USART_Cmd(USART2, ENABLE);                                          //ʹ�ܴ���2
}

u8 uart2_rec_flag=0;
u16 rec_len=0;
u16 rec_ding_len=0;
void USART2_IRQHandler(void)                	            //����2�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)   //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART2);                     //(USART1->DR);	//ָ����Ƶķ�ʽ;	      
		if(uart2_rec_flag=='Y')
		{
			uart2_rx_buffer[rec_len]=Res;
			rec_len++;
			if(rec_len == DATA_LEN)
			{
				rec_len=0;
				uart2_rec_flag='F';
				USART_Cmd(USART2, DISABLE);                 //ʹ�ܴ���2
			}
			return;
		}
		else if(uart2_rec_flag=='T')                        //�������õ����ݽ���;
		{
			uart2_rx_buffer[rec_len]=Res;
			rec_len++;
			if(rec_len == 8)
			{
				rec_ding_len = uart2_rx_buffer[7]<<8;
				rec_ding_len |= uart2_rx_buffer[6];
				rec_ding_len += 10;          //����ͷβ;
			}
			if((rec_len>7) && (rec_len == rec_ding_len))
			{
				rec_len=0;
				uart2_rec_flag='P';
				memcpy(parameters,uart2_rx_buffer,RX2_BUFF_SIZE);
				USART_Cmd(USART2, DISABLE);  //ʧ�ܴ���2
			}
			return;
		}
		else if(uart2_rec_flag=='X')         //�������õ����ݽ���;
		{
			uart2_rx_buffer[rec_len]=Res;
			rec_len++;
			if((uart2_rx_buffer[rec_len-2] == 0xA1) && (uart2_rx_buffer[rec_len-1] == 0xFC))
			{
				ring_buffer_write(&usart2_rx_fifo, (const uint8_t*)uart2_rx_buffer, rec_len);
				ring_buffer_write(&usart2_rx_fifo1, (const uint8_t*)uart2_rx_buffer, rec_len);
				uart2_rec_flag='U';
				rec_len = 0;

				if(uart2_rx_buffer[4] == 0x01)
				{
					if(uart2_rx_buffer[8] == 0x03)ganRaojPower = 0x03;
					else if(uart2_rx_buffer[8] == 0x02)ganRaojPower = 0x02;
					else if(uart2_rx_buffer[8] == 0x01)ganRaojPower = 0x01;
					else if(uart2_rx_buffer[8] == 0x00)ganRaojPower = 0x00;

					if(uart2_rx_buffer[7] == 0x01)fashejiPwer = 1;
					if(uart2_rx_buffer[7] == 0x00)fashejiPwer = 0;
					ganRaojFlag = 1;
				}
                
			}
			return;
		}

		//�ҵ�ͷ���Ҽ�������;
		if(rec_len == 0)
		{
			uart2_rec_flag = 1;
			rec_len ++;
			uart2_rx_buffer[0] = Res;
		}
		else if(rec_len == 1)
		{
			uart2_rx_buffer[1] = Res;
			if((uart2_rx_buffer[0] == HEAD_BYTE_1) && (uart2_rx_buffer[1] == HEAD_BYTE_2))//ң��ָ��;
			{
				uart2_rec_flag='Y';
				rec_len=2;
			}
			else if((uart2_rx_buffer[0] == 0xCF) && (uart2_rx_buffer[0] == 0x1A))         //����״̬�Ĳ���ָ�����ͷΪ0xCF1A;
			{
				uart2_rec_flag='T';
				rec_len=2;
			}
            else if((uart2_rx_buffer[0] == 0xCF) && (uart2_rx_buffer[1] == 0x1A))
            {
                uart2_rec_flag='X';
                rec_len=2;
            }
			else
			{
				uart2_rx_buffer[0] = uart2_rx_buffer[1];
				rec_len = 1;
				uart2_rec_flag=0;
			}
			
		}
	}
} 


//����1111111111111111111111111111111111111111111111111
//���ս��ս��ս��ս��ս��ս��ս��ս��գ�DMA��������ж�
u8 uart1_rec_flag=0;
void DMA2_Stream5_IRQHandler(void) 
{
    printf("enterx uart1_rec_flag:2\r\n");
    //DMA_Cmd(DMA2_Stream5,DISABLE);
    //USART1->CR1 = USART1->CR1&0xfffffffB;	        //һ�ν�����ɣ��رս��գ�ֱ���´�ͬ���ź������ٴ򿪽���
    DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);	
    if(DMA_GetCurrentMemoryTarget(DMA2_Stream5)==0)
    {
        printf("enter uart1_rec_flag:2\r\n");
        uart1_rec_flag=2;					        //�ȴ�MEMORY 1 flag=2 ��ʾ����DMA MEM1 ,MEM2�����ݿɶ�
        //LED0=!LED0;
    }
    else		
    {
        printf("enter uart1_rec_flag:1\r\n");
        uart1_rec_flag=1;		
        //LED1=!LED1;
    }  
}

//����111111111111111111111111111111111
//���ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͣ�DMA��������ж�
void DMA2_Stream7_IRQHandler(void) 
{
	DMA_Cmd(DMA2_Stream7,DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);	
	DMA_SetCurrDataCounter(DMA2_Stream7,TX1_BUFF_SIZE);
	u1_tx_idle=1;																				//U1����������
}


//����22222222222222222222222222222
//���ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͣ�DMA��������ж�
void DMA1_Stream6_IRQHandler(void) 
{
	DMA_Cmd(DMA1_Stream6,DISABLE);
	DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);	
	DMA_SetCurrDataCounter(DMA1_Stream6,TX2_BUFF_SIZE); 
	u2_tx_idle=1;																							//U2����������
}


// ����33333333333333
// ��ʼ��--------------------
void uart3_init(u32 bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);        //ʹ��GPIOAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);       //ʹ��USART3ʱ��

    //����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);    //GPIOA9����ΪUSART1
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);    //GPIOA10����ΪUSART1

    //USART3�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;    //GPIOA2��GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	        //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;              //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                //����
    GPIO_Init(GPIOB,&GPIO_InitStructure);                       //��ʼ��PA9��PA10

    //USART3 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;                     //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;             //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART3, &USART_InitStructure);                       //��ʼ������USART3

    USART_ClearFlag(USART3, USART_FLAG_TC);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);          //��������ж�

    //USART3 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;       //����1�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��VIC�Ĵ�����

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);

    //����ʹ��DMA
    MY_TX_DMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)&uart3_tx_buffer[0],TX3_BUFF_SIZE);
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
    DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);

    USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���2
}

void USART3_IRQHandler(void)                	            //����2�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)   //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART3);//(USART1->DR);	    //��ȡ���յ�������;
		Res = Res;
	}
}

//����333333333333333
//���ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͣ�DMA��������ж�
void DMA1_Stream3_IRQHandler(void) 
{
	DMA_Cmd(DMA1_Stream3,DISABLE);
	DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);	
	DMA_SetCurrDataCounter(DMA1_Stream3,TX3_BUFF_SIZE); 
	u3_tx_idle=1;																							//U2����������
}


// ����444444444
// ��ʼ��--------------------
void uart4_init(u32 bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);    //ʹ��GPIOAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);    //ʹ��UART4ʱ��

    //����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA9����ΪUSART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA10����ΪUSART1

    //UART4�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  //GPIOA2��GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	    //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //����
    GPIO_Init(GPIOA,&GPIO_InitStructure);                   //��ʼ��PA9��PA10

    //UART4 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;                 //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;         //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //�շ�ģʽ
    USART_Init(UART4, &USART_InitStructure );                                       //��ʼ������UART4


    USART_ClearFlag(UART4, USART_FLAG_TC);

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//��������ж�

    //UART4 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;        //����4�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��VIC�Ĵ�����

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);

    //����ʹ��DMA
    MY_TX_DMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)&uart4_tx_buffer[0],TX4_BUFF_SIZE);
    USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
    DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);

    USART_Cmd(UART4, ENABLE);                               //ʹ�ܴ���4
}

void UART4_IRQHandler(void)                	                //����2�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)    //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(UART4);//(USART1->DR);	    //��ȡ���յ�������;
		Res = Res;
	}
}

//����444444444444444444
//���ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͣ�DMA��������ж�
void DMA1_Stream4_IRQHandler(void) 
{
	DMA_Cmd(DMA1_Stream4,DISABLE);
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4);	
	DMA_SetCurrDataCounter(DMA1_Stream4,byte_read_u4); 
	u4_tx_idle=1;																							//U2����������
}

// ����666666666666666666666666666666666666666666666666666666666666666666
// ����666666666666666666666666666666666666666666666666666666666666666666
// ����666666666666666666666666666666666666666666666666666666666666666666
// ����666666666666666666666666666666666666666666666666666666666666666666
// ��ʼ��--------------------
// FPGA��ARM֮��Ԥ����һ·����
void uart6_init(u32 bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);    //ʹ��GPIOʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);   //ʹ��UARTʱ��

    //����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIO����
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIO����

    //UART6�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	    //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //����
    GPIO_Init(GPIOC,&GPIO_InitStructure);                   //��ʼ��

    //UART6 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;                                     //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //�շ�ģʽ
    USART_Init(USART6, &USART_InitStructure);                                       //��ʼ������UART4


    USART_ClearFlag(USART6, USART_FLAG_TC);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);          //��������ж�

    //UART6 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;       //����6�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��VIC�Ĵ�����
 
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);

    //����ʹ��DMA
    MY_TX_DMA_Config(DMA2_Stream6,DMA_Channel_5,(u32)&USART6->DR,(u32)&uart6_tx_buffer[0],TX6_BUFF_SIZE);
    USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
    DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);

    USART_Cmd(USART6, ENABLE);                              //ʹ�ܴ���2
}

u8 uart6_rec_flag=0;
u16 u6_rec_len=0;

void USART6_IRQHandler(void)                	            //����2�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)   //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART6);                     //(USART6->DR);	//ָ����Ƶķ�ʽ;	
		if(uart6_rec_flag=='H')
		{
			uart6_rx_buffer[u6_rec_len]=Res;
			u6_rec_len++;
			if((uart6_rx_buffer[u6_rec_len-2] == 0xA1) && (uart6_rx_buffer[u6_rec_len-1] == 0xFC))
			{
					memcpy(uart2_tx_buffer,uart6_rx_buffer,u6_rec_len+1);
					memcpy(uart4_tx_buffer,uart6_rx_buffer,u6_rec_len+1);
					u6_rec_len=0;
					uart6To2Len = u6_rec_len;
					uart6_rec_flag='W';
					//USART_Cmd(USART6, DISABLE);               //ʧ�ܴ���6
			}
			return;
		}
		
		//�ҵ�ͷ���Ҽ�������;
		if(u6_rec_len == 0)
		{
			uart6_rec_flag = 1;
			u6_rec_len ++;
			uart6_rx_buffer[0] = Res;
		}
		else if(u6_rec_len == 1)
		{
			uart6_rx_buffer[1] = Res;
			if((uart6_rx_buffer[0] == 0xCF) && (uart6_rx_buffer[1] == 0x1A))
			{
				uart6_rec_flag='H';
				u6_rec_len=2;
			}
			else
			{
				uart6_rx_buffer[0] = uart6_rx_buffer[1];
				u6_rec_len = 1;
				uart6_rec_flag=1;
			}
		}
	}
}

//����666666666666666666666666
//���ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͷ��ͣ�DMA��������ж�
void DMA2_Stream6_IRQHandler(void) 
{
	DMA_Cmd(DMA2_Stream6,DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream6,DMA_IT_TCIF6);	
	DMA_SetCurrDataCounter(DMA2_Stream6,byte_read_u6); 
	u6_tx_idle=1;																			//U2����������
}

