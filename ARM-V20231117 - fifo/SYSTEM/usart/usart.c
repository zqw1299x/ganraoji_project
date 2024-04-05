#include "sys.h"
#include "usart.h"	
#include "string.h"
#include "led.h"
//////////////////////////
#include "delay.h"
//////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4探索者开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
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
__align(4) char uart2_tx_buffer[TX2_BUFF_SIZE]={0x96,0x96,0x15};//对外上传数据数组
#elif OUTINTERFACE == PLANE
__align(4) char uart2_tx_buffer[TX2_BUFF_SIZE]={0xEB,0x90,0x40};	//对外上传数据数组
#elif OUTINTERFACE == _8511
__align(4) char uart2_tx_buffer[TX2_BUFF_SIZE]={HEAD_BYTE_1,HEAD_BYTE_2};	//对外上传数据数组
#else
__align(4) char uart2_tx_buffer[TX2_BUFF_SIZE]={0xEB,0x90,0xfc};
#endif
__align(4) char uart2_temp_buffer[TX2_BUFF_SIZE]={0xEB,0x90,0x40};	//对外上传数据数组
__align(4) char uart2_jiexi_buffer[100]={0xEB,0x90,0x40};	//对外上传数据数组
//对内;
//__align(4) char uart1_tx_buffer[TX1_BUFF_SIZE]={'$','G',0,0,14};
__align(4) char uart1_tx_buffer[TX1_BUFF_SIZE]={0xEB,0x90,0x01};        //向FPGA 下发指令数组
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
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{
	x = x; 
}
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((UART4->SR&0X40)==0);//循环发送,直到发送完毕   
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

/*********************串口接收fifo相关**********************/

RingBuffer usart2_rx_fifo;
RingBuffer usart2_rx_fifo1;

//将数据写入缓冲区
uint32_t ring_buffer_write(RingBuffer *rb, const uint8_t *data, uint32_t count)
{
    uint32_t i;
    for(i = 0; i < count; i++)
    {
        uint32_t next_tail = (rb->tail + 1) % FIFO_BUFFER_SIZE;
        if(next_tail == rb->head)
        {
            //缓冲区已满
            return i;
        }
        rb->buffer[rb->tail] = data[i];             //写入数据
        rb->tail = next_tail;                       //更新尾指针
    }
    return count;
}

// 将数据从缓冲区读取
uint32_t ring_buffer_read(RingBuffer *rb, uint8_t *data, uint32_t count)
{
    uint32_t i;
    for(i = 0; i < count; i++)
    {
        if(rb->head == rb->tail)
        {
            //缓冲区为空
            return i;
        }
        data[i] = rb->buffer[rb->head];                  //读数据
        rb->head = (rb->head + 1) % FIFO_BUFFER_SIZE;    //更新指针
    }
    return count;
}
/************************************************************/

//RX外设到内存DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:外设地址
//mar0:存储器0地址
//mar1:存储器1地址
//ndtr:数据传输量  
void MY_RX_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0,u32 mar1, u16 ndtr)
{ 
    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)                          //得到当前stream是属于DMA2还是DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   //DMA2时钟使能 
        
    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);   //DMA1时钟使能 
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}      //等待DMA可配置 

    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                                    //通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;                         //DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = mar0;                           //DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //外设到存储器模式
    DMA_InitStructure.DMA_BufferSize = ndtr;                                //数据传输量 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //使用普通模式 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //外设突发单次传输  
    DMA_DoubleBufferModeConfig(DMA_Streamx,mar1, DMA_Memory_0);
    DMA_DoubleBufferModeCmd(DMA_Streamx, ENABLE);
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                              //初始化DMA Stream
} 


void MY_RX2_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0, u16 ndtr)
{ 
    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)                              //得到当前stream是属于DMA2还是DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);       //DMA2时钟使能 
        
    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);       //DMA1时钟使能 
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}          //等待DMA可配置 

    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                        //通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;             //DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = mar0;               //DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;     //外设到存储器模式
    DMA_InitStructure.DMA_BufferSize = ndtr;                    //数据传输量 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //使用普通模式 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //外设突发单次传输  
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                              //初始化DMA Stream	
} 

//TX内存到外设DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:外设地址
//mar0:存储器地址
//ndtr:数据传输量  
void MY_TX_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0, u16 ndtr)
{ 
    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)                          //得到当前stream是属于DMA2还是DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   //DMA2时钟使能 
        
    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);   //DMA1时钟使能 
    }
    DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}      //等待DMA可配置 

    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                    //通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;         //DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = mar0;           //DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; //外设到存储器模式
    DMA_InitStructure.DMA_BufferSize = ndtr;                //数据传输量 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //使用普通模式 
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //外设突发单次传输  
    DMA_Init(DMA_Streamx, &DMA_InitStructure);                              //初始化DMA Stream	
} 
  	

//串口111111111111111111111111111111111111111111111111111111
//串口111111111111111111111111111111111111111111111111111111
//串口111111111111111111111111111111111111111111111111111111
//串口111111111111111111111111111111111111111111111111111111
//串口111111111111111111111111111111111111111111111111111111
//串口111111111111111111111111111111111111111111111111111111
//串口111111111111111111111111111111111111111111111111111111
//串口111111111111111111111111111111111111111111111111111111
//bound:波特率
void uart1_init(u32 bound)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);                //使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);               //使能USART1时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);             //GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);            //GPIOA10复用为USART1

    //USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;             //GPIOA9与GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	                //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure);                               //初始化PA9，PA10

    //USART1 初始化设置
    USART_InitStructure.USART_BaudRate = bound;                                     //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode =  USART_Mode_Tx;	                            //收发模式
    USART_Init(USART1, &USART_InitStructure);                                       //初始化串口1

    USART_ClearFlag(USART1, USART_FLAG_TC);

    //DMA接收通道中断配置
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);    

    //DMA发送通道中断配置
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
    DMA_Cmd(DMA2_Stream5,ENABLE);			//先使能DMA接收后使能串口，防止出现接收上溢
    USART_Cmd(USART1, ENABLE);              //使能串口1 
}

// 串口222222222222222222222222222222222222222222222222222222
// 串口222222222222222222222222222222222222222222222222222222
// 串口222222222222222222222222222222222222222222222222222222
// 串口222222222222222222222222222222222222222222222222222222
// 串口222222222222222222222222222222222222222222222222222222
// 串口222222222222222222222222222222222222222222222222222222
// 串口222222222222222222222222222222222222222222222222222222
// 初始化----------------------115200，DMA收发 //
void uart2_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);                //使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);               //使能USART2时钟

    //串口2对应引脚复用映射
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);             //GPIOA9复用为USART2
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);             //GPIOA10复用为USART2

    //USART2端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;              //GPIOA2与GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	                //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure);                               //初始化PA9，PA10

    //USART2 初始化设置
    USART_InitStructure.USART_BaudRate = bound;                         //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;         //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;              //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                 //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //收发模式
    USART_Init(USART2, &USART_InitStructure);                                       //初始化串口2

    USART_ClearFlag(USART2, USART_FLAG_TC);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                      //开启相关中断

    //USART2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                   //串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;             //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		            //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			            //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	                                    //根据指定的参数初始化VIC寄存器

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //发送使用DMA
    //MY_RX_DMA_Config(DMA1_Stream5,DMA_Channel_4,(u32)&USART1->DR,(u32)uart2_rx_buffer,(u32)uart2_rx_buffer1,RX2_BUFF_SIZE);
    MY_TX_DMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)&uart2_tx_buffer[0],TX2_BUFF_SIZE);
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
    DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);

    USART_Cmd(USART2, ENABLE);                                          //使能串口2
}

u8 uart2_rec_flag=0;
u16 rec_len=0;
u16 rec_ding_len=0;
void USART2_IRQHandler(void)                	            //串口2中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)   //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART2);                     //(USART1->DR);	//指令控制的方式;	      
		if(uart2_rec_flag=='Y')
		{
			uart2_rx_buffer[rec_len]=Res;
			rec_len++;
			if(rec_len == DATA_LEN)
			{
				rec_len=0;
				uart2_rec_flag='F';
				USART_Cmd(USART2, DISABLE);                 //使能串口2
			}
			return;
		}
		else if(uart2_rec_flag=='T')                        //参数配置的内容接收;
		{
			uart2_rx_buffer[rec_len]=Res;
			rec_len++;
			if(rec_len == 8)
			{
				rec_ding_len = uart2_rx_buffer[7]<<8;
				rec_ding_len |= uart2_rx_buffer[6];
				rec_ding_len += 10;          //包含头尾;
			}
			if((rec_len>7) && (rec_len == rec_ding_len))
			{
				rec_len=0;
				uart2_rec_flag='P';
				memcpy(parameters,uart2_rx_buffer,RX2_BUFF_SIZE);
				USART_Cmd(USART2, DISABLE);  //失能串口2
			}
			return;
		}
		else if(uart2_rec_flag=='X')         //参数配置的内容接收;
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

		//找到头并且加以区分;
		if(rec_len == 0)
		{
			uart2_rec_flag = 1;
			rec_len ++;
			uart2_rx_buffer[0] = Res;
		}
		else if(rec_len == 1)
		{
			uart2_rx_buffer[1] = Res;
			if((uart2_rx_buffer[0] == HEAD_BYTE_1) && (uart2_rx_buffer[1] == HEAD_BYTE_2))//遥测指令;
			{
				uart2_rec_flag='Y';
				rec_len=2;
			}
			else if((uart2_rx_buffer[0] == 0xCF) && (uart2_rx_buffer[0] == 0x1A))         //调试状态的参数指令接收头为0xCF1A;
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


//串口1111111111111111111111111111111111111111111111111
//接收接收接收接收接收接收接收接收接收，DMA传输完成中断
u8 uart1_rec_flag=0;
void DMA2_Stream5_IRQHandler(void) 
{
    printf("enterx uart1_rec_flag:2\r\n");
    //DMA_Cmd(DMA2_Stream5,DISABLE);
    //USART1->CR1 = USART1->CR1&0xfffffffB;	        //一次接收完成，关闭接收，直到下次同步信号来，再打开接收
    DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);	
    if(DMA_GetCurrentMemoryTarget(DMA2_Stream5)==0)
    {
        printf("enter uart1_rec_flag:2\r\n");
        uart1_rec_flag=2;					        //先传MEMORY 1 flag=2 表示正在DMA MEM1 ,MEM2的数据可读
        //LED0=!LED0;
    }
    else		
    {
        printf("enter uart1_rec_flag:1\r\n");
        uart1_rec_flag=1;		
        //LED1=!LED1;
    }  
}

//串口111111111111111111111111111111111
//发送发送发送发送发送发送发送发送发送，DMA传输完成中断
void DMA2_Stream7_IRQHandler(void) 
{
	DMA_Cmd(DMA2_Stream7,DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);	
	DMA_SetCurrDataCounter(DMA2_Stream7,TX1_BUFF_SIZE);
	u1_tx_idle=1;																				//U1发送完数据
}


//串口22222222222222222222222222222
//发送发送发送发送发送发送发送发送发送，DMA传输完成中断
void DMA1_Stream6_IRQHandler(void) 
{
	DMA_Cmd(DMA1_Stream6,DISABLE);
	DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);	
	DMA_SetCurrDataCounter(DMA1_Stream6,TX2_BUFF_SIZE); 
	u2_tx_idle=1;																							//U2发送完数据
}


// 串口33333333333333
// 初始化--------------------
void uart3_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);        //使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);       //使能USART3时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);    //GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);    //GPIOA10复用为USART1

    //USART3端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;    //GPIOA2与GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	        //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;              //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                //上拉
    GPIO_Init(GPIOB,&GPIO_InitStructure);                       //初始化PA9，PA10

    //USART3 初始化设置
    USART_InitStructure.USART_BaudRate = bound;                     //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART3, &USART_InitStructure);                       //初始化串口USART3

    USART_ClearFlag(USART3, USART_FLAG_TC);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);          //开启相关中断

    //USART3 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;       //串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器、

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);

    //发送使用DMA
    MY_TX_DMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)&uart3_tx_buffer[0],TX3_BUFF_SIZE);
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
    DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);

    USART_Cmd(USART3, ENABLE);  //使能串口2
}

void USART3_IRQHandler(void)                	            //串口2中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)   //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART3);//(USART1->DR);	    //读取接收到的数据;
		Res = Res;
	}
}

//串口333333333333333
//发送发送发送发送发送发送发送发送发送，DMA传输完成中断
void DMA1_Stream3_IRQHandler(void) 
{
	DMA_Cmd(DMA1_Stream3,DISABLE);
	DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);	
	DMA_SetCurrDataCounter(DMA1_Stream3,TX3_BUFF_SIZE); 
	u3_tx_idle=1;																							//U2发送完数据
}


// 串口444444444
// 初始化--------------------
void uart4_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);    //使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);    //使能UART4时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA10复用为USART1

    //UART4端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  //GPIOA2与GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	    //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure);                   //初始化PA9，PA10

    //UART4 初始化设置
    USART_InitStructure.USART_BaudRate = bound;                 //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;         //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //收发模式
    USART_Init(UART4, &USART_InitStructure );                                       //初始化串口UART4


    USART_ClearFlag(UART4, USART_FLAG_TC);

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断

    //UART4 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;        //串口4中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器、

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);

    //发送使用DMA
    MY_TX_DMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)&uart4_tx_buffer[0],TX4_BUFF_SIZE);
    USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
    DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);

    USART_Cmd(UART4, ENABLE);                               //使能串口4
}

void UART4_IRQHandler(void)                	                //串口2中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)    //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(UART4);//(USART1->DR);	    //读取接收到的数据;
		Res = Res;
	}
}

//串口444444444444444444
//发送发送发送发送发送发送发送发送发送，DMA传输完成中断
void DMA1_Stream4_IRQHandler(void) 
{
	DMA_Cmd(DMA1_Stream4,DISABLE);
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4);	
	DMA_SetCurrDataCounter(DMA1_Stream4,byte_read_u4); 
	u4_tx_idle=1;																							//U2发送完数据
}

// 串口666666666666666666666666666666666666666666666666666666666666666666
// 串口666666666666666666666666666666666666666666666666666666666666666666
// 串口666666666666666666666666666666666666666666666666666666666666666666
// 串口666666666666666666666666666666666666666666666666666666666666666666
// 初始化--------------------
// FPGA与ARM之间预留的一路串口
void uart6_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);    //使能GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);   //使能UART时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIO复用
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIO复用

    //UART6端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	    //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOC,&GPIO_InitStructure);                   //初始化

    //UART6 初始化设置
    USART_InitStructure.USART_BaudRate = bound;                                     //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //收发模式
    USART_Init(USART6, &USART_InitStructure);                                       //初始化串口UART4


    USART_ClearFlag(USART6, USART_FLAG_TC);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);          //开启相关中断

    //UART6 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;       //串口6中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器、
 
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;        
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);

    //发送使用DMA
    MY_TX_DMA_Config(DMA2_Stream6,DMA_Channel_5,(u32)&USART6->DR,(u32)&uart6_tx_buffer[0],TX6_BUFF_SIZE);
    USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
    DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);

    USART_Cmd(USART6, ENABLE);                              //使能串口2
}

u8 uart6_rec_flag=0;
u16 u6_rec_len=0;

void USART6_IRQHandler(void)                	            //串口2中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)   //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART6);                     //(USART6->DR);	//指令控制的方式;	
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
					//USART_Cmd(USART6, DISABLE);               //失能串口6
			}
			return;
		}
		
		//找到头并且加以区分;
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

//串口666666666666666666666666
//发送发送发送发送发送发送发送发送发送，DMA传输完成中断
void DMA2_Stream6_IRQHandler(void) 
{
	DMA_Cmd(DMA2_Stream6,DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream6,DMA_IT_TCIF6);	
	DMA_SetCurrDataCounter(DMA2_Stream6,byte_read_u6); 
	u6_tx_idle=1;																			//U2发送完数据
}

