#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "ff.h"
//////////////////////////////////////////////////////////////////////////////////	 
//extern u8 OUTINTERFACE;

#define _119		0
#define SANYUAN     1
#define PLANE		2
#define _8511		3//��ģʽҪ�������ص�Э�������޸ĺ�֡ͷ;
#define _70debug    4

#define OUTINTERFACE _70debug
////////////////////////////////////////////////////////////////////////////////// 
#define FIFO_BUFFER_SIZE     4096

#define EN_USART1_RX 1		//ʹ�ܣ�1��/��ֹ��0������1����
#define EN_USART2_RX 1		//ʹ�ܣ�1��/��ֹ��0������1����	  
#define RX1_BUFF_SIZE 37    //16384
#define TX1_BUFF_SIZE 220    //220

#define TX3_BUFF_SIZE 4
#define RX3_BUFF_SIZE 100
#define TX4_BUFF_SIZE 1024
#define RX4_BUFF_SIZE 100

#define TX6_BUFF_SIZE 300
#define RX6_BUFF_SIZE 300 

#if OUTINTERFACE == SANYUAN
	#define TX2_BUFF_SIZE 25
	#define RX2_BUFF_SIZE 41
#elif OUTINTERFACE == _119
	#define RX2_BUFF_SIZE 20
	#define TX2_BUFF_SIZE 256
#elif OUTINTERFACE == PLANE
	#define TX2_BUFF_SIZE 67
	#define RX2_BUFF_SIZE 20
#elif OUTINTERFACE == _8511
	#define TX2_BUFF_SIZE 30
	#define RX2_BUFF_SIZE 13
#elif OUTINTERFACE == _70debug
	#define TX2_BUFF_SIZE 1024
	#define RX2_BUFF_SIZE 1024
#endif

#define REC 0
#define SND 1
//#define		HEAD_INTERNAL_1		0xEB		//�ڲ���ͷ1�ֽ� ԭ0XEB
//#define		HEAD_INTERNAL_2		0x90		//�ڲ���ͷ2�ֽ� ԭ0X90

#define		HEAD_INTERNAL_1		0x55		//�ڲ���ͷ1�ֽ� ԭ0XEB
#define		HEAD_INTERNAL_2		0x55		//�ڲ���ͷ2�ֽ� ԭ0X90
#define		HEAD_INTERNAL_3		0xAA		//�ڲ���ͷ1�ֽ� ԭ0XEB
#define		HEAD_INTERNAL_4		0xAA		//�ڲ���ͷ2�ֽ� ԭ0X90

#if OUTINTERFACE == SANYUAN
	#define		HEAD_BYTE_1		0x7E		//��ͷ1�ֽ� ԭ0XEB
	#define		HEAD_BYTE_2		0x7E		//��ͷ2�ֽ� ԭ0X90
	#define		DATA_LEN	    0x29		//���յ��ֽ�����
#elif OUTINTERFACE == _119//�˲���Ϊ��һ���Խӱ����������˻����ͺ����
	#define		HEAD_BYTE_1		0xEB		//��ͷ1�ֽ� ԭ0XEB
	#define		HEAD_BYTE_2		0x90		//��ͷ2�ֽ� ԭ0X90
	#define		DATA_LEN		0x14		//���յ��ֽ�����
#elif OUTINTERFACE == _8511
//�˲���Ϊ8511�Խ�60����Э��涨
	#define		HEAD_BYTE_1		0xDD		//��ͷ1�ֽ� ԭ0XEB
	#define		HEAD_BYTE_2		0x77		//��ͷ2�ֽ� ԭ0X90
	#define		DATA_LEN		0x0D		//���յ��ֽ�����

//������8511�ṩЭ��Խӱ������������˻�ģ��
//	#define		HEAD_BYTE_1		0xEB		//��ͷ1�ֽ� ԭ0XEB
//	#define		HEAD_BYTE_2		0x90		//��ͷ2�ֽ� ԭ0X90
//	#define		DATA_LEN			0x0D		//���յ��ֽ�����

#elif OUTINTERFACE == _70debug
	#define		HEAD_BYTE_1		0xDD		//��ͷ1�ֽ� ԭ0XEB
	#define		HEAD_BYTE_2		0x77		//��ͷ2�ֽ� ԭ0X90
	#define		DATA_LEN		0x0D		//���յ��ֽ�����
#else
	#define		HEAD_BYTE_1		0xEB		//��ͷ1�ֽ� ԭ0XEB
	#define		HEAD_BYTE_2		0x90		//��ͷ2�ֽ� ԭ0X90
	#define		DATA_LEN		0x14		//���յ��ֽ�����
	
#endif

#define OFFSETS_BASE  0x03E8 	//����FLASH �����ַ
#define Param0_OFFSET 0x0000
#define Param1_OFFSET 0x00DC
#define Param2_OFFSET 0x01B8

#define OFFSETS_ADDR_0  0x0000 	//����FLASH �����ַ
#define OFFSETS_ADDR_1  0x03E8 	//����FLASH �����ַ
#define OFFSETS_ADDR_2  0x07D0 	//����FLASH �����ַ
#define OFFSETS_ADDR_3  0x0BB8 	//����FLASH �����ַ
#define OFFSETS_ADDR_4  0x0FA0 	//����FLASH �����ַ
#define OFFSETS_ADDR_5  0x1388 	//����FLASH �����ַ
#define OFFSETS_ADDR_6  0x1770 	//����FLASH �����ַ
#define OFFSETS_ADDR_7  0x1B58 	//����FLASH �����ַ

typedef struct {
    uint8_t buffer[FIFO_BUFFER_SIZE];    //����������
    uint32_t head;                       //������ͷָ��
    uint32_t tail;                       //������βָ��
} RingBuffer;

extern char uart1_rx_buffer0[RX1_BUFF_SIZE];
extern char uart1_rx_buffer1[RX1_BUFF_SIZE];
extern char uart2_rx_buffer[RX2_BUFF_SIZE];
extern char uart3_rx_buffer[RX3_BUFF_SIZE];
extern unsigned char uart6_rx_buffer[RX6_BUFF_SIZE];
extern unsigned char uart6_tx_buffer[RX6_BUFF_SIZE];

extern char uart1_tx_buffer[TX1_BUFF_SIZE];
extern char uart2_tx_buffer[TX2_BUFF_SIZE];
extern char uart2_temp_buffer[TX2_BUFF_SIZE];
extern char uart2_jiexi_buffer[100];
extern char uart4_tx_buffer[TX4_BUFF_SIZE];
extern char parameters[RX2_BUFF_SIZE];
extern char ALL_parameters[10000];//����дflash;
extern char ALL_parameters_yanzheng[10000];
extern char uart3_tx_buffer[TX3_BUFF_SIZE];
extern u8 uart1_rec_flag;
extern u8 uart2_rec_flag;
extern u8 uart6_rec_flag;
extern FIL file;					//�ļ�������1

extern u32 cnt_t;
extern u32 bw;

extern char u1_tx_idle;
extern char u2_tx_idle;
extern char u3_tx_idle;
extern char u4_tx_idle;
extern char u6_tx_idle;
extern u16  uart2To6Len;
extern u16  uart6To2Len;

extern RingBuffer usart2_rx_fifo;
extern RingBuffer usart2_rx_fifo1;
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void uart4_init(u32 bound);
void uart6_init(u32 bound);

void uart3_send_u2_tx_buff(void);
void uart4_send_u2_tx_buff(void);
void DMA1_Stream6_Send_Set(u16 ndtr);
void MY_TX_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar0, u16 ndtr);

uint32_t ring_buffer_write(RingBuffer *rb, const uint8_t *data, uint32_t count);
uint32_t ring_buffer_read(RingBuffer *rb, uint8_t *data, uint32_t count);
void logPrintU4(int len);

#endif

//#define MARK_TYPE 2
//#define MARK_TARGET_BIND 3
//#define MARK_JAM_TYPE 4
//#define MARK_FIRST_TARGET_LEN 5
//#define MARK_SECOND_FIRST_DELAY 8
//#define MARK_THIRD_SECOND_DELAY 11
//#define MARK_FOURTH_THIRD_DELAY 14
//#define MARK_SENSITIVITY 17				//1B
//#define MARK_DOPPLER_MOD_0 18			//3B
//#define MARK_DOPPLER_MOD_1 22			//3B
//#define MARK_DOPPLER_MOD_2 26			//3B
//#define MARK_DOPPLER_MOD_3 30			//3B
//#define MARK_RECIEVE_WEAKEN 34 		//1B
//#define MARK_UNREAL_TARGET_NUM 35
//#define MARK_PWM 36
//#define MARK_NOISE_FM 38
//#define MARK_NOISE_AM 41 				//1B
//#define MARK_TRANSMIT_TIME 42 	//2B
//#define MARK_STOP_TIME 44 			//2B
//#define MARK_SEND_WEAKEN 46 		//1B
//#define MARK_PRIORITY 47
//#define MARK_SAME_UNREAL_TARGET_NUM 48
