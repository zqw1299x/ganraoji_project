#ifndef __RNG_H
#define __RNG_H	 
#include "sys.h" 
 //////////////////////////////////////////////////////////////////////////////////	 


	
u8  RNG_Init(void);			//RNG��ʼ�� 
u32 RNG_Get_RandomNum(void);//�õ������
u32 RNG_Get_RandomRange(u32 min,u32 max);//����[min,max]��Χ�������
void Get_RandomArray(u32 *buff,u8 cnt,u32 min,u32 max);
#endif

















