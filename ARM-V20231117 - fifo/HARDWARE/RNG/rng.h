#ifndef __RNG_H
#define __RNG_H	 
#include "sys.h" 
 //////////////////////////////////////////////////////////////////////////////////	 


	
u8  RNG_Init(void);			//RNG初始化 
u32 RNG_Get_RandomNum(void);//得到随机数
u32 RNG_Get_RandomRange(u32 min,u32 max);//生成[min,max]范围的随机数
void Get_RandomArray(u32 *buff,u8 cnt,u32 min,u32 max);
#endif

















