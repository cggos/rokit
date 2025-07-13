#ifndef __MYENUM_H_
#define __MYENUM_H_


////JTAG模式设置定义		JTAG mode setting definition
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00


//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//For specific implementation ideas, please refer to Chapter 5 (pages 87-92) of <<CM3 Authoritative Guide>>.

//IO口操作宏定义	IO port operation macro definition
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射	IO port address mapping
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!	IO port operation is only for a single IO port!
//确保n的值小于16!	Make sure the value of n is less than 16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 Output
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 Input

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 Output
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 Input

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 Output
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 Input

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 Output
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 Input

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 Output
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入 Input

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 Output
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入 Input

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 Output
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入 Input

#endif

