#include "bsp.h"

void bsp_init(void)
{
	
	DIY_NVIC_PriorityGroupConfig(2);	  //设置中断分组   //Set interrupt grouping
	delay_init();	    	            //延时函数初始化	 //Delay function initialization
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口    //Close JTAG interface
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试 //Opening the SWD interface allows for debugging using the motherboard's SWD interface
	uart_init(115200);

}


void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //开启辅助时钟	  Activate auxiliary clock  
	AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24] Clear MAPR [26:24]
	AFIO->MAPR|=temp;       //设置jtag模式 Set jtag mode
} 


/**************************************************************************
Function: Set NVIC group
Input   : NVIC_Group
Output  : none
函数功能：设置中断分组
入口参数：NVIC_Group:NVIC分组 0~4 总共5组 	
返回  值：无
**************************************************************************/ 
void DIY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
	u32 temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//取后三位 Take the last three
	temp1<<=8;
	temp=SCB->AIRCR;  //读取先前的设置  Read previous settings
	temp&=0X0000F8FF; //清空先前分组   Clear previous groups
	temp|=0X05FA0000; //写入钥匙  Write the key
	temp|=temp1;	   
	SCB->AIRCR=temp;  //设置分组	  Set grouping   	  				   
}

