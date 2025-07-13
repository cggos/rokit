#include "bsp_motor_usart.hpp"

#define  mySerial Serial
SoftwareSerial printSerial(2, 3);//2:RX 3:TX

//电机串口初始化	serial port initialization
void Usart_init (void)
{
  mySerial.begin(115200);
  printSerial.begin(115200);
  printSerial.println("please wait...");
}


/************************************************
  函数名称 ： Send_Motor_U8		Function name: Send_Motor_U8
  功    能 ： UART发送一个字符	Function: UART sends a character
  参    数 ： Data --- 数据		Parameter: Data --- data
  返 回 值 ： 无					Return value: None
*************************************************/
void Send_Motor_U8(uint8_t Data)
{
  mySerial.write(Data);
}

/************************************************
  函数名称 ： Send_Motor_ArrayU8	Function name: Send_Motor_ArrayU8
  功    能 ： 串口发送N个字符		Function: Serial port sends N characters
  参    数 ： pData ---- 字符串	Parameter: pData ---- string
            Length --- 长度		Length --- length
  返 回 值 ： 无					Return value: None
*************************************************/
void Send_Motor_ArrayU8(uint8_t *pData, uint16_t Length)
{
  while (Length--)
  {
    Send_Motor_U8(*pData);
    pData++;
  }
}

void Motor_USART_Recieve(void)
{
  char Rx_Temp = 0;

  if (mySerial.available())
  {
    Rx_Temp = char(mySerial.read());
    Deal_Control_Rxtemp(Rx_Temp);
  }
}


