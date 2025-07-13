#ifndef __BSP_MOTOR_USART_H_
#define __BSP_MOTOR_USART_H_

#include <Arduino.h>
#include "app_motor_usart.hpp"
#include <SoftwareSerial.h>

extern SoftwareSerial printSerial;
//#define  printSerial Serial

void Usart_init (void);
void Send_Motor_U8(uint8_t Data);
void Send_Motor_ArrayU8(uint8_t *pData, uint16_t Length);
void Motor_USART_Recieve(void);



#endif

