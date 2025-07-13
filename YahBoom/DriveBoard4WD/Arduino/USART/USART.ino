#include <Arduino.h>
#include "app_motor_usart.hpp"
#include "bsp_motor_usart.hpp"

#define UPLOAD_DATA 1  //0:不接受数据 1:接收总的编码器数据 2:接收实时的编码器 3:接收电机当前速度 mm/s
                       //0: Do not receive data 1: Receive total encoder data 2: Receive real-time encoder 3: Receive current motor speed mm/s

#define MOTOR_TYPE 2   //1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
                       //1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor

int i = 0;
char buffer[50];
char string1[10],string2[10],string3[10],string4[10];

void setup(){
  
  Usart_init();
    //先关闭上报 Close the report first
  send_upload_data(false,false,false);
  
  #if MOTOR_TYPE == 1
	send_motor_type(1);//配置电机类型	Configure motor type
	delay(100);
	send_pulse_phase(30);//配置减速比 查电机手册得出	Configure the reduction ratio. Check the motor manual to find out
	delay(100);
	send_pulse_line(11);//配置磁环线 查电机手册得出	Configure the magnetic ring wire. Check the motor manual to get the result.
	delay(100);
	send_wheel_diameter(67.00);//配置轮子直径,测量得出		Configure the wheel diameter and measure it
	delay(100);
	send_motor_deadzone(1600);//配置电机死区,实验得出	Configure the motor dead zone, and the experiment shows
	delay(100);
    
  #elif MOTOR_TYPE == 2
  send_motor_type(2);
	delay(100);
	send_pulse_phase(20);
	delay(100);
	send_pulse_line(13);
	delay(100);
	send_wheel_diameter(48.00);
	delay(100);
	send_motor_deadzone(1300);
	delay(100);
    
  #elif MOTOR_TYPE == 3
  send_motor_type(3);
	delay(100);
	send_pulse_phase(45);
	delay(100);
	send_pulse_line(13);
	delay(100);
	send_wheel_diameter(68.00);
	delay(100);
	send_motor_deadzone(1250);
	delay(100);
    
  #elif MOTOR_TYPE == 4
  send_motor_type(4);
	delay(100);
	send_pulse_phase(48);
	delay(100);
	send_motor_deadzone(1000);
	delay(100);
    
  #elif MOTOR_TYPE == 5
  send_motor_type(1);
	delay(100);
	send_pulse_phase(40);
	delay(100);
	send_pulse_line(11);
	delay(100);
	send_wheel_diameter(67.00);
	delay(100);
	send_motor_deadzone(1600);
	delay(100);
  #endif
  
  //给电机模块发送需要上报的数据  Send the data that needs to be reported to the motor module
  #if UPLOAD_DATA == 1
  send_upload_data(true,false,false);delay(10);
  #elif UPLOAD_DATA == 2
  send_upload_data(false,true,false);delay(10);
  #elif UPLOAD_DATA == 3
  send_upload_data(false,false,true);delay(10);
  #endif

} 

void loop(){
  
      Motor_USART_Recieve();
      if(g_recv_flag == 1)
    {
      g_recv_flag = 0;
      #if MOTOR_TYPE == 4
        Contrl_Pwm(i*2,i*2,i*2,i*2);
      #else
        Contrl_Speed(i,i,i,i);   //值为-1000~1000 The value is -1000~1000
      #endif

        Deal_data_real();
        //delay(100);
      #if UPLOAD_DATA == 1
        sprintf(buffer,"M1:%d,M2:%d,M3:%d,M4:%d\r\n",Encoder_Now[0],Encoder_Now[1],Encoder_Now[2],Encoder_Now[3]);
        printSerial.println(buffer);
      #elif UPLOAD_DATA == 2
        sprintf(buffer,"M1:%d,M2:%d,M3:%d,M4:%d\r\n",Encoder_Offset[0],Encoder_Offset[1],Encoder_Offset[2],Encoder_Offset[3]);
        printSerial.println(buffer);
      #elif UPLOAD_DATA == 3
        dtostrf(g_Speed[0], 4, 2, string1);
        dtostrf(g_Speed[1], 4, 2, string2);
        dtostrf(g_Speed[2], 4, 2, string3);
        dtostrf(g_Speed[3], 4, 2, string4);
        sprintf(buffer,"M1:%s,M2:%s,M3:%s,M4:%s\r\n",string1,string2,string3,string4);
        printSerial.println(buffer);
      #endif

      i++;
      if (i == 1000) i = 0;
      
    }

}
