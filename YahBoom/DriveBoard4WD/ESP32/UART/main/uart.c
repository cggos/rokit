#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "uart_module.h"
#include "app_motor_uart.h"

#define delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))// 毫秒延时宏 | Millisecond delay macro

#define UPLOAD_DATA 3  //0:不接受数据 1:接收总的编码器数据 2:接收实时的编码器 3:接收电机当前速度 mm/s
					   //0: Do not receive data 1: Receive total encoder data 2: Receive real-time encoder 3: Receive current motor speed mm/s

#define MOTOR_TYPE 2   //1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
                       //1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor

// 电机控制任务 | Motor Control Task
void MotorControl_Task(void *arg) {
    static int i = 0;
    while(1) {
		if(g_recv_flag == 1)// 接收标志检查 | Check reception flag
		{
			g_recv_flag = 0;// 重置标志 | Reset flag

			// 根据电机类型选择控制方式 | Select control mode by motor type
			#if MOTOR_TYPE == 4
			Contrl_Pwm(i*20,i*20,i*20,i*20);// PWM控制模式 | PWM control
			#else
			Contrl_Speed(i*10,i*10,i*10,i*10);// 速度控制模式 | Speed control
			#endif
			
			Deal_data_real();
			
			#if UPLOAD_DATA == 1
				printf("M1:%d,M2:%d,M3:%d,M4:%d\r\n",Encoder_Now[0],Encoder_Now[1],Encoder_Now[2],Encoder_Now[3]);
			#elif UPLOAD_DATA == 2
				printf("M1:%d,M2:%d,M3:%d,M4:%d\r\n",Encoder_Offset[0],Encoder_Offset[1],Encoder_Offset[2],Encoder_Offset[3]);
			#elif UPLOAD_DATA == 3
				printf("M1:%.2f,M2:%.2f,M3:%.2f,M4:%.2f\r\n",g_Speed[0],g_Speed[1],g_Speed[2],g_Speed[3]);
			#endif
			
			i = (i < 100) ? i+1 : 0;
			delay_ms(100);
		}
		delay_ms(1);// 防止任务卡死 | Preventing tasks from getting stuck
    }
}
	

void app_main(void)
{
	//初始化UART1通信 | Initialize UART1 communication
	uart1_init();
	printf("pelase wait...\r\n");
	
	//先关闭上报	Close the report first
	send_upload_data(false,false,false);
    delay_ms(10);
	
    #if MOTOR_TYPE == 1
	send_motor_type(1);//配置电机类型	Configure motor type
	delay_ms(100);
	send_pulse_phase(30);//配置减速比 查电机手册得出	Configure the reduction ratio. Check the motor manual to find out
	delay_ms(100);
	send_pulse_line(11);//配置磁环线 查电机手册得出	Configure the magnetic ring wire. Check the motor manual to get the result.
	delay_ms(100);
	send_wheel_diameter(67.00);//配置轮子直径,测量得出		Configure the wheel diameter and measure it
	delay_ms(100);
	send_motor_deadzone(1600);//配置电机死区,实验得出	Configure the motor dead zone, and the experiment shows
	delay_ms(100);
    
    #elif MOTOR_TYPE == 2
    send_motor_type(2);
	delay_ms(100);
	send_pulse_phase(20);
	delay_ms(100);
	send_pulse_line(13);
	delay_ms(100);
	send_wheel_diameter(48.00);
	delay_ms(100);
	send_motor_deadzone(1300);
	delay_ms(100);
    
    #elif MOTOR_TYPE == 3
    send_motor_type(3);
	delay_ms(100);
	send_pulse_phase(45);
	delay_ms(100);
	send_pulse_line(13);
	delay_ms(100);
	send_wheel_diameter(68.00);
	delay_ms(100);
	send_motor_deadzone(1250);
	delay_ms(100);
    
    #elif MOTOR_TYPE == 4
    send_motor_type(4);
	delay_ms(100);
	send_pulse_phase(48);
	delay_ms(100);
	send_motor_deadzone(1000);
	delay_ms(100);
    
    #elif MOTOR_TYPE == 5
    send_motor_type(1);
	delay_ms(100);
	send_pulse_phase(40);
	delay_ms(100);
	send_pulse_line(11);
	delay_ms(100);
	send_wheel_diameter(67.00);
	delay_ms(100);
	send_motor_deadzone(1600);
	delay_ms(100);
    #endif
	
	//给电机模块发送需要上报的数据	Send the data that needs to be reported to the motor module
	#if UPLOAD_DATA == 1
	send_upload_data(true,false,false);delay_ms(10);
	#elif UPLOAD_DATA == 2
	send_upload_data(false,true,false);delay_ms(10);
	#elif UPLOAD_DATA == 3
	send_upload_data(false,false,true);delay_ms(10);
	#endif
	
    xTaskCreate(
        UART_Process_Task, 		// 任务函数	Task Function
        "UART_Process",         // 任务名称	Task Name
        4096,                   // 堆栈大小（字节）	Stack size (bytes)
        NULL, 
        2,                      // 优先级（数值越大优先级越高）	Priority (the larger the value, the higher the priority)
        NULL
    );
	xTaskCreate(MotorControl_Task, "MotorCtrl", 4096, NULL, 1, NULL);

}
