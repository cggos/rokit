#include "AllHeader.h"

#define UPLOAD_DATA 2  //1:接收总的编码器数据 2:接收实时的编码器
					   //1: Receive total encoder data 2: Receive real-time encoder

#define MOTOR_TYPE 1   //1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
                       //1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor

int main(void)
{	
	int i;
	bsp_init();
	
	TIM3_Init();
	
	//电机模块iic通信初始化	Motor module iic communication initialization
	IIC_Motor_Init();
	
	printf("pelase wait...\r\n");
	
    #if MOTOR_TYPE == 1
	Set_motor_type(1);//配置电机类型	Configure motor type
	delay_ms(100);
	Set_Pluse_Phase(30);//配置减速比 查电机手册得出	Configure the reduction ratio. Check the motor manual to find out
	delay_ms(100);
	Set_Pluse_line(11);//配置磁环线 查电机手册得出	Configure the magnetic ring wire. Check the motor manual to get the result.
	delay_ms(100);
	Set_Wheel_dis(67.00);//配置轮子直径,测量得出		Configure the wheel diameter and measure it
	delay_ms(100);
	Set_motor_deadzone(1600);//配置电机死区,实验得出	Configure the motor dead zone, and the experiment shows
	delay_ms(100);
    
    #elif MOTOR_TYPE == 2
    Set_motor_type(2);
	delay_ms(100);
	Set_Pluse_Phase(20);
	delay_ms(100);
	Set_Pluse_line(13);
	delay_ms(100);
	Set_Wheel_dis(48.00);
	delay_ms(100);
	Set_motor_deadzone(1300);
	delay_ms(100);
    
    #elif MOTOR_TYPE == 3
    Set_motor_type(3);
	delay_ms(100);
	Set_Pluse_Phase(45);
	delay_ms(100);
	Set_Pluse_line(13);
	delay_ms(100);
	Set_Wheel_dis(68.00);
	delay_ms(100);
	Set_motor_deadzone(1250);
	delay_ms(100);
    
    #elif MOTOR_TYPE == 4
    Set_motor_type(4);
	delay_ms(100);
	Set_Pluse_Phase(48);
	delay_ms(100);
	Set_motor_deadzone(1000);
	delay_ms(100);
    
    #elif MOTOR_TYPE == 5
    Set_motor_type(1);
	delay_ms(100);
	Set_Pluse_Phase(40);
	delay_ms(100);
	Set_Pluse_line(11);
	delay_ms(100);
	Set_Wheel_dis(67.00);
	delay_ms(100);
	Set_motor_deadzone(1600);
	delay_ms(100);
    #endif

	while(1)
	{
        
		for(i=0;i<100;i++)
		{
            #if MOTOR_TYPE == 4
            control_pwm(i*20,i*20,i*20,i*20);
            #else
			control_speed(i*10,i*10,i*10,i*10);
            #endif
            delay_ms(100);
			
			#if UPLOAD_DATA == 1
			Read_ALL_Enconder();
			printf("M1:%d\t M2:%d\t M3:%d\t M4:%d\t \r\n",Encoder_Now[0],Encoder_Now[1],Encoder_Now[2],Encoder_Now[3]);
			#elif UPLOAD_DATA == 2
			Read_10_Enconder();
			printf("M1:%d\t M2:%d\t M3:%d\t M4:%d\t \r\n",Encoder_Offset[0],Encoder_Offset[1],Encoder_Offset[2],Encoder_Offset[3]);
			#endif
			
			if(i>=100)i=0;
		}

	}
	
}

