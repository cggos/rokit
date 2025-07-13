import smbus
import struct
import time

UPLOAD_DATA = 1  #1:接收总的编码器数据 2:接收实时的编码器
                 #1: Receive total encoder data 2: Receive real-time encoder

MOTOR_TYPE = 1  #1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
                #1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor

# 创建I2C通信对象   Create I2C communication object
bus = smbus.SMBus(1)  # 1代表I2C总线号，这里可能要根据自己驱动板所在的I2C总线来修改    
                       #1 represents the I2C bus number. You may need to modify it according to the I2C bus where your driver board is located.

# I2C地址   I2C Address
MOTOR_MODEL_ADDR = 0x26

# I2C寄存器定义 I2C Register Definition
MOTOR_TYPE_REG = 0x01
MOTOR_DEADZONE_REG = 0x02
MOTOR_PLUSELINE_REG = 0x03
MOTOR_PLUSEPHASE_REG = 0x04
WHEEL_DIA_REG = 0x05
SPEED_CONTROL_REG = 0x06
PWM_CONTROL_REG = 0x07

READ_TEN_M1_ENCODER_REG = 0x10
READ_TEN_M2_ENCODER_REG = 0x11
READ_TEN_M3_ENCODER_REG = 0x12
READ_TEN_M4_ENCODER_REG = 0x13

READ_ALLHIGH_M1_REG = 0x20
READ_ALLLOW_M1_REG = 0x21
READ_ALLHIGH_M2_REG = 0x22
READ_ALLLOW_M2_REG = 0x23
READ_ALLHIGH_M3_REG = 0x24
READ_ALLLOW_M3_REG = 0x25
READ_ALLHIGH_M4_REG = 0x26
READ_ALLLOW_M4_REG = 0x27

# 全局变量  Global variables
encoder_offset = [0] * 4
encoder_now = [0] * 4

# 写数据到I2C寄存器 Write data to I2C register
def i2c_write(addr, reg, data):
    bus.write_i2c_block_data(addr, reg, data)

# 从I2C寄存器读取数据   Reading data from I2C registers
def i2c_read(addr, reg, length):
    return bus.read_i2c_block_data(addr, reg, length)

# 浮点数转字节  Floating point number to byte
def float_to_bytes(f):
    return struct.pack('<f', f) # 小端字节序 Little-endian

# 配置电机类型  Configure motor type
def set_motor_type(data):
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_TYPE_REG, [data])

# 配置死区  Configuring Dead Zone
def set_motor_deadzone(data):
    buf = [(data >> 8) & 0xFF, data & 0xFF]
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_DEADZONE_REG, buf)

# 配置磁环线    Configuring magnetic loop
def set_pluse_line(data):
    buf = [(data >> 8) & 0xFF, data & 0xFF]
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_PLUSELINE_REG, buf)

# 配置减速比    Configure the reduction ratio
def set_pluse_phase(data):
    buf = [(data >> 8) & 0xFF, data & 0xFF]
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_PLUSEPHASE_REG, buf)

# 配置轮子直径  Configuration Diameter
def set_wheel_dis(data):
    bytes_data = float_to_bytes(data)
    i2c_write(MOTOR_MODEL_ADDR, WHEEL_DIA_REG, list(bytes_data))

# 控制速度  Controlling Speed
def control_speed(m1, m2, m3, m4):
    speeds = [
        (m1 >> 8) & 0xFF, m1 & 0xFF,
        (m2 >> 8) & 0xFF, m2 & 0xFF,
        (m3 >> 8) & 0xFF, m3 & 0xFF,
        (m4 >> 8) & 0xFF, m4 & 0xFF
    ]
    i2c_write(MOTOR_MODEL_ADDR, SPEED_CONTROL_REG, speeds)

# 控制PWM(适用于无编码器的电机) Control PWM (for motors without encoder)
def control_pwm(m1, m2, m3, m4):
    pwms = [
        (m1 >> 8) & 0xFF, m1 & 0xFF,
        (m2 >> 8) & 0xFF, m2 & 0xFF,
        (m3 >> 8) & 0xFF, m3 & 0xFF,
        (m4 >> 8) & 0xFF, m4 & 0xFF
    ]
    i2c_write(MOTOR_MODEL_ADDR, PWM_CONTROL_REG, pwms)

# 读取编码器数据    Read encoder data
def read_10_encoder():
    global encoder_offset
    formatted_values = []
    for i in range(4):
        reg = READ_TEN_M1_ENCODER_REG + i
        buf = i2c_read(MOTOR_MODEL_ADDR, reg, 2)
        encoder_offset[i] = (buf[0] << 8) | buf[1]
        if encoder_offset[i] & 0x8000:  # 检查最高位（符号位）是否为 1  Check if the highest bit (sign bit) is 1
            encoder_offset[i] -= 0x10000 # 将其转为负数 Turn it into a negative number
        formatted_values.append("M{}:{}".format(i + 1, encoder_offset[i]))
    return ", ".join(formatted_values)

def read_all_encoder():
    global encoder_now
    formatted_values = []
    for i in range(4):
        high_reg = READ_ALLHIGH_M1_REG + (i * 2)
        low_reg = READ_ALLLOW_M1_REG + (i * 2)
        high_buf = i2c_read(MOTOR_MODEL_ADDR, high_reg, 2)
        low_buf = i2c_read(MOTOR_MODEL_ADDR, low_reg, 2)
        
        high_val = high_buf[0] <<8 | high_buf[1]
        low_val = low_buf[0] <<8 | low_buf[1]
        
        encoder_val = (high_val << 16) | low_val
        
        # 处理符号扩展，假设 32 位有符号整数    Handles sign extension, assuming 32-bit signed integers
        if encoder_val >= 0x80000000:  # 如果大于 2^31，说明应该是负数  If it is greater than 2^31, it should be a negative number
            encoder_val -= 0x100000000  # 将其转为负数  Turn it into a negative number
        encoder_now[i] = encoder_val
        formatted_values.append("M{}:{}".format(i + 1, encoder_now[i]))
    return ", ".join(formatted_values) 

##以下的参数根据自己的实际使用电机配置即可，只要配置一次即可，电机驱动板有断电保存功能
##The following parameters can be configured according to the actual motor you use. You only need to configure it once. The motor driver board has a power-off saving function.
def set_motor_parameter():

    if MOTOR_TYPE == 1:
        set_motor_type(1)  # 配置电机类型   Configure motor type
        time.sleep(0.1)
        set_pluse_phase(30)  # 配置减速比，查电机手册得出   Configure the reduction ratio and check the motor manual for the result.
        time.sleep(0.1)
        set_pluse_line(11)  # 配置磁环线，查电机手册得出    Configure the magnetic ring wire and check the motor manual to get the result
        time.sleep(0.1)
        set_wheel_dis(67.00)  # 配置轮子直径，测量得出  Configure the wheel diameter and measure it
        time.sleep(0.1)
        set_motor_deadzone(1600)  # 配置电机死区，实验得出  Configure the motor dead zone, and the experiment shows
        time.sleep(0.1)

    elif MOTOR_TYPE == 2:
        set_motor_type(2)
        time.sleep(0.1)
        set_pluse_phase(20)
        time.sleep(0.1)
        set_pluse_line(13)
        time.sleep(0.1)
        set_wheel_dis(48.00)
        time.sleep(0.1)
        set_motor_deadzone(1200)
        time.sleep(0.1)

    elif MOTOR_TYPE == 3:
        set_motor_type(3)
        time.sleep(0.1)
        set_pluse_phase(45)
        time.sleep(0.1)
        set_pluse_line(13)
        time.sleep(0.1)
        set_wheel_dis(68.00)
        time.sleep(0.1)
        set_motor_deadzone(1250)
        time.sleep(0.1)

    elif MOTOR_TYPE == 4:
        set_motor_type(4)
        time.sleep(0.1)
        set_pluse_phase(48)
        time.sleep(0.1)
        set_motor_deadzone(1000)
        time.sleep(0.1)

    elif MOTOR_TYPE == 5:
        set_motor_type(1)
        time.sleep(0.1)
        set_pluse_phase(40)
        time.sleep(0.1)
        set_pluse_line(11)
        time.sleep(0.1)
        set_wheel_dis(67.00)
        time.sleep(0.1)
        set_motor_deadzone(1600)
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        t = 0
        print("please wait...")
        set_motor_parameter() # 设置自己的电机参数  Set your own motor parameters

        while True:
            t += 10
            M1 = t
            M2 = t
            M3 = t
            M4 = t

            if MOTOR_TYPE == 4:
                control_pwm(M1*2, M2*2, M3*2, M4*2)
            else:
                control_speed(M1, M2, M3, M4)#直接发送命令控制电机  Send commands directly to control the motor
            
            if t> 1000 or t < -1000:
                t = 0

            if UPLOAD_DATA == 1:
                now_string = read_all_encoder()  # 读取累计编码器数据   Read the accumulated encoder data
                print(now_string)
            elif UPLOAD_DATA == 2:
                offset_string = read_10_encoder()  # 读取实时编码器数据 Read real-time encoder data
                print(offset_string)          
            time.sleep(0.1)

    except KeyboardInterrupt:
            control_pwm(0, 0, 0, 0)#让电机停下来  Stop the motor
