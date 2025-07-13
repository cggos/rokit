from machine import UART, Pin
import time

UPLOAD_DATA = 3  #0:不接受数据 1:接收总的编码器数据 2:接收实时的编码器 3:接收电机当前速度 mm/s
                 #0: Do not receive data 1: Receive total encoder data 2: Receive real-time encoder 3: Receive current motor speed mm/s

MOTOR_TYPE = 1  #1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
                #1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor

# 串口初始化    Serial port initialization
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))  # 使用 UART0，TX=GP0，RX=GP1  Use UART0, TX=GP0, RX=GP1

# 接收缓存  Receive Buffer
recv_buffer = ""

# 发送数据  Sending Data
def send_data(data):
    uart.write(data.encode())  # 将字符串转换为字节后发送    Convert the string to bytes before sending
    time.sleep(0.01)  # 延时确保数据发送完成    Delay to ensure data transmission is completed

# 接收数据  Receiving Data
def receive_data():
    global recv_buffer
    if uart.any() > 0:  # 检查串口缓冲区是否有数据  Check if there is data in the serial port buffer
        recv_buffer += uart.read(uart.any()).decode()  # 读取并解码数据  Read and decode data
        
        # 按结束符 "#" 分割消息 Split the message by the ending character "#"
        messages = recv_buffer.split("#")
        recv_buffer = messages[-1]
        
        if len(messages) > 1:
            return messages[0] + "#"  #返回一条完整的消息   Return a complete message
    return None

# 配置电机类型  Configure motor type
def set_motor_type(data):
    TYPE = data
    send_data("$mtype:{}#".format(TYPE))

# 配置死区  Configuring Dead Zone
def set_motor_deadzone(data):
    DZ = data
    send_data("$deadzone:{}#".format(DZ))

# 配置磁环线    Configuring magnetic loop
def set_pluse_line(data):
    LINE = data
    send_data("$mline:{}#".format(LINE))

# 配置减速比    Configure the reduction ratio
def set_pluse_phase(data):
    PHASE = data
    send_data("$mphase:{}#".format(PHASE))

# 配置轮子直径  Configuration Diameter
def set_wheel_dis(data):
    WHEEL = data
    send_data("$wdiameter:{}#".format(WHEEL))

# 控制速度  Controlling Speed
def control_speed(m1, m2, m3, m4):
    send_data("$spd:{},{},{},{}#".format(m1, m2, m3, m4))

# 控制PWM(适用于无编码器的电机) Control PWM (for motors without encoder)
def control_pwm(m1, m2, m3, m4):
    send_data("$pwm:{},{},{},{}#".format(m1, m2, m3, m4))

# 解析接收到的数据  Parsing received data
def parse_data(data):
    data = data.strip()  # 去掉两端的空格或换行符   Remove spaces or line breaks at both ends

    if data.startswith("$MAll:"):
        values_str = data[6:-1]  # 去除 "$MAll:" 和 "#" Remove "$MAll:" and "#"
        values = list(map(int, values_str.split(',')))  # 分割并转换为整数  Split and convert to integer
        parsed = ', '.join([f"M{i+1}:{value}" for i, value in enumerate(values)])
        return parsed
    elif data.startswith("$MTEP:"):
        values_str = data[6:-1]
        values = list(map(int, values_str.split(',')))
        parsed = ', '.join([f"M{i+1}:{value}" for i, value in enumerate(values)])
        return parsed
    elif data.startswith("$MSPD:"):
        values_str = data[6:-1]
        values = [float(value) if '.' in value else int(value) for value in values_str.split(',')]
        parsed = ', '.join([f"M{i+1}:{value}" for i, value in enumerate(values)])
        return parsed

#需要接收数据的开关	Switch that needs to receive data
def send_upload_command(mode):
    if mode == 0:
        send_data("$upload:0,0,0#")
    elif mode == 1:
        send_data("$upload:1,0,0#")
    elif mode == 2:
        send_data("$upload:0,1,0#")
    elif mode == 3:
        send_data("$upload:0,0,1#")

##以下的参数根据自己的实际使用电机配置即可，只要配置一次即可，电机驱动板有断电保存功能
##The following parameters can be configured according to the actual motor you use. You only need to configure it once. The motor driver board has a power-off saving function.
def set_motor_parameter():

    if MOTOR_TYPE == 1:
        set_motor_type(1)  # 配置电机类型
        time.sleep(0.1)
        set_pluse_phase(30)  # 配置减速比，查电机手册得出
        time.sleep(0.1)
        set_pluse_line(11)  # 配置磁环线，查电机手册得出
        time.sleep(0.1)
        set_wheel_dis(67.00)  # 配置轮子直径，测量得出
        time.sleep(0.1)
        set_motor_deadzone(1600)  # 配置电机死区，实验得出
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
        set_motor_deadzone(1300)
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
        send_upload_command(UPLOAD_DATA)#给电机模块发送需要上报的数据	Send the data that needs to be reported to the motor module
        time.sleep(0.1)
        set_motor_parameter()#设计电机参数  Design motor parameters

        while True:
            received_message = receive_data()  # 接收消息    Receiving Messages
            if received_message:    # 如果有数据返回 If there is data returned
                parsed = parse_data(received_message) # 解析数据 Parsing the data
                if parsed:
                    print(parsed)  # 打印解析后的数据   Print the parsed data

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

            time.sleep(0.1)

    except KeyboardInterrupt:
        control_pwm(0, 0, 0, 0)#让电机停下来  Stop the motor
    finally:
        uart.deinit()  # 关闭串口 Close the serial port