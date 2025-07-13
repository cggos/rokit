# coding: UTF-8

import serial
import time


class Ctrl4WDSerial(object):
    def __init__(self, UPLOAD_DATA=0, MOTOR_TYPE=4):
        """
            UPLOAD_DATA = 0  # 0:不接受数据 1:接收总的编码器数据 2:接收实时的编码器 3:接收电机当前速度 mm/s
            # 0: Do not receive data 1: Receive total encoder data 2: Receive real-time encoder 3: Receive current motor speed mm/s

            MOTOR_TYPE = 4  # 1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
            # 1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor

        :param UPLOAD_DATA:
        :param MOTOR_TYPE:
        """
        self.MOTOR_TYPE = MOTOR_TYPE

        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        self.send_upload_command(UPLOAD_DATA)
        time.sleep(0.1)
        self.set_motor_parameter(MOTOR_TYPE)

        self.recv_buffer = ""

    # 发送数据  Sending Data
    def send_data(self, data):
        self.ser.write(data.encode())  # 将字符串转换为字节后发送    Convert the string to bytes before sending
        time.sleep(0.01)  # 延时确保数据发送完成    Delay to ensure data transmission is completed

    # 接收数据  Receiving Data
    def receive_data(self):
        self.recv_buffer = ""  # ????
        if self.ser.in_waiting > 0:  # 检查串口缓冲区是否有数据  Check if there is data in the serial port buffer
            self.recv_buffer += self.ser.read(self.ser.in_waiting).decode()  # 读取并解码数据  Read and decode data

            # 按结束符 "#" 分割消息 Split the message by the ending character "#"
            messages = self.recv_buffer.split("#")
            self.recv_buffer = messages[-1]

            if len(messages) > 1:
                return messages[0] + "#"  # 返回一条完整的消息   Return a complete message
        return None

    # 配置电机类型  Configure motor type
    def set_motor_type(self, data):
        TYPE = data
        self.send_data("$mtype:{}#".format(TYPE))

    # 配置死区  Configuring Dead Zone
    def set_motor_deadzone(self, data):
        DZ = data
        self.send_data("$deadzone:{}#".format(DZ))

    # 配置磁环线    Configuring magnetic loop
    def set_pluse_line(self, data):
        LINE = data
        self.send_data("$mline:{}#".format(LINE))

    # 配置减速比    Configure the reduction ratio
    def set_pluse_phase(self, data):
        PHASE = data
        self.send_data("$mphase:{}#".format(PHASE))

    # 配置轮子直径  Configuration Diameter
    def set_wheel_dis(self, data):
        WHEEL = data
        self.send_data("$wdiameter:{}#".format(WHEEL))

    # 控制速度  Controlling Speed
    def control_speed(self, m1, m2, m3, m4):
        self.send_data("$spd:{},{},{},{}#".format(m1, m2, m3, m4))

    # 控制PWM(适用于无编码器的电机) Control PWM (for motors without encoder)
    def control_pwm(self, m1, m2, m3, m4):
        self.send_data("$pwm:{},{},{},{}#".format(m1, m2, m3, m4))

    # 解析接收到的数据  Parsing received data
    def parse_data(self, data):
        data = data.strip()  # 去掉两端的空格或换行符   Remove spaces or line breaks at both ends

        if data.startswith("$MAll:"):
            values_str = data[6:-1]  # 去除 "$MAll:" 和 "#" Remove "$MAll:" and "#"
            values = list(map(int, values_str.split(',')))  # 分割并转换为整数  Split and convert to integer
            parsed = ', '.join([f"M{i + 1}:{value}" for i, value in enumerate(values)])
            return parsed
        elif data.startswith("$MTEP:"):
            values_str = data[6:-1]
            values = list(map(int, values_str.split(',')))
            parsed = ', '.join([f"M{i + 1}:{value}" for i, value in enumerate(values)])
            return parsed
        elif data.startswith("$MSPD:"):
            values_str = data[6:-1]
            values = [float(value) if '.' in value else int(value) for value in values_str.split(',')]
            parsed = ', '.join([f"M{i + 1}:{value}" for i, value in enumerate(values)])
            return parsed

        return None

    # 需要接收数据的开关	Switch that needs to receive data
    def send_upload_command(self, mode):
        if mode == 0:
            self.send_data("$upload:0,0,0#")
        elif mode == 1:
            self.send_data("$upload:1,0,0#")
        elif mode == 2:
            self.send_data("$upload:0,1,0#")
        elif mode == 3:
            self.send_data("$upload:0,0,1#")

    ##以下的参数根据自己的实际使用电机配置即可，只要配置一次即可，电机驱动板有断电保存功能
    ##The following parameters can be configured according to the actual motor you use. You only need to configure it once. The motor driver board has a power-off saving function.
    def set_motor_parameter(self, MOTOR_TYPE):
        if MOTOR_TYPE == 1:
            self.set_motor_type(1)  # 配置电机类型   Configure motor type
            time.sleep(0.1)
            self.set_pluse_phase(
                30)  # 配置减速比，查电机手册得出   Configure the reduction ratio and check the motor manual for the result.
            time.sleep(0.1)
            self.set_pluse_line(
                11)  # 配置磁环线，查电机手册得出    Configure the magnetic ring wire and check the motor manual to get the result
            time.sleep(0.1)
            self.set_wheel_dis(67.00)  # 配置轮子直径，测量得出  Configure the wheel diameter and measure it
            time.sleep(0.1)
            self.set_motor_deadzone(1600)  # 配置电机死区，实验得出  Configure the motor dead zone, and the experiment shows
            time.sleep(0.1)

        elif MOTOR_TYPE == 2:
            self.set_motor_type(2)
            time.sleep(0.1)
            self.set_pluse_phase(20)
            time.sleep(0.1)
            self.set_pluse_line(13)
            time.sleep(0.1)
            self.set_wheel_dis(48.00)
            time.sleep(0.1)
            self.set_motor_deadzone(1300)
            time.sleep(0.1)

        elif MOTOR_TYPE == 3:
            self.set_motor_type(3)
            time.sleep(0.1)
            self.set_pluse_phase(45)
            time.sleep(0.1)
            self.set_pluse_line(13)
            time.sleep(0.1)
            self.set_wheel_dis(68.00)
            time.sleep(0.1)
            self.set_motor_deadzone(1250)
            time.sleep(0.1)

        elif MOTOR_TYPE == 4:
            self.set_motor_type(4)
            time.sleep(0.1)
            self.set_pluse_phase(48)
            time.sleep(0.1)
            self.set_motor_deadzone(1000)
            time.sleep(0.1)

        elif MOTOR_TYPE == 5:
            self.set_motor_type(1)
            time.sleep(0.1)
            self.set_pluse_phase(40)
            time.sleep(0.1)
            self.set_pluse_line(11)
            time.sleep(0.1)
            self.set_wheel_dis(67.00)
            time.sleep(0.1)
            self.set_motor_deadzone(1600)
            time.sleep(0.1)

    def move_forward(self, value=500, is_forward=True):
        if not is_forward:
            value = -value

        if self.MOTOR_TYPE == 4:
            self.control_pwm(value, value, value, value)
        else:
            self.control_speed(value, value, value, value)

    def move_backward(self, value=500):
        self.move_forward(value, False)

    # Stop the motor
    def stop(self):
        self.control_pwm(0, 0, 0, 0)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        self.ser.close()


if __name__ == "__main__":
    MOTOR_TYPE = 4

    ctrl_4wd = Ctrl4WDSerial(0, MOTOR_TYPE)

    try:
        t = 0
        print("please wait...")

        while True:
            received_message = ctrl_4wd.receive_data()
            if received_message:
                parsed = ctrl_4wd.parse_data(received_message)
                if parsed:
                    print(parsed)

            t += 10
            M1 = t
            M2 = t
            M3 = t
            M4 = t

            if MOTOR_TYPE == 4:
                ctrl_4wd.control_pwm(M1 * 2, M2 * 2, M3 * 2, M4 * 2)
            else:
                ctrl_4wd.control_speed(M1, M2, M3, M4)

            if t > 1000 or t < -1000:
                t = 0

            time.sleep(0.1)
    except KeyboardInterrupt:
        ctrl_4wd.stop()
    finally:
        ctrl_4wd.ser.close()
