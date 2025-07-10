# pip install Jetson.GPIO

import Jetson.GPIO as GPIO
import time


class JetsonGPIO:
    def __init__(self):
        # Test pin 40: I2S_SDOUT
        # Test pin 32: GPIO9
        self.output_pin = 31

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.output_pin, GPIO.OUT, initial=GPIO.LOW)

    def __del__(self):
        GPIO.output(self.output_pin, GPIO.LOW)
        GPIO.cleanup()

    def set_hight(self):
        GPIO.output(self.output_pin, GPIO.HIGH)

    def set_low(self):
        GPIO.output(self.output_pin, GPIO.LOW)

    def blink(self):
        count = 0
        while True:
            GPIO.output(self.output_pin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(self.output_pin, GPIO.LOW)
            time.sleep(0.5)
            count += 1
            if count > 5:
                break
        GPIO.output(self.output_pin, GPIO.HIGH)


if __name__ == "__main__":
    jetio = JetsonGPIO()
    jetio.set_hight()
    time.sleep(5)
    jetio.blink()
