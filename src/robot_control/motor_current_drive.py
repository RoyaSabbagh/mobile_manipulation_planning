from pyduino import *
import time
from matplotlib import pyplot as plot


class motor:

    def __init__(self):
        self.a = Arduino()
        # if your arduino was running on a serial port other than '/dev/ttyACM0/'
        # declare: a = Arduino(serial_port='/dev/ttyXXXX')

        time.sleep(3)
        # sleep to ensure sample time for computer to make serial connection

        self.PWM_pin = 11
        self.Enable_pin = 22
        self.Direction_pin = 24

        self.a.set_pin_mode(self.PWM_pin, 'O')
        self.a.set_pin_mode(self.Direction_pin, 'O')
        self.a.set_pin_mode(self.Enable_pin, 'O')

        self.Enable = 0
        self.a.digital_write(Enable_pin, self.Enable)
        time.sleep(10)

    def on(self):
        self.Direction = 0
        self.Enable = 1
        self.Current = 40

        self.a.digital_write(Direction_pin, self.Direction)
        self.a.digital_write(Enable_pin, self.Enable)
        self.a.analog_write(PWM_pin, self.Current)
        time.sleep(10)

    def off(self):
        self.Enable = 0
        self.a.digital_write(Enable_pin, self.Enable)
        time.sleep(10)

    def close(self):
        self.Current = 50
        self.a.analogWrite(PWM_pin, self.Current)
        time.sleep(1000)

        self.Current = 100
        self.a.analogWrite(PWM_pin, self.Current)
        time.sleep(1000)

    def stay_put(self, current):
        self.a.analogWrite(PWM_pin, current)
        time.sleep(1000)


    def open(self):
        self.Current = 20
        self.a.analogWrite(PWM_pin, self.Current)
        time.sleep(1000)

        self.Current = 50
        self.a.analogWrite(PWM_pin, self.Current)
        time.sleep(1000)


