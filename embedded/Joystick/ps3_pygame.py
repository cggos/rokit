import pygame, sys
from time import sleep

from YahBoom.DriveBoard4WD.RPi5.USART.Ctrl4WDSerial import Ctrl4WDSerial


class JoystickCtrl:
    def __init__(self, move_forward, move_backward, move_right=None, move_left=None):
        # setup the pygame window
        pygame.init()
        # window = pygame.display.set_mode((200, 200), 0, 32)

        # how many joysticks connected to computer?
        joystick_count = pygame.joystick.get_count()
        print("There is " + str(joystick_count) + " joystick/s")

        if joystick_count == 0:
            # if no joysticks, quit program safely
            print("Error, I did not find any joysticks")
            pygame.quit()
            sys.exit()
        else:
            # initialise joystick
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

        print('Initialized Joystick : %s' % self.joystick.get_name())

        self.axes = self.joystick.get_numaxes()
        self.buttons = self.joystick.get_numbuttons()
        self.hats = self.joystick.get_numhats()

        print("There is " + str(self.axes) + " axes")
        print("There is " + str(self.buttons) + " button/s")
        print("There is " + str(self.hats) + " hat/s")

        self.move_forward = move_forward
        self.move_backward = move_backward

    def getAxis(self, number):
        # when nothing is moved on an axis, the VALUE IS NOT EXACTLY ZERO
        # so this is used not "if joystick value not zero"
        if self.joystick.get_axis(number) < -0.1 or self.joystick.get_axis(number) > 0.1:
            # value between 1.0 and -1.0
            print("Axis ID is %s, Axis value is %s" % (number, self.joystick.get_axis(number)))

    def getButton(self, number):
        # returns 1 or 0 – pressed or not
        if self.joystick.get_button(number):
            # just prints id of button
            print("Button ID is %s" % (number))

    def getHat(self, number):
        if self.joystick.get_hat(number) != (0, 0):
            # returns tuple with values either 1, 0 or -1
            print("Hat ID is %s, Hat value is %s, %s" % (number, self.joystick.get_hat(number)[0],
                                                         self.joystick.get_hat(number)[1]))

    def process(self):
        while True:
            for event in pygame.event.get():
                # Check if one of the joysticks has moved
                if event.type == pygame.JOYAXISMOTION:
                    print(f"JOYAXISMOTION: Axis ID is {event.axis}, Axis value is {event.value}")
                    if event.axis == 4:
                        if -1.0 < event.value < 0.0:
                            if self.move_forward is not None:
                                self.move_forward()
                        if 0.0 < event.value < 1.0:
                            if self.move_backward is not None:
                                self.move_backward()

                # loop through events, if window shut down, quit program
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            if self.axes != 0:
                for i in range(self.axes):
                    self.getAxis(i)
            if self.buttons != 0:
                for i in range(self.buttons):
                    self.getButton(i)
            if self.hats != 0:
                for i in range(self.hats):
                    self.getHat(i)
            sleep(0.5)


if __name__ == '__main__':
    ctrl_4wd = Ctrl4WDSerial()

    js_ctrl = JoystickCtrl(ctrl_4wd.move_forward, ctrl_4wd.move_backward)
    js_ctrl.process()
