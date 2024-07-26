import time
from Motor import *
from ADC import *
import RPi.GPIO as GPIO
import random

IR01 = 14
IR02 = 15
IR03 = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR01,GPIO.IN)
GPIO.setup(IR02,GPIO.IN)
GPIO.setup(IR03,GPIO.IN)


def stop():
     PWM.setMotorModel(0,0,0,0)  # stop
     time.sleep(1)
def turnRight():
     PWM.setMotorModel(2500, 2500, -3000, -3000)# Right
     time.sleep(0.5)
def turnLeft():
    PWM.setMotorModel(3000, 3000, -2500, -2500)  # Left
    time.sleep(0.5)
def back():
     PWM.setMotorModel(-700, -700, -700, -700) # Back
     time.sleep(0.8)
def spin():
    PWM.setMotorModel()



# def chooseDirectionWhileReversing():
#         # Example: Randomly choose left or right when reversing
#         direction = random.choice(['left', 'right'])
#         if direction == 'left':
#             turnLeft()
#             print("Left")
#         else:
#             turnRight()
#             print("Right")         


class Light:
    def run(self):
        while True:
            self.adc=Adc()
            self.LMR=0x00
            if GPIO.input(IR01)==True:
                self.LMR=(self.LMR | 4)
            if GPIO.input(IR02)==True:
                self.LMR=(self.LMR | 2)
            if GPIO.input(IR03)==True:
                self.LMR=(self.LMR | 1)

            if self.LMR==2: # forward
                print("moving forward")
                PWM.setMotorModel(0,0,0,00)
            elif self.LMR==4:
                stop()
                back()
                #PWM.setMotorModel(2500,2500,-3000,-3000) # right
                turnRight() 
                print("RIGHT")
            elif self.LMR==6:
                # stop()
                # back()
                PWM.setMotorModel(2000,2000,-4000,-4000) # sharp right
                print("sharp right")
            elif self.LMR==1:
                stop()
                #back() #added
                turnLeft()
                #PWM.setMotorModel(3000,3000,-2500,-2500) # left
                turnLeft() #- called twice in statement
                print("left")
            elif self.LMR==3:
                # stop()
                # back()
                PWM.setMotorModel(4000,4000,-2000,-2000) # sharp left
                print("sharp left")
            elif self.LMR==7: # stop
                stop() 
                back()
                #chooseDirectionWhileReversing() 
            else:
                PWM.setMotorModel(000,000,000,000) # to go forward at the beginning

            time.sleep(1)  

    def detect(self):
        self.LMR=0x00
        if GPIO.input(IR01)==True:
            self.LMR=(self.LMR | 4)
        if GPIO.input(IR02)==True:
            self.LMR=(self.LMR | 2)
        if GPIO.input(IR03)==True:
            self.LMR=(self.LMR | 1)
        print(self.LMR)
        return self.LMR



infrared=Light()
if __name__=='__main__':
    print ('Program is starting ... ')
    try:
        infrared.run()
        infrared.detect()
    except KeyboardInterrupt: # When 'Ctrl+C' is pressed, the child program destroy() will be 
    #executed.
        PWM.setMotorModel(0,0,0,0)




        
    # Light.py Code
    # 
    # def run(self):
    #     try:
    #         self.adc=Adc()
    #         self.PWM=Motor()
    #         self.PWM.setMotorModel(0,0,0,0)
    #         while True:
    #             L = self.adc.recvADC(0)
    #             R = self.adc.recvADC(1)
    #             if L < 2.99 and R < 2.99 :
    #                 self.PWM.setMotorModel(600,600,600,600)
    #             elif abs(L-R)<0.15:
    #                 self.PWM.setMotorModel(0,0,0,0)
                    
    #             elif L > 3 or R > 3:
    #                 if L > R :
    #                     self.PWM.setMotorModel(-1200,-1200,1400,1400)
                        
    #                 elif R > L :
    #                     self.PWM.setMotorModel(1400,1400,-1200,-1200)
                    
    #     except KeyboardInterrupt:
    #        led_Car.PWM.setMotorModel(0,0,0,0) 

