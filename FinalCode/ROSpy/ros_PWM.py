#!/usr/bin/env python3

import rospy
import serial
import time
from std_msgs.msg import Int64MultiArray

# SER1 for PWM Sending 
SER1 = serial.Serial('/dev/ttyUSB2', baudrate = 115200, timeout = 1) # PWM SENT
time.sleep(1)
PWMdata = [0,0,0,0,0,0,0,0]
MOTOR_ADR=[31,32,33,34,35,36,37,38]
MOTOR_DIR=[0,0,0,0,0,0,0,0]
MOTOR_PWM=[0,0,0,0,200,200,200,200]

def callback(data):
    global PWMdata
    PWMdata = data.data

        

def SentPWMtoARD1():
    DATA="O"
    for i in range(0,8):
        ADD=str(MOTOR_ADR[i])
        DIR=str(MOTOR_DIR[i])
        DATA=DATA+ADD+DIR
        PWM=[int(MOTOR_PWM[i]/100)%10,int(MOTOR_PWM[i]/10)%10,int(MOTOR_PWM[i]/1)%10]
        for j in range(0,3):
            DATA=DATA+str(PWM[j])
    DATA=DATA+"C"
    SER1.write(bytes(DATA,'utf-8'))
    return DATA  

def listener():
    rospy.init_node('get_PWM', anonymous=True)
    rospy.Subscriber("Sent_Encoder", Int64MultiArray, callback) 

if __name__ == "__main__":
    while not rospy.is_shutdown():
        listener()
        print(PWMdata)

        for i in range (0,8):
            if PWMdata[i]>=0:
                MOTOR_DIR[i]=0
            else:
                MOTOR_DIR[i]=1

            MOTOR_PWM[i]=abs(PWMdata[i])
        # print(MOTOR_DIR)
        # print(MOTOR_PWM)
        PWM=SentPWMtoARD1()
        # print(PWM)

        time.sleep(0.0001)