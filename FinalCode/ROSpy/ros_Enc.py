#!/usr/bin/env python3

import rospy
import serial
import time
import numpy as np
import math
from std_msgs.msg import Int64MultiArray

# SER2 and SER3 for Recieving ENC
SER2 = serial.Serial('/dev/ttyUSB1', baudrate = 230400, timeout = 0) # ENC 4 RotateMotor
SER3 = serial.Serial('/dev/ttyUSB0', baudrate = 230400, timeout = 0) # ENC 4 DriveMotor
time.sleep(1)

flushtime=time.time()
pidtime=time.time()
pidinterval=3 #pid every x second

PWM_DATA=[0,0,0,0,0,0,0,0]

ROTATE_TARGET=[180,180,180,180]
MOTOR_DEG=[0,0,0,0]

TPR=[30,30,30,30]
MOTOR_TICK=[0,0,0,0]
prevMOTOR_TICK=[0,0,0,0]
MOTOR_RPM=[0,0,0,0]
RotateENC_Complete=1
DriveENC_Complete=0


def RecievedENCfromARD2():
    global RotateENC_Complete,MOTOR_DEG
    ARD2Data = (SER2.readline()[:-2]).decode('ascii') #ENC from DriveMotor
    RotateENC=ARD2Data.split(',')
    if(len(RotateENC)==5 and RotateENC[0]=='ROT'):
        for i in range (1,5):
            if len(RotateENC[i])>0:
                if RotateENC[i].isnumeric()==True:
                    MOTOR_DEG[i-1]=int(RotateENC[i])
                elif (RotateENC[i][0]=='-' and RotateENC[i][1:].isnumeric()==True):
                    MOTOR_DEG[i-1]=int(RotateENC[i][1:])*(-1)
                else:
                    pass
            else:
                pass    
        RotateENC_Complete=1
        return MOTOR_DEG
    else:
        pass

def RecievedENCfromARD3():
    global DriveENC_Complete,MOTOR_TICK
    ARD3Data = (SER3.readline()[:-2]).decode('ascii') #ENC from DriveMotor
    DriveENC=ARD3Data.split(',')
    if(len(DriveENC)==5 and DriveENC[0]=='DRV'):
        for i in range (1,5):
            if len(DriveENC[i])>0:
                if DriveENC[i].isnumeric()==True:
                    MOTOR_TICK[i-1]=int(DriveENC[i])
                elif (DriveENC[i][0]=='-' and DriveENC[i][1:].isnumeric()==True):
                    MOTOR_TICK[i-1]=int(DriveENC[i][1:])*(-1)
                else:
                    pass
            else:
                pass    
        DriveENC_Complete=1
        return MOTOR_TICK
    else:
        pass

def PositionControl(target,curr,k):
    
    if curr<=(target+2) and curr>=(target-2) :
        PWM_DATA[k]=0
    elif (target-2)>curr:
        PWM_DATA[k]=40
    elif (target+2)<curr:
        PWM_DATA[k]=-40



spd_prev_error=[0,0,0,0]
spd_eintegral=[0,0,0,0]
def PID_SpeedControl(target,curr,dt,k):
    
    kp=[1,1,1,1]
    ki=[0,0,0,0]
    kd=[0,0,0,0]

    error=target-curr

    spd_eintegral[k]=spd_eintegral[k]+(error*dt)
    ediff=(error-spd_prev_error[k])/dt
    spd_prev_error[k]=error

    u=(kp[k]*error)+(ki[k]*spd_eintegral[k])+(kd[k]*ediff)

    pwm=int(u)
    if pwm>255:
        pwm=255
    elif pwm<-255:
        pwm=-255
    
    return pwm

if __name__ == "__main__":
    
    pub = rospy.Publisher('Sent_Encoder', Int64MultiArray, queue_size=10)
    rospy.init_node('encoder', anonymous=True) 

    while not rospy.is_shutdown():
        RecievedENCfromARD2()
        RecievedENCfromARD3()

        if(RotateENC_Complete==1 and DriveENC_Complete==1):
            
            print(MOTOR_DEG)
            for k in range(0,4):
                PositionControl(ROTATE_TARGET[k],MOTOR_DEG[k],k)
            SER2.flush()
            deltatime=time.time()-pidtime
            if(deltatime>=pidinterval):
                dt=deltatime/60
                # print(MOTOR_TICK)
                for k in range(0,4):
                    
                    MOTOR_RPM[k]=(MOTOR_TICK[k]-prevMOTOR_TICK[k])/TPR[k]/dt
                    prevMOTOR_TICK[k]=MOTOR_TICK[k]

                    # PWM_DATA[k]=PID_PositionControl(0,MOTOR_DEG[k],dt,k)
                    # PWM_DATA[k+4]=PID_SpeedControl(100,MOTOR_RPM[k],dt,k)
                # print(MOTOR_RPM)
                # print(PWM_DATA)
                pidtime=time.time()                

            if(time.time()-flushtime>=10):
                SER3.flush()
                flushtime==time.time()
            RotateENC_Complete=1
            DriveENC_Complete=0
            
            sent_Enc = Int64MultiArray()
            sent_Enc.data = PWM_DATA
            pub.publish(sent_Enc)