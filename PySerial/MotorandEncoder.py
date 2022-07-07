#!/usr/bin/env python3

import rospy
import serial
import time
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# SER1 for PWM Sending // SER2 and SER3 for Recieving ENC
# SER1 = serial.Serial('/dev/tty', baudrate = 115200, timeout = 1) # PWM SENT
# SER2 = serial.Serial('/dev/tty', baudrate = 250000, timeout = 1) # ENC 4 front Motor
# SER3 = serial.Serial('/dev/tty', baudrate = 250000, timeout = 1) # ENC 4 rear Motor
time.sleep(1)

vx=0
omega = 0

def callback(data):
    global vx
    global omega
    vx = data.linear.x
    omega = data.angular.z
    # print(vx)
    # print(omega)

x = [-0.2555,0.2555,-0.2555,0.2555]
y = [0.3725,0.3725,-0.3725,-0.3725]
wheelradius=0.0735

def getTheta(v,w):
    a = []
    b = []
    
    for i in range(len(x)):
        if w!=0:
            angle=math.atan(y[i]/(v/w - x[i]))
        else:
            angle=0
        a.append(angle)
        b.append(angle*180/math.pi)
    
    return b

def getVelocity(v,w):
    a = []
    b = []
    for i in range(len(x)):
        if w!=0:
            vel=w*math.sqrt(math.pow(v/w - x[i],2) + math.pow(y[i],2))
        else:
            vel=v
        a.append(vel)
        b.append(vel*60/(2*math.pi*wheelradius))

    return b

#for Drive Motor usually use Kp and Ki
# DriveMotor_Target=[0,0,0,0]
DriveMotor_Target=getVelocity(vx,omega)
print(DriveMotor_Target)
drv_kp=[1,1,1,1]
drv_ki=[0,0,0,0]
drv_kd=[0,0,0,0]

#for Rotate Motor usually use only Kp
# RotateMotor_Target=[0,90,180,270]
RotateMotor_Target=getTheta(vx,omega)
print(RotateMotor_Target)
rot_kp=[1,1,1,1]
rot_ki=[0.5,0.5,0.5,0.5]
rot_kd=[0,0,0,0]

RotateENC_Complete=0
MOTOR_DEG=[0,0,0,0]
rot_prev_time=[time.time(),time.time(),time.time(),time.time()]

DriveENC_Complete=0
MOTOR_TPR=[100,100,100,100] #Motor Tick pre Rev (For DriveMotor)
MOTOR_TICK=[0,0,0,0]
MOTOR_TICK_prev=[0,0,0,0]
drv_prev_time=[time.time(),time.time(),time.time(),time.time()]


MOTOR_ADR=[31,32,33,34,35,36,37,38]
MOTOR_DIR=[0,1,0,1,0,1,0,1]
MOTOR_PWM=[5,100,15,20,25,30,35,40]

# def SentPWMtoARD1():
#     DATA="O"
#     for i in range(0,8):
#         ADD=str(MOTOR_ADR[i])
#         DIR=str(MOTOR_DIR[i])
#         DATA=DATA+ADD+DIR
#         PWM=[int(MOTOR_PWM[i]/100)%10,int(MOTOR_PWM[i]/10)%10,int(MOTOR_PWM[i]/1)%10]
#         for j in range(0,3):
#             DATA=DATA+str(PWM[j])
#     DATA=DATA+"C"
#     # SER1.write(bytes(DATA,'utf-8'))
#     return DATA   

# def RecievedENCfromARD2():

#     global RotateENC_Complete,MOTOR_DEG

#     ARD2Data = (SER2.readline()[:-2]).decode('ascii') #ENC from DriveMotor
#     RotateENC=ARD2Data.split(',')
#     if(len(RotateENC)==4 and 
#         RotateENC[0][0]=='A' and RotateENC[1][0]=='B' 
#         and RotateENC[2][0]=='C' and RotateENC[3][0]=='D'):
#         for i in range (0,4):
#             MOTOR_DEG[i]=int(RotateENC[i][1:])
#         RotateENC_Complete=1
#         return MOTOR_DEG
#     else:
#         print("ARD2 Error")
#         RotateENC_Complete=0
#         return 0
    
# def RecievedENCfromARD3():

#     global DriveENC_Complete,MOTOR_TICK

#     ARD3Data = (SER3.readline()[:-2]).decode('ascii') #ENC from DriveMotor
#     DriveENC=ARD3Data.split(',')
#     if(len(DriveENC)==4 and 
#         DriveENC[0][0]=='E' and DriveENC[1][0]=='F' 
#         and DriveENC[2][0]=='G' and DriveENC[3][0]=='H'):
#         for i in range (0,4):
#             MOTOR_TICK[i]=int(DriveENC[i][1:])
#         DriveENC_Complete=1
#         return MOTOR_TICK
#     else:
#         print("ARD3 Error")
#         DriveENC_Complete=0
#         return 0

# pos_prev_error=[0,0,0,0]
# pos_eintegral=[0,0,0,0]

# def PID_PositionControl(target_val,curr_val,dt,k):
    
#     global rot_kp,rot_ki,rot_kd,pos_prev_error,pos_eintegral

#     error=target_val-curr_val
#     pos_eintegral[k]=pos_eintegral[k]+(error*dt)
#     ediff=(error-pos_prev_error[k])/dt

#     u=(rot_kp[k]*error)+(rot_ki[k]*pos_eintegral[k])+(rot_kd[k]*ediff)

#     if u>0:
#         dir=0
#     else:
#         dir=1
    
#     pwm=abs(u)
#     if pwm>255:
#         pwm=255

#     MOTOR_DIR[k]=dir
#     MOTOR_PWM[k]=pwm

#     pos_prev_error[k]=error

# spd_prev_error=[0,0,0,0]
# spd_eintegral=[0,0,0,0]

# def PID_SpeedControl(target_val,curr_val,dt,k):
    
#     global drv_kp,drv_ki,drv_kd,spd_prev_error,spd_eintegral

#     error=target_val-curr_val
#     spd_eintegral[k]=spd_eintegral[k]+(error*dt)
#     ediff=(error-spd_prev_error[k])/dt

#     u=(drv_kp[k]*error)+(drv_ki[k]*spd_eintegral[k])+(drv_kd[k]*ediff)

#     if u>0:
#         dir=0
#     else:
#         dir=1
    
#     pwm=abs(u)
#     if pwm>255:
#         pwm=255

#     MOTOR_DIR[k+4]=dir
#     MOTOR_PWM[k+4]=pwm

#     spd_prev_error[k]=error


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)

    # PWM=SentPWMtoARD1()
    # print(PWM)

    # DEG=RecievedENCfromARD2()
    # if(RotateENC_Complete==1):
    #     print(DEG)
    #     for i in range (0,4):
    #         curr_time=time.time()
    #         rot_deltatime=(curr_time-rot_prev_time[i])/60
    #         PID_PositionControl(RotateMotor_Target[i],MOTOR_DEG,rot_deltatime,i)
        
    #     RotateENC_Complete=0

    # TICK=RecievedENCfromARD3()
    # if(DriveENC_Complete==1):
    #     # print(TICK)
    #     for j in range (0,4):
    #         curr_time=time.time()
    #         drv_deltatime=(curr_time-drv_prev_time[j])/60
    #         delta_TICK=MOTOR_TICK[j]-MOTOR_TICK_prev[j]
    #         RPM=(delta_TICK/MOTOR_TPR[j])/drv_deltatime

    #         PID_SpeedControl(DriveMotor_Target[j],RPM,drv_deltatime,j)

    #         MOTOR_TICK_prev[0]=MOTOR_TICK[0]
        
    #     DriveENC_Complete=0

    time.sleep(0.001)
    
    # rospy.spin()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        listener()
        DriveMotor_Target=getVelocity(vx,omega)
        print(DriveMotor_Target)
        RotateMotor_Target=getTheta(vx,omega)
        # print(RotateMotor_Target)
        # print(vx)
        # print(type(vx))
        # print(omega)
        # print(type(omega))