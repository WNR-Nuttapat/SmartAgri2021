#!/usr/bin/env python3
from math import pi
from matplotlib.pyplot import vlines
import serial
import time
import rospy
from std_msgs.msg import UInt32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

wheelradius=3.5
wheelspan=29.5
Right_TPR=970
Left_TPR=980
prev_right_tick=0
prev_left_tick=0
prev_time = 0

def getValues():

    arduinoData = ser.readline()[:-2]#.decode('ascii')
    return arduinoData


if __name__ == "__main__":
    
    ser = serial.Serial('/dev/ttyUSB0', baudrate = 115200, timeout = 1)
    rospy.init_node('read_ser', anonymous=True)
    pub = rospy.Publisher('serial_read', UInt32, queue_size=10)
    #odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    pub_vel = rospy.Publisher('cmd_vel',Twist,queue_size=10)

    time.sleep(1)

    curr_time=rospy.Time.now()
    prev_time=rospy.Time.now()
    
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        ## DataSending ##
        ## Motor Commanding -> index0=RM , index1=LM
        MOTOR_ADR=[31,32]           ##Fix address 31->RM , 32->LM
        MOTOR_DIR=[0,0]             ##Motor Direction 0->Foward , 1->Backward
        MOTOR_PWM=[255,255]         ##Motor PWM (0-255)

        DATA="O"  ##First Bit of DATA
        for i in range(0,2):
                    
            ADD=str(MOTOR_ADR[i])
            DIR=str(MOTOR_DIR[i])
            DATA=DATA+ADD+DIR
            PWM=[int(MOTOR_PWM[i]/100)%10,int(MOTOR_PWM[i]/10)%10,int(MOTOR_PWM[i]/1)%10]
            for j in range(0,3):
                DATA=DATA+str(PWM[j])
        DATA=DATA+"C"   ##Last Bit of DATA

        # print(DATA)
        ser.write(bytes(DATA,'utf-8')) #Sent DATA to Arduino

        
        curr_time=rospy.Time.now()
        
        ## DataRecieving ##    
        Response=getValues()        ##Get DATA from Arduino
        stringData=Response.decode('ascii')
        stringData=stringData.split(',')    ##Split into 2 string
        # print(stringData)
        
        data=[]  
        if len(stringData)==2:
            Right_tick=int(stringData[0][1:])
            Left_tick=int(stringData[1][1:])
    
            delta_Right_tick=Right_tick-prev_right_tick
            delta_Left_tick=Left_tick-prev_left_tick
            delta_time=(curr_time-prev_time).to_sec()
            #print(type(delta_time))

            dr=2*pi*wheelradius*(delta_Right_tick/Right_TPR)
            dl=2*pi*wheelradius*(delta_Left_tick/Left_TPR)

            vr=dr/delta_time
            vl=dl/delta_time

            vc=(vr+vl)/2
            omega=wheelradius*(vr-vl)/wheelspan
            
            data.append(vc)
            print(vc)   
            data.append(omega)
            print(omega)  
            prev_right_tick=Right_tick
            prev_left_tick=Left_tick
            prev_time=curr_time        

        if data:
            rospy.loginfo(data)
            pub.publish(data)
            #pub_vel.publish(data)

        prev_time=curr_time
        time.sleep(0.00001)
