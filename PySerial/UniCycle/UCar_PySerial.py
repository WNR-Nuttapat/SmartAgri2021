import serial
import time

n=0

ser = serial.Serial('COM7', baudrate = 115200, timeout = 1)
time.sleep(1)

def getValues():
    
    arduinoData = ser.readline()[:-2]#.decode('ascii')
    return arduinoData


while(1):

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

    ## DataRecieving ##    
    Response=getValues()        ##Get DATA from Arduino
    stringData=Response.decode('ascii')
    stringData=stringData.split(',')    ##Split into 2 string
    # print(stringData)
    
    data=[] #Store the Encoder Tick -> Index0-RM , Index1-LM 
    for i in range(0,len(stringData)):
        data.append(int(stringData[i][1:])) ##Convert String to In
        # print(data[i])
    
    time.sleep(0.00001)