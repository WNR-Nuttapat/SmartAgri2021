import serial
import time

SER4 = serial.Serial('COM7', baudrate = 1000000, timeout = 1) #
SER5 = serial.Serial('COM12', baudrate = 9600, timeout = 1)

time.sleep(1)

MPU_Complete=0
GPS_Complete=0

def getMPUDATA():
    
    global MPU_Complete,GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ
    
    arduinoData = SER4.readline()[:-2]
    stringData=arduinoData.decode('ascii')
    stringData=stringData.split(',')    ##Split into 2 string

    MPU_Complete=0

    if (len(stringData)==2 and 
        stringData[0][0]=='G' and stringData[1][0]=='A'):
        
        GyroData=(stringData[0].split('/'))[1:]
        AccelData=(stringData[1].split('/'))[1:]

        if(len(GyroData)==3 and len(AccelData)==3):

            GyroX=GyroData[0]
            GyroY=GyroData[1]
            GyroZ=GyroData[2]

            AccelX=AccelData[0]
            AccelY=AccelData[1]
            AccelZ=AccelData[2]
            
            MPU_Complete=1

        return GyroData,AccelData
    
    else :
        
        MPU_Complete=0
        print("MPU Error")

        return 0

def getGPSDATA():
    
    global GPS_Complete,GPS_time,GPS_NS,GPS_EW,GPS_latitude,GPS_longitude

    arduinoData = (SER5.readline().decode('ascii')).split(",")
    if (arduinoData[0]=='$GPGGA' and len(arduinoData)==15):
        datagps=arduinoData
        count=0
        for i in range (0,15):
            count+=len(datagps[i])
        if(count>19):
            GPS_time = datagps[1] 
            GPS_NS = datagps[3]
            GPS_EW = datagps[5]
            GPS_latitude = float(datagps[2])/100
            GPS_longitude = float(datagps[4])/100

            GPS_Complete=1

            return GPS_time,GPS_NS,GPS_EW,GPS_latitude,GPS_longitude
        else:
            print("GPS FAIL")
            GPS_Complete=0
            return 0

while(1):

    x=getMPUDATA()
    if MPU_Complete==1:
        print(x)  

        MPU_Complete=0
    
    y=getGPSDATA()
    if GPS_Complete==1:
        print(y)

        GPS_Complete=0

    time.sleep(0.001)
