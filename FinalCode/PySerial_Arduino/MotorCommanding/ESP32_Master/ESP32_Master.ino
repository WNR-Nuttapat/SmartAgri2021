//Using SPI Communication From ESP32(Master) to STM32(Slave)

#include <SPI.h>

//define VSPI
#define VSPI_MOSI 23// 23,13
#define VSPI_MISO 19// 19,12
#define VSPI_CLK  18// 18,14

volatile byte SS_PIN[] = {0, 5, 4};
volatile byte SS_TTPIN = 3;
volatile byte SS_PINPOS = 0;

char SerialData[51];
int PWMData[8] = {100, 100, 100, 100, 100, 100, 100, 100};
int PWMDataMapped[8];

void setup() {
  Serial.begin(115200);

  pinMode(VSPI_MOSI, OUTPUT);
  pinMode(VSPI_MISO, INPUT);
  pinMode(VSPI_CLK, OUTPUT);

  for (byte k = 1; k < SS_TTPIN; k++) {
    pinMode(SS_PIN[k], OUTPUT);
    digitalWrite(SS_PIN[k], HIGH);
  }

  SPI.beginTransaction(SPISettings (1000000, MSBFIRST, SPI_MODE0));
  SPI.begin();

}

void loop() {

  if (Serial.available() > 0) {
    int recieved = Serial.read();
    SerialData[50] = recieved;
    for (int i = 0; i < 51; i++) {
      SerialData[i] = SerialData[i + 1];
    }
    if (SerialData[0] == 'O' && SerialData[49] == 'C') {
      Serial.print('C');
      //Serial.println("C");  //Serial.write('\0');
      for (int j = 0; j < 8; j++) {
        int sym = 0;
        if (SerialData[(6 * j) + 3] == '1')
          sym = -1;
        else if (SerialData[(6 * j) + 3] == '0')
          sym = 1;
        int val = ((SerialData[(6 * j) + 4] - '0') * 100) +
                  ((SerialData[(6 * j) + 5] - '0') * 10) +
                  ((SerialData[(6 * j) + 6] - '0') * 1);

        PWMData[j] = sym * val;
        Serial.print(' '); Serial.print(PWMData[j]);
      }
      Serial.println("")
      DataSent_AllData(1, 1);
      DataSent_AllData(2, 1);
    }
    else {

    }
  }
  else {

  }
  //  DataSent_AllData(1, 1);
  //  DataSent_AllData(2, 1);
}

void DataSent_RotateMotor(int FLM, int FRM, int RLM, int RRM) {
  
  byte RotateMotor_Data[12];
  RotateMotor_Data[0] = B00000001;           //FLM Address
  RotateMotor_Data[1] = (FLM >> 8);
  RotateMotor_Data[2] = (FLM & B11111111);
  RotateMotor_Data[3] = B00000010;           //FRM Address
  RotateMotor_Data[4] = (FRM >> 8);
  RotateMotor_Data[5] = (FRM & B11111111);
  RotateMotor_Data[6] = B00000011;           //RLM Address
  RotateMotor_Data[7] = (RLM >> 8);
  RotateMotor_Data[8] = (RLM & B11111111);
  RotateMotor_Data[9] = B00000100;           //RRM Address
  RotateMotor_Data[10] = (RRM >> 8);
  RotateMotor_Data[11] = (RRM & B11111111);

  digitalWrite(SS_PIN[SS_PINPOS], LOW);
  for (int i = 0; i < 12; i++) {
    SPI.transfer(RotateMotor_Data[i]);
    delayMicroseconds(50);
    //Serial.print(RotateMotor_Data[i], BIN); Serial.print(' ');
  }
  //Serial.println();
  digitalWrite(SS_PIN[SS_PINPOS], HIGH);
  delayMicroseconds(50);

}

void DataSent_DriveMotor(int FLM, int FRM, int RLM, int RRM) {
  byte DriveMotor_Data[12];
  DriveMotor_Data[0] = B00000101;           //FLM Address
  DriveMotor_Data[1] = (FLM >> 8);
  DriveMotor_Data[2] = (FLM & B11111111);
  DriveMotor_Data[3] = B00000110;           //FRM Address
  DriveMotor_Data[4] = (FRM >> 8);
  DriveMotor_Data[5] = (FRM & B11111111);
  DriveMotor_Data[6] = B00000111;           //RLM Address
  DriveMotor_Data[7] = (RLM >> 8);
  DriveMotor_Data[8] = (RLM & B11111111);
  DriveMotor_Data[9] = B00001000;           //RRM Address
  DriveMotor_Data[10] = (RRM >> 8);
  DriveMotor_Data[11] = (RRM & B11111111);

  digitalWrite(SS_PIN[SS_PINPOS], LOW);
  for (int i = 0; i < 12; i++) {
    SPI.transfer(DriveMotor_Data[i]);
    delayMicroseconds(50);
    //Serial.print(DriveMotor_Data[i], BIN); Serial.print(' ');
  }
  //Serial.println();
  digitalWrite(SS_PIN[SS_PINPOS], HIGH);
  delayMicroseconds(50);

}

void DataSent_AllData(byte slaveno, int delaytime) {
  SS_PINPOS = slaveno;
  for (int k = 0; k < 8; k++) {
    PWMDataMapped[k] = PWMData[k] + 1500;
  }
  DataSent_RotateMotor(PWMDataMapped[0], PWMDataMapped[1],PWMDataMapped[2], PWMDataMapped[3]);
  DataSent_DriveMotor(PWMDataMapped[4], PWMDataMapped[5],PWMDataMapped[6], PWMDataMapped[7]);
  SS_PINPOS = 0;
  delay(delaytime);
}
