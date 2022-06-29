#include <SPI.h>

//define VSPI
#define VSPI_MOSI 23// 23,13
#define VSPI_MISO 19// 19,12
#define VSPI_CLK  18// 18,14
//#define SS1 5
//#define SS2 4
int n;
volatile byte SS_PIN[] = {0, 5, 4};
volatile byte SS_TTPIN = 3;
volatile byte SS_PINPOS = 0;

char SerialData[51];
//int PWMData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int PWMData[8] = { -250, -150, -50, 0, 50, 100, 200};
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
    //    lcd.setCursor(0, 0);  lcd.print("   ");
    //    lcd.setCursor(0, 0);  lcd.print(Serial.available());
    int recieved = Serial.read();
    //    lcd.setCursor(3, 0);  lcd.print("   ");
    //    lcd.setCursor(3, 0);  lcd.print(recieved);

    SerialData[50] = recieved;
    for (int i = 0; i < 51; i++) {
      SerialData[i] = SerialData[i + 1];
    }
    if (SerialData[0] == 'O' && SerialData[49] == 'C') {
      Serial.print('C'); Serial.println(n);
      //Serial.println("C");  //Serial.write('\0');
      //      lcd.setCursor(0, 1);  lcd.print("CP#");
      //      lcd.setCursor(3, 1);  lcd.print(n);
      n++;
      //byte clmn[] = {0, 4, 8, 12, 0, 4, 8, 12};
      //byte rw[] = {0, 0, 0, 0, 1, 1, 1, 1};
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
        //Serial.println(PWMData[j]);
        //delay(200);
        //lcd.setCursor(clmn[j], rw[j]);  lcd.print("    ");
        //lcd.setCursor(clmn[j], rw[j]);  lcd.print(sym * val);
      }
      DataSent_AllData(1, 1);
      DataSent_AllData(2, 1);
    }
    else {
      //Serial.println("F");  //Serial.write('\0');
    }
  }
  else {
    //Serial.println("N");  //Serial.write('\0');
    //DataSent_AllData(1, 5);
    //DataSent_AllData(2, 5);
  }

  //  DataSent_AllData(1, 5);
  //  DataSent_AllData(2, 5);
  //  delay(10);
  //  Serial.println(millis());
}

void DataSent_FrontRightMotor(int RotateM, int DriveM) {
  //FrontRightMotor
  byte FrontRight_Data[6];
  FrontRight_Data[0] = B00000001;               //Address
  FrontRight_Data[1] = (RotateM >> 8);          //RotateMotor
  FrontRight_Data[2] = (RotateM & B11111111);   //DriveMotor
  FrontRight_Data[3] = B00000010;               //Address
  FrontRight_Data[4] = (DriveM >> 8);           //RotateMotor
  FrontRight_Data[5] = (DriveM & B11111111);    //DriveMotor

  digitalWrite(SS_PIN[SS_PINPOS], LOW);
  for (int i = 0; i < 6; i++) {
    SPI.transfer(FrontRight_Data[i]);
    delayMicroseconds(10);
    //Serial.print(FrontRight_Data[i], BIN); Serial.print(' ');
  }
  //Serial.println();
  digitalWrite(SS_PIN[SS_PINPOS], HIGH);
  delayMicroseconds(40);

}

void DataSent_FrontLeftMotor(int RotateM, int DriveM) {
  //FrontLeftMotor
  byte FrontLeft_Data[6];
  FrontLeft_Data[0] = B00000011;               //Address
  FrontLeft_Data[1] = (RotateM >> 8);          //RotateMotor
  FrontLeft_Data[2] = (RotateM & B11111111);   //DriveMotor
  FrontLeft_Data[3] = B00000100;               //Address
  FrontLeft_Data[4] = (DriveM >> 8);           //RotateMotor
  FrontLeft_Data[5] = (DriveM & B11111111);    //DriveMotor

  digitalWrite(SS_PIN[SS_PINPOS], LOW);
  for (int i = 0; i < 6; i++) {
    SPI.transfer(FrontLeft_Data[i]);
    delayMicroseconds(10);
    //Serial.print(FrontLeft_Data[i], BIN); Serial.print(' ');
  }
  //Serial.println();
  digitalWrite(SS_PIN[SS_PINPOS], HIGH);
  delayMicroseconds(40);

}

void DataSent_RearRightMotor(int RotateM, int DriveM) {
  //RearRightMotor
  byte RearRight_Data[6];
  RearRight_Data[0] = B00000101;               //Address
  RearRight_Data[1] = (RotateM >> 8);          //RotateMotor
  RearRight_Data[2] = (RotateM & B11111111);   //DriveMotor
  RearRight_Data[3] = B00000110;               //Address
  RearRight_Data[4] = (DriveM >> 8);           //RotateMotor
  RearRight_Data[5] = (DriveM & B11111111);    //DriveMotor

  digitalWrite(SS_PIN[SS_PINPOS], LOW);
  for (int i = 0; i < 6; i++) {
    SPI.transfer(RearRight_Data[i]);
    delayMicroseconds(10);
    //Serial.print(RearRight_Data[i], BIN); Serial.print(' ');
  }
  //Serial.println();
  digitalWrite(SS_PIN[SS_PINPOS], HIGH);
  delayMicroseconds(40);

}

void DataSent_RearLeftMotor(int RotateM, int DriveM) {
  //RearLeftMotor
  byte RearLeft_Data[6];
  RearLeft_Data[0] = B00000111;               //Address
  RearLeft_Data[1] = (RotateM >> 8);          //RotateMotor
  RearLeft_Data[2] = (RotateM & B11111111);   //DriveMotor
  RearLeft_Data[3] = B00001000;               //Address
  RearLeft_Data[4] = (DriveM >> 8);           //RotateMotor
  RearLeft_Data[5] = (DriveM & B11111111);    //DriveMotor

  digitalWrite(SS_PIN[SS_PINPOS], LOW);
  for (int i = 0; i < 6; i++) {
    SPI.transfer(RearLeft_Data[i]);
    delayMicroseconds(10);
    //Serial.print(RearLeft_Data[i], BIN); Serial.print(' ');
  }
  //Serial.println();
  digitalWrite(SS_PIN[SS_PINPOS], HIGH);
  delayMicroseconds(40);

}

void DataSent_AllData(byte slaveno, int delaytime) {
  SS_PINPOS = slaveno;
  for (int k = 0; k < 8; k++) {
    PWMDataMapped[k] = PWMData[k]+1500;
  }
  DataSent_FrontRightMotor(PWMDataMapped[0], PWMDataMapped[1]);
  DataSent_FrontLeftMotor(PWMDataMapped[2], PWMDataMapped[3]);
  DataSent_RearRightMotor(PWMDataMapped[4], PWMDataMapped[5]);
  DataSent_RearLeftMotor(PWMDataMapped[6], PWMDataMapped[7]);
  SS_PINPOS = 0;
  delay(delaytime);
}
