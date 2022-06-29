#include <SPI.h>
#define SPI1_NSS_PIN PA4
uint8_t count = 0;
uint8_t data[25];
volatile bool ButtonData8bit[4];
unsigned int NumberData16;
byte DataAccepted = 0;

int PWMData[8];

#define RM_DIR PA2
#define RM_PWM PA0
#define LM_DIR PA3
#define LM_PWM PA1

#define UDM_DIR PB10
#define UDM_PWM PB0
#define GRPM_DIR PB11
#define GRPM_PWM PB1

void RM_Drive(int RMpwm) {
  if (RMpwm >= 0)
    digitalWrite(RM_DIR, LOW);
  else
    digitalWrite(RM_DIR, HIGH);

  analogWrite(RM_PWM, abs(RMpwm));
}

void LM_Drive(int LMpwm) {
  if (LMpwm >= 0)
    digitalWrite(LM_DIR, LOW);
  else
    digitalWrite(LM_DIR, HIGH);

  analogWrite(LM_PWM, abs(LMpwm));
}

void UDM_Drive(int UDpwm) {
  if (UDpwm >= 0)
    digitalWrite(UDM_DIR, LOW);
  else
    digitalWrite(UDM_DIR, HIGH);

  analogWrite(UDM_PWM, abs(UDpwm));
}

void GRPM_Drive(int GRPpwm) {
  if (GRPpwm >= 0)
    digitalWrite(GRPM_DIR, LOW);
  else
    digitalWrite(GRPM_DIR, HIGH);

  analogWrite(GRPM_PWM, abs(GRPpwm));
}

void Spi_recieve() {
  uint8_t msg = SPI.transfer(++count);
  //Serial.println(msg,BIN);
  for (byte i = 0; i < 24; i++) {
    data[i] = data[i + 1];
  }
  data[24] = msg;
}

void DataCheck() {

  if ( data[0] == B00000001 && data[3] == B00000010 &&
       data[6] == B00000011 && data[9] == B00000100 &&
       data[12] == B00000101 && data[15] == B00000110 &&
       data[18] == B00000111 && data[21] == B00001000 ) {

    //        for(int i=0;i<22;i++){
    //          Serial1.println(data[i],BIN);
    //        }

    //Check FrontRightMotor Data
    NumberData16bitDecoder(data[1], data[2]);
    PWMData[0] = NumberData16;
    NumberData16 = 0;
    NumberData16bitDecoder(data[4], data[5]);
    PWMData[1] = NumberData16;
    NumberData16 = 0;

    //Check FrontLeftMotor Data
    NumberData16bitDecoder(data[7], data[8]);
    PWMData[2] = NumberData16;
    NumberData16 = 0;
    NumberData16bitDecoder(data[10], data[11]);
    PWMData[3] = NumberData16;
    NumberData16 = 0;

    //Check RearRightMotor Data
    NumberData16bitDecoder(data[13], data[14]);
    PWMData[4] = NumberData16;
    NumberData16 = 0;
    NumberData16bitDecoder(data[16], data[17]);
    PWMData[5] = NumberData16;
    NumberData16 = 0;

    //Check FrontLeftMotor Data
    NumberData16bitDecoder(data[19], data[20]);
    PWMData[6] = NumberData16;
    NumberData16 = 0;
    NumberData16bitDecoder(data[22], data[23]);
    PWMData[7] = NumberData16;
    NumberData16 = 0;

    DataAccepted = 1;
    //Serial.println("DataAccepted");
    for (int j = 0; j < 8; j++) {
      PWMData[j]=PWMData[j]-1500;
      Serial.println(PWMData[j]);
    }

  }
  else {
//    Serial.println("DataFailed");
    DataAccepted = 0;
  }
}

void ButtonData8bitDecoder(byte temp) {

  //  //Reset Previous Data
  //  ButtonData8bit[0] = 0;
  //  ButtonData8bit[1] = 0;
  //  ButtonData8bit[2] = 0;
  //  ButtonData8bit[3] = 0;

  //Get New Data
  ButtonData8bit[0] = (temp >> 0)&B1;
  ButtonData8bit[1] = (temp >> 2)&B1;
  ButtonData8bit[2] = (temp >> 4)&B1;
  ButtonData8bit[3] = (temp >> 6)&B1;

}

void ButtonData8bitReset() {
  //Reset Previous Data
  ButtonData8bit[0] = 0;
  ButtonData8bit[1] = 0;
  ButtonData8bit[2] = 0;
  ButtonData8bit[3] = 0;
}

void NumberData16bitDecoder(byte temp1, byte temp2) {

  //  NumberData16 = 0;
  NumberData16 = (temp1 << 8) | temp2;

}


void setup() {

  Serial.begin(115200);
  Serial.print("RDY");
  SPI.begin();
  SPI.beginTransactionSlave(SPISettings(1000000, MSBFIRST, SPI_MODE0, DATA_SIZE_8BIT));
  pinMode(SPI1_NSS_PIN, INPUT);

  pinMode(RM_PWM, PWM);
  pinMode(RM_DIR, OUTPUT);
  pinMode(LM_PWM, PWM);
  pinMode(LM_DIR, OUTPUT);

  pinMode(UDM_PWM, PWM);
  pinMode(UDM_DIR, OUTPUT);
  pinMode(GRPM_PWM, PWM);
  pinMode(GRPM_DIR, OUTPUT);

  RM_Drive(0);
  LM_Drive(0);

  UDM_Drive(0);
  GRPM_Drive(0);

}

void loop() {

  Spi_recieve();
  DataCheck();
  if (DataAccepted) {
    
  }

}
