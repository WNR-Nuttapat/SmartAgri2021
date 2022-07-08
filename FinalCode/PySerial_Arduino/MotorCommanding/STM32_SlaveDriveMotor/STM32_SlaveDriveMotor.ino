#include <SPI.h>
#define SPI1_NSS_PIN PA4
uint8_t count = 0;
uint8_t data[25];
volatile bool ButtonData8bit[4];
unsigned int NumberData16;
byte DataAccepted = 0;

int PWMData[8];

#define FRM_DIR PA2
#define FRM_PWM PA0
#define RRM_DIR PA3
#define RRM_PWM PA1

#define FLM_DIR PB10
#define FLM_PWM PB0
#define RLM_DIR PB11
#define RLM_PWM PB1

void FRM_Drive(int FRMpwm) {
  if (FRMpwm >= 0)
    digitalWrite(FRM_DIR, LOW);
  else
    digitalWrite(FRM_DIR, HIGH);

  FRMpwm = abs(FRMpwm);
  if (FRMpwm > 255)
    FRMpwm = 255;

  analogWrite(FRM_PWM, FRMpwm);
}

void RRM_Drive(int RRMpwm) {
  if (RRMpwm >= 0)
    digitalWrite(RRM_DIR, LOW);
  else
    digitalWrite(RRM_DIR, HIGH);

  RRMpwm = abs(RRMpwm);
  if (RRMpwm > 255)
    RRMpwm = 255;

  analogWrite(RRM_PWM, RRMpwm);
}

void FLM_Drive(int FLMpwm) {
  if (FLMpwm >= 0)
    digitalWrite(FLM_DIR, LOW);
  else
    digitalWrite(FLM_DIR, HIGH);

  FLMpwm = abs(FLMpwm);
  if (FLMpwm > 255)
    FLMpwm = 255;

  analogWrite(FLM_PWM, FLMpwm);
}

void RLM_Drive(int RLMpwm) {
  if (RLMpwm >= 0)
    digitalWrite(RLM_DIR, LOW);
  else
    digitalWrite(RLM_DIR, HIGH);

  RLMpwm = abs(RLMpwm);
  if (RLMpwm > 255)
    RLMpwm = 255;

  analogWrite(RLM_PWM, RLMpwm);
}

void Spi_recieve() {
  uint8_t msg = SPI.transfer(++count);
  //  Serial.println(msg,BIN);
  for (byte i = 0; i < 24; i++) {
    data[i] = data[i + 1];
  }
  data[24] = msg;
}

void DataCheck() {

  DataAccepted = 0;

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
    Serial.println("DataAccepted");
    for (int j = 0; j < 8; j++) {
      PWMData[j] = PWMData[j] - 1500;
      //      Serial.println(PWMData[j]);
    }

  }
  else {
    //    Serial.println("DataFailed");
    DataAccepted = 0;
  }
}

void ButtonData8bitDecoder(byte temp) {
  ButtonData8bit[0] = (temp >> 0)&B1;
  ButtonData8bit[1] = (temp >> 2)&B1;
  ButtonData8bit[2] = (temp >> 4)&B1;
  ButtonData8bit[3] = (temp >> 6)&B1;
}

void NumberData16bitDecoder(byte temp1, byte temp2) {
  NumberData16 = (temp1 << 8) | temp2;
}


void setup() {

  Serial.begin(115200);
  Serial.print("RDY");
  SPI.begin();
  SPI.beginTransactionSlave(SPISettings(1000000, MSBFIRST, SPI_MODE0, DATA_SIZE_8BIT));
  pinMode(SPI1_NSS_PIN, INPUT);

  pinMode(FRM_PWM, PWM);
  pinMode(FRM_DIR, OUTPUT);
  pinMode(RRM_PWM, PWM);
  pinMode(RRM_DIR, OUTPUT);
  pinMode(FLM_PWM, PWM);
  pinMode(FLM_DIR, OUTPUT);
  pinMode(RLM_PWM, PWM);
  pinMode(RLM_DIR, OUTPUT);

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  FRM_Drive(0);
  RRM_Drive(0);
  FLM_Drive(0);
  RLM_Drive(0);

}

void loop() {

  Spi_recieve();
  DataCheck();
  if (DataAccepted) {
    FLM_Drive(PWMData[4]);
    FRM_Drive(PWMData[5]);
    RLM_Drive(PWMData[6]);
    RRM_Drive(PWMData[7]);
    DataAccepted = 0;
  }

}
