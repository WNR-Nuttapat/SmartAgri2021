#include "HardwareTimer.h"

//Right Motor
#define IN1_RM PA2
#define IN2_RM PA3
#define PWMpin_RM PB8
#define ENCA_RM PA0
#define ENCB_RM PA1

//Left Motor
#define IN1_LM PA4
#define IN2_LM PA5
#define PWMpin_LM PB9
#define ENCA_LM PA6
#define ENCB_LM PA7

HardwareTimer tim2(2);
HardwareTimer tim3(3);
//Pulses per revolution
#define RM_PPR   970
#define LM_PPR   980

unsigned long RM_FR = 0;
void RM_RevUpdate() {
  if (tim2.getDirection()) {
    RM_FR--;
  } else {
    RM_FR++;
  }
}

unsigned long LM_FR = 0;
void LM_RevUpdate() {
  if (tim3.getDirection()) {
    LM_FR--;
  } else {
    LM_FR++;
  }
}

void MotorDriveR(int PWM) {
  if (PWM > 0) {
    digitalWrite(IN1_RM, HIGH);
    digitalWrite(IN2_RM, LOW);
  }
  else if (PWM < 0) {
    digitalWrite(IN1_RM, LOW);
    digitalWrite(IN2_RM, HIGH);
  }
  else {
    digitalWrite(IN1_RM, LOW);
    digitalWrite(IN2_RM, LOW);
  }
  analogWrite(PWMpin_RM, abs(PWM));
}

void MotorDriveL(int PWM) {
  if (PWM > 0) {
    digitalWrite(IN1_LM, HIGH);
    digitalWrite(IN2_LM, LOW);
  }
  else if (PWM < 0) {
    digitalWrite(IN1_LM, LOW);
    digitalWrite(IN2_LM, HIGH);
  }
  else {
    digitalWrite(IN1_LM, LOW);
    digitalWrite(IN2_LM, LOW);
  }
  analogWrite(PWMpin_LM, abs(PWM));
}

void setup() {

  //PWM
  pinMode(PWMpin_RM, OUTPUT);
  pinMode(PWMpin_LM, OUTPUT);

  //Directiton
  pinMode(IN1_RM, OUTPUT); //Motor1A
  pinMode(IN2_RM, OUTPUT); //Motor1B
  pinMode(IN1_LM, OUTPUT); //Motor2A
  pinMode(IN2_LM, OUTPUT); //Motor2B

  //Encoder
  pinMode(ENCA_RM, INPUT_PULLUP);
  pinMode(ENCB_RM, INPUT_PULLUP);
  tim2.pause(); //stop...
  tim2.setMode(1, TIMER_ENCODER); //set mode, the channel is not used when in this mode (but it must be [1..4]).
  tim2.setPrescaleFactor(1); //normal for encoder to have the lowest or no prescaler.
  tim2.setOverflow(RM_PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps.
  tim2.refresh();   //update PSC,OVF value
  tim2.setCount(0);          //reset the counter.
  tim2.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction.
  tim2.attachInterrupt(0, RM_RevUpdate); //channel must be 0 here
  tim2.resume();                 //start the encoder...

  pinMode(ENCA_LM, INPUT_PULLUP);
  pinMode(ENCB_LM, INPUT_PULLUP);
  tim3.pause(); //stop...
  tim3.setMode(1, TIMER_ENCODER); //set mode, the channel is not used when in this mode (but it must be [1..4]).
  tim3.setPrescaleFactor(1); //normal for encoder to have the lowest or no prescaler.
  tim3.setOverflow(LM_PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps.
  tim3.refresh();   //update PSC,OVF value
  tim3.setCount(0);          //reset the counter.
  tim3.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction.
  tim3.attachInterrupt(0, LM_RevUpdate); //channel must be 0 here
  tim3.resume();                 //start the encoder...

  Serial1.begin(115200);
  pinMode(PB12, OUTPUT);

  MotorDriveR(0);
  MotorDriveL(0);

}

unsigned long interval = 0;
bool a = 0;
int n;
char SerialData[51];
int PWMData[2] = {0, 0};

void loop() {
  // put your main code here, to run repeatedly:

  unsigned int RM_Count = tim2.getCount();
  unsigned int LM_Count = tim3.getCount();
  //Serial1.println(RM_Count);  Serial1.println(LM_Count);
  unsigned int RM_Pulse = (RM_PPR * RM_FR) + RM_Count;
  unsigned int LM_Pulse = (LM_PPR * LM_FR) + LM_Count;
  //Serial1.println(RM_Pulse);  Serial1.println(LM_Pulse);

  if (millis() - interval >= 250) {
    Serial1.print('A');  Serial1.print(RM_Pulse); Serial1.print(',');
    Serial1.print('B');  Serial1.print(LM_Pulse); Serial1.print('\0'); Serial1.print('\n');

    interval = millis(); //update interval for user.
  }

  if (Serial1.available() > 0) {
    int recieved = Serial1.read();
    SerialData[50] = recieved;
    for (int i = 0; i < 51; i++) {
      SerialData[i] = SerialData[i + 1];
    }
    if (SerialData[0] == 'O' && SerialData[13] == 'C') {
      //Serial1.print('C'); Serial1.println(n);
      //a=!a;
      //digitalWrite(PB12,a);
      //Serial.println("C");  //Serial.write('\0');
      //      lcd.setCursor(0, 1);  lcd.print("CP#");
      //      lcd.setCursor(3, 1);  lcd.print(n);
      n++;
      //byte clmn[] = {0, 4, 8, 12, 0, 4, 8, 12};
      //byte rw[] = {0, 0, 0, 0, 1, 1, 1, 1};
      for (int j = 0; j < 2; j++) {
        int sym = 0;
        if (SerialData[(6 * j) + 3] == '1')
          sym = -1;
        else if (SerialData[(6 * j) + 3] == '0')
          sym = 1;
        int val = ((SerialData[(6 * j) + 4] - '0') * 100) +
                  ((SerialData[(6 * j) + 5] - '0') * 10) +
                  ((SerialData[(6 * j) + 6] - '0') * 1);

        PWMData[j] = sym * val;
        //Serial1.println(PWMData[j]);

        //lcd.setCursor(clmn[j], rw[j]);  lcd.print("    ");
        //lcd.setCursor(clmn[j], rw[j]);  lcd.print(sym * val);

      }
      MotorDriveR(PWMData[0]);
      MotorDriveL(PWMData[1]);

    }
    else {
      //Serial.println("F");  //Serial.write('\0');
    }
  }
  else {
    //Serial.println("N");  //Serial.write('\0');
  }


  //Serial1.println(RM_Pulse);  Serial1.println(LM_Pulse);
  //Serial1.print(RM_Count);
  //Serial1.print(',');  Serial1.print(RM_Rev);
  //Serial1.print(',');  Serial1.print(LM_Count);
  //Serial1.print(',');  Serial1.print(LM_Rev);
  //Serial1.print(','); Serial1.print(n);
  //Serial1.print('\n');
  //n++;
  //delay(100);

}
