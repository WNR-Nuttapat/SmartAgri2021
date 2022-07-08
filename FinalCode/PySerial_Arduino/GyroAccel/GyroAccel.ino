//Get data from Gyroscope and Accelerometer using MPU9250 Module

#include <Wire.h>

#define CALIBRATION_NUMBER 1000

uint8_t buffer[2] = {0, 0};
int16_t rawGyro[3] = {0, 0, 0}, rawAccel[3] = {0, 0, 0};
int16_t rawTemp = 0;
int32_t calGyro[3] = {0, 0, 0}, calAccel[3] = {0, 0, 0};
double intGyro[3], gyro[3], accel[3], angleAccel[3] = {0, 0, 0};
double accelMag = 0;
//GET Angle From gyro[3],accel[3]

void calibration_Gyro();
void getData(int16_t* _gyro, int16_t* _accel, int16_t& _temp);

uint32_t prevMicros = 0, deltaMicros = 0;
void processing_Gyro();

void printValue();

void setup() {
  Wire.setClock(400000);
  Wire.begin();

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  calibration_Gyro();

  Serial1.begin(1000000);
  prevMicros = micros();

}

void loop() {
  processing_Gyro();
  printValue();

}

void printValue() {
  Serial1.print("G");
  for (uint8_t i = 0; i < 3; i++) {
    Serial1.print("/");
    Serial1.print(gyro[i]);
  }
  Serial1.print(",A");
  for (uint8_t i = 0; i < 3; i++) {
    Serial1.print("/");
    Serial1.print(accel[i]);
  }
  Serial1.println("");
}

void processing_Gyro() {

  getData(rawGyro, rawAccel, rawTemp);
  for (uint8_t i = 0; i < 3; i++) {
    intGyro[i] = (double)(rawGyro[i] - calGyro[i]) / 131.0;
    if (i < 2) {
      accel[i] = (double)(rawAccel[i] - calAccel[i]) / 16384.0;
    }
    else {
      accel[i] = (double)(rawAccel[i]) / 16384.0;
    }
  }
  deltaMicros = micros() - prevMicros;
  for (uint8_t i = 0; i < 3; i++) {
    gyro[i] += (double)intGyro[i] * deltaMicros / 1000000;
  }
  gyro[0] -= (double)gyro[1] * sin(intGyro[2] * deltaMicros / 1000000 / 57.296);
  gyro[1] += (double)gyro[0] * sin(intGyro[2] * deltaMicros / 1000000 / 57.296);
  prevMicros = micros();

  accelMag = sqrt(pow(accel[0], 2) + pow(accel[1], 2) + pow(accel[2], 2));
  angleAccel[0] = asin((double)accel[1] / accelMag) * 57.296;
  angleAccel[1] = asin((double)accel[0] / accelMag) * 57.296 * -1;

  gyro[0] = 0.96 * gyro[0] + 0.04 * angleAccel[0];
  gyro[1] = 0.96 * gyro[1] + 0.04 * angleAccel[1];

}

void calibration_Gyro() {
  for (uint16_t j = 0 ; j < CALIBRATION_NUMBER ; j++) {
    getData(rawGyro, rawAccel, rawTemp);
    for (uint8_t i = 0; i < 3; i++) {
      calGyro[i] += (int32_t)rawGyro[i];
      calAccel[i] += (int32_t)rawAccel[i];
    }
  }
  for (uint8_t i = 0; i < 3; i++) {
    calGyro[i] /= (int32_t)CALIBRATION_NUMBER;
    calAccel[i] /= (int32_t)CALIBRATION_NUMBER;
  }

}

void getData(int16_t* _gyro, int16_t* _accel, int16_t& _temp) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);

  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 2; j++) {
      buffer[j] = Wire.read();
    }
    (*_accel) = (int16_t)buffer[0] << 8 | buffer[1];
    _accel++;

  }
  for (uint8_t i = 0; i < 2; i++) {
    buffer[i] = Wire.read();
  }
  _temp = (int16_t)buffer[0] << 8 | buffer[1];
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 2; j++) {
      buffer[j] = Wire.read();
    }
    (*_gyro) = (int16_t)buffer[0] << 8 | buffer[1];
    _gyro++;

  }
  Wire.endTransmission(true);
}
