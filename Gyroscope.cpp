#include "arduino.h"
#include "Gyroscope.h"
#include <i2c_t3.h>
#include "filter.h"

GyroData::GyroData(){

}


void GyroData::Registers() {                                                 //MPU-6050
  Wire.begin();
  Wire.setClock(400000L);
  Wire.beginTransmission(MPU6050);
  Wire.write(0x6B);                                                          // PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                          // activate
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x1B);                                                          //gyro dps register
  Wire.write(0x08);                                                          //00001000 (500dps scale)
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x1C);                                                          //accelerometer sensisitivity register
  Wire.write(0x10);                                                          //00010000  (8g full scale range)
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x1A);                                                          // 
  Wire.write(0x00);                                                         //(Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();

}

void GyroData::Data() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050, 14);

  while (Wire.available() < 14);
  axis[1] = Wire.read() << 8 | Wire.read();                                 //Accelerometer
  axis[2] = Wire.read() << 8 | Wire.read();
  axis[3] = Wire.read() << 8 | Wire.read();
  int temperature = Wire.read() << 8 | Wire.read();
  axis[4] = Wire.read() << 8 | Wire.read();                                 //Gyroscope - pitch
  axis[5] = Wire.read() << 8 | Wire.read();                                 //Roll
  axis[6] = Wire.read() << 8 | Wire.read();                                 //Yaw
                                      //if (cal_int == 2000) {
  axis[4] -= 50; //gyro_axis_cal[1];
  axis[5] -= -30; //gyro_axis_cal[2];
  axis[6] -= -4;//gyro_axis_cal[3];
             //}
  //Initialisers
  filter FilterRoll;
  filter FilterPitch;
  filter FilterYaw;
  filter FilterX;
  filter FilterY;
  filter FilterZ;
  
  roll = FilterRoll.MovingAvg(axis[5], 0.01); // Gyroscope
  pitch = FilterPitch.MovingAvg(axis[4], 0.01);
  yaw = FilterYaw.MovingAvg(axis[6], 0.01) * (-1);
  
  x = FilterX.MovingAvg(axis[1], 0.01);          // Accelerometer
  y = FilterY.MovingAvg(axis[2], 0.01);
  z = FilterZ.MovingAvg(axis[3], 0.01) * (-1);
}

  void GyroData::Calibrate() {
 /* for (int cal_int = 0; cal_int < 2000; cal_int++) {
    if (cal_int % 15 == 0)digitalWriteFast(led, !digitalReadFast(led));
    gyro_data();
    delay(300);
    Gyro.axis_cal[1] += Gyro.axis[1];
    Gyro.axis_cal[2] += Gyro.axis[2];
    Gyro.axis_cal[3] += Gyro_axis[3];

  }
  Gyro.axis_cal[1] /= 2000;
  Gyro.axis_cal[2] /= 2000;
  Gyro.axis_cal[3] /= 2000;*/
  }

 int GyroData::getGyroRoll() const {
  return roll;
 }
 int GyroData::getGyroPitch() const {
  return pitch;
 }
 int GyroData::getGyroYaw() const {
  return yaw;
 }
 int GyroData::getAccX() const {
  return x;
 }
 int GyroData::getAccY() const {
  return y;
 }
 int GyroData::getAccZ() const {
  return z;
 }

GyroData gyro = GyroData();

