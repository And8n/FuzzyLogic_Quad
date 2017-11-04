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
  roll /= 65.5;
  pitch /= 65.5;
  yaw /= 65.5;
  
  x = FilterX.MovingAvg(axis[1], 0.01);          // Accelerometer
  y = FilterY.MovingAvg(axis[2], 0.01);
  z = FilterZ.MovingAvg(axis[3], 0.01) * (-1);

  CalcAngle(roll, pitch, yaw, x, y ,z);
}

void GyroData::CalcAngle(float roll, float pitch, float yaw, float x, float y, float z){

      roll_angle += roll * 0.0000611;
    pitch_angle += pitch * 0.0000611;

    pitch_angle -= roll_angle * sin(yaw * 0.000001066);
    roll_angle += pitch_angle * sin(yaw * 0.000001066);

    acc_vector = sqrt((x * x) + (y * y) + (z * z));

    if (abs(y) < acc_vector) {
      pitch_acc = asin((float)y / acc_vector) * 57.296;
    }
    if (abs(x) < acc_vector) {
      roll_acc = asin((float)x / acc_vector) * -57.296;
    }

    pitch_acc -= 1.0;  // 1.0;
    roll_acc -= -1.0; // -1.0;

    pitch_angle = pitch_angle * 0.9995 + pitch_acc * 0.0005;
    roll_angle = roll_angle * 0.9995 + roll_acc * 0.0005;
    pitch_adjust = pitch_angle * 15.0;
    roll_adjust = roll_angle * 15.0;

}
float GyroData::getPitchAcc() const{
 return pitch_acc;
}
float GyroData::getRollAcc() const{
  return roll_acc;
}
float GyroData::getRollAngle() const{
  return roll_angle;
}
float GyroData::getRollAdjust() const{
  return roll_adjust;
}
float GyroData::getPitchAngle() const{
  return pitch_angle;
}
float GyroData::getPitchAdjust() const{
  return pitch_adjust;
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

