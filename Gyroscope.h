#ifndef Gyroscope_h
#define Gyroscope_h
#define MPU6050 0x68


class GyroData {
    int axis[7], axis_cal[4];
    int x, y, z;
    float pitch;
    float roll;
    float yaw;
  public:
    GyroData();
    void Data();
    void Registers();
    void Calibrate();
    int getGyroRoll() const;
    int getGyroPitch() const;
    int getGyroYaw() const;
    int getAccX() const;
    int getAccY() const;
    int getAccZ() const;
};

extern GyroData gyro;

#endif

