#ifndef Gyroscope_h
#define Gyroscope_h
#define MPU6050 0x68


class GyroData {
private:
    int axis[7], axis_cal[4];
    int x, y, z;
    float pitch;
    float roll;
    float yaw;
    float roll_angle, roll_adjust, pitch_angle, pitch_adjust;
    float acc_vector, pitch_acc, roll_acc;

  public:
    GyroData();
    void CalcAngle(float roll, float pitch, float yaw, float x, float y, float z);
    void Data();
    void Registers();
    void Calibrate();
    float getRollAcc() const;
    float getPitchAcc() const;
    float getRollAdjust() const;
    float getRollAngle() const;
    float getPitchAdjust() const;
    float getPitchAngle() const;
    int getGyroRoll() const;
    int getGyroPitch() const;
    int getGyroYaw() const;
    int getAccX() const;
    int getAccY() const;
    int getAccZ() const;
};

extern GyroData gyro;

#endif

