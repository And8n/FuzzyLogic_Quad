#ifndef controller_h
#define controller_h
#include "functions.h"


class Fuzzy: public functions {
  private:
    float e, de, re;
    float a[13], u[13], w[13];
    const float h = 1.0;
    float fuzzy_output;
    float pos_e, pos_d, pos_r, neg_e, neg_d, neg_r, gau_e;
    float e_last;
    float out_max;
    float e_limit;
    float de_limit;
    float re_limit;
    float g_limit;
    float output_gain;
    float rate_gain;

  public:
    Fuzzy(); //Constructor
    Fuzzy(float e_limit, float de_limit, float re_limit, float g_limit, float output_gain, float out_max); // Overload constructor

    float Controller(float rollInput, float setpoint, bool Level);
    void setLimits(float e_limit, float de_limit, float re_limit, float g_limit, float output_gain, float out_max);
    void setOutputMax(float out_max);
    void setGainI(float i_gain);
    void Reset();
};

class PID: public functions {
  private:
    float pid_output;
    float error_p, error_i, error_d, error_p_last;
    float p_gain, i_gain, d_gain, pid_max;

  public:
    PID(float p_gain, float i_gain, float d_gain, int output_max);

    float Controller(float gyro_input, float setpoint);
    void setPidGains(float p_gain, float i_gain, float d_gain);
    void setOutputMax(float newOutputMax);
    void setGainP(float p_gain);
    void setGainI(float i_gain);
    void setGainD(float d_gain);
    void Reset();
};

#endif
