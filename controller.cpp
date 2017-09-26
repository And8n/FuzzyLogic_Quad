#include "arduino.h"
#include "controller.h"
#include "functions.h"



Fuzzy::Fuzzy()                                                                     // constructor
{
  e_limit = 0;
  de_limit = 0;
  re_limit = 0;
  g_limit = 0;
  output_gain = 0;
  out_max = 0;
  
  } 
Fuzzy::Fuzzy(float e_limit, float de_limit, float re_limit, float g_limit, float output_gain, float out_max)  //overload constructor
{
  this->e_limit = e_limit;
  this->de_limit = de_limit;
  this->re_limit = re_limit;
  this->g_limit = g_limit;
  this->output_gain = output_gain;
  this->out_max = out_max;
}

void Fuzzy::setLimits(float e_limit, float de_limit, float re_limit, float g_limit, float output_gain, float out_max)
{
  this->e_limit = e_limit;
  this->de_limit = de_limit;
  this->re_limit = re_limit;
  this->g_limit = g_limit;
  this->output_gain = output_gain;
  this->out_max = out_max;
}
void Fuzzy::setOutputMax(float out_max){
  this->out_max = out_max;
}
void Fuzzy::setGainI(float rate_gain){
  this->rate_gain = rate_gain;
}


float Fuzzy::Controller(float roll_input, float setpoint, bool level) {
   
  
  if (level) {
    w[1] = 1.0;
    w[2] = 0.35;
    w[3] = 1.0;
    w[4] = 0.7;
  } if (!level) {
    w[1] = 1.3;
    w[2] = 0.15;
    w[3] = 1.0;
    w[4] = 0;
  }

  e = (setpoint - roll_input);
  re += e * rate_gain;  
  de = (e - e_last);

  e = Function.Limiter(e, e_limit, (-1)*e_limit); //Limits the output of e
  re = Function.Limiter(re, re_limit, (-1)*re_limit);
  de = Function.Limiter(de, de_limit, (-1)*de_limit);

  pos_e = Function.Positive(e, e_limit); //linear posetive function (centre point 0.5 with output from 0 to 1)
  pos_d = Function.Positive(de, de_limit);
  pos_r = Function.Positive(re, re_limit);

  neg_e = Function.Negative(e, e_limit);
  neg_d = Function.Negative(de, de_limit);
  neg_r = Function.Negative(re, re_limit);

  gau_e = Function.Gauss(e, g_limit); //Gaussian function with center point at 0
  // gauD = function.gauss(de, g);

  a[1] = neg_e * neg_d; //r1  //rule 1 and 2 acts as the Proportional part from a PID
  a[2] = pos_e * pos_d; //r2  //

  a[3] = neg_e * pos_d; //r3  //rule 3 and 4 acts as the derivative part from a PID
  a[4] = pos_e * neg_d; //r4  //

  a[5] = neg_e * neg_r; //r5  //rule 5 and 6 acts as the integral part from a PID
  a[6] = pos_e * pos_r; //r6  //

  a[7] = gau_e * pos_d; //r7  //gaussian function rule further reducing overshoot - places more weight on D term when gauE = 1
  a[8] = gau_e * neg_d; //r8  //

  //output MF
  u[1] = a[1] * w[1] * (-1) * h;          //h - Singleton output (can be replaced with function)
  u[2] = a[2] * w[1] * h;

  u[3] = a[3] * w[2] * h;
  u[4] = a[4] * w[2] * (-1) * h;

  u[5] = a[5] * w[3] * (-1) * h;
  u[6] = a[6] * w[3] * h;

  u[7] = a[7] * w[4] * h;
  u[8] = a[8] * w[4] * (-1) * h;

  fuzzy_output = ((u[1] + u[2] + u[3] + u[4] + u[5] + u[6] + u[7] + u[8]) / (a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7] + a[8])); //Centroidal Defuzzifier
  fuzzy_output *= output_gain;

  fuzzy_output = Function.Limiter(fuzzy_output, out_max, (-1)*out_max);
 
  e_last = e;

  return fuzzy_output;
}

void Fuzzy::Reset(){
  this->e = 0;
  this->de = 0;
  this->re = 0;
  this->fuzzy_output = 0;
}

//////////////////////PID////////////////////////////////////////////////////////
PID::PID(float p_gain, float i_gain, float d_gain, int pid_max){ //CONSTRUCTOR
  this->p_gain = p_gain;
  this->i_gain = i_gain;
  this->d_gain = d_gain;
  this->pid_max = pid_max;
}

void PID::Reset(){
  this->error_p = 0;
  this->error_d = 0;
  this->error_i = 0;
  this->pid_output = 0;
}

float PID::Controller(float gyro_input, float setpoint){
  error_p = setpoint - gyro_input;
  error_i += i_gain * error_p;
  
  error_i = Function.Limiter(error_i, pid_max, (-1)*pid_max);
  error_d = error_p - error_p_last;

  pid_output = (p_gain * error_p) + error_i + (d_gain * error_d);
  pid_output = Function.Limiter(pid_output, pid_max, (-1)*pid_max);
  error_p_last = error_p;

  return pid_output;
}

void PID::setPidGains(float p_gain, float i_gain, float d_gain){
  this->p_gain = p_gain;
  this->i_gain = i_gain;
  this->d_gain = d_gain;  
}
void PID::setOutputMax(float pid_max){
  this->pid_max = pid_max;
}
void PID::setGainP(float p_gain){
  this->p_gain = p_gain;
}
void PID::setGainI(float i_gain){
  this->i_gain = i_gain;
}
void PID::setGainD(float d_gain){
  this->d_gain = d_gain;
}


