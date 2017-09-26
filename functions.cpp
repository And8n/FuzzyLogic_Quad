#include "arduino.h"
#include "functions.h"




functions::functions() {
  // cuntructor will be called each time an object is created
}

int functions::ConvertStick(int in) {
  int in2 = in - 1500;

  if (in > 1500)
    out = (in2 * in2) / 1000 + (in2) / 2 + 1500;
  else if (in < 1500)
    out = 1500 - (in2 * in2) / 1000 + in2 / 2;
  return out;
}

int functions::ConvertThrottle(int in) {
  int Throttle = sqrt(in - 1000) * 31.63 + 1000;
  return Throttle;
}

float functions::Gauss(float  x, float a)
{
  float y;
  y = exp((-1 * (x * x)) / (2 * a)); //centre is 0
  return y;
}
float functions::Negative(float x, float a)
{
  float y;
  y = (a - x) / (2 * a); // centre is 0.5
  return y;
}
float functions::Positive(float x, float a)
{
  float y;
  y = (a + x) / (2 * a); // centre is 0.5
  return y;
}
float functions::positiveNL(float x)//from -10 to 10
{
  float y;
  y = (((x * x * x + x) / 2020.0) + 0.5); // centre is 0.5
  return y;
}
float functions::negativeNL(float x)//from -10 to 10
{
  float y;
  y = (((-x * x * x - x) / 2020) + 0.5); // centre is 0.5
  return y;
}
float functions::Limiter(float input, float upperLimit, float lowerLimit){
    if (input >= upperLimit)input = upperLimit;
  else if (input <= (lowerLimit))input = lowerLimit;
  return input;
}

functions Function = functions();
