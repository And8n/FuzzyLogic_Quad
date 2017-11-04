#ifndef functions_h
#define functions_h

class functions {

    int out;
    
  public:

    functions();

    long Pyth(long x, long y, long z);
    int ConvertStick(int in);
    int ConvertThrottle(int in);
    float Gauss(float x, float a);
    float Positive(float x, float a);
    float Negative(float x, float a);
    float positiveNL(float x);
    float negativeNL(float x);
    float Limiter(float input, float upperLimit, float lowerLimit);
};

extern functions Function;

#endif

