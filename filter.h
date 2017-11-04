#ifndef filter_h
#define filter_h

class filter {
  private:
    float out;
    float diff;
    int count;
    
  public:
    filter();
   
   
    float MovingAvg(float in, float snap);
    int MovingAvg(int in, float snap);
    long MovingAvg(long in, float snap);
    //void Flag();
};


#endif

