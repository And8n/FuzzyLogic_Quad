#include "arduino.h"
#include "filter.h"




filter::filter() {
  // cuntructor will be called each time an object is created
}







int filter::MovingAvg(int in, float snap) {
  diff = abs(in - out);
  
  float y = 1 / ((diff * snap) + 1);
  y = (1 - y) * 2;
  if (y > 1)y = 1;
  out += (in - out) * y;
  
  return out;
}

float filter::MovingAvg(float in, float snap) {
  diff = abs(in - out);
  
  float y = 1 / ((diff * snap) + 1);
  y = (1 - y) * 2;
  if (y > 1)y = 1;
  out += (in - out) * y;
  
  return out;
}
long filter::MovingAvg(long in, float snap) {
  diff = abs(in - out);

  float y = 1 / ((diff * snap) + 1);
  y = (1 - y) * 2;
  if (y > 1)y = 1;
  out += (in - out) * y;
  
  return out;
}


//filterclass filter = filterclass();
