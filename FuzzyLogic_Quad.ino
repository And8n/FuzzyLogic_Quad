#include <arduino.h>
#include "filter.h"
#include "functions.h"
#include "Gyroscope.h"
#include "controller.h"

#define led 13
#define SETA GPIOA_PSOR
#define CLRA GPIOA_PCOR
#define SETB GPIOB_PSOR
#define CLRB GPIOB_PCOR
#define SETC GPIOC_PSOR
#define CLRC GPIOC_PCOR
#define SETD GPIOD_PSOR
#define CLRD GPIOD_PCOR

unsigned long loop_timer, esc_loop_timer;
unsigned long counter[8];

struct Initialise {
  byte flag[7];
  byte start0;
  boolean autoLevel;
} init;

struct Controller {
  int16_t out[5];
  int16_t in[5];
} ESC;

struct GyroscopeData {
 
  float output;
  float setpoint;
  float Adjust;
  float gyro;
  float acc;
  float angle;
 
  float input;
} roll, pitch, yaw, throttle,acc;

struct Receiver {
  volatile uint16_t pitch, roll, throttle, yaw, SWITCH;
  volatile uint16_t Channel[8];
} Receiver;

//Variable End//

void setup() {
  Serial.begin(57600);
  init.start0 = 0;
  init.flag[1] = 1;
  init.flag[2] = 1;
  init.flag[3] = 1;
  init.flag[4] = 1;
  init.flag[5] = 1;

  pinMode(led, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, INPUT);
  attachInterrupt(2, ISR1, CHANGE);
  attachInterrupt(3, ISR2, CHANGE);
  attachInterrupt(4, ISR3, CHANGE);
  attachInterrupt(5, ISR4, CHANGE);
  attachInterrupt(10, ISR5, CHANGE);
 
  digitalWriteFast(led, HIGH);

  gyro.Registers();

  for (int cal_int = 0; cal_int < 1000; cal_int++) {
    SETD = (1 << 4); //6
    SETD = (1 << 2); //7
    SETD = (1 << 3); //8
    SETC = (1 << 3); //9
    delayMicroseconds(125);
    CLRD = (1 << 4); //6
    CLRD = (1 << 2); //7
    CLRD = (1 << 3); //8
    CLRC = (1 << 3); //9
    delayMicroseconds(3000);
  }

  Receiver.throttle = Function.ConvertThrottle(Receiver.Channel[3]);
  Receiver.yaw = Function.ConvertStick(Receiver.Channel[4]);


  while (Receiver.throttle < 990 || Receiver.throttle > 1050 || Receiver.yaw < 1400) {
    init.start0++;
    SETD = (1 << 4); //6
    SETD = (1 << 2); //7
    SETD = (1 << 3); //8
    SETC = (1 << 3); //9
    delayMicroseconds(125);
    CLRD = (1 << 4); //6
    CLRD = (1 << 2); //7
    CLRD = (1 << 3); //8
    CLRC = (1 << 3); //9
    delay(3);
    if (init.start0 == 200) {
      digitalWriteFast(led, !digitalRead(led));
      init.start0 = 0;
    }
  }



  init.start0 = 0;
  //batteryV = (analogRead(2)) * 1.2317;
  digitalWriteFast(led, LOW);
  loop_timer = micros();

}



void loop() {
  //Create instances
  Fuzzy FuzzyRoll(750, 100, 300, 10, 850.0, 400);
  Fuzzy FuzzyPitch(750, 100, 300, 10, 850.0, 400);
  Fuzzy FuzzyYaw(750, 100, 300, 10, 0, 400);
  //PID PidYaw(2.5, 0.003, 0, 400);

  //////////////
  
  bool FLYING = true;
  while (FLYING == true) {
    
  init.flag[1] = 1;
  init.flag[2] = 1;
  init.flag[3] = 1;
  init.flag[4] = 1;
  init.flag[5] = 1;

   /* for (int i = 0; i <= 5; i++) {
      init.flag[i] = 1;
    }*/

    Receiver.roll = Function.ConvertStick(Receiver.Channel[1]);
    Receiver.pitch = Function.ConvertStick(Receiver.Channel[2]);
    Receiver.throttle = Function.ConvertThrottle(Receiver.Channel[3]);
    Receiver.yaw = Function.ConvertStick(Receiver.Channel[4]);
    Receiver.SWITCH = Receiver.Channel[5];

    if (Receiver.SWITCH >= 1800) {
      init.autoLevel = 1;
      init.start0 = 2;

      FuzzyPitch.setGainI(0.006);
      FuzzyRoll.setGainI(0.006);
     
    } if (Receiver.SWITCH <= 1800 && Receiver.SWITCH >= 1300) {
      init.autoLevel = 0;
      init.start0 = 2;

      FuzzyPitch.setGainI(0.035);
      FuzzyRoll.setGainI(0.035);
      
    } if (Receiver.SWITCH < 1300) {
      init.start0 = 1;
      pitch.angle = gyro.getPitchAcc();
      roll.angle = gyro.getRollAcc();

      FuzzyRoll.Reset();
      FuzzyPitch.Reset();
      FuzzyYaw.Reset();
    }


        //Alternative start up sequence if no 3 way switch
    /* 
      if (throttle < 1050 && yaw < 1050)start0 = 1;
      if (start0 == 1 && throttle < 1050 && yaw > 1450) {
      start0 = 2;
      pitch.angle = pitch.acc_angle;
      roll.angle = roll.acc_angle;

      FuzzyRoll.Reset();
      FuzzyPitch.Reset();
      PidYaw.Reset();
      }
      if (start0 == 2 && throttle < 1050 && yaw > 1950)start0 = 0;
    */
  

    gyro.Data(); //Call gyro data
    
    roll.input = gyro.getGyroRoll();
    pitch.input = gyro.getGyroPitch();
    yaw.input = gyro.getGyroYaw();

    roll.setpoint = 0;
    pitch.setpoint = 0;
    yaw.setpoint = 0;

    if (Receiver.roll > 1508) roll.setpoint = (Receiver.roll - 1508);
    else if (Receiver.roll < 1492) roll.setpoint = (Receiver.roll - 1492);

    if (Receiver.pitch > 1508) pitch.setpoint = (1508 - Receiver.pitch);
    else if (Receiver.pitch < 1492) pitch.setpoint = (1492 - Receiver.pitch);

    if (Receiver.throttle > 1050) {
      if (Receiver.yaw > 1508)yaw.setpoint = (Receiver.yaw - 1508);
      else if (Receiver.yaw < 1492)yaw.setpoint = (Receiver.yaw - 1492);
    }

   // throttle.setpoint = Receiver.throttle;
    if (Receiver.throttle < 1100)Receiver.throttle = 1100;

    if (!init.autoLevel) { // rate mode
      roll.setpoint /= 1.2;
      pitch.setpoint /= 1.2;
      yaw.setpoint /= 1.2;
    }
    else if (init.autoLevel) { //for auto level subtract roll/pitch adjust
      roll.setpoint -= gyro.getRollAdjust();
      roll.setpoint /= 1.2;
      pitch.setpoint -= gyro.getPitchAdjust();
      pitch.setpoint /= 1.2;
      yaw.setpoint /= 1.2;
    }


    roll.output = FuzzyRoll.Controller(roll.input, roll.setpoint, init.autoLevel);

    pitch.output = FuzzyPitch.Controller(pitch.input, pitch.setpoint, init.autoLevel);

    yaw.output = FuzzyYaw.Controller(yaw.input, yaw.setpoint, false);

    //yaw.output = PidYaw.Controller(yaw.input, yaw.setpoint);

    ////////////////////////Battery volatage reader////////////////////////
    // int16_t batteryV; // simply add voltage devider to power source and connect to analog input
    // batteryV = batteryV * 0.92 + (analogRead(2)) * 0.09853;   //0.09853 = 0.08 * 1.2317.
    // if (batteryV < 1000 && batteryV > 600)digitalWriteFast(led, HIGH);

    if (init.start0 == 2) {
      if (Receiver.throttle > 1800) Receiver.throttle = 1800;

      ESC.in[1] = Receiver.throttle + pitch.output - roll.output + yaw.output;//(front-right - CCW)
      ESC.in[2] = Receiver.throttle - pitch.output - roll.output - yaw.output;//(rear-right - CW)
      ESC.in[3] = Receiver.throttle - pitch.output + roll.output + yaw.output;//(rear-left - CCW)
      ESC.in[4] = Receiver.throttle + pitch.output + roll.output - yaw.output;//(front-left - CW)

       // Remove Comments if using voltage read
      /* if (batteryV < 1240 && batteryV > 800) {
         esc_1 += esc_1 * ((1240 - batteryV) / (float)3500);
         esc_2 += esc_2 * ((1240 - batteryV) / (float)3500);
         esc_3 += esc_3 * ((1240 - batteryV) / (float)3500);
         esc_4 += esc_4 * ((1240 - batteryV) / (float)3500);
        }*/
      if (ESC.in[1] < 1100) ESC.in[1] = 1100;     // 0x44C = 1100
      if (ESC.in[2] < 1100) ESC.in[2] = 1100;
      if (ESC.in[3] < 1100) ESC.in[3] = 1100;
      if (ESC.in[4] < 1100) ESC.in[4] = 1100;

      if (ESC.in[1] > 2000)ESC.in[1] = 2000;    // 0x7D0 = 2000
      if (ESC.in[2] > 2000)ESC.in[2] = 2000;
      if (ESC.in[3] > 2000)ESC.in[3] = 2000;
      if (ESC.in[4] > 2000)ESC.in[4] = 2000;
    }
    else {
      ESC.in[1] = 1000;    // 0x3E8 = 1000
      ESC.in[2] = 1000;
      ESC.in[3] = 1000;
      ESC.in[4] = 1000;
    }

    ESC.out[1] = (float)ESC.in[1] / 8.0; // divide by 8 for one shot (2000/8=250 pulse)
    ESC.out[2] = (float)ESC.in[2] / 8.0;
    ESC.out[3] = (float)ESC.in[3] / 8.0;
    ESC.out[4] = (float)ESC.in[4] / 8.0;

    if ((micros() - loop_timer) > 4050)digitalWriteFast(led, HIGH);
    while ((micros() - loop_timer) < 4000);

    loop_timer = micros();

    SETD = (1 << 2); //7
    SETD = (1 << 3); //8
    SETC = (1 << 3); //9
    SETD = (1 << 4); //6

    unsigned long timer[5];
    timer[1] = (ESC.out[1] + loop_timer);
    timer[2] = (ESC.out[2] + loop_timer);
    timer[3] = (ESC.out[3] + loop_timer);
    timer[4] = (ESC.out[4] + loop_timer);

    while (digitalReadFast(6) || digitalReadFast(7) || digitalReadFast(8) || digitalReadFast(9))
    {
      esc_loop_timer = micros();
      if (timer[1] <= esc_loop_timer)CLRD = (1 << 3); //8
      if (timer[2] <= esc_loop_timer)CLRD = (1 << 2); //7
      if (timer[3] <= esc_loop_timer)CLRD = (1 << 4); //6
      if (timer[4] <= esc_loop_timer)CLRC = (1 << 3); //9
    }
  }// end flying
}//END

//////////////////////////////////////////////////////////////////////////////////////////////////////

void ISR1() {

  if (init.flag[1] == 1) {
    if (digitalReadFast(2) != 0) {
      counter[1] = micros();
      return;
    }
    else if (micros() - counter[1] > 999) {
      if (micros() - counter[1] < 2001) {
        Receiver.Channel[1] = (uint16_t)(micros() - counter[1]);//roll
        init.flag[1] = 0;
      }
    }
  }
}
void ISR2() {

  if (init.flag[2] == 1) {

    if (digitalReadFast(3) != 0) {
      counter[2] = micros();
      return;
    }
    else if (micros() - counter[2] > 999) {
      if (micros() - counter[2] < 2001) {
        Receiver.Channel[2] = (uint16_t)(micros() - counter[2]); //pitch
        init.flag[2] = 0;
      }
    }
  }
}
void ISR3() {

  if (init.flag[3] == 1) {
    if (digitalReadFast(4) != 0) {
      counter[3] = micros();
      return;
    }
    else if (micros() - counter[3] > 999) {
      if (micros() - counter[3] < 2001) {
        Receiver.Channel[3] = (uint16_t)(micros() - counter[3]); // throttle
        init.flag[3] = 1;
      }
    }
  }
}
void ISR4() {

  if (init.flag[4] == 1) {
    if (digitalReadFast(5) != 0) {
      counter[4] = micros();
      return;
    }
    else if (micros() - counter[4] > 999) {
      if (micros() - counter[4] < 2001) {
        Receiver.Channel[4] = (uint16_t)(micros() - counter[4]);//yaw
        init.flag[4] = 0;
      }
    }
  }
}
void ISR5() {

  if (init.flag[5] == 1) {
    if (digitalReadFast(10) != 0) {
      counter[5] = micros();
      return;
    }
    else if (micros() - counter[5] > 999) {
      if (micros() - counter[5] < 2001) {
        Receiver.Channel[5] = (uint16_t)(micros() - counter[5]);//3 way switch
        init.flag[5] = 0;
      }
    }
  }
}





