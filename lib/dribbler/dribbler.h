#ifndef MBED_dribbler_H
#define MBED_dribbler_H

#include "mbed.h"

class dribbler {
     public:
      dribbler(PinName pin_1_, PinName pin_2_);
      void test();
      void kick(uint8_t speed = 100);
      void hold(uint8_t speed = 50);
      void stop();

     private:
      PwmOut pin_1;
      PwmOut pin_2;
};

#endif