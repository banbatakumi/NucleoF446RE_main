#ifndef MBED_MOVING_AVE_H
#define MBED_MOVING_AVE_H

#include "mbed.h"

class MovingAve {
     public:
      void Compute(int16_t* input_);
      void SetLength(uint8_t length_);
      void Reset();

     private:
      int16_t data[100];
      uint8_t cnt;
      uint8_t length;
};

#endif