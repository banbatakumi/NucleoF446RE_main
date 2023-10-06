#include "moving_ave.h"

#include "mbed.h"

void MovingAve::SetLength(uint8_t length_) {
      this->length = length_;
}

void MovingAve::Compute(int16_t* input_) {
      if (cnt >= length) cnt = 0;
      data[cnt] = *input_;
      *input_ = 0;
      for (uint8_t i = 0; i < length; i++) {
            *input_ += data[i];
      }
      *input_ /= length;
      cnt++;
}

void MovingAve::Reset() {
      for (uint8_t i = 0; i < length; i++) {
            data[i] = 0;
      }
}