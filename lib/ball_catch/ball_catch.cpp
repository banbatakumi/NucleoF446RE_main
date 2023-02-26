#include "ball_catch.h"

#include "mbed.h"

ball_catch::ball_catch(PinName left_, PinName right_) : left(left_), right(right_) {
}

void ball_catch::read() {
      for (uint8_t count = 0; count < IR_NUM; count++) value[count] = 0;
      for (uint16_t count = 0; count < READ_NUMBER_OF_TIME; count++) {
            value[0] += left;
            value[1] += right;
      }

      for (uint8_t count = 0; count < IR_NUM; count++) {
            for (uint8_t i = SAMPLE_NUMBER - 1; i > 0; i--) sample_value[i][count] = sample_value[i - 1][count];
            sample_value[0][count] = value[count];
            value[count] = 0;
            for (uint8_t i = 0; i < SAMPLE_NUMBER; i++) value[count] += sample_value[i][count];
            value[count] = value[count] * (1 - RC) + pre_value[count] * RC;
            pre_value[count] = value[count];
            value[count] = (READ_NUMBER_OF_TIME * SAMPLE_NUMBER - value[count]) * (100.000 / (READ_NUMBER_OF_TIME * SAMPLE_NUMBER));
      }
}

uint8_t ball_catch::get_left() {
      return value[0];
}

uint8_t ball_catch::get_right() {
      return value[1];
}

uint8_t ball_catch::get() {
      return (value[0] + value[1]) / 2;
}