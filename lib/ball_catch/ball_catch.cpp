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

      if (moving_average_count == MOVING_AVERAGE_COUNT_NUMBER) moving_average_count = 0;
      for (uint8_t count = 0; count < IR_NUM; count++) {
            value[count] = (READ_NUMBER_OF_TIME - value[count]) * (100.00000 / READ_NUMBER_OF_TIME);

            tmp_value[count][moving_average_count] = value[count];
            value[count] = 0;
            for (uint8_t count_1 = 0; count_1 < MOVING_AVERAGE_COUNT_NUMBER; count_1++) value[count] += tmp_value[count][count_1];
            value[count] /= MOVING_AVERAGE_COUNT_NUMBER;
      }
      moving_average_count++;
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