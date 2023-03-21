#include "line.h"

#include "mbed.h"

line::line(PinName led_pin_, PinName front_1_, PinName front_2_, PinName right_1_, PinName right_2_, PinName right_3_, PinName back_1_, PinName back_2_, PinName back_3_, PinName left_1_, PinName left_2_, PinName left_3_)
    : led_pin(led_pin_), front_1(front_1_), front_2(front_2_), right_1(right_1_), right_2(right_2_), right_3(right_3_), back_1(back_1_), back_2(back_2_), back_3(back_3_), left_1(left_1_), left_2(left_2_), left_3(left_3_) {
      threshold = 7;
}

void line::led(bool intensity) {
      led_pin = intensity;
}

void line::read() {
      value[0] = front_1.read() * 1000;
      value[1] = front_2.read() * 1000;
      value[2] = right_1.read() * 1000;
      value[3] = right_2.read() * 1000;
      value[4] = right_3.read() * 1000;
      value[5] = back_1.read() * 1000;
      value[6] = back_2.read() * 1000;
      value[7] = back_3.read() * 1000;
      value[8] = left_1.read() * 1000;
      value[9] = left_2.read() * 1000;
      value[10] = left_3.read() * 1000;

      if (moving_average_count == MOVING_AVERAGE_COUNT_NUMBER) moving_average_count = 0;
      for (uint8_t count = 0; count < LINE_NUM; count++) {
            tmp_value[count][moving_average_count] = value[count];
            value[count] = 0;
            for (uint8_t count_1 = 0; count_1 < MOVING_AVERAGE_COUNT_NUMBER; count_1++) value[count] += tmp_value[count][count_1];
            value[count] /= MOVING_AVERAGE_COUNT_NUMBER;
            tf[count] = value[count] > reaction[count] + threshold ? 1 : 0;   // 反応したかのチェック
      }
      moving_average_count++;
}

void line::set() {
      led_pin = 1;
      wait_us(100000);
      for (uint8_t count = 0; count < LINE_NUM; count++) reaction[count] = 0;
      for (uint16_t count = 0; count < REACTION_AVERAGE_NUMBER_OF_TIMES; count++) {
            reaction[0] += front_1.read() * 1000;
            reaction[1] += front_2.read() * 1000;
            reaction[2] += right_1.read() * 1000;
            reaction[3] += right_2.read() * 1000;
            reaction[4] += right_3.read() * 1000;
            reaction[5] += back_1.read() * 1000;
            reaction[6] += back_2.read() * 1000;
            reaction[7] += back_3.read() * 1000;
            reaction[8] += left_1.read() * 1000;
            reaction[9] += left_2.read() * 1000;
            reaction[10] += left_3.read() * 1000;
      }
      for (uint8_t count = 0; count < LINE_NUM; count++) reaction[count] = reaction[count] / REACTION_AVERAGE_NUMBER_OF_TIMES;
      led_pin = 0;
}

bool line::all() {
      if (tf[0] || tf[1] || tf[2] || tf[3] || tf[4] || tf[5] || tf[6] || tf[7] || tf[8] || tf[9] || tf[10]) {
            return 1;
      } else {
            return 0;
      }
}

bool line::front() {
      return tf[0] || tf[1] ? 1 : 0;
}

bool line::right() {
      return tf[2] || tf[3] || tf[4] ? 1 : 0;
}

bool line::back() {
      return tf[5] || tf[6] || tf[7] ? 1 : 0;
}

bool line::left() {
      return tf[8] || tf[9] || tf[10] ? 1 : 0;
}

bool line::check(uint8_t line_number) {
      return tf[line_number] ? 1 : 0;
}

uint16_t line::get_value(uint8_t line_number) {
      return value[line_number];
}