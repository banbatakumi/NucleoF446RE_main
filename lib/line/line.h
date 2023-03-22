#ifndef MBED_LINE_H
#define MBED_LINE_H

#include "mbed.h"

#define REACTION_AVERAGE_NUMBER_OF_TIMES 10000
#define MOVING_AVERAGE_COUNT_NUMBER 10
#define LINE_NUM 11

class line {
     public:
      line(PinName led_pin_, PinName front_1_, PinName front_2_, PinName right_1_, PinName right_2_, PinName right_3_, PinName back_1_, PinName back_2_, PinName back_3_, PinName left_1_, PinName left_2_, PinName left_3_);

      void led(bool intensity);
      void read();
      void set();
      bool all();
      bool front();
      bool right();
      bool back();
      bool left();
      bool check(uint8_t line_number);
      uint16_t get_value(uint8_t line_number);

      uint16_t threshold;

     private:
      PwmOut led_pin;
      AnalogIn front_1;
      AnalogIn front_2;
      AnalogIn right_1;
      AnalogIn right_2;
      AnalogIn right_3;
      AnalogIn back_1;
      AnalogIn back_2;
      AnalogIn back_3;
      AnalogIn left_1;
      AnalogIn left_2;
      AnalogIn left_3;

      bool tf[LINE_NUM];
      uint16_t value[LINE_NUM];
      uint16_t tmp_value[LINE_NUM][MOVING_AVERAGE_COUNT_NUMBER];
      uint8_t moving_average_count;
      uint32_t reaction[LINE_NUM];
};

#endif