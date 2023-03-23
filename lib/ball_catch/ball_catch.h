#ifndef MBED_BALL_CATCH_H
#define MBED_BALL_CATCH_H

#include "mbed.h"

#define READ_NUMBER_OF_TIME 500   // ボールセンサを読む回数
#define MOVING_AVERAGE_COUNT_NUMBER 75   // 移動平均フィルタの回数
#define IR_NUM 2
class ball_catch {
     public:
      ball_catch(PinName left_, PinName right_);
      void read();
      uint8_t get_left();
      uint8_t get_right();
      uint8_t get();

     private:
      DigitalIn left;
      DigitalIn right;

      int16_t value[IR_NUM];
      int16_t tmp_value[4][MOVING_AVERAGE_COUNT_NUMBER];

      uint8_t moving_average_count;
};

#endif