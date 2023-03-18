#ifndef MBED_BALL_CATCH_H
#define MBED_BALL_CATCH_H

#include "mbed.h"

#define READ_NUMBER_OF_TIME 100   // ボールセンサを読む回数
#define SAMPLE_NUMBER 20   // n回分の過去に読んだ値の使用
#define IR_NUM 2
#define RC 0.9
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

      uint16_t sample_value[SAMPLE_NUMBER][IR_NUM];
      int32_t value[IR_NUM], pre_value[IR_NUM];
};

#endif