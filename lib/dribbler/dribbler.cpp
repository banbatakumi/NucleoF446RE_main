#include "dribbler.h"

#include "mbed.h"

dribbler::dribbler(PinName pin_1_, PinName pin_2_) : pin_1(pin_1_), pin_2(pin_2_) {
      pin_1 = 0;
      pin_2 = 0;
}

void dribbler::test() {
      pin_1 = 0.9;
      pin_2 = 0;
      wait_us(1500000);
      pin_1 = 0;
      pin_2 = 0;
      wait_us(250000);
      pin_1 = 0;
      pin_2 = 0.9;
      wait_us(500000);
}

void dribbler::hold(uint8_t speed) {
      pin_1 = speed;
      pin_2 = 0;
}
void dribbler::kick(uint8_t speed) {
      pin_1 = 0;
      pin_2 = speed;
}
void dribbler::stop() {
      pin_1 = 0;
      pin_2 = 0;
}