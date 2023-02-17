#include <mbed.h>

#include "line.h"
#include "motor.h"

#define PI 3.1415926535   // 円周率

motor _motor(PA_10, PB_3, PB_5, PB_4, PA_8, PB_10, PA_5, PA_9);   //  45度、135度、225度、315度
line _line(PC_9, PC_2, PC_3, PC_0, PC_1, PB_0, PA_4, PA_1, PA_0, PC_4, PB_1, PC_5);   // ledピン、前ライン、右ライン、後ライン、左ライン

PwmOut Dribbler_1(PC_6);
PwmOut Dribbler_2(PC_8);

int main() {
      // put your setup code here, to run once:
      _line.led(1);

      while (1) {
            // put your main code here, to run repeatedly:
            _motor.run(0, 0);
            Dribbler_1 = 0;
            Dribbler_2 = 0;
      }
}