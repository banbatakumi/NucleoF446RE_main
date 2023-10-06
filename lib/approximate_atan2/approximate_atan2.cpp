#include "approximate_atan2.h"

int16_t MyAtan2(int16_t y_, int16_t x_, uint8_t accuracy_level_) {
      int16_t x = abs(x_);
      int16_t y = abs(y_);
      float z;
      bool c;

      c = y < x;
      if (c == 1) {
            z = (float)y / x;
      } else {
            z = (float)x / y;
      }

      // ４次曲線になるほど精度が悪くなる
      int16_t a;
      if (accuracy_level_ = 0) {
            a = z * (-1556 * z + 6072);   // 2次曲線近似
      } else if (accuracy_level_ = 1) {
            a = z * (z * (-448 * z - 954) + 5894);   // 3次曲線近似
      } else if (accuracy_level_ = 2) {
            a = z * (z * (z * (829 * z - 2011) - 58) + 5741);   // 4次曲線近似
      }

      if (c == 1) {
            if (x_ > 0) {
                  if (y_ < 0) a *= -1;
            }
            if (x_ < 0) {
                  if (y_ > 0) a = 18000 - a;
                  if (y_ < 0) a = a - 18000;
            }
      }

      if (c != 1) {
            if (x_ > 0) {
                  if (y_ > 0) a = 9000 - a;
                  if (y_ < 0) a = a - 9000;
            }
            if (x_ < 0) {
                  if (y_ > 0) a = a + 9000;
                  if (y_ < 0) a = -a - 9000;
            }
      }

      a /= 100;
      return a;
}
