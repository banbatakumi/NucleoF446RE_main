#include "simplify_deg.h"

int SimplifyDeg(int deg_) {
      int deg = deg_;
      while (deg > 180) {
            deg -= 360;
      }
      while (deg < -180) {
            deg += 360;
      }

      return deg;
}