#include "hcsr04.h"

HCSR04::HCSR04(PinName TrigPin, PinName EchoPin) : trigger(TrigPin), echo(EchoPin) {
      pulsetime.stop();
      pulsetime.reset();
      echo.rise(this, &HCSR04::isr_rise);
      echo.fall(this, &HCSR04::isr_fall);
      trigger = 0;
}

HCSR04::~HCSR04() {
}

void HCSR04::isr_rise(void) {
      pulsetime.start();
}
void HCSR04::start(void) {
      trigger = 0;
      wait_us(2);
      trigger = 1;
      wait_us(10);
      trigger = 0;
}

void HCSR04::isr_fall(void) {
      pulsetime.stop();
      pulsedur = pulsetime.read_us();
      distance = (pulsedur * 343) / 20000;

      if (distance_moving_average_count == DISTANCE_MOVING_AVERAGE_COUNT_NUMBER) distance_moving_average_count = 0;
      tmp_distance[distance_moving_average_count] = distance;
      distance = 0;
      for (uint8_t count = 0; count < DISTANCE_MOVING_AVERAGE_COUNT_NUMBER; count++) distance += tmp_distance[count];
      distance /= DISTANCE_MOVING_AVERAGE_COUNT_NUMBER;
      distance_moving_average_count++;

      pulsetime.reset();
}

void HCSR04::rise(void (*fptr)(void)) {
      echo.rise(fptr);
}
void HCSR04::fall(void (*fptr)(void)) {
      echo.fall(fptr);
}

uint16_t HCSR04::get() {
      return distance;
}
uint16_t HCSR04::get_pulse_us() {
      return pulsedur;
}