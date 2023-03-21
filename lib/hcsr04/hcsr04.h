#ifndef MBED_HCSR04_H
#define MBED_HCSR04_H

#include "mbed.h"

#define DISTANCE_MOVING_AVERAGE_COUNT_NUMBER 50
class HCSR04 {
     public:
      HCSR04(PinName TrigPin, PinName EchoPin);
      ~HCSR04();

      uint16_t get(void);

      uint16_t get_pulse_us(void);

      void start(void);
      void isr_rise(void);
      void isr_fall(void);
      void fall(void (*fptr)(void));
      void rise(void (*fptr)(void));

     private:
      Timer pulsetime;
      DigitalOut trigger;
      InterruptIn echo;

      uint16_t distance_moving_average_count, tmp_distance[DISTANCE_MOVING_AVERAGE_COUNT_NUMBER];
      uint16_t pulsedur;
      uint16_t distance;
};

#endif