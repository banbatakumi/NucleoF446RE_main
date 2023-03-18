#ifndef MBED_HCSR04_H
#define MBED_HCSR04_H

#include "mbed.h"

#define RC 0.9

class HCSR04 {
     public:
      HCSR04(PinName TrigPin, PinName EchoPin);
      ~HCSR04();

      uint16_t get_dist_cm(void);

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
      uint16_t pulsedur;
      uint16_t distance, pre_distance;
};

#endif