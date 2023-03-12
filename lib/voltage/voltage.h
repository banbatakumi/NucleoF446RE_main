#ifndef MBED_VOLTAGE_H
#define MBED_VOLTAGE_H

#include "mbed.h"

#define VOLTAGE_RC 0.9
#define CHANGE_VOLTAGE 0.0002125037

class voltage {
     public:
      voltage(PinName battery_voltage_pin_, PinName vcc_voltage_pin_);
      void read();
      float get_battery_voltage();
      float get_vcc_voltage();

     private:
      AnalogIn battery_voltage_pin;
      AnalogIn vcc_voltage_pin;

      float battery_voltage_value, vcc_voltage_value;
      float pre_battery_voltage_value, pre_vcc_voltage_value;

      Timer sampling_timer;
};

#endif