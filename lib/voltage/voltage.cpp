#include "voltage.h"

#include "mbed.h"

voltage::voltage(PinName battery_voltage_pin_, PinName vcc_voltage_pin_) : battery_voltage_pin(battery_voltage_pin_), vcc_voltage_pin(vcc_voltage_pin_) {
      sampling_timer.start();
      battery_voltage_value = 10;
      vcc_voltage_value = 5;
}

void voltage::read() {
      if (sampling_timer > 1) {
            pre_battery_voltage_value = battery_voltage_value;
            battery_voltage_value = (battery_voltage_pin.read_u16() * 5.0 / 1023.0 / 23.0) * (1 - VOLTAGE_RC) + pre_battery_voltage_value * VOLTAGE_RC;   // 電圧のRCフィルタリング

            pre_vcc_voltage_value = vcc_voltage_value;
            vcc_voltage_value = (vcc_voltage_pin.read_u16() * 5.0 / 1023.0 / 23.0) * (1 - VOLTAGE_RC) + pre_vcc_voltage_value * VOLTAGE_RC;   // 電圧のRCフィルタリング

            sampling_timer.reset();
      }
}

float voltage::get_battery_voltage() {
      return battery_voltage_value;
}

float voltage::get_vcc_voltage() {
      return vcc_voltage_value;
}