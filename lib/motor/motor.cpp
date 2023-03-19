#include "motor.h"

#include "mbed.h"

motor::motor(PinName motor_1_1_, PinName motor_1_2_, PinName motor_2_1_, PinName motor_2_2_, PinName motor_3_1_, PinName motor_3_2_, PinName motor_4_1_, PinName motor_4_2_) : motor_1_1(motor_1_1_), motor_1_2(motor_1_2_), motor_2_1(motor_2_1_), motor_2_2(motor_2_2_), motor_3_1(motor_3_1_), motor_3_2(motor_3_2_), motor_4_1(motor_4_1_), motor_4_2(motor_4_2_) {
      motor_1_1 = 0;
      motor_1_2 = 0;
      motor_2_1 = 0;
      motor_2_2 = 0;
      motor_3_1 = 0;
      motor_3_2 = 0;
      motor_4_1 = 0;
      motor_4_2 = 0;

      d_timer.start();
}

void motor::run(int16_t move_angle, int16_t move_speed, int8_t robot_angle) {
      angle = move_angle;
      speed = move_speed;
      if (move_speed > POWER_LIMIT) move_speed = POWER_LIMIT;   // 速度が上限を超えていないか
      for (uint8_t count = 0; count < MOTOR_NUM; count++) power[count] = sin((move_angle - (45 + count * 90)) * PI / 180.00000) * move_speed * (count < 2 ? -1 : 1);   // 角度とスピードを各モーターの値に変更

      // モーターの最大パフォーマンス発揮
      maximum_power = 0;
      for (uint8_t count = 0; count < MOTOR_NUM; count++) {
            if (maximum_power < abs(power[count])) maximum_power = abs(power[count]);
      }
      for (uint8_t count = 0; count < MOTOR_NUM; count++) power[count] *= float(move_speed) / maximum_power;

      // PD姿勢制御
      p = robot_angle - yaw;   // 比例
      if (d_timer.read() > D_PERIODO) {
            d = p - pre_p;   // 微分
            pre_p = p;
            d_timer.reset();
      }
      
      if(robot_angle == 0){
            pd = p * KP + d * KD;
      } else {
            pd = p * KP * 2 + d * KD;
      }
      if (abs(pd) > PD_LIMIT) pd = PD_LIMIT * (abs(pd) / pd);

      if (moving_average_count == MOVING_AVERAGE_COUNT_NUMBER) moving_average_count = 0;
      for (uint8_t count = 0; count < MOTOR_NUM; count++) {
            if (robot_angle == 0) {
                  power[count] += count < 2 ? -pd : pd;
            } else {
                  power[count] += count < 2 && (count == 1 || count == 2) ? -pd : pd;
            }
            if (abs(power[count]) > POWER_LIMIT) power[count] = POWER_LIMIT * (abs(power[count]) / power[count]);   // モーターの上限値超えた場合の修正

            tmp_power[count][moving_average_count] = power[count];
            power[count] = 0;
            for (uint8_t count_1 = 0; count_1 < MOVING_AVERAGE_COUNT_NUMBER; count_1++) power[count] += tmp_power[count][count_1];
            power[count] /= MOVING_AVERAGE_COUNT_NUMBER;
      }
      moving_average_count++;

      motor_1_1 = abs(power[0]) < MIN_BRAKE ? 1 : (power[0] > 0 ? power[0] * 0.01000 : 0);
      motor_1_2 = abs(power[0]) < MIN_BRAKE ? 1 : (power[0] < 0 ? power[0] * -0.01000 : 0);
      motor_2_1 = abs(power[1]) < MIN_BRAKE ? 1 : (power[1] > 0 ? power[1] * 0.01000 : 0);
      motor_2_2 = abs(power[1]) < MIN_BRAKE ? 1 : (power[1] < 0 ? power[1] * -0.01000 : 0);
      motor_3_1 = abs(power[2]) < MIN_BRAKE ? 1 : (power[2] > 0 ? power[2] * 0.01000 : 0);
      motor_3_2 = abs(power[2]) < MIN_BRAKE ? 1 : (power[2] < 0 ? power[2] * -0.01000 : 0);
      motor_4_1 = abs(power[3]) < MIN_BRAKE ? 1 : (power[3] > 0 ? power[3] * 0.01000 : 0);
      motor_4_2 = abs(power[3]) < MIN_BRAKE ? 1 : (power[3] < 0 ? power[3] * -0.01000 : 0);
}

void motor::set_pwm() {
      motor_1_1.period_us(MOTOR_FREQUENCY);
      motor_1_2.period_us(MOTOR_FREQUENCY);
      motor_2_1.period_us(MOTOR_FREQUENCY);
      motor_2_2.period_us(MOTOR_FREQUENCY);
      motor_3_1.period_us(MOTOR_FREQUENCY);
      motor_3_2.period_us(MOTOR_FREQUENCY);
      motor_4_1.period_us(MOTOR_FREQUENCY);
      motor_4_2.period_us(MOTOR_FREQUENCY);
}

void motor::brake(uint16_t brake_time) {
      motor_1_1 = 1;
      motor_1_2 = 1;
      motor_2_1 = 1;
      motor_2_2 = 1;
      motor_3_1 = 1;
      motor_3_2 = 1;
      motor_4_1 = 1;
      motor_4_2 = 1;
      wait_us(brake_time * 1000);
}

void motor::free() {
      motor_1_1 = 0;
      motor_1_2 = 0;
      motor_2_1 = 0;
      motor_2_2 = 0;
      motor_3_1 = 0;
      motor_3_2 = 0;
      motor_4_1 = 0;
      motor_4_2 = 0;
}

int16_t motor::motor_1() {
      return power[0];
}
int16_t motor::motor_2() {
      return power[1];
}
int16_t motor::motor_3() {
      return power[2];
}
int16_t motor::motor_4() {
      return power[3];
}
int16_t motor::move_angle() {
      return angle;
}
int16_t motor::move_speed() {
      return speed;
}