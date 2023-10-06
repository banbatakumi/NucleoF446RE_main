#include "motor.h"

#include "mbed.h"

Motor::Motor(PinName motor_1_a_, PinName motor_1_b_, PinName motor_2_a_, PinName motor_2_b_, PinName motor_3_a_, PinName motor_3_b_, PinName motor_4_a_, PinName motor_4_b_) : motor_1_a(motor_1_a_), motor_1_b(motor_1_b_), motor_2_a(motor_2_a_), motor_2_b(motor_2_b_), motor_3_a(motor_3_a_), motor_3_b(motor_3_b_), motor_4_a(motor_4_a_), motor_4_b(motor_4_b_) {
      motor_1_a = 0;
      motor_1_b = 0;
      motor_2_a = 0;
      motor_2_b = 0;
      motor_3_a = 0;
      motor_3_b = 0;
      motor_4_a = 0;
      motor_4_b = 0;

      motor1Ave.SetLength(MOVING_AVG_LENGTH);
      motor2Ave.SetLength(MOVING_AVG_LENGTH);
      motor3Ave.SetLength(MOVING_AVG_LENGTH);
      motor4Ave.SetLength(MOVING_AVG_LENGTH);

      attitudeControlPID.SetGain(KP, KI, KD);
      attitudeControlPID.SetSamplingPeriod(PID_SAMPLING_PERIOD);
      attitudeControlPID.SelectType(PI_D_TYPE);

      d_timer.start();
}

void Motor::Run(int16_t moving_dir_, uint8_t moving_speed_, int16_t robot_angle_, uint8_t robot_angle_mode_, uint8_t pid_limit_) {
      int16_t power[MOTOR_QTY];
      uint8_t moving_speed = moving_speed_;

      if (moving_speed > POWER_LIMIT) moving_speed = POWER_LIMIT;

      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            power[i] = MySin(moving_dir_ - (45 + i * 90)) * moving_speed * (i < 2 ? -1 : 1);   // 角度とスピードを各モーターの値に変更
      }

      // モーターの最大パフォーマンス発揮
      uint8_t max_power = 0;
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            if (max_power < abs(power[i])) {
                  max_power = abs(power[i]);
            }
      }
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            power[i] *= float(moving_speed_) / max_power;
      }

      // PID姿勢制御
      attitudeControlPID.Compute(yaw, robot_angle_);
      attitudeControlPID.SetLimit(pid_limit_);
      float tmp_pid = attitudeControlPID.Get();

      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            // ボールを捕捉しながら回転するために姿勢制御を与えるモーターを制限
            if (robot_angle_mode_ == 0) {
                  power[i] += i < 2 ? tmp_pid * -1 : tmp_pid;
            } else if (robot_angle_mode_ == 1) {   // ボールを前に捕捉した状態
                  if (i == 1 || i == 2) {
                        power[i] += i < 2 ? tmp_pid * -2 : tmp_pid * 2;
                  }
            } else if (robot_angle_mode_ == 2) {   // ボールを右に捕捉した状態
                  if (i == 2 || i == 3) {
                        power[i] += i < 2 ? tmp_pid * -2 : tmp_pid * 2;
                  }
            } else if (robot_angle_mode_ == 3) {   // ボールを後ろに捕捉した状態
                  if (i == 0 || i == 3) {
                        power[i] += i < 2 ? tmp_pid * -2 : tmp_pid * 2;
                  }
            } else if (robot_angle_mode_ == 4) {   // ボールを左に捕捉した状態
                  if (i == 0 || i == 1) {
                        power[i] += i < 2 ? tmp_pid * -2 : tmp_pid * 2;
                  }
            }

            // モーターの上限値を超えた場合の修正
            if (abs(power[i]) > POWER_LIMIT) {
                  power[i] = POWER_LIMIT * (abs(power[i]) / power[i]);
            }
      }

      // 移動平均フィルタ
      motor1Ave.Compute(&power[0]);
      motor2Ave.Compute(&power[1]);
      motor3Ave.Compute(&power[2]);
      motor4Ave.Compute(&power[3]);

      // モーターへ出力
      motor_1_a = abs(power[0]) < MIN_BRAKE ? 1 : (power[0] > 0 ? power[0] * 0.01000 : 0);
      motor_1_b = abs(power[0]) < MIN_BRAKE ? 1 : (power[0] < 0 ? power[0] * -0.01000 : 0);
      motor_2_a = abs(power[1]) < MIN_BRAKE ? 1 : (power[1] > 0 ? power[1] * 0.01000 : 0);
      motor_2_b = abs(power[1]) < MIN_BRAKE ? 1 : (power[1] < 0 ? power[1] * -0.01000 : 0);
      motor_3_a = abs(power[2]) < MIN_BRAKE ? 1 : (power[2] > 0 ? power[2] * 0.01000 : 0);
      motor_3_b = abs(power[2]) < MIN_BRAKE ? 1 : (power[2] < 0 ? power[2] * -0.01000 : 0);
      motor_4_a = abs(power[3]) < MIN_BRAKE ? 1 : (power[3] > 0 ? power[3] * 0.01000 : 0);
      motor_4_b = abs(power[3]) < MIN_BRAKE ? 1 : (power[3] < 0 ? power[3] * -0.01000 : 0);
}

void Motor::SetPwmPeriod(uint16_t pwm_period_) {
      motor_1_a.period_us(pwm_period_);
      motor_1_b.period_us(pwm_period_);
      motor_2_a.period_us(pwm_period_);
      motor_2_b.period_us(pwm_period_);
      motor_3_a.period_us(pwm_period_);
      motor_3_b.period_us(pwm_period_);
      motor_4_a.period_us(pwm_period_);
      motor_4_b.period_us(pwm_period_);
}

void Motor::Brake(uint16_t brake_time_) {
      motor_1_a = 1;
      motor_1_b = 1;
      motor_2_a = 1;
      motor_2_b = 1;
      motor_3_a = 1;
      motor_3_b = 1;
      motor_4_a = 1;
      motor_4_b = 1;
      wait_us(brake_time_ * 1000);
}

void Motor::Free() {
      motor_1_a = 0;
      motor_1_b = 0;
      motor_2_a = 0;
      motor_2_b = 0;
      motor_3_a = 0;
      motor_3_b = 0;
      motor_4_a = 0;
      motor_4_b = 0;
}