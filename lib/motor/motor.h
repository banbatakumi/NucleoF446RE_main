#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"
#include "moving_ave.h"
#include "pid.h"
#include "simplify_deg.h"
#include "sin_cos_table.h"

#define PI 3.1415926535   // 円周率

#define MIN_BRAKE 5   // モーターの最小値ブレーキ
#define POWER_LIMIT 75   // モーターの最大パワー

#define KP 1.000   // 姿勢制御比例ゲイン
#define KI 0   // 姿制御積分ゲイン
#define KD 0.100   // 姿制御微分ゲイン
#define PID_SAMPLING_PERIOD 0.01

#define MOVING_AVG_LENGTH 10   // 移動平均フィルタの回数

#define MOTOR_QTY 4
class Motor {
     public:
      Motor(PinName motor_1_a_, PinName motor_1_b_, PinName motor_2_a_, PinName motor_2_b_, PinName motor_3_a_, PinName motor_3_b_, PinName motor_4_a_, PinName motor_4_b_);
      void Run(int16_t moving_dir_ = 0, uint8_t moving_speed_ = 0, int16_t robot_angle_ = 0, uint8_t robot_angle_mode_ = 0, uint8_t pid_limit_ = POWER_LIMIT);
      void SetPwmPeriod(uint16_t pwm_period_ = 30000);
      void Brake(uint16_t brake_time_ = 0);
      void Free();

#define FRONT 1
#define RIGHT 2
#define BACK 3
#define LEFT 4

      int16_t yaw;

     private:
      MovingAve motor1Ave;
      MovingAve motor2Ave;
      MovingAve motor3Ave;
      MovingAve motor4Ave;

      PID attitudeControlPID;

      PwmOut motor_1_a;
      PwmOut motor_1_b;
      PwmOut motor_2_a;
      PwmOut motor_2_b;
      PwmOut motor_3_a;
      PwmOut motor_3_b;
      PwmOut motor_4_a;
      PwmOut motor_4_b;

      Timer d_timer;
};

#endif