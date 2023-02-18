#include <mbed.h>

#include "dribbler.h"
#include "line.h"
#include "motor.h"

#define PI 3.1415926535   // 円周率

motor Motor(PA_10, PB_3, PB_5, PB_4, PA_8, PB_10, PA_5, PA_9);   //  45度、135度、225度、315度
line Line(PC_9, PC_1, PB_0, PC_2, PC_3, PC_0, PA_1, PA_0, PA_4, PC_5, PB_1, PC_4);   // ledピン、前ライン、右ライン、後ライン、左ライン
dribbler Dribbler(PC_8, PC_6);

Serial arduino_ir(PC_10, PC_11);   // TX, RX
Serial arduino_imu(PC_12, PD_2);   // TX, RX
Serial arduino_ui(PA_2, PA_3);   // TX, RX

void line_move(uint8_t* line_tf, int16_t line_move_speed, int16_t move_speed);

int16_t yaw, set_yaw;

void ir_getc();
void imu_getc();
void ui_getc();

Timer line_timer;

int main() {
      Line.led(1);
      Line.set();

      uint8_t mode;

      uint8_t line_tf;
      int16_t move_speed = 50, line_move_speed = 80;

      arduino_ir.baud(28800);
      arduino_imu.baud(28800);
      arduino_ui.baud(9600);

      arduino_ir.attach(ir_getc, Serial::RxIrq);
      arduino_imu.attach(imu_getc, Serial::RxIrq);
      arduino_ui.attach(ui_getc, Serial::RxIrq);

      while (1) {
            Motor.yaw = yaw;
            Line.read();

            if (mode == 0) {
                  line_move(&line_tf, line_move_speed, move_speed);
                  if (line_tf == 0) {
                        Motor.run(90, move_speed);
                  }
            } else if (mode == 1) {
            } else if (mode == 2) {
            } else if (mode == 3) {
            }
      }
}

void ir_getc() {   // IMU情報の取得
      if (arduino_ir.getc() == 'a') {
      }
}

void imu_getc() {   // IMU情報の取得
      if (arduino_imu.getc() == 'a') {
            int16_t yaw_plus, yaw_minus;
            yaw_plus = arduino_imu.getc();
            yaw_minus = arduino_imu.getc();
            yaw = (yaw_plus == 0 ? yaw_minus * -1 : yaw_plus) - set_yaw;
            yaw -= yaw > 180 ? 360 : (yaw < -180 ? -360 : 0);
      }
}

void ui_getc() {   // IMU情報の取得
      if (arduino_ui.getc() == 'a') {
      }
}

void line_move(uint8_t* line_tf, int16_t line_move_speed, int16_t move_speed) {
      bool line_tf_x_unit[2], line_tf_y_unit[2];
      static int16_t tmp_move_angle;
      static uint8_t line_tf_x, line_tf_y;
      static int16_t line_result_vector_x, line_result_vector_y, pre_line_result_vector_x, pre_line_result_vector_y, line_back_angle;

      if (Line.check_all() == 1) {
            line_timer.start();
            line_timer.reset();
            if (*line_tf == 0) Motor.brake(10);

            if (Line.check_right() == 1 && line_tf_x <= 2) line_tf_x = 1;
            if (Line.check(4) && line_tf_x == 1) line_tf_x = 3;
            if (Line.check_left() == 1 && line_tf_x <= 2) line_tf_x = 2;
            if (Line.check(10) && line_tf_x == 2) line_tf_x = 4;
            if (Line.check_front() == 1 && line_tf_y <= 2) line_tf_y = 1;
            if (Line.check(1) && line_tf_y == 1) line_tf_y = 3;
            if (Line.check_back() == 1 && line_tf_y <= 2) line_tf_y = 2;
            if (Line.check(7) && line_tf_y == 2) line_tf_y = 4;
            if ((Line.check(0) == 1 && Line.check_back() == 1) || (Line.check(5) == 1 && Line.check_front() == 1)) line_tf_y = 0;
            if ((Line.check(2) == 1 && Line.check_left() == 1) || (Line.check(8) == 1 && Line.check_right() == 1)) line_tf_x = 0;
      }
      for (uint8_t count = 0; count < 2; count++) {
            line_tf_x_unit[count] = line_tf_x == count + 1 || line_tf_x == count + 3 ? 1 : 0;
            line_tf_y_unit[count] = line_tf_y == count + 1 || line_tf_y == count + 3 ? 1 : 0;
      }

      if (*line_tf == 0) {
            if (line_tf_y_unit[0] == 1) *line_tf = 1;
            if (line_tf_x_unit[0] == 1) *line_tf = 2;
            if (line_tf_y_unit[1] == 1) *line_tf = 3;
            if (line_tf_x_unit[1] == 1) *line_tf = 4;
      }

      if (*line_tf != 0) {
            if (Line.check_all() == 1) {
                  line_result_vector_x = Line.check_right() + Line.check_left() * -1;
                  line_result_vector_y = Line.check_front() + Line.check_back() * -1;
                  if (pre_line_result_vector_x != line_result_vector_x || pre_line_result_vector_y != line_result_vector_y) line_back_angle = atan2(line_result_vector_x, line_result_vector_y) / PI * 180.000 + 180.5;
                  pre_line_result_vector_x = line_result_vector_x;
                  pre_line_result_vector_y = line_result_vector_y;
                  if (line_back_angle > 180) line_back_angle -= 360;

                  if (line_tf_x_unit[0] == 1 && line_tf_y_unit[0] == 1) {
                        tmp_move_angle = line_back_angle > 135 || line_back_angle < -45 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_x_unit[0] == 1 && line_tf_y_unit[1] == 1) {
                        tmp_move_angle = line_back_angle > -135 && line_back_angle < 45 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_x_unit[1] == 1 && line_tf_y_unit[1] == 1) {
                        tmp_move_angle = line_back_angle > -45 && line_back_angle < 135 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_x_unit[1] == 1 && line_tf_y_unit[0] == 1) {
                        tmp_move_angle = line_back_angle > 45 || line_back_angle < -135 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_y_unit[0]) {
                        tmp_move_angle = line_back_angle < -90 || line_back_angle > 90 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_y_unit[1]) {
                        tmp_move_angle = line_back_angle > -90 && line_back_angle < 90 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_x_unit[0]) {
                        tmp_move_angle = line_back_angle > -180 && line_back_angle < 0 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_x_unit[1]) {
                        tmp_move_angle = line_back_angle > 0 && line_back_angle < 180 ? line_back_angle : line_back_angle - 180;
                  }
                  Motor.run(tmp_move_angle,line_move_speed);

            } else {
                  if (line_timer.read() > 0) {
                        line_timer.stop();
                        line_timer.reset();
                        line_tf_x = 0;
                        line_tf_y = 0;
                        *line_tf = 0;
                  } else {
                        Motor.brake(50);
                  }
            }
      }
}