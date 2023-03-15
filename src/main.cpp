#include <mbed.h>

#include "ball_catch.h"
#include "dribbler.h"
#include "hcsr04.h"
#include "line.h"
#include "motor.h"
#include "voltage.h"

HCSR04 usensor(PB_13, PB_7);

#define PI 3.1415926535   // 円周率

#define BALL_FOLLOW_RANGE 55.000   // 回り込み時にボールを追い始める角度
#define BALL_FOLLOW_DEPTH_AWAY 30.000
#define BALL_FOLLOW_DEPTH_NEAR 85.000

#define LINE_BALL_P 2.5
#define LINE_BALL_WAIT_TIME 3
#define LINE_BRAKE_TIME 50

#define SERIAL_BAUD 38400

#define CAM_CENTER 106

motor Motor(PA_10, PB_3, PB_5, PB_10, PA_9, PA_8, PA_5, PB_6);   //  45度、135度、225度、315度
line Line(PC_9, PC_1, PB_0, PC_2, PC_3, PC_0, PA_1, PA_0, PA_4, PC_5, PB_1, PC_4);   // ledピン、前ライン、右ライン、後ライン、左ライン
dribbler Dribbler(PC_8, PC_6);
ball_catch Ball_catch(PB_9, PB_8);
voltage Voltage(PA_6, PA_7);

Serial arduino_ir(PC_10, PC_11);   // TX, RX
Serial arduino_imu(PC_12, PD_2);   // TX, RX
Serial arduino_ui(PA_2, PA_3);   // TX, RX

void line_move(uint8_t* line_tf, int16_t line_move_speed, int16_t move_speed);

void offence_move();
void diffence_move();

int16_t ball_angle, ball_distance;
void ir();

int16_t yaw, set_yaw, goal_angle = 0, goal_wide = 0, goal_angle_mode = 0, imu_validation = 0;
void imu();

uint8_t mode = 0, move_speed = 0, line_move_speed = 0;
void ui();

Timer line_timer;

Timer dribbler_timer;

int main() {
      Line.led(1);
      Line.set();

      uint8_t line_tf = 0;

      arduino_ir.baud(SERIAL_BAUD);
      arduino_imu.baud(SERIAL_BAUD);
      arduino_ui.baud(SERIAL_BAUD);

      arduino_ir.attach(ir, Serial::RxIrq);
      arduino_imu.attach(imu, Serial::RxIrq);
      arduino_ui.attach(ui, Serial::RxIrq);

      while (1) {
            Motor.yaw = yaw;
            Line.read();
            Ball_catch.read();
            Voltage.read();
            usensor.start();

            if (mode == 0) {
                  Motor.free();
            } else if (mode == 1) {
                  line_move(&line_tf, line_move_speed, move_speed);

                  if (line_tf == 0) {
                        if (ball_distance == 0) {
                              if (goal_angle_mode == 0 || goal_angle == 0) {
                                    Motor.run(usensor.get_dist_cm() > 100 ? 180 : 0, abs(100 - usensor.get_dist_cm()) * 2);
                              } else {
                                    Motor.run(usensor.get_dist_cm() > 100 ? (goal_angle > 0 ? 135 : -135) : (goal_angle > 0 ? 45 : -45), abs(100 - usensor.get_dist_cm()) * 2 + abs(goal_angle) * 2);
                              }
                        } else {
                              offence_move();
                        }
                  }
            } else if (mode == 2) {
                  diffence_move();
            } else if (mode == 3) {
                  Motor.run(0, 0);
            } else if (mode == 4) {
                  line_move(&line_tf, line_move_speed, move_speed);

                  if (line_tf == 0) {
                        if (ball_distance == 0) {
                              Motor.run(0, 0);
                        } else {
                              Motor.run(ball_angle, move_speed);
                        }
                  }
            }
      }
}

void ir() {
      if (arduino_ir.getc() == 'a') {
            uint8_t ball_angle_plus, ball_angle_minus;
            ball_angle_plus = arduino_ir.getc();
            ball_angle_minus = arduino_ir.getc();
            ball_distance = arduino_ir.getc();

            ball_angle = ball_angle_plus == 0 ? ball_angle_minus * -1 : ball_angle_plus;
            // ball_angle += yaw;
            ball_angle -= ball_angle > 180 ? 360 : (ball_angle < -180 ? -360 : 0);
      }
}

void imu() {
      if (arduino_imu.getc() == 'a') {
            imu_validation = 1;
            uint8_t yaw_plus, yaw_minus, yellow_angle, yellow_wide, blue_angle, blue_wide;
            yaw_plus = arduino_imu.getc();
            yaw_minus = arduino_imu.getc();
            yellow_angle = arduino_imu.getc();
            yellow_wide = arduino_imu.getc();
            blue_angle = arduino_imu.getc();
            blue_wide = arduino_imu.getc();

            yaw = (yaw_plus == 0 ? yaw_minus * -1 : yaw_plus) - set_yaw;
            yaw -= yaw > 180 ? 360 : (yaw < -180 ? -360 : 0);

            goal_angle = 0;
            goal_wide = 0;
            if (goal_angle_mode == 1) goal_angle = yellow_angle - CAM_CENTER;
            if (goal_angle_mode == 2) goal_angle = blue_angle - CAM_CENTER;
            if (goal_angle == CAM_CENTER * -1) goal_angle = 0;
            if (goal_angle_mode == 1) goal_wide = yellow_wide;
            if (goal_angle_mode == 2) goal_wide = blue_wide;
      }
}

void ui() {
      uint8_t display_mode;

      if (arduino_ui.getc() == 'a') {
            display_mode = arduino_ui.getc();
            if (display_mode == 0) {
                  mode = arduino_ui.getc();
                  move_speed = arduino_ui.getc();
                  line_move_speed = arduino_ui.getc();
                  goal_angle_mode = arduino_ui.getc();

                  if (mode == 0) {
                        arduino_ui.putc('a');
                        arduino_ui.putc(imu_validation);
                  }
            } else if (display_mode == 1) {
                  uint8_t yaw_plus, yaw_minus;

                  if (arduino_ui.getc() == 1) set_yaw = yaw + set_yaw;

                  yaw_plus = yaw > 0 ? yaw : 0;
                  yaw_minus = yaw < 0 ? yaw * -1 : 0;

                  arduino_ui.putc('a');
                  arduino_ui.putc(yaw_plus);
                  arduino_ui.putc(yaw_minus);
            } else if (display_mode == 2) {
                  move_speed = arduino_ui.getc();
                  line_move_speed = arduino_ui.getc();
            } else if (display_mode == 3) {
                  if (arduino_ui.getc() == 1) Line.set();

                  arduino_ui.putc('a');
                  arduino_ui.putc(Line.check(0));
                  arduino_ui.putc(Line.check(1));
                  arduino_ui.putc(Line.check(2));
                  arduino_ui.putc(Line.check(3));
                  arduino_ui.putc(Line.check(4));
                  arduino_ui.putc(Line.check(5));
                  arduino_ui.putc(Line.check(6));
                  arduino_ui.putc(Line.check(7));
                  arduino_ui.putc(Line.check(8));
                  arduino_ui.putc(Line.check(9));
                  arduino_ui.putc(Line.check(10));
            } else if (display_mode == 4) {
                  uint8_t ball_angle_plus, ball_angle_minus;

                  ball_angle_plus = ball_angle > 0 ? ball_angle : 0;
                  ball_angle_minus = ball_angle < 0 ? ball_angle * -1 : 0;

                  arduino_ui.putc('a');
                  arduino_ui.putc(ball_angle_plus);
                  arduino_ui.putc(ball_angle_minus);
                  arduino_ui.putc(ball_distance);
            } else if (display_mode == 5) {
                  uint8_t dribbler_test;

                  dribbler_test = arduino_ui.getc();

                  if (dribbler_test == 0) Dribbler.stop();
                  if (dribbler_test == 1) Dribbler.hold();
                  if (dribbler_test == 2) Dribbler.kick();
            } else if (display_mode == 6) {
                  arduino_ui.putc('a');
                  arduino_ui.putc(Ball_catch.get());
                  arduino_ui.putc(Ball_catch.get_left());
                  arduino_ui.putc(Ball_catch.get_right());
            } else if (display_mode == 7) {
                  goal_angle_mode = arduino_ui.getc();

                  arduino_ui.putc('a');
                  arduino_ui.putc(goal_angle + CAM_CENTER);
                  arduino_ui.putc(goal_wide);
            } else if (display_mode == 8) {
                  goal_angle_mode = arduino_ui.getc();

                  arduino_ui.putc('a');
                  arduino_ui.putc(usensor.get_dist_cm());
            }
      }
}
void offence_move() {
      int16_t tmp_move_speed, tmp_move_angle, robot_angle = 0;
      float move_angle_decrement;

      // 方向

      if (Ball_catch.get() > 50) {
            tmp_move_angle = Ball_catch.get_right() - Ball_catch.get_left();
      } else if (abs(ball_angle) <= BALL_FOLLOW_RANGE) {
            tmp_move_angle = ball_angle + (90.000 / BALL_FOLLOW_RANGE * ball_angle);
      } else {
            move_angle_decrement = ((ball_distance < BALL_FOLLOW_DEPTH_AWAY ? BALL_FOLLOW_DEPTH_AWAY : ball_distance) - BALL_FOLLOW_DEPTH_AWAY) / float(BALL_FOLLOW_DEPTH_NEAR - BALL_FOLLOW_DEPTH_AWAY);
            tmp_move_angle = (ball_angle > 0 ? 90 : -90) + (ball_angle * move_angle_decrement);
      }
      if (tmp_move_angle > 180) tmp_move_angle -= 360;
      if (tmp_move_angle < -180) tmp_move_angle += 360;

      // 速度
      /*
            if (Ball_catch.get() > 50) {
                  tmp_move_speed = move_speed;
            } else if (abs(ball_angle) <= 90) {
                  tmp_move_speed = ((move_speed - 30) * (abs(ball_angle) / 90.000)) + 30 + (100 - ball_distance) * 1.5;
            } else {
                  tmp_move_speed = move_speed;
            }*/

      if (abs(ball_angle) <= 90) {
            tmp_move_speed = ((move_speed - 30) * (abs(ball_angle) / 90.000)) + 30 + (100 - ball_distance) * 1.5;
      } else {
            tmp_move_speed = move_speed;
      }

      if (Ball_catch.get() > 50) {
            if (Ball_catch.get() > 60) dribbler_timer.start();
            if (dribbler_timer.read() < 0.5) Dribbler.hold();
      }

      if (dribbler_timer.read() > 0.25) {
            if (goal_angle_mode == 0 || goal_angle == 0) {
                  tmp_move_angle = 0;
            } else {
                  tmp_move_angle += goal_angle / 1.5 > 40 ? (goal_angle > 0 ? 40 : -40) : goal_angle / 1.5;
            }

            tmp_move_speed = move_speed;
            if (abs(goal_angle) > 40) {
                  dribbler_timer.stop();
            } else {
                  dribbler_timer.start();
            }

            if (dribbler_timer.read() > 0.5) {
                  Dribbler.kick();
            }
            if (dribbler_timer.read() > 0.7) {
                  tmp_move_speed = 0;
            }
      }

      if (dribbler_timer.read() > 1.5 || abs(ball_angle) > 60) {
            dribbler_timer.reset();
            dribbler_timer.stop();
            robot_angle = 0;
      }

      if (tmp_move_speed > move_speed) tmp_move_speed = move_speed;
      Motor.run(tmp_move_angle, tmp_move_speed, robot_angle);   // 回り込み
}

void diffence_move() {
      int16_t tmp_move_speed, tmp_move_angle;

      tmp_move_speed = abs(ball_angle) * 3;

      if (Line.check_front() && Line.check_back()) {
            tmp_move_angle = abs(ball_angle) > 90 ? 180 : 0;
      } else if (Line.check_right() && Line.check_left()) {
            tmp_move_angle = ball_angle > 0 ? 90 : -90;
      } else if (Line.check_front()) {
            tmp_move_angle = ball_angle > 0 ? 30 : -30;
      } else if (Line.check_back()) {
            tmp_move_angle = ball_angle > 0 ? 150 : -150;
      } else if (Line.check_right()) {
            tmp_move_angle = abs(ball_angle) > 90 ? 150 : 30;
      } else if (Line.check_left()) {
            tmp_move_angle = abs(ball_angle) > 90 ? -150 : -30;
      } else {
            tmp_move_angle = ball_angle > 0 ? 120 : -120;
      }

      if (tmp_move_speed > move_speed) tmp_move_speed = move_speed;

      Motor.run(tmp_move_angle, tmp_move_speed);   // 回り込み
}

void line_move(uint8_t* line_tf, int16_t line_move_speed, int16_t move_speed) {
      bool line_tf_x_unit[2], line_tf_y_unit[2];
      static int16_t tmp_move_angle, tmp_move_speed;
      static uint8_t line_tf_x, line_tf_y;
      static int16_t line_result_vector_x, line_result_vector_y, pre_line_result_vector_x, pre_line_result_vector_y, line_back_angle, line_wait;

      if (Line.check_front() && Ball_catch.get() > 50) {
            Dribbler.kick();
      } else if (Ball_catch.get() > 50) {
            Dribbler.hold();
      } else {
            Dribbler.stop();
      }

      if (Line.check_all()) {
            line_timer.start();
            line_timer.reset();
            if (*line_tf == 0) Motor.brake(LINE_BRAKE_TIME);

            if (Line.check_right() && line_tf_x <= 2) line_tf_x = 1;
            if (Line.check(4) && line_tf_x == 1) line_tf_x = 3;
            if (Line.check_left() && line_tf_x <= 2) line_tf_x = 2;
            if (Line.check(10) && line_tf_x == 2) line_tf_x = 4;
            if (Line.check_front() && line_tf_y <= 2) line_tf_y = 1;
            if (Line.check(1) && line_tf_y == 1) line_tf_y = 3;
            if (Line.check_back() && line_tf_y <= 2) line_tf_y = 2;
            if (Line.check(7) && line_tf_y == 2) line_tf_y = 4;
            if (Line.check(5) && Line.check_front()) line_tf_y = 0;
            if ((Line.check(2) && Line.check_left()) || (Line.check(8) && Line.check_right())) line_tf_x = 0;
      }
      for (uint8_t count = 0; count < 2; count++) {
            line_tf_x_unit[count] = line_tf_x == count + 1 || line_tf_x == count + 3 ? 1 : 0;
            line_tf_y_unit[count] = line_tf_y == count + 1 || line_tf_y == count + 3 ? 1 : 0;
      }

      if (*line_tf == 0) {
            line_wait = 0;
            if (line_tf_y_unit[0]) *line_tf = 1;
            if (line_tf_x_unit[0]) *line_tf = 2;
            if (line_tf_y_unit[1]) *line_tf = 3;
            if (line_tf_x_unit[1]) *line_tf = 4;
      }

      if (*line_tf != 0) {
            if (Line.check_all() == 1) {
                  line_result_vector_x = Line.check_right() + Line.check_left() * -1;
                  line_result_vector_y = Line.check_front() + Line.check_back() * -1;
                  if (pre_line_result_vector_x != line_result_vector_x || pre_line_result_vector_y != line_result_vector_y) line_back_angle = atan2(line_result_vector_x, line_result_vector_y) / PI * 180.000 + 180.5;
                  pre_line_result_vector_x = line_result_vector_x;
                  pre_line_result_vector_y = line_result_vector_y;
                  if (line_back_angle > 180) line_back_angle -= 360;

                  if (line_tf_x_unit[0] && line_tf_y_unit[0]) {
                        tmp_move_angle = line_back_angle > 135 || line_back_angle < -45 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_x_unit[0] && line_tf_y_unit[1]) {
                        tmp_move_angle = line_back_angle > -135 && line_back_angle < 45 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_x_unit[1] && line_tf_y_unit[1]) {
                        tmp_move_angle = line_back_angle > -45 && line_back_angle < 135 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_x_unit[1] && line_tf_y_unit[0]) {
                        tmp_move_angle = line_back_angle > 45 || line_back_angle < -135 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_y_unit[0]) {
                        tmp_move_angle = line_back_angle < -90 || line_back_angle > 90 ? line_back_angle : line_back_angle - 180;
                  } else if (line_tf_y_unit[1]) {
                        tmp_move_angle = line_back_angle > -90 && line_back_angle < 90 ? line_back_angle : line_back_angle - 180;
                        tmp_move_angle += ball_angle > 0 ? 30 : -30;
                  } else if (line_tf_x_unit[0]) {
                        tmp_move_angle = line_back_angle > -180 && line_back_angle < 0 ? line_back_angle : line_back_angle - 180;
                        tmp_move_angle += abs(ball_angle) > 90 ? -30 : 30;
                  } else if (line_tf_x_unit[1]) {
                        tmp_move_angle = line_back_angle > 0 && line_back_angle < 180 ? line_back_angle : line_back_angle - 180;
                        tmp_move_angle += abs(ball_angle) > 90 ? 30 : -30;
                  }
                  Motor.run(tmp_move_angle, line_move_speed);

            } else {
                  if (line_timer.read() > 0.1) {
                        line_tf_x = 0;
                        line_tf_y = 0;
                        if (line_timer.read() > LINE_BALL_WAIT_TIME) {
                              line_timer.stop();
                              line_timer.reset();
                              *line_tf = 0;
                        }

                        if (*line_tf == 1 && abs(ball_angle) < 45) {
                              tmp_move_angle = ball_angle > 0 ? 70 : -70;
                              tmp_move_speed = abs(ball_angle) * LINE_BALL_P;
                        } else if (*line_tf == 2 && ball_angle > 15) {
                              if (ball_distance > 75) {
                                    tmp_move_angle = abs(ball_angle) < 45 ? 20 : 180;
                                    tmp_move_speed = abs(abs(ball_angle) - 45) * LINE_BALL_P;
                              } else {
                                    tmp_move_angle = ball_angle - 45;
                                    tmp_move_speed = move_speed;
                              }
                        } else if (*line_tf == 3 && abs(ball_angle) > 15) {
                              if (abs(ball_angle) > 120) {
                                    tmp_move_speed = 0;
                              } else if (abs(ball_angle) > 60) {
                                    tmp_move_angle = ball_angle > 0 ? 120 : -120;
                                    tmp_move_speed = move_speed;
                              } else {
                                    tmp_move_angle = ball_angle > 0 ? 90 : -90;
                                    tmp_move_speed = move_speed;
                              }
                        } else if (*line_tf == 4 && ball_angle < -15) {
                              if (ball_distance > 75) {
                                    tmp_move_angle = abs(ball_angle) < 45 ? -20 : 180;
                                    tmp_move_speed = abs(abs(ball_angle) - 45) * LINE_BALL_P;
                              } else {
                                    tmp_move_angle = ball_angle + 45;
                                    tmp_move_speed = move_speed;
                              }
                        } else {
                              line_timer.stop();
                              line_timer.reset();
                              *line_tf = 0;
                        }

                        if (tmp_move_speed > move_speed) tmp_move_speed = move_speed;
                        Motor.run(tmp_move_angle, tmp_move_speed);
                  } else {
                        Motor.brake();
                  }
            }
      }
}