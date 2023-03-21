#include <mbed.h>

#include "ball_catch.h"
#include "dribbler.h"
#include "hcsr04.h"
#include "line.h"
#include "motor.h"
#include "voltage.h"

#define PI 3.1415926535   // 円周率

#define BALL_FOLLOW_RANGE 125   // 回り込み時にボールを追い始める角度(標準値:130)
#define BALL_FOLLOW_DEPTH_AWAY -10.000   // ボールが遠い時の回り込みの深さ(標準値:30)
#define BALL_FOLLOW_DEPTH_NEAR 90.000   // ボールが近い時の回り込みの深さ(標準値:90)

#define LINE_BALL_P 2.5   // ライン待機時にボールを追う比例制御のゲイン
#define LINE_DISTANCE_P 1.5
#define LINE_BALL_WAIT_TIME 3   // ライン待機時にボールを追う時間
#define LINE_BRAKE_TIME 10   // ラインを踏んだ時のブレーキ時間(μs)
#define LINE_WAIT_BRAKE_TIME 0.1   // ラインを踏んだ後のブレーキ時間(s)

#define IMU_SERIAL_BAUD 38400   // UART通信速度
#define IR_SERIAL_BAUD 38400   // UART通信速度
#define UI_SERIAL_BAUD 19200   // UART通信速度

#define CAM_CENTER 106   // カメラのセンター合わせ

motor Motor(PA_10, PB_3, PB_5, PB_10, PA_9, PA_8, PA_5, PB_6);   //  45度、135度、225度、315度
line Line(PC_9, PC_1, PB_0, PC_2, PC_3, PC_0, PA_1, PA_0, PA_4, PC_5, PB_1, PC_4);   // ledピン、前ライン、右ライン、後ライン、左ライン
dribbler Dribbler(PC_8, PC_6);
ball_catch Ball_catch(PB_9, PB_8);
voltage Voltage(PA_6, PA_7);
HCSR04 usensor(PB_13, PB_7);   // 超音波センサ

Serial arduino_ir(PC_10, PC_11);   // TX, RX
Serial arduino_imu(PC_12, PD_2);   // TX, RX
Serial arduino_ui(PA_2, PA_3);   // TX, RX

void line_move(uint8_t* line_tf, int16_t line_move_speed, int16_t move_speed);

void offence_move();
void diffence_move();

int16_t ball_angle, ball_distance;
void ir();

int16_t yaw, set_yaw, goal_angle = 0, goal_height = 0, goal_angle_mode = 0, imu_validation = 0;
void imu();

uint8_t mode = 0, move_speed = 70, line_move_speed = 80, catch_move_speed = 70;
void ui();

Timer line_timer;

Timer line_wait_timer;

Timer dribbler_timer;

Timer diffence_timer;

int main() {
      Line.led(1);
      Line.set();

      uint8_t line_tf = 0;

      arduino_ir.baud(IR_SERIAL_BAUD);
      arduino_imu.baud(IMU_SERIAL_BAUD);
      arduino_ui.baud(UI_SERIAL_BAUD);

      arduino_ir.attach(ir, Serial::RxIrq);
      arduino_imu.attach(imu, Serial::RxIrq);
      arduino_ui.attach(ui, Serial::RxIrq);

      while (1) {
            Motor.yaw = yaw;
            Line.read();
            Ball_catch.read();
            Voltage.read();
            usensor.start();

            if (Voltage.get_battery_voltage() < 6) {
            } else {
            }

            if (mode == 0) {
                  Motor.free();
            } else if (mode == 1) {
                  line_move(&line_tf, line_move_speed, move_speed);

                  if (line_tf == 0) {
                        if (ball_distance == 0) {   // ボールない時
                              if (goal_angle_mode == 0 || goal_angle == 0) {   // ゴールが見えていない時
                                    Motor.run(usensor.get() > 80 ? 180 : 0, abs(80 - usensor.get()) * 3);
                              } else {
                                    Motor.run(usensor.get() > 80 ? (goal_angle > 0 ? 135 : -135) : (goal_angle > 0 ? 45 : -45), abs(80 - usensor.get()) * 3 + abs(goal_angle) * 2);
                              }
                        } else {
                              offence_move();
                        }
                  }
            } else if (mode == 2) {
                  diffence_move();
            } else if (mode == 3) {
                  diffence_timer.start();
                  if (diffence_timer.read() > 0.9 && diffence_timer.read() < 1 && !Line.back()) diffence_timer.stop();
                  if(diffence_timer.read() < 1){
                        Dribbler.hold();
                        Motor.run(Line.right() ? -30 : 30, 30, 0);
                  } else if (diffence_timer.read() < 2.5) {
                        int i;
                        i = (diffence_timer.read() - 1) * 150;
                        Motor.run(0, 0, i > 180 ? i - 360 : i);
                  } else if (diffence_timer.read() < 4) {
                        Motor.run(0, 0, 0);
                  } else {
                        diffence_timer.stop();
                        diffence_timer.reset();
                  }
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

void ir() {   // ir用のマイコンとの通信
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

void imu() {   // imu用のマイコンとの通信
      if (arduino_imu.getc() == 'a') {
            imu_validation = 1;
            uint8_t yaw_plus, yaw_minus, yellow_angle, yellow_height, blue_angle, blue_height;
            yaw_plus = arduino_imu.getc();
            yaw_minus = arduino_imu.getc();
            yellow_angle = arduino_imu.getc();
            yellow_height = arduino_imu.getc();
            blue_angle = arduino_imu.getc();
            blue_height = arduino_imu.getc();

            yaw = (yaw_plus == 0 ? yaw_minus * -1 : yaw_plus) - set_yaw;
            yaw -= yaw > 180 ? 360 : (yaw < -180 ? -360 : 0);

            goal_angle = 0;
            goal_height = 0;
            if (goal_angle_mode == 1) goal_angle = yellow_angle - CAM_CENTER;
            if (goal_angle_mode == 2) goal_angle = blue_angle - CAM_CENTER;
            if (goal_angle == CAM_CENTER * -1) goal_angle = 0;
            if (goal_angle_mode == 1) goal_height = yellow_height;
            if (goal_angle_mode == 2) goal_height = blue_height;
      }
}

void ui() {   // ui用のマイコンとの通信
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
                  catch_move_speed = arduino_ui.getc();
            } else if (display_mode == 3) {
                  if (arduino_ui.getc() == 1) Line.set();

                  arduino_ui.putc('a');
                  arduino_ui.putc(Line.all());
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
                  arduino_ui.putc(goal_height);
                  arduino_ui.putc(usensor.get());
            } else if (display_mode == 8) {
                  goal_angle_mode = arduino_ui.getc();

                  arduino_ui.putc('a');
                  arduino_ui.putc(usensor.get());
            }
      }
}
void offence_move() {   // アタックモード
      int16_t tmp_move_speed, tmp_move_angle, robot_angle = 0;
      float move_angle_decrement;

      // 方向

      if (abs(ball_angle) <= (BALL_FOLLOW_RANGE - ball_distance > 90 ? 90 : BALL_FOLLOW_RANGE - ball_distance)) {
            tmp_move_angle = ball_angle + (90.00000 / (BALL_FOLLOW_RANGE - ball_distance) * ball_angle);
      } else {
            move_angle_decrement = ((ball_distance < BALL_FOLLOW_DEPTH_AWAY ? BALL_FOLLOW_DEPTH_AWAY : ball_distance) - BALL_FOLLOW_DEPTH_AWAY) / float(BALL_FOLLOW_DEPTH_NEAR - BALL_FOLLOW_DEPTH_AWAY);
            tmp_move_angle = (ball_angle > 0 ? 90 : -90) + (ball_angle * move_angle_decrement);
      }
      if (tmp_move_angle > 180) tmp_move_angle -= 360;
      if (tmp_move_angle < -180) tmp_move_angle += 360;

      // 速度
      if (abs(ball_angle) <= 90) {
            tmp_move_speed = ((move_speed - 10) * (abs(ball_angle) / 90.000)) + 10 + (100 - ball_distance);
      } else {
            tmp_move_speed = move_speed;
      }
      tmp_move_speed = move_speed;

      // ドリブラー制御
      if (Ball_catch.get() > 50) {
            if (Ball_catch.get() > 60) dribbler_timer.start();
            if (dribbler_timer.read() < 0.4) Dribbler.hold();
      }
      if (goal_angle_mode != 0 && goal_angle != 0 && Ball_catch.get() > 50) robot_angle = goal_angle / 1.25 + yaw;
      if (dribbler_timer.read() > 0.1) {   // ボール補足時
            tmp_move_speed = catch_move_speed;
            if (dribbler_timer.read() > 0.5 && goal_height > 15) {
                  Dribbler.kick();
                  tmp_move_angle = 0;
            } else if (dribbler_timer.read() > 0.4) {
                  Dribbler.stop();
                  tmp_move_angle = 0;
            }
            if (abs(goal_angle / 1.25 + yaw) > 50) tmp_move_angle += goal_angle / 1.25 + yaw > 0 ? 30 : -30;
      }

      if (dribbler_timer.read() > 1.5 || abs(ball_angle) > 45) {
            dribbler_timer.reset();
            dribbler_timer.stop();
            robot_angle = 0;
      }

      // if (tmp_move_speed > move_speed) tmp_move_speed = move_speed;
      Motor.run(tmp_move_angle, tmp_move_speed, robot_angle);   // 回り込み
}

void diffence_move() {   // ディフェンスモード
      int16_t tmp_move_speed, tmp_move_angle, distance_p, back_distance;

      back_distance = 60;

      //
      tmp_move_speed = abs(ball_angle) * 3 + abs(back_distance - usensor.get()) * 3;

      distance_p = abs(back_distance - usensor.get());
      if (distance_p > 60) distance_p = 60;

      if (Line.right()) {
            tmp_move_angle = -90;
            tmp_move_speed = line_move_speed;
      } else if (Line.left()) {
            tmp_move_angle = 90;
            tmp_move_speed = line_move_speed;
      } else {
            if (usensor.get() > back_distance) {
                  if (ball_angle > 0) {
                        tmp_move_angle = 90 + distance_p;
                  } else {
                        tmp_move_angle = -90 - distance_p;
                  }
            } else {
                  if (ball_angle > 0) {
                        tmp_move_angle = 90 - distance_p;
                  } else {
                        tmp_move_angle = -90 + distance_p;
                  }
            }
      }
      //

      if (abs(ball_angle) < 30) {
            diffence_timer.start();
            if (diffence_timer.read() > 6) {
                  diffence_timer.stop();
                  diffence_timer.reset();
            } else if (diffence_timer.read() > 5) {
                  tmp_move_angle = Ball_catch.get_right() - Ball_catch.get_left();
                  tmp_move_speed = move_speed;
            }
      } else {
            diffence_timer.stop();
            diffence_timer.reset();
      }
      if (tmp_move_speed > move_speed) tmp_move_speed = move_speed;

      Motor.run(tmp_move_angle, tmp_move_speed);   // 回り込み
}

void line_move(uint8_t* line_tf, int16_t line_move_speed, int16_t move_speed) {   // ライン処理
      bool tf_x_unit[2], tf_y_unit[2];
      static int16_t tmp_move_angle, tmp_move_speed;
      static uint8_t tf_x, tf_y;
      static int16_t result_vector_x, result_vector_y, pre_result_vector_x, pre_result_vector_y, line_back_angle;
      static uint8_t wait, wait_angle;

      // ドリブラー制御
      if ((Line.front() || Line.right() || Line.left()) && Ball_catch.get() > 50) {
            Dribbler.kick();
      } else if (Ball_catch.get() > 50) {
            Dribbler.hold();
      } else {
            Dribbler.stop();
      }

      // どこのラインを最初に踏んだか記憶
      if (Line.all() && wait == 0) {
            line_timer.start();
            line_timer.reset();

            if (Line.right() && tf_x <= 2) tf_x = 1;
            if (Line.check(4) && tf_x == 1) tf_x = 3;
            if (Line.left() && tf_x <= 2) tf_x = 2;
            if (Line.check(10) && tf_x == 2) tf_x = 4;
            if (Line.front() && tf_y <= 2) tf_y = 1;
            if (Line.check(1) && tf_y == 1) tf_y = 3;
            if (Line.back() && tf_y <= 2) tf_y = 2;
            if (Line.check(7) && tf_y == 2) tf_y = 4;
            // if (Line.check(5) && Line.front()) tf_y = 0;
            // if ((Line.check(2) && Line.left()) || (Line.check(8) && Line.right())) tf_x = 0;

            if (*line_tf == 0) Motor.brake(LINE_BRAKE_TIME);   // ラインを踏んだ時のブレーキ
      }
      for (uint8_t count = 0; count < 2; count++) {
            tf_x_unit[count] = tf_x == count + 1 || tf_x == count + 3 ? 1 : 0;
            tf_y_unit[count] = tf_y == count + 1 || tf_y == count + 3 ? 1 : 0;
      }

      if (tf_y_unit[0]) *line_tf = 1;
      if (tf_x_unit[0]) *line_tf = 2;
      if (tf_y_unit[1]) *line_tf = 3;
      if (tf_x_unit[1]) *line_tf = 4;
      if (tf_y_unit[0] && tf_x_unit[0]) *line_tf = 5;
      if (tf_y_unit[1] && tf_x_unit[0]) *line_tf = 6;
      if (tf_y_unit[1] && tf_x_unit[1]) *line_tf = 7;
      if (tf_y_unit[0] && tf_x_unit[1]) *line_tf = 8;

      if (*line_tf != 0) {   // コートに戻る動作
            if (Line.all() && wait == 0) {
                  result_vector_x = Line.right() + Line.left() * -1;
                  result_vector_y = Line.front() + Line.back() * -1;
                  if (pre_result_vector_x != result_vector_x || pre_result_vector_y != result_vector_y) line_back_angle = atan2(result_vector_x, result_vector_y) / PI * 180.000 + 180.5;
                  pre_result_vector_x = result_vector_x;
                  pre_result_vector_y = result_vector_y;
                  if (line_back_angle > 180) line_back_angle -= 360;

                  if (tf_x_unit[0] && tf_y_unit[0]) {
                        tmp_move_angle = line_back_angle > 135 || line_back_angle < -45 ? line_back_angle : line_back_angle - 180;
                  } else if (tf_x_unit[0] && tf_y_unit[1]) {
                        tmp_move_angle = line_back_angle > -135 && line_back_angle < 45 ? line_back_angle : line_back_angle - 180;
                  } else if (tf_x_unit[1] && tf_y_unit[1]) {
                        tmp_move_angle = line_back_angle > -45 && line_back_angle < 135 ? line_back_angle : line_back_angle - 180;
                  } else if (tf_x_unit[1] && tf_y_unit[0]) {
                        tmp_move_angle = line_back_angle > 45 || line_back_angle < -135 ? line_back_angle : line_back_angle - 180;
                  } else if (tf_y_unit[0]) {
                        tmp_move_angle = line_back_angle < -90 || line_back_angle > 90 ? line_back_angle : line_back_angle - 180;
                  } else if (tf_y_unit[1]) {
                        tmp_move_angle = line_back_angle > -90 && line_back_angle < 90 ? line_back_angle : line_back_angle - 180;
                  } else if (tf_x_unit[0]) {
                        tmp_move_angle = line_back_angle > -180 && line_back_angle < 0 ? line_back_angle : line_back_angle - 180;
                  } else if (tf_x_unit[1]) {
                        tmp_move_angle = line_back_angle > 0 && line_back_angle < 180 ? line_back_angle : line_back_angle - 180;
                  }
                  Motor.run(tmp_move_angle, line_move_speed);

            } else {   // ライン待機動作
                  if (line_timer.read() > LINE_WAIT_BRAKE_TIME) {
                        tf_x = 0;
                        tf_y = 0;
                        if (line_timer.read() > LINE_BALL_WAIT_TIME || Ball_catch.get() > 55) {
                              line_timer.stop();
                              line_timer.reset();
                              *line_tf = 0;
                              wait = 0;
                        }

                        if (*line_tf == 1 && abs(ball_angle) < 60) {
                              wait = 1;
                              tmp_move_speed = abs(ball_angle) * LINE_BALL_P + (100 - ball_distance) * LINE_DISTANCE_P;
                              if (Line.check(1)) {
                                    tmp_move_angle = ball_angle > 0 ? 120 : -120;
                              } else {
                                    tmp_move_angle = ball_angle > 0 ? 60 : -60;
                              }
                              if (Line.right() || Line.back() || Line.left()) {
                                    wait = 0;
                              }
                        } else if (*line_tf == 2 && ball_angle > 0) {
                              wait = 1;
                              if (usensor.get() < 60) {
                                    wait_angle = ball_distance > 80 ? 90 : 120;
                                    tmp_move_speed = abs(abs(ball_angle) - wait_angle) * LINE_BALL_P + (100 - ball_distance) * LINE_DISTANCE_P;
                                    if (Line.check(2)) {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? -150 : -30;
                                    } else if (Line.check(3)) {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? -135 : -45;
                                    } else {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? 150 : 30;
                                    }
                              } else {
                                    tmp_move_speed = abs(abs(ball_angle) - 45) * LINE_BALL_P + (100 - ball_distance) * LINE_DISTANCE_P;
                                    if (Line.check(3)) {
                                          tmp_move_angle = abs(ball_angle) > 45 ? -150 : -30;
                                    } else {
                                          tmp_move_angle = abs(ball_angle) > 45 ? 150 : ball_angle;
                                          if (ball_distance < 60) {
                                                line_timer.stop();
                                                line_timer.reset();
                                                *line_tf = 0;
                                                wait = 0;
                                          }
                                    }
                              }
                              if (Line.check(4) || Line.front() || Line.back() || Line.left()) {
                                    wait = 0;
                              }
                        } else if (*line_tf == 3 && abs(ball_angle) > 15) {
                              wait = 1;
                              tmp_move_speed = abs(ball_angle) * LINE_BALL_P + (100 - ball_distance) * LINE_DISTANCE_P;
                              if (Line.check(6)) {
                                    tmp_move_angle = ball_angle > 0 ? 60 : -60;
                              } else {
                                    tmp_move_angle = ball_angle > 0 ? 120 : -120;
                              }
                              if (Line.check(7) || Line.front() || Line.right() || Line.left()) {
                                    wait = 0;
                              }
                        } else if (*line_tf == 4 && ball_angle < 0) {
                              wait = 1;
                              if (usensor.get() < 60) {
                                    wait_angle = ball_distance > 80 ? 90 : 120;
                                    tmp_move_speed = abs(abs(ball_angle) - wait_angle) * LINE_BALL_P + (100 - ball_distance) * LINE_DISTANCE_P;
                                    if (Line.check(8)) {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? 150 : 30;
                                    } else if (Line.check(9)) {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? 135 : 45;
                                    } else {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? -150 : -30;
                                    }
                              } else {
                                    tmp_move_speed = abs(abs(ball_angle) - 45) * LINE_BALL_P + (100 - ball_distance) * 2;
                                    if (Line.check(9)) {
                                          tmp_move_angle = abs(ball_angle) > 45 ? 150 : 30;
                                    } else {
                                          tmp_move_angle = abs(ball_angle) > 45 ? -150 : ball_angle;
                                          if (ball_distance < 60) {
                                                line_timer.stop();
                                                line_timer.reset();
                                                *line_tf = 0;
                                                wait = 0;
                                          }
                                    }
                              }
                              if (Line.check(10) || Line.front() || Line.right() || Line.back()) {
                                    wait = 0;
                              }
                        } else if (*line_tf == 6 && abs(ball_angle) < 90) {
                              wait = 1;
                              tmp_move_speed = move_speed;
                              if (Line.check(2)) {
                                    tmp_move_angle = -30;
                              } else if (Line.check(3)) {
                                    tmp_move_angle = -45;
                              } else {
                                    tmp_move_angle = 30;
                              }
                              if (Line.check(4) || Line.check(7) || Line.front() || Line.left()) {
                                    wait = 0;
                              }
                        } else if (*line_tf == 7 && abs(ball_angle) < 90) {
                              wait = 1;
                              tmp_move_speed = move_speed;
                              if (Line.check(8)) {
                                    tmp_move_angle = 30;
                              } else if (Line.check(9)) {
                                    tmp_move_angle = 45;
                              } else {
                                    tmp_move_angle = -30;
                              }
                              if (Line.check(10) || Line.check(7) || Line.front() || Line.right()) {
                                    wait = 0;
                              }
                        } else {
                              line_timer.stop();
                              line_timer.reset();
                              *line_tf = 0;
                              wait = 0;
                        }

                        if (tmp_move_speed > move_speed) tmp_move_speed = move_speed;
                        Motor.run(tmp_move_angle, tmp_move_speed);
                  } else {
                        Motor.brake();
                  }
            }
      }
}