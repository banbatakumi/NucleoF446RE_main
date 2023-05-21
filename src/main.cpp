#include <mbed.h>

#include "ball_catch.h"
#include "dribbler.h"
#include "hcsr04.h"
#include "line.h"
#include "motor.h"
#include "voltage.h"

#define PI 3.1415926535   // 円周率

#define BALL_FOLLOW_RANGE 140   // 回り込み時にボールを追い始める角度(標準値:130)
#define BALL_FOLLOW_DEPTH_AWAY -35.000   // ボールが遠い時の回り込みの深さ(標準値:30)
#define BALL_FOLLOW_DEPTH_NEAR 85.000   // ボールが近い時の回り込みの深さ(標準値:90)
#define BALL_CATCH_P 1.5
#define BALL_FOLLOW_D 20.000
#define D_PERIODO 0.01

#define LINE_BALL_P 2.5   // ライン待機時にボールを追う比例制御のゲイン
#define LINE_DISTANCE_P 2.5
#define LINE_BALL_WAIT_TIME 3   // ライン待機時にボールを追う時間
#define LINE_BRAKE_TIME 10   // ラインを踏んだ時のブレーキ時間(μs)
#define LINE_WAIT_BRAKE_TIME 0.05   // ラインを踏んだ後のブレーキ時間(s)

#define IMU_SERIAL_BAUD 38400   // UART通信速度
#define IR_SERIAL_BAUD 38400   // UART通信速度
#define UI_SERIAL_BAUD 19200   // UART通信速度

#define CAM_CENTER 105   // カメラのセンター合わせ

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

int16_t yaw, set_yaw, goal_angle = 0, pre_goal_angle = 0, goal_height = 0, goal_angle_mode = 0, imu_validation = 0;
void imu();

uint8_t mode = 0, move_speed = 70, line_move_speed = 80, catch_move_speed = 70;
void ui();

Timer line_timer;

Timer line_wait_timer;

Timer line_back_timer;

Timer dribbler_timer;

Timer diffence_timer;

Timer ball_follow_pd_timer;

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
                              Motor.run(0, 0);
                        } else {
                              offence_move();
                        }
                  }
            } else if (mode == 2) {
                  diffence_move();
            } else if (mode == 3) {
                  Motor.run(0, 0, 45, 1);
                  // Motor.run(0, 0, (goal_angle / 1.25 + yaw));
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
            pre_goal_angle = abs(goal_angle) > 30 ? (goal_angle > 0 ? 1 : -1) : pre_goal_angle;
      }
}

void ui() {   // ui用のマイコンとの通信
      uint8_t display_mode;

      if (arduino_ui.getc() == 'a') {
            display_mode = arduino_ui.getc();
            if (display_mode != 0) imu_validation = 0;
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
      int16_t pre_p, p, d;
      float move_angle_decrement;

      // 方向

      move_angle_decrement = ((ball_distance < BALL_FOLLOW_DEPTH_AWAY ? BALL_FOLLOW_DEPTH_AWAY : ball_distance) - BALL_FOLLOW_DEPTH_AWAY) / float(BALL_FOLLOW_DEPTH_NEAR - BALL_FOLLOW_DEPTH_AWAY);
      if (abs(ball_angle) <= (BALL_FOLLOW_RANGE - ball_distance > 90 ? 90 : BALL_FOLLOW_RANGE - ball_distance)) {
            tmp_move_angle = (90.00000 / (BALL_FOLLOW_RANGE - ball_distance) * ball_angle);
      } else {
            tmp_move_angle = (ball_angle > 0 ? 90 : -90);
      }

      tmp_move_angle += (ball_angle * move_angle_decrement);

      if (abs(ball_angle) < 90) tmp_move_angle += (Ball_catch.get_right() - Ball_catch.get_left()) * BALL_CATCH_P * (ball_distance / 90.000) * ((90 - abs(ball_angle)) / 90.000);

      if (tmp_move_angle > 180) tmp_move_angle -= 360;
      if (tmp_move_angle < -180) tmp_move_angle += 360;

      // 速度
      if (Ball_catch.get() > 50) {
            tmp_move_speed = catch_move_speed;
      } else if (abs(ball_angle) <= 100) {
            ball_follow_pd_timer.start();
            p = 0 - (((move_speed - 40) * (abs(ball_angle) / 90.000)) + 40 + (BALL_FOLLOW_DEPTH_NEAR - ball_distance));
            d = p - pre_p;
            pre_p = p;
            if (ball_follow_pd_timer.read() > D_PERIODO) {
                  d = p - pre_p;   // 微分
                  pre_p = p;
                  ball_follow_pd_timer.reset();
            }
            tmp_move_speed = abs(p + d * BALL_FOLLOW_D);
      } else {
            ball_follow_pd_timer.stop();
            tmp_move_speed = move_speed;
      }

      // ドリブラー制御
      if (Ball_catch.get() > 50) {
            if (Ball_catch.get() > 60) dribbler_timer.start();
            if (dribbler_timer.read() < 0.2) Dribbler.hold();
      }

      if (goal_angle_mode != 0 && abs(ball_angle) < 90 && Ball_catch.get() > 50 && goal_height > 10) robot_angle = goal_angle / 1.25 + yaw;
      if (dribbler_timer.read() > 0.1) {   // ボール補足時
            tmp_move_speed = catch_move_speed;
            if (goal_angle_mode != 0 && goal_angle != 0 && (abs(goal_angle) > 25 || goal_height < 15)) {
                  dribbler_timer.stop();
            } else {
                  dribbler_timer.start();
            }
            if (dribbler_timer.read() > 0.3) {
                  Dribbler.kick();
                  tmp_move_angle = 0;
            } else if (dribbler_timer.read() > 0.2) {
                  Dribbler.stop();
                  tmp_move_angle = 0;
            }
            if (abs(goal_angle / 1.25 + yaw) > 50) tmp_move_angle += goal_angle / 1.25 + yaw > 0 ? 30 : -30;
      }

      if (dribbler_timer.read() > 2 || abs(ball_angle) > 30) {
            dribbler_timer.reset();
            dribbler_timer.stop();
            robot_angle = 0;
      }

      if (tmp_move_speed > catch_move_speed) tmp_move_speed = move_speed;
      Motor.run(tmp_move_angle, tmp_move_speed, robot_angle);   // 回り込み
}

void diffence_move() {   // ディフェンスモード
      int16_t tmp_move_speed, tmp_move_angle, distance_p, back_distance;

      back_distance = 60;

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
      int16_t robot_angle = 0;
      static int16_t tmp_move_angle, tmp_move_speed, shoot_robot_angle, pre_goal_angle_line;
      static uint8_t tf_x, tf_y;
      static int16_t result_vector_x, result_vector_y, pre_result_vector_x, pre_result_vector_y, line_back_angle;
      static uint8_t wait, wait_angle;

      // ドリブラー制御
      if (Line.front() && Ball_catch.get() > 50) {
            Dribbler.kick();
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
            if (Line.check(5) && Line.front()) tf_y = 0;
            if ((Line.check(2) && Line.left()) || (Line.check(8) && Line.right())) tf_x = 0;

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
                        if (line_timer.read() > (*line_tf == 1 ? 7.5 : LINE_BALL_WAIT_TIME) || (Ball_catch.get() > 55 && *line_tf != 1)) {
                              line_timer.stop();
                              line_timer.reset();
                              *line_tf = 0;
                              wait = 0;
                        }

                        if (*line_tf == 1 && abs(ball_angle) < 45) {
                              if (wait == 0) {
                                    line_back_timer.reset();
                                    line_back_timer.stop();
                              }
                              wait = 1;
                              if (Ball_catch.get() > 60) line_back_timer.start();
                              // if (Ball_catch.get() < 10) line_back_timer.reset();
                              if (line_back_timer.read() > 1 && line_back_timer.read() < 4) {
                                    shoot_robot_angle = 1;
                                    robot_angle = (line_back_timer.read() - 1) * (pre_goal_angle_line == 1 ? -65 : 65);
                                    Dribbler.hold(90);

                              } else if (line_back_timer.read() < 1) {
                                    if (Ball_catch.get() > 50) Dribbler.hold();
                                    tmp_move_speed = 0;
                                    robot_angle = 0;
                                    pre_goal_angle_line = pre_goal_angle == 1 ? 1 : -1;
                              } else if (line_back_timer.read() >= 5) {
                                    line_back_timer.reset();
                                    line_back_timer.stop();
                                    Dribbler.stop();
                                    robot_angle = 0;
                                    wait = 0;
                              } else if (line_back_timer.read() >= 4) {
                                    if (Ball_catch.get() > 50) Dribbler.hold(90);
                                    robot_angle = (pre_goal_angle_line == 1 ? 120 : -120);
                              }
                              if (Line.check(1) || Line.check(3) || Line.check(4) || Line.check(6) || Line.check(7) || Line.check(9) || Line.check(10)) wait = 0;
                              /*
                              if (Ball_catch.get() > 60) line_back_timer.start();
                              if (Ball_catch.get() < 30) line_back_timer.reset();
                              if (line_back_timer.read() > 1 && line_back_timer.read() < 2) {
                                    tmp_move_speed = 25;
                                    tmp_move_angle = 180;
                              } else if (line_back_timer.read() < 1) {
                                    Dribbler.hold();
                                    tmp_move_speed = 0;
                              } else if (line_back_timer.read() > 2) {
                                    line_back_timer.reset();
                                    line_back_timer.stop();
                                    Dribbler.stop();
                              }*/

                              /*
                              wait = 1;
                              tmp_move_speed = abs(ball_angle) * LINE_BALL_P + (100 - ball_distance) * LINE_DISTANCE_P;
                              if (Line.check(0)) {
                                    tmp_move_angle = ball_angle > 0 ? 120 : -120;
                              } else if (Line.check(1)) {
                                    tmp_move_angle = ball_angle > 0 ? 135 : -135;
                              } else {
                                    tmp_move_angle = ball_angle > 0 ? 60 : -60;
                              }
                              if (Line.right() || Line.back() || Line.left()) {
                                    wait = 0;
                              }*/
                        } else if (*line_tf == 2 && ball_angle > 0) {
                              wait = 1;
                              if (usensor.get() < 60) {
                                    wait_angle = ball_distance > 80 ? 90 : 120;
                                    tmp_move_speed = abs(abs(ball_angle) - wait_angle) * LINE_BALL_P + (BALL_FOLLOW_DEPTH_NEAR - ball_distance) * LINE_DISTANCE_P;
                                    if (Line.check(2)) {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? -150 : -30;
                                    } else if (Line.check(3)) {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? -135 : -45;
                                    } else {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? 150 : 30;
                                    }
                              } else {
                                    tmp_move_speed = abs(abs(ball_angle) - 45) * LINE_BALL_P + (BALL_FOLLOW_DEPTH_NEAR - ball_distance) * LINE_DISTANCE_P;
                                    if (Line.check(2)) {
                                          tmp_move_angle = abs(ball_angle) > 45 ? -150 : -30;
                                    } else if (Line.check(3)) {
                                          tmp_move_angle = abs(ball_angle) > 45 ? -135 : -45;
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
                              if (Line.check(4) || Line.front() || Line.back() || Line.left()) wait = 0;
                        } else if (*line_tf == 3 && abs(ball_angle) > 15) {
                              wait = 1;
                              tmp_move_speed = abs(ball_angle) * LINE_BALL_P + (BALL_FOLLOW_DEPTH_NEAR - ball_distance) * LINE_DISTANCE_P;
                              if (Line.check(5)) {
                                    tmp_move_angle = ball_angle > 0 ? 60 : -60;
                              } else if (Line.check(6)) {
                                    tmp_move_angle = ball_angle > 0 ? 45 : -45;
                              } else {
                                    tmp_move_angle = ball_angle > 0 ? 120 : -120;
                              }
                              if (Line.check(7) || Line.front() || Line.right() || Line.left()) wait = 0;
                        } else if (*line_tf == 4 && ball_angle < 0) {
                              wait = 1;
                              if (usensor.get() < 60) {
                                    wait_angle = ball_distance > 80 ? 90 : 120;
                                    tmp_move_speed = abs(abs(ball_angle) - wait_angle) * LINE_BALL_P + (BALL_FOLLOW_DEPTH_NEAR - ball_distance) * LINE_DISTANCE_P;
                                    if (Line.check(8)) {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? 150 : 30;
                                    } else if (Line.check(9)) {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? 135 : 45;
                                    } else {
                                          tmp_move_angle = abs(ball_angle) > wait_angle ? -150 : -30;
                                    }
                              } else {
                                    tmp_move_speed = abs(abs(ball_angle) - 45) * LINE_BALL_P + (BALL_FOLLOW_DEPTH_NEAR - ball_distance) * 2;
                                    if (Line.check(8)) {
                                          tmp_move_angle = abs(ball_angle) > 45 ? 150 : 30;
                                    } else if (Line.check(9)) {
                                          tmp_move_angle = abs(ball_angle) > 45 ? 135 : 45;
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
                              if (Line.check(10) || Line.front() || Line.right() || Line.back()) wait = 0;
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
                              if (Line.check(4) || Line.check(7) || Line.front() || Line.left()) wait = 0;
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
                              if (Line.check(10) || Line.check(7) || Line.front() || Line.right()) wait = 0;
                        } else {
                              line_timer.stop();
                              line_timer.reset();
                              *line_tf = 0;
                              wait = 0;
                        }

                        if (tmp_move_speed > move_speed) tmp_move_speed = move_speed;
                        Motor.run(tmp_move_angle, tmp_move_speed, robot_angle, shoot_robot_angle);
                  } else {
                        if (*line_tf != 0) Motor.brake();
                  }
            }
      }
}