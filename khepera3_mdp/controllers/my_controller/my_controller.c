/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  A controller moving the khepera III and its gripper.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SENSOR_NUMBER 16
#define RANGE (1024 / 2)
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))
#define GRIPPER_MOTOR_MAX_SPEED 2.0
#define sqrt_2 1.41421356237


static WbDeviceTag left_motor_kedy, right_motor_kedy;

static int time_step = 0;
double step_time = (0.5*sqrt_2/0.0212)/10;

static void initialize_kedy() {
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  left_motor_kedy = wb_robot_get_device("left wheel motor kedy");
  right_motor_kedy = wb_robot_get_device("right wheel motor kedy");
  wb_motor_set_position(left_motor_kedy, INFINITY);
  wb_motor_set_position(right_motor_kedy, INFINITY);
  wb_motor_set_velocity(left_motor_kedy, 0.0);
  wb_motor_set_velocity(right_motor_kedy, 0.0);
}

void step_kedy(double seconds) {
  const double ms = seconds * 1000.0;
  int elapsed_time = 0;
  while (elapsed_time < ms) {
    wb_robot_step(time_step);
    elapsed_time += time_step;
  }
}

void moveForwards_kedy(){
  wb_motor_set_velocity(left_motor_kedy, 10);
  wb_motor_set_velocity(right_motor_kedy, 10);
}

void turn_right_kedy(double speed) {
  wb_motor_set_velocity(left_motor_kedy, speed);
  wb_motor_set_velocity(right_motor_kedy, -speed);
}

void turn_left_kedy(double speed) {
  wb_motor_set_velocity(left_motor_kedy, -speed);
  wb_motor_set_velocity(right_motor_kedy, speed);
}

void stop_kedy(double seconds) {
  wb_motor_set_velocity(left_motor_kedy, 0.0);
  wb_motor_set_velocity(right_motor_kedy, 0.0);
  step_kedy(seconds);
}

void saga_git_kedy(){
  turn_right_kedy(3.5);
  step_kedy(1.0);
  stop_kedy(0.1);
  step_kedy(0.1);
  wb_motor_set_velocity(left_motor_kedy, (0.5/2)/0.0212);
  wb_motor_set_velocity(right_motor_kedy, (0.5/2)/0.0212);
  step_kedy(2.0);
  stop_kedy(1.0);
  turn_left_kedy(3.5);
  step_kedy(1.0);
  stop_kedy(0.1);
  step_kedy(0.1);
}

void sola_git_kedy(){
  turn_left_kedy(3.5);
  step_kedy(1.0);
  stop_kedy(1.0);
  step_kedy(1.0);
  wb_motor_set_velocity(left_motor_kedy, (0.5/2)/0.0212);
  wb_motor_set_velocity(right_motor_kedy, (0.5/2)/0.0212);
  step_kedy(2.0);
  stop_kedy(1.0);
  turn_right_kedy(3.5);
  step_kedy(1.0);
  stop_kedy(0.1);
  step_kedy(0.1);
}

void sol_alta_git_kedy(){
  turn_left_kedy(5.25);
  step_kedy(1.0);
  stop_kedy(0.1);
  step_kedy(1.0);
  wb_motor_set_velocity(left_motor_kedy, (0.5/2)/0.0212);
  wb_motor_set_velocity(right_motor_kedy, (0.5/2)/0.0212);
  step_kedy(2.0);
  stop_kedy(1.0);
  turn_right_kedy(5.25);
  step_kedy(1.0);
  stop_kedy(0.1);
  step_kedy(0.1);
}


int main() {
  initialize_kedy();
}