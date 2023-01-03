/*
 * Copyright 1996-2022 Cyberbotics Ltd.
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
 
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/motor.h>
#include "mdp_run_run.h"
#include "all.h"
#include "any.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "find.h"
#include "ifWhileCond.h"
#include "mdp_run_run.h"
#include "mdp_run_run_data.h"
#include "mdp_run_run_initialize.h"
#include "mdp_run_run_terminate.h"
#include "mdp_run_run_types.h"
#include "minOrMax.h"
#include "nnz.h"
#include "nonSingletonDim.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include "useConstantDim.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rtwtypes.h"
#include "tmwtypes.h"
#include "useConstantDim.h"
#include <math.h>
#include <time.h>


#include "all.c"
#include "any.c"
#include "eml_rand_mt19937ar_stateful.c"
#include "find.c"
#include "ifWhileCond.c"
#include "mdp_run_run_data.c"
#include "mdp_run_run_initialize.c"
#include "mdp_run_run_terminate.c"
#include "minOrMax.c"
#include "nnz.c"
#include "nonSingletonDim.c"
#include "rand.c"
#include "rt_nonfinite.c"
#include "rtGetInf.c"
#include "rtGetNaN.c"
#include "useConstantDim.c"

#define TIME_STEP 32
#define MAX_SENSOR_NUMBER 16
#define RANGE (1024 / 2)
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))
#define GRIPPER_MOTOR_MAX_SPEED 2.0
#define sqrt_2 1.4142


static WbDeviceTag sensors[MAX_SENSOR_NUMBER];
static WbDeviceTag gripper_motors[3];
static WbDeviceTag left_motor, right_motor;
static WbDeviceTag left_position_sensor, right_position_sensor;


static const int num_sensors = 5;
static int time_step = 0;
double step_time = (0.5*sqrt_2/0.0212)/10;


static void initialize() {
  /* necessary to initialize Webots */
  const char *name;
  char num;
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();
  name = wb_robot_get_name();
  num = name[0];
  
  switch (num) {
    case '1':
      char sensors_name[5];
      sprintf(sensors_name, "%s", "us0");
      
      int i;
      for (i = 0; i < num_sensors; i++) {
        sensors[i] = wb_robot_get_device(sensors_name);
        wb_distance_sensor_enable(sensors[i], time_step);
    
        if ((i + 1) >= 10) {
          sensors_name[2] = '1';
          sensors_name[3]++;
    
          if ((i + 1) == 10) {
            sensors_name[3] = '0';
            sensors_name[4] = '\0';
          }
        } else {
          sensors_name[2]++;
        }
      }
    
      gripper_motors[0] = wb_robot_get_device("horizontal_motor");
      gripper_motors[1] = wb_robot_get_device("left_finger_motor");
      gripper_motors[2] = wb_robot_get_device("right_finger_motor");
      
      left_motor = wb_robot_get_device("left wheel motor");
      right_motor = wb_robot_get_device("right wheel motor");
      
      wb_motor_set_position(left_motor, INFINITY);
      wb_motor_set_position(right_motor, INFINITY);
      wb_motor_set_velocity(left_motor, 0.0);
      wb_motor_set_velocity(right_motor, 0.0);
      
      left_position_sensor = wb_robot_get_device("left wheel sensor");
      right_position_sensor = wb_robot_get_device("right wheel sensor");
      wb_position_sensor_enable(left_position_sensor, TIME_STEP);
      wb_position_sensor_enable(right_position_sensor, TIME_STEP);
      break;
    case '2':
      left_motor = wb_robot_get_device("left wheel motor");
      right_motor = wb_robot_get_device("right wheel motor");
      
      wb_motor_set_position(left_motor, INFINITY);
      wb_motor_set_position(right_motor, INFINITY);
      wb_motor_set_velocity(left_motor, 0.0);
      wb_motor_set_velocity(right_motor, 0.0);
      
      left_position_sensor = wb_robot_get_device("left wheel sensor");
      right_position_sensor = wb_robot_get_device("right wheel sensor");
      wb_position_sensor_enable(left_position_sensor, TIME_STEP);
      wb_position_sensor_enable(right_position_sensor, TIME_STEP);
      break;
  }
}

void step(double seconds) {
  const double ms = seconds * 1000.0;
  int elapsed_time = 0;
  while (elapsed_time < ms) {    
    wb_robot_step(time_step);
    elapsed_time += time_step;
  }
}

void moveArms(double position) {
  wb_motor_set_velocity(gripper_motors[0], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[0], position);
}

void moveFingers(double position) {
  wb_motor_set_velocity(gripper_motors[1], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_velocity(gripper_motors[2], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[1], position);
  wb_motor_set_position(gripper_motors[2], -position);
}

void stop(double seconds) {
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  step(seconds);
}

void turn_right(double speed) {
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, -speed);
}

void turn_left(double speed) {
  wb_motor_set_velocity(left_motor, -speed);
  wb_motor_set_velocity(right_motor, speed);
}

void ileri_git(){
  const char *name;
  char num;
  int flag = 0;

  name = wb_robot_get_name();
  num = name[0];
  
  switch (num) {
    case '1':
      const double ms = 2 * 1000.0;
      int elapsed_time = 0;
      double sensors_value[num_sensors];
      
      while (elapsed_time < ms) {
        wb_motor_set_velocity(left_motor, (0.5/2)/0.0212);
        wb_motor_set_velocity(right_motor, (0.5/2)/0.0212);
        int i;
        for (i = 0; i < num_sensors; i++)
        {
          sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);  
        }
        
        if (sensors_value[2]>7500) {
          wb_motor_set_velocity(left_motor, 0.0);
          wb_motor_set_velocity(right_motor, 0.0);
          moveArms(-3.0);
          step(1.0);
          stop(0.1);
          moveFingers(0.42);
          step(1.0);
          moveArms(0.0);
          step(1);
          flag = 1;

        }
        else {
          wb_robot_step(time_step);
          elapsed_time += time_step;
        }
      }
      if (flag == 0) {
        stop(3.1);
      }
      stop(4.3+5+step_time);
      break;
      }
}

void geri_git(){
  const char *name;
  char num;
  int flag = 0;

  name = wb_robot_get_name();
  num = name[0];
  
  switch (num) {
    case '1':
    turn_right(3.45);
    step(2.0);
    stop(0.1);
  
    const double ms = 2 * 1000.0;
    int elapsed_time = 0;
    double sensors_value[num_sensors];
    
    while (elapsed_time < ms) {
      wb_motor_set_velocity(left_motor, (0.5/2)/0.0212);
      wb_motor_set_velocity(right_motor, (0.5/2)/0.0212);
      int i;
      for (i = 0; i < num_sensors; i++)
      {
        sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);  
      }
      
      if (sensors_value[2]>7500) {
        wb_motor_set_velocity(left_motor, 0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        moveArms(-3.0);
        step(1.0);
        stop(0.1);
        moveFingers(0.42);
        step(1.0);
        moveArms(0.0);
        step(1);
        flag = 1;

      }
      else {
        wb_robot_step(time_step);
        elapsed_time += time_step;
      }
    }
      if (flag == 0) {
        stop(3.1);
      }
      stop(0.1);
      turn_left(3.45);
      step(2.0);
      stop(0.1+5+step_time);
      break;
  }
}

void saga_git(){
  const char *name;
  char num;
  int flag = 0;

  name = wb_robot_get_name();
  num = name[0];
  
  switch (num) {
    case '1':
      turn_right(3.45);
      step(1.0);
      stop(0.1);
      
      const double ms = 2 * 1000.0;
      int elapsed_time = 0;
      double sensors_value[num_sensors];
      
      while (elapsed_time < ms) {
        wb_motor_set_velocity(left_motor, (0.5/2)/0.0212);
        wb_motor_set_velocity(right_motor, (0.5/2)/0.0212);
        int i;
        for (i = 0; i < num_sensors; i++)
        {
          sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);  
        }
        
        if (sensors_value[2]>8000) {
          wb_motor_set_velocity(left_motor, 0.0);
          wb_motor_set_velocity(right_motor, 0.0);
          moveArms(-3.0);
          step(1.0);
          stop(0.1);
          moveFingers(0.42);
          step(1.0);
          moveArms(0.0);
          step(1);
          flag = 1;
        }
        else {
          wb_robot_step(time_step);
          elapsed_time += time_step;
        }
      }
      if (flag == 0) {
        stop(3.1);
      }
      stop(0.1);
      turn_left(3.45);
      step(1.0);
      stop(2.1+5+step_time);
      break;
    }
}

void sola_git(){
  const char *name;
  char num;
  int flag = 0;


  name = wb_robot_get_name();
  num = name[0];
  
  switch (num) {
    case '1':
      turn_left(3.45);
      step(1.0);
      stop(0.1);
      
      const double ms = 2 * 1000.0;
      int elapsed_time = 0;
      double sensors_value[num_sensors];
      
      while (elapsed_time < ms) {
        wb_motor_set_velocity(left_motor, (0.5/2)/0.0212);
        wb_motor_set_velocity(right_motor, (0.5/2)/0.0212);
        int i;
        for (i = 0; i < num_sensors; i++)
        {
          sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);  
        }
        
        if (sensors_value[2]>7500) {
          wb_motor_set_velocity(left_motor, 0.0);
          wb_motor_set_velocity(right_motor, 0.0);
          moveArms(-3.0);
          step(1.0);
          stop(0.1);
          moveFingers(0.42);
          step(1.0);
          moveArms(0.0);
          step(1);
          flag = 1;
        }
        else {
          wb_robot_step(time_step);
          elapsed_time += time_step;
        }
      }
      if (flag == 0) {
        stop(3.1);
      }
      stop(0.1);
      turn_right(3.45);
      step(1.0);
      stop(2.1+5+step_time);
      break;
  }
}

////////////////////////////////////////////////////////////////

void sol_alta_git_kedy(){
  const char *name;
  char num;

  name = wb_robot_get_name();
  num = name[0];
  
  switch (num) {
    case '1':
      break;
    case '2':
      turn_left(3.45*1.5);
      step(1.0);
      stop(1.0);
      wb_motor_set_velocity(left_motor, 10);
      wb_motor_set_velocity(right_motor, 10);
      step(step_time);
      stop(1.0);
      turn_right(3.45*1.5);
      step(1.0);
      stop(1.0);
      stop(9.4);
      break;
}
}

void sag_alta_git_kedy(){
  const char *name;
  char num;

  name = wb_robot_get_name();
  num = name[0];
  
  switch (num) {
    case '1':
      break;
    case '2':
      turn_right(3.45*1.5);
      step(1.0);
      stop(1.0);
      wb_motor_set_velocity(left_motor, 10);
      wb_motor_set_velocity(right_motor, 10);
      step(step_time);
      stop(1.0);
      turn_left(3.45*1.5);
      step(1.0);
      stop(1.0);
      stop(9.4);
      break;
}
}

void sag_uste_git_kedy(){
  const char *name;
  char num;

  name = wb_robot_get_name();
  num = name[0];
  
  switch (num) {
    case '1':
      break;
    case '2':
      turn_right(3.45/2);
      step(1.0);
      stop(1.0);
      wb_motor_set_velocity(left_motor, 10);
      wb_motor_set_velocity(right_motor, 10);
      step(step_time);
      stop(1.0);
      turn_left(3.45/2);
      step(1.0);
      stop(1.0);
      stop(9.4);
      break;
}
}

void sol_uste_git_kedy(){
  const char *name;
  char num;

  name = wb_robot_get_name();
  num = name[0];
  
  switch (num) {
    case '1':
      break;
    case '2':
      turn_left(3.45/2);
      step(1.0);
      stop(1.0);
      wb_motor_set_velocity(left_motor, 10);
      wb_motor_set_velocity(right_motor, 10);
      step(step_time);
      stop(1.0);
      turn_right(3.45/2);
      step(1.0);
      stop(1.0);
      stop(9.4);
      break;
}
}

///////////////////////////////////////////////////////////////

void mdp_run_run(void)
{
  static const double dv[16] = {0.7, 0.1, 0.1, 0.1, 0.1, 0.7, 0.1, 0.1,
                                0.1, 0.1, 0.7, 0.1, 0.1, 0.1, 0.1, 0.7};
  static const signed char rotation[16] = {0,  1,  2, 1, 0, -1, -2, -1,
                                           -2, -1, 0, 1, 2, 1,  0,  -1};
  double reward[64];
  double value[64];
  double reward_cheese[16];
  double x_data[16];
  double action_fear[4];
  double b_reward_cheese[4];
  double cheese[2];
  double d;
  double d1;
  double dis1_idx_0;
  double dis1_idx_1;
  double dis2_idx_0;
  double dis3_idx_0;
  double dis3_idx_1;
  double dis4_idx_0;
  double dis4_idx_1;
  double dis_idx_0_tmp;
  double dis_idx_1_tmp;
  double eklenti;
  double fear;
  double hunger;
  double iteration;
  double mice_idx_0;
  double mice_idx_1;
  double v_next_idx_0;
  double v_next_idx_1;
  double v_next_idx_2;
  double v_next_idx_3;
  int ind_fear_data[4];
  int ind_hunger_data[4];
  int a_size[2];
  int action_size[2];
  int ind_hunger_size[2];
  int r_size[2];
  int x_size[2];
  int a;
  int b_i;
  int b_reward_cheese_tmp;
  int exitg1;
  int i;
  int it;
  int j;
  int loop_ub;
  int peynir_flag;
  int reward_cheese_tmp;
  int z_tmp;
  signed char cat_array[16];
  signed char z[16];
  signed char tmp_data[4];
  signed char cat_idx_0;
  signed char cat_idx_1;
  signed char i1;
  boolean_T c_reward_cheese[16];
  boolean_T c_i[8];
  boolean_T r_data[5];
  boolean_T x[4];
  boolean_T mice[2];
  boolean_T action_data;
  if (!isInitialized_mdp_run_run) {
    mdp_run_run_initialize();
  }
  
  mice_idx_0 = 1.0;/* Farenin BaÅŸlangÄ±Ã§ Konumu (x)*/
  cheese[0] = 5.0;/* Peynirin BaÅŸlangÄ±Ã§ Konumu (x)*/
  mice_idx_1 = 1.0;/* Farenin BaÅŸlangÄ±Ã§ Konumu (y)*/
  cheese[1] = 5.0;/* Peynirin BaÅŸlangÄ±Ã§ Konumu (y)*/
  
  const char *name;
  char num;

  name = wb_robot_get_name();
  num = name[0];
  
  switch (num) {
    case '1':
      break;
    case '2':
      stop(9.4);
      break;
      }
  
  for (i = 0; i < 8; i++) { /* Kedinin BulunacaÄŸÄ± KonumlarÄ± Hesaplar*/
    cat_array[i] = (signed char)(5 - rotation[i]);
    cat_array[i + 8] = (signed char)(5 - rotation[i + 8]);
  }
  cat_idx_0 = cat_array[0];/* Kedinin BaÅŸlangÄ±Ã§ Konumu (x)*/
  cat_idx_1 = cat_array[8];/* Kedinin BaÅŸlangÄ±Ã§ Konumu (y)*/
  
  for (b_i = 0; b_i < 64; b_i++) { /*Ã–dÃ¼l DeÄŸerlerini Hesaplar*/
    value[b_i] = 0.0;
    reward[b_i] = 0.01;
  }
  reward[36] = 10.0;
  reward[(cat_array[0] + ((cat_array[8] - 1) << 3)) - 1] = -50.0;
  for (b_i = 0; b_i < 16; b_i++) {
    z[b_i] = 0;
  }
  it = 0;
  for (i = 0; i < 3; i++) {
    z[it] = (signed char)(i - 1);
    z[it + 8] = -1;
    it++;
    if (i - 1 != 0) {
      z[it] = (signed char)(i - 1);
      z[it + 8] = 0;
      it++;
    }
    z[it] = (signed char)(i - 1);
    z[it + 8] = 1;
    it++;
  }
  for (b_i = 0; b_i < 16; b_i++) {
    reward_cheese[b_i] = 5.0 - (double)z[b_i];
  }
  for (b_i = 0; b_i < 2; b_i++) {
    it = b_i << 3;
    for (reward_cheese_tmp = 0; reward_cheese_tmp < 8; reward_cheese_tmp++) {
      z_tmp = reward_cheese_tmp + it;
      z[z_tmp] = (signed char)(cat_array[it] - z[z_tmp]);
    }
  }
  for (i = 0; i < 8; i++) {
    for (j = 0; j < 8; j++) {
      for (b_i = 0; b_i < 8; b_i++) {
        c_i[b_i] = (i + 1 == (int)reward_cheese[b_i]);
      }
      if (any(c_i)) {
        for (b_i = 0; b_i < 8; b_i++) {
          c_i[b_i] = (j + 1 == (int)reward_cheese[b_i + 8]);
        }
        if (any(c_i)) {
          it = i + (j << 3);
          reward[it] += 10.0;
        }
      }
      for (b_i = 0; b_i < 8; b_i++) {
        c_i[b_i] = (i + 1 == z[b_i]);
      }
      if (any(c_i)) {
        for (b_i = 0; b_i < 8; b_i++) {
          c_i[b_i] = (j + 1 == z[b_i + 8]);
        }
        if (any(c_i)) {
          it = i + (j << 3);
          reward[it] += -50.0;
        }
      }
    }
  }
  
  hunger = 0.2;
  fear = 0.0;
  peynir_flag = 0;
  iteration = 0.0;
  
  do {
  
    exitg1 = 0;
    dis_idx_0_tmp = mice_idx_0 - (double)cat_idx_0;
    dis_idx_1_tmp = mice_idx_1 - (double)cat_idx_1;
    dis1_idx_0 = mice_idx_0 - cheese[0];
    dis2_idx_0 = mice_idx_0 - cheese[0];
    dis3_idx_1 = mice_idx_1 - cheese[1];
    dis4_idx_1 = mice_idx_1 - cheese[1];
    for (z_tmp = 0; z_tmp < 20; z_tmp++) { /*20 Ä°terasyon Boyunca BÃ¼tÃ¼n HaritanÄ±n DeÄŸerlerini Hesaplar*/
      for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
          v_next_idx_0 = 0.0;
          v_next_idx_1 = 0.0;
          v_next_idx_2 = 0.0;
          v_next_idx_3 = 0.0;
          if (j + 2 <= 8) {
            v_next_idx_0 = value[i + ((j + 1) << 3)];
          }
          if (j >= 1) {
            v_next_idx_1 = value[i + ((j - 1) << 3)];
          }
          if (i >= 1) {
            v_next_idx_2 = value[(i + (j << 3)) - 1];
          }
          if (i + 2 <= 8) {
            v_next_idx_3 = value[(i + (j << 3)) + 1];
          }
          memcpy(&reward_cheese[0], &dv[0], 16U * sizeof(double));
          if (j + 2 > 8) {
            reward_cheese[0] = 0.0;
            reward_cheese[1] = 0.0;
            reward_cheese[2] = 0.0;
            reward_cheese[3] = 0.0;
          }
          if (j < 1) {
            reward_cheese[4] = 0.0;
            reward_cheese[5] = 0.0;
            reward_cheese[6] = 0.0;
            reward_cheese[7] = 0.0;
          }
          if (i < 1) {
            reward_cheese[8] = 0.0;
            reward_cheese[9] = 0.0;
            reward_cheese[10] = 0.0;
            reward_cheese[11] = 0.0;
          }
          if (i + 2 > 8) {
            reward_cheese[12] = 0.0;
            reward_cheese[13] = 0.0;
            reward_cheese[14] = 0.0;
            reward_cheese[15] = 0.0;
          }
          for (a = 0; a < 4; a++) {
            d = reward_cheese[a + 4];
            dis3_idx_0 = reward_cheese[a + 8];
            dis4_idx_0 = reward_cheese[a + 12];
            d1 = reward_cheese[a];
            eklenti = ((d1 + d) + dis3_idx_0) + dis4_idx_0;
            if (eklenti != 1.0) {
              eklenti =
                  (1.0 - eklenti) /
                  (double)((((d1 != 0.0) + (d != 0.0)) + (dis3_idx_0 != 0.0)) +
                           (dis4_idx_0 != 0.0));
              if (d1 != 0.0) {
                reward_cheese[a] = d1 + eklenti;
              }
              if (d != 0.0) {
                d += eklenti;
                reward_cheese[a + 4] = d;
              }
              if (dis3_idx_0 != 0.0) {
                dis3_idx_0 += eklenti;
                reward_cheese[a + 8] = dis3_idx_0;
              }
              if (dis4_idx_0 != 0.0) {
                dis4_idx_0 += eklenti;
                reward_cheese[a + 12] = dis4_idx_0;
              }
            }
          }
          dis3_idx_0 = (mice_idx_0 - (double)cat_idx_0) + -1.0;
          dis4_idx_0 = (mice_idx_0 - (double)cat_idx_0) + 1.0;
          dis1_idx_1 = (mice_idx_1 - (double)cat_idx_1) + 1.0;
          eklenti = (mice_idx_1 - (double)cat_idx_1) + -1.0;
          action_fear[0] =
              sqrt(dis_idx_0_tmp * dis_idx_0_tmp + dis1_idx_1 * dis1_idx_1);
          action_fear[1] =
              sqrt(dis_idx_0_tmp * dis_idx_0_tmp + eklenti * eklenti);
          action_fear[2] =
              sqrt(dis3_idx_0 * dis3_idx_0 + dis_idx_1_tmp * dis_idx_1_tmp);
          action_fear[3] =
              sqrt(dis4_idx_0 * dis4_idx_0 + dis_idx_1_tmp * dis_idx_1_tmp);
          eklenti = maximum(action_fear);
          x[0] = (eklenti == action_fear[0]);
          x[1] = (eklenti == action_fear[1]);
          x[2] = (eklenti == action_fear[2]);
          x[3] = (eklenti == action_fear[3]);
          eml_find(x, ind_hunger_data, ind_hunger_size);
          loop_ub = ind_hunger_size[1];
          if (0 <= loop_ub - 1) {
            memcpy(&ind_fear_data[0], &ind_hunger_data[0],
                   loop_ub * sizeof(int));
          }
          dis3_idx_0 = (mice_idx_0 - cheese[0]) + -1.0;
          dis4_idx_0 = (mice_idx_0 - cheese[0]) + 1.0;
          dis1_idx_1 = (mice_idx_1 - cheese[1]) + 1.0;
          eklenti = (mice_idx_1 - cheese[1]) + -1.0;
          action_fear[0] =
              sqrt(dis1_idx_0 * dis1_idx_0 + dis1_idx_1 * dis1_idx_1);
          action_fear[1] = sqrt(dis2_idx_0 * dis2_idx_0 + eklenti * eklenti);
          action_fear[2] =
              sqrt(dis3_idx_0 * dis3_idx_0 + dis3_idx_1 * dis3_idx_1);
          action_fear[3] =
              sqrt(dis4_idx_0 * dis4_idx_0 + dis4_idx_1 * dis4_idx_1);
          eklenti = minimum(action_fear);
          x[0] = (eklenti == action_fear[0]);
          x[1] = (eklenti == action_fear[1]);
          x[2] = (eklenti == action_fear[2]);
          x[3] = (eklenti == action_fear[3]);
          eml_find(x, ind_hunger_data, ind_hunger_size);
          for (a = 0; a < 4; a++) {
            reward_cheese_tmp = a << 2;
            for (b_i = 0; b_i < 4; b_i++) {
              b_reward_cheese_tmp = b_i << 2;
              c_reward_cheese[b_reward_cheese_tmp] =
                  (reward_cheese[reward_cheese_tmp] != 0.0);
              c_reward_cheese[b_reward_cheese_tmp + 1] =
                  (reward_cheese[reward_cheese_tmp + 1] != 0.0);
              c_reward_cheese[b_reward_cheese_tmp + 2] =
                  (reward_cheese[reward_cheese_tmp + 2] != 0.0);
              c_reward_cheese[b_reward_cheese_tmp + 3] =
                  (reward_cheese[reward_cheese_tmp + 3] != 0.0);
            }
            all(c_reward_cheese, x);
            if (ifWhileCond(x)) {
              a_size[0] = 1;
              a_size[1] = loop_ub;
              for (b_i = 0; b_i < loop_ub; b_i++) {
                x[b_i] = (a + 1 == ind_fear_data[b_i]);
              }
              if (b_ifWhileCond(x, a_size)) {
                reward_cheese_tmp = a << 2;
                reward_cheese[reward_cheese_tmp] += fear;
                reward_cheese_tmp = (a << 2) + 1;
                reward_cheese[reward_cheese_tmp] += fear;
                reward_cheese_tmp = (a << 2) + 2;
                reward_cheese[reward_cheese_tmp] += fear;
                reward_cheese_tmp = (a << 2) + 3;
                reward_cheese[reward_cheese_tmp] += fear;
              } else {
                action_fear[0] = reward_cheese[a];
                action_fear[1] = reward_cheese[a + 4];
                action_fear[2] = reward_cheese[a + 8];
                action_fear[3] = reward_cheese[a + 12];
                eklenti = fear / ((double)intnnz(action_fear) - 1.0);
                reward_cheese_tmp = a << 2;
                reward_cheese[reward_cheese_tmp] -= eklenti;
                reward_cheese_tmp = (a << 2) + 1;
                reward_cheese[reward_cheese_tmp] -= eklenti;
                reward_cheese_tmp = (a << 2) + 2;
                reward_cheese[reward_cheese_tmp] -= eklenti;
                reward_cheese_tmp = (a << 2) + 3;
                reward_cheese[reward_cheese_tmp] -= eklenti;
              }
              a_size[0] = 1;
              it = ind_hunger_size[1];
              a_size[1] = ind_hunger_size[1];
              for (b_i = 0; b_i < it; b_i++) {
                x[b_i] = (a + 1 == ind_hunger_data[b_i]);
              }
              if (b_ifWhileCond(x, a_size)) {
                reward_cheese_tmp = a << 2;
                reward_cheese[reward_cheese_tmp] += hunger;
                reward_cheese_tmp = (a << 2) + 1;
                reward_cheese[reward_cheese_tmp] += hunger;
                reward_cheese_tmp = (a << 2) + 2;
                reward_cheese[reward_cheese_tmp] += hunger;
                reward_cheese_tmp = (a << 2) + 3;
                reward_cheese[reward_cheese_tmp] += hunger;
              } else {
                a_size[0] = 1;
                it = ind_hunger_size[1];
                a_size[1] = ind_hunger_size[1];
                for (b_i = 0; b_i < it; b_i++) {
                  x[b_i] = (a + 1 != ind_hunger_data[b_i]);
                }
                if (b_ifWhileCond(x, a_size)) {
                  action_fear[0] = reward_cheese[a];
                  action_fear[1] = reward_cheese[a + 4];
                  action_fear[2] = reward_cheese[a + 8];
                  action_fear[3] = reward_cheese[a + 12];
                  eklenti = hunger / ((double)intnnz(action_fear) - 1.0);
                  reward_cheese_tmp = a << 2;
                  reward_cheese[reward_cheese_tmp] -= eklenti;
                  reward_cheese_tmp = (a << 2) + 1;
                  reward_cheese[reward_cheese_tmp] -= eklenti;
                  reward_cheese_tmp = (a << 2) + 2;
                  reward_cheese[reward_cheese_tmp] -= eklenti;
                  reward_cheese_tmp = (a << 2) + 3;
                  reward_cheese[reward_cheese_tmp] -= eklenti;
                }
              }
            }
          }
          for (a = 0; a < 4; a++) {
            d = reward_cheese[a];
            dis3_idx_0 = d;
            if (d >= 1.0) {
              dis3_idx_0 = 1.0;
              d = 1.0;
            }
            if (dis3_idx_0 <= 0.0) {
              d = 0.0;
            }
            dis4_idx_0 = reward_cheese[a + 4];
            dis3_idx_0 = dis4_idx_0;
            if (dis4_idx_0 >= 1.0) {
              dis3_idx_0 = 1.0;
              dis4_idx_0 = 1.0;
            }
            if (dis3_idx_0 <= 0.0) {
              dis4_idx_0 = 0.0;
            }
            d1 = reward_cheese[a + 8];
            dis3_idx_0 = d1;
            if (d1 >= 1.0) {
              dis3_idx_0 = 1.0;
              d1 = 1.0;
            }
            if (dis3_idx_0 <= 0.0) {
              d1 = 0.0;
            }
            eklenti = reward_cheese[a + 12];
            dis3_idx_0 = eklenti;
            if (eklenti >= 1.0) {
              dis3_idx_0 = 1.0;
              eklenti = 1.0;
            }
            if (dis3_idx_0 <= 0.0) {
              eklenti = 0.0;
            }
            dis1_idx_1 = ((d + dis4_idx_0) + d1) + eklenti;
            d /= dis1_idx_1;
            reward_cheese[a] = d;
            dis3_idx_0 = d * v_next_idx_0;
            d = dis4_idx_0 / dis1_idx_1;
            reward_cheese[a + 4] = d;
            dis3_idx_0 += d * v_next_idx_1;
            d = d1 / dis1_idx_1;
            reward_cheese[a + 8] = d;
            dis3_idx_0 += d * v_next_idx_2;
            d = eklenti / dis1_idx_1;
            reward_cheese[a + 12] = d;
            dis3_idx_0 += d * v_next_idx_3;
            b_reward_cheese[a] = dis3_idx_0;
          }
          b_i = i + (j << 3);
          d = value[b_i]; /*DeÄŸerleri Hesaplar*/
          d += 0.9 * ((reward[b_i] + 0.9 * maximum(b_reward_cheese)) - d);
          value[b_i] = d;
        }
      }
    }
    v_next_idx_0 = 0.0; /*Policy HesaplanÄ±r*/
    v_next_idx_1 = 0.0;
    v_next_idx_2 = 0.0;
    v_next_idx_3 = 0.0;
    if (mice_idx_1 + 1.0 <= 8.0) {
      v_next_idx_0 =
          value[((int)mice_idx_0 + (((int)(mice_idx_1 + 1.0) - 1) << 3)) - 1];
    }
    if (mice_idx_1 - 1.0 >= 1.0) {
      v_next_idx_1 =
          value[((int)mice_idx_0 + (((int)(mice_idx_1 - 1.0) - 1) << 3)) - 1];
    }
    if (mice_idx_0 - 1.0 >= 1.0) {
      v_next_idx_2 =
          value[((int)(mice_idx_0 - 1.0) + (((int)mice_idx_1 - 1) << 3)) - 1];
    }
    if (mice_idx_0 + 1.0 <= 8.0) {
      v_next_idx_3 =
          value[((int)(mice_idx_0 + 1.0) + (((int)mice_idx_1 - 1) << 3)) - 1];
    }
    memcpy(&reward_cheese[0], &dv[0], 16U * sizeof(double));
    if (mice_idx_1 + 1.0 > 8.0) {
      reward_cheese[0] = 0.0;
      reward_cheese[1] = 0.0;
      reward_cheese[2] = 0.0;
      reward_cheese[3] = 0.0;
    }
    if (mice_idx_1 - 1.0 < 1.0) {
      reward_cheese[4] = 0.0;
      reward_cheese[5] = 0.0;
      reward_cheese[6] = 0.0;
      reward_cheese[7] = 0.0;
    }
    if (mice_idx_0 - 1.0 < 1.0) {
      reward_cheese[8] = 0.0;
      reward_cheese[9] = 0.0;
      reward_cheese[10] = 0.0;
      reward_cheese[11] = 0.0;
    }
    if (mice_idx_0 + 1.0 > 8.0) {
      reward_cheese[12] = 0.0;
      reward_cheese[13] = 0.0;
      reward_cheese[14] = 0.0;
      reward_cheese[15] = 0.0;
    }
    for (a = 0; a < 4; a++) {
      d = reward_cheese[a + 4];
      dis3_idx_0 = reward_cheese[a + 8];
      dis4_idx_0 = reward_cheese[a + 12];
      d1 = reward_cheese[a];
      eklenti = ((d1 + d) + dis3_idx_0) + dis4_idx_0;
      if (eklenti != 1.0) {
        eklenti = (1.0 - eklenti) /
                  (double)((((d1 != 0.0) + (d != 0.0)) + (dis3_idx_0 != 0.0)) +
                           (dis4_idx_0 != 0.0));
        if (d1 != 0.0) {
          reward_cheese[a] = d1 + eklenti;
        }
        if (d != 0.0) {
          d += eklenti;
          reward_cheese[a + 4] = d;
        }
        if (dis3_idx_0 != 0.0) {
          dis3_idx_0 += eklenti;
          reward_cheese[a + 8] = dis3_idx_0;
        }
        if (dis4_idx_0 != 0.0) {
          dis4_idx_0 += eklenti;
          reward_cheese[a + 12] = dis4_idx_0;
        }
      }
    }
    dis4_idx_1 = dis_idx_0_tmp * dis_idx_0_tmp;
    action_fear[0] =
        sqrt(dis4_idx_1 + (dis_idx_1_tmp + 1.0) * (dis_idx_1_tmp + 1.0));
    action_fear[1] =
        sqrt(dis4_idx_1 + (dis_idx_1_tmp + -1.0) * (dis_idx_1_tmp + -1.0));
    dis3_idx_1 = dis_idx_1_tmp * dis_idx_1_tmp;
    action_fear[2] =
        sqrt((dis_idx_0_tmp + -1.0) * (dis_idx_0_tmp + -1.0) + dis3_idx_1);
    action_fear[3] =
        sqrt((dis_idx_0_tmp + 1.0) * (dis_idx_0_tmp + 1.0) + dis3_idx_1);
    eklenti = maximum(action_fear);
    x[0] = (eklenti == action_fear[0]);
    x[1] = (eklenti == action_fear[1]);
    x[2] = (eklenti == action_fear[2]);
    x[3] = (eklenti == action_fear[3]);
    eml_find(x, ind_hunger_data, ind_hunger_size);
    it = ind_hunger_size[1];
    loop_ub = ind_hunger_size[1];
    if (0 <= loop_ub - 1) {
      memcpy(&ind_fear_data[0], &ind_hunger_data[0], loop_ub * sizeof(int));
    }
    d = mice_idx_0 - cheese[0];
    dis1_idx_0 = d;
    dis2_idx_0 = d;
    dis3_idx_0 = d + -1.0;
    d++;
    dis4_idx_0 = d;
    d = mice_idx_1 - cheese[1];
    action_fear[0] = sqrt(dis1_idx_0 * dis1_idx_0 + (d + 1.0) * (d + 1.0));
    action_fear[1] = sqrt(dis2_idx_0 * dis2_idx_0 + (d + -1.0) * (d + -1.0));
    eklenti = d * d;
    action_fear[2] = sqrt(dis3_idx_0 * dis3_idx_0 + eklenti);
    action_fear[3] = sqrt(dis4_idx_0 * dis4_idx_0 + eklenti);
    eklenti = minimum(action_fear);
    x[0] = (eklenti == action_fear[0]);
    x[1] = (eklenti == action_fear[1]);
    x[2] = (eklenti == action_fear[2]);
    x[3] = (eklenti == action_fear[3]);
    eml_find(x, ind_hunger_data, ind_hunger_size);
    for (a = 0; a < 4; a++) {
      reward_cheese_tmp = a << 2;
      for (b_i = 0; b_i < 4; b_i++) {
        b_reward_cheese_tmp = b_i << 2;
        c_reward_cheese[b_reward_cheese_tmp] =
            (reward_cheese[reward_cheese_tmp] != 0.0);
        c_reward_cheese[b_reward_cheese_tmp + 1] =
            (reward_cheese[reward_cheese_tmp + 1] != 0.0);
        c_reward_cheese[b_reward_cheese_tmp + 2] =
            (reward_cheese[reward_cheese_tmp + 2] != 0.0);
        c_reward_cheese[b_reward_cheese_tmp + 3] =
            (reward_cheese[reward_cheese_tmp + 3] != 0.0);
      }
      all(c_reward_cheese, x);
      if (ifWhileCond(x)) {
        a_size[0] = 1;
        a_size[1] = it;
        for (b_i = 0; b_i < it; b_i++) {
          x[b_i] = (a + 1 == ind_fear_data[b_i]);
        }
        if (b_ifWhileCond(x, a_size)) {
          reward_cheese_tmp = a << 2;
          reward_cheese[reward_cheese_tmp] += fear;
          reward_cheese_tmp = (a << 2) + 1;
          reward_cheese[reward_cheese_tmp] += fear;
          reward_cheese_tmp = (a << 2) + 2;
          reward_cheese[reward_cheese_tmp] += fear;
          reward_cheese_tmp = (a << 2) + 3;
          reward_cheese[reward_cheese_tmp] += fear;
        } else {
          action_fear[0] = reward_cheese[a];
          action_fear[1] = reward_cheese[a + 4];
          action_fear[2] = reward_cheese[a + 8];
          action_fear[3] = reward_cheese[a + 12];
          eklenti = fear / ((double)intnnz(action_fear) - 1.0);
          reward_cheese_tmp = a << 2;
          reward_cheese[reward_cheese_tmp] -= eklenti;
          reward_cheese_tmp = (a << 2) + 1;
          reward_cheese[reward_cheese_tmp] -= eklenti;
          reward_cheese_tmp = (a << 2) + 2;
          reward_cheese[reward_cheese_tmp] -= eklenti;
          reward_cheese_tmp = (a << 2) + 3;
          reward_cheese[reward_cheese_tmp] -= eklenti;
        }
        a_size[0] = 1;
        loop_ub = ind_hunger_size[1];
        a_size[1] = ind_hunger_size[1];
        for (b_i = 0; b_i < loop_ub; b_i++) {
          x[b_i] = (a + 1 == ind_hunger_data[b_i]);
        }
        if (b_ifWhileCond(x, a_size)) {
          reward_cheese_tmp = a << 2;
          reward_cheese[reward_cheese_tmp] += hunger;
          reward_cheese_tmp = (a << 2) + 1;
          reward_cheese[reward_cheese_tmp] += hunger;
          reward_cheese_tmp = (a << 2) + 2;
          reward_cheese[reward_cheese_tmp] += hunger;
          reward_cheese_tmp = (a << 2) + 3;
          reward_cheese[reward_cheese_tmp] += hunger;
        } else {
          a_size[0] = 1;
          loop_ub = ind_hunger_size[1];
          a_size[1] = ind_hunger_size[1];
          for (b_i = 0; b_i < loop_ub; b_i++) {
            x[b_i] = (a + 1 != ind_hunger_data[b_i]);
          }
          if (b_ifWhileCond(x, a_size)) {
            action_fear[0] = reward_cheese[a];
            action_fear[1] = reward_cheese[a + 4];
            action_fear[2] = reward_cheese[a + 8];
            action_fear[3] = reward_cheese[a + 12];
            eklenti = hunger / ((double)intnnz(action_fear) - 1.0);
            reward_cheese_tmp = a << 2;
            reward_cheese[reward_cheese_tmp] -= eklenti;
            reward_cheese_tmp = (a << 2) + 1;
            reward_cheese[reward_cheese_tmp] -= eklenti;
            reward_cheese_tmp = (a << 2) + 2;
            reward_cheese[reward_cheese_tmp] -= eklenti;
            reward_cheese_tmp = (a << 2) + 3;
            reward_cheese[reward_cheese_tmp] -= eklenti;
          }
        }
      }
    }
    for (a = 0; a < 4; a++) {
      d = reward_cheese[a];
      dis3_idx_0 = d;
      if (d >= 1.0) {
        dis3_idx_0 = 1.0;
        d = 1.0;
      }
      if (dis3_idx_0 <= 0.0) {
        d = 0.0;
      }
      dis4_idx_0 = reward_cheese[a + 4];
      dis3_idx_0 = dis4_idx_0;
      if (dis4_idx_0 >= 1.0) {
        dis3_idx_0 = 1.0;
        dis4_idx_0 = 1.0;
      }
      if (dis3_idx_0 <= 0.0) {
        dis4_idx_0 = 0.0;
      }
      d1 = reward_cheese[a + 8];
      dis3_idx_0 = d1;
      if (d1 >= 1.0) {
        dis3_idx_0 = 1.0;
        d1 = 1.0;
      }
      if (dis3_idx_0 <= 0.0) {
        d1 = 0.0;
      }
      eklenti = reward_cheese[a + 12];
      dis3_idx_0 = eklenti;
      if (eklenti >= 1.0) {
        dis3_idx_0 = 1.0;
        eklenti = 1.0;
      }
      if (dis3_idx_0 <= 0.0) {
        eklenti = 0.0;
      }
      dis1_idx_1 = ((d + dis4_idx_0) + d1) + eklenti;
      d /= dis1_idx_1;
      reward_cheese[a] = d;
      dis3_idx_0 = d * v_next_idx_0;
      d = dis4_idx_0 / dis1_idx_1;
      reward_cheese[a + 4] = d;
      dis3_idx_0 += d * v_next_idx_1;
      d = d1 / dis1_idx_1;
      reward_cheese[a + 8] = d;
      dis3_idx_0 += d * v_next_idx_2;
      d = eklenti / dis1_idx_1;
      reward_cheese[a + 12] = d;
      dis3_idx_0 += d * v_next_idx_3;
      b_reward_cheese[a] = dis3_idx_0;
    }
    action_fear[0] = b_reward_cheese[0];
    action_fear[1] = b_reward_cheese[1];
    action_fear[2] = b_reward_cheese[2];
    action_fear[3] = b_reward_cheese[3];
    eklenti = maximum(action_fear);
    z_tmp = 0;
    if (eklenti == b_reward_cheese[0]) {
      z_tmp = 1;
    }
    if (eklenti == b_reward_cheese[1]) {
      z_tmp++;
    }
    if (eklenti == b_reward_cheese[2]) {
      z_tmp++;
    }
    if (eklenti == b_reward_cheese[3]) {
      z_tmp++;
    }
    it = 0;
    if (eklenti == b_reward_cheese[0]) {
      tmp_data[0] = 1;
      it = 1;
    }
    if (eklenti == b_reward_cheese[1]) {
      tmp_data[it] = 2;
      it++;
    }
    if (eklenti == b_reward_cheese[2]) {
      tmp_data[it] = 3;
      it++;
    }
    if (eklenti == b_reward_cheese[3]) {
      tmp_data[it] = 4;
    }
    x_size[0] = z_tmp;
    x_size[1] = 4;
    for (b_i = 0; b_i < 4; b_i++) {
      for (reward_cheese_tmp = 0; reward_cheese_tmp < z_tmp;
           reward_cheese_tmp++) {
        x_data[reward_cheese_tmp + z_tmp * b_i] =
            reward_cheese[(tmp_data[reward_cheese_tmp] + (b_i << 2)) - 1];
      }
    }
    useConstantDim(x_data, x_size, nonSingletonDim(x_size));
    if (x_size[0] != 0) {
      cat_idx_0 = 4;
    } else {
      cat_idx_0 = 0;
    }
    
    ////////////////////////////////////////////////////////////
    
    
    /*eklenti = b_rand();*/
            
    eklenti=(float)rand()/((float)RAND_MAX/1);
    printf("%f",eklenti);
    r_size[0] = 1;
    r_size[1] = cat_idx_0 + 1;
    r_data[0] = (eklenti > 0.0);
    ////////////////////////////////////////////////////////////
    
    loop_ub = cat_idx_0;
    for (b_i = 0; b_i < loop_ub; b_i++) {
      r_data[b_i + 1] = (eklenti > x_data[b_i]);
    }
    b_eml_find(r_data, r_size, (int *)&it, ind_hunger_size);
    loop_ub = ind_hunger_size[1];
    for (b_i = 0; b_i < loop_ub; b_i++) {
      cat_idx_0 = (signed char)((signed char)(it - 1) + 1);
    }
    action_size[0] = 1;
    action_size[1] = ind_hunger_size[1];
    loop_ub = ind_hunger_size[1];
    for (b_i = 0; b_i < loop_ub; b_i++) {
      action_data = (cat_idx_0 == 1);
    }
    
    
    if (b_ifWhileCond((boolean_T *)&action_data, action_size)) {
      mice_idx_1++; /*Fareyi YukarÄ± TaÅŸÄ±r*/
      ileri_git();
    } else {
      action_size[0] = 1;
      action_size[1] = ind_hunger_size[1];
      loop_ub = ind_hunger_size[1];
      for (b_i = 0; b_i < loop_ub; b_i++) {
        action_data = (cat_idx_0 == 2);
      }
      if (b_ifWhileCond((boolean_T *)&action_data, action_size)) {
        mice_idx_1--;/*Fareyi AÅŸaÄŸÄ± TaÅŸÄ±r*/
        geri_git();
      } else {
        action_size[0] = 1;
        action_size[1] = ind_hunger_size[1];
        loop_ub = ind_hunger_size[1];
        for (b_i = 0; b_i < loop_ub; b_i++) {
          action_data = (cat_idx_0 == 3);
        }
        if (b_ifWhileCond((boolean_T *)&action_data, action_size)) {
          mice_idx_0--;/*Fareyi Sola TaÅŸÄ±r*/
          sola_git();
        } else {
          action_size[0] = 1;
          action_size[1] = ind_hunger_size[1];
          loop_ub = ind_hunger_size[1];
          for (b_i = 0; b_i < loop_ub; b_i++) {
            action_data = (cat_idx_0 == 4);
          }
          if (b_ifWhileCond((boolean_T *)&action_data, action_size)) {
            mice_idx_0++;/*Fareyi SaÄŸa TaÅŸÄ±r*/
            saga_git();
          }
        }
      }
    }
    
    iteration++;
    if (rtIsInf(iteration)) {
      eklenti = rtNaN;
    } else {
      eklenti = fmod(iteration, 8.0);
    }
    
    /////////////////////////////////////////////////////////////////////////////
    
    cat_idx_0 = cat_array[(int)(eklenti + 1.0) - 1];
    dis1_idx_0 = mice_idx_0 - (double)cat_idx_0;
    cat_idx_1 = cat_array[(int)(eklenti + 1.0) + 7];
    dis1_idx_1 = mice_idx_1 - (double)cat_idx_1;
    
    if (eklenti == 0) {
      sol_uste_git_kedy();
      }
    if (eklenti == 1) {
      sol_alta_git_kedy();
      }
    if (eklenti == 2) {
      sol_alta_git_kedy();
      }        
    if (eklenti == 3) {
      sag_alta_git_kedy();
      }    
    if (eklenti == 4) {
      sag_alta_git_kedy();
      }  
    if (eklenti == 5) {
      sag_uste_git_kedy();
      }  
    if (eklenti == 6) {
      sag_uste_git_kedy();
      } 
    if (eklenti == 6) {
      sol_uste_git_kedy();
      } 
      
    
    /////////////////////////////////////////////////////////////////////////////
    
    fear += (sqrt(dis4_idx_1 + dis3_idx_1) -
             sqrt(dis1_idx_0 * dis1_idx_0 + dis1_idx_1 * dis1_idx_1)) /
            1000.0;
    if (peynir_flag == 0) {
      for (b_i = 0; b_i < 64; b_i++) {
        reward[b_i] = 0.01;
      }
      reward[((int)cheese[0] + (((int)cheese[1] - 1) << 3)) - 1] = 10.0;
      if (rtIsInf(iteration)) {
        d = rtNaN;
        d1 = rtNaN;
      } else {
        d = fmod(iteration, 8.0);
        d1 = fmod(iteration, 8.0);
      }
      reward[(cat_array[(int)(d + 1.0) - 1] +
              ((cat_array[(int)(d1 + 1.0) + 7] - 1) << 3)) -
             1] = -50.0;
      for (b_i = 0; b_i < 16; b_i++) {
        z[b_i] = 0;
      }
      it = 0;
      for (i = 0; i < 3; i++) {
        z[it] = (signed char)(i - 1);
        z[it + 8] = -1;
        it++;
        if (i - 1 != 0) {
          z[it] = (signed char)(i - 1);
          z[it + 8] = 0;
          it++;
        }
        z[it] = (signed char)(i - 1);
        z[it + 8] = 1;
        it++;
      }
      for (b_i = 0; b_i < 2; b_i++) {
        reward_cheese_tmp = b_i << 3;
        for (it = 0; it < 8; it++) {
          z_tmp = it + reward_cheese_tmp;
          i1 = z[z_tmp];
          reward_cheese[z_tmp] = cheese[b_i] - (double)i1;
          z[z_tmp] = (signed char)(cat_array[((int)(eklenti + 1.0) +
                                              reward_cheese_tmp) -
                                             1] -
                                   i1);
        }
      }
      for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
          for (b_i = 0; b_i < 8; b_i++) {
            c_i[b_i] = ((double)i + 1.0 == reward_cheese[b_i]);
          }
          if (any(c_i)) {
            for (b_i = 0; b_i < 8; b_i++) {
              c_i[b_i] = ((double)j + 1.0 == reward_cheese[b_i + 8]);
            }
            if (any(c_i)) {
              it = i + (j << 3);
              reward[it] += 10.0;
            }
          }
          for (b_i = 0; b_i < 8; b_i++) {
            c_i[b_i] = (i + 1 == z[b_i]);
          }
          if (any(c_i)) {
            for (b_i = 0; b_i < 8; b_i++) {
              c_i[b_i] = (j + 1 == z[b_i + 8]);
            }
            if (any(c_i)) {
              it = i + (j << 3);
              reward[it] += -50.0;
            }
          }
        }
      }
    } else {
      for (b_i = 0; b_i < 64; b_i++) {
        reward[b_i] = 0.01;
      }
      reward[0] = 10.0;
      if (rtIsInf(iteration)) {
        d = rtNaN;
        d1 = rtNaN;
      } else {
        d = fmod(iteration, 8.0);
        d1 = fmod(iteration, 8.0);
      }
      reward[(cat_array[(int)(d + 1.0) - 1] +
              ((cat_array[(int)(d1 + 1.0) + 7] - 1) << 3)) -
             1] = -50.0;
      for (b_i = 0; b_i < 16; b_i++) {
        z[b_i] = 0;
      }
      it = 0;
      for (i = 0; i < 3; i++) {
        z[it] = (signed char)(i - 1);
        z[it + 8] = -1;
        it++;
        if (i - 1 != 0) {
          z[it] = (signed char)(i - 1);
          z[it + 8] = 0;
          it++;
        }
        z[it] = (signed char)(i - 1);
        z[it + 8] = 1;
        it++;
      }
      for (b_i = 0; b_i < 16; b_i++) {
        reward_cheese[b_i] = 1.0 - (double)z[b_i];
      }
      for (b_i = 0; b_i < 2; b_i++) {
        it = b_i << 3;
        for (reward_cheese_tmp = 0; reward_cheese_tmp < 8;
             reward_cheese_tmp++) {
          z_tmp = reward_cheese_tmp + it;
          z[z_tmp] = (signed char)(cat_array[((int)(eklenti + 1.0) + it) - 1] -
                                   z[z_tmp]);
        }
      }
      for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
          for (b_i = 0; b_i < 8; b_i++) {
            c_i[b_i] = (i + 1 == (int)reward_cheese[b_i]);
          }
          if (any(c_i)) {
            for (b_i = 0; b_i < 8; b_i++) {
              c_i[b_i] = (j + 1 == (int)reward_cheese[b_i + 8]);
            }
            if (any(c_i)) {
              it = i + (j << 3);
              reward[it] += 10.0;
            }
          }
          for (b_i = 0; b_i < 8; b_i++) {
            c_i[b_i] = (i + 1 == z[b_i]);
          }
          if (any(c_i)) {
            for (b_i = 0; b_i < 8; b_i++) {
              c_i[b_i] = (j + 1 == z[b_i + 8]);
            }
            if (any(c_i)) {
              it = i + (j << 3);
              reward[it] += -50.0;
            }
          }
        }
      }
      cheese[0] = mice_idx_0;
      cheese[1] = mice_idx_1;
    }
        
    mice[0] = (mice_idx_0 == cheese[0]);
    mice[1] = (mice_idx_1 == cheese[1]);
    if (b_all(mice) && (peynir_flag == 0)) {
      hunger = 0.0;
      peynir_flag = 1;
    } else {
      hunger += 0.001;
    }
    mice[0] = (mice_idx_0 == cat_array[(int)(eklenti + 1.0) - 1]); /*Kediyi Hareket Ettirir*/
    mice[1] = (mice_idx_1 == cat_array[(int)(eklenti + 1.0) + 7]);
    if (b_all(mice)) {
      exitg1 = 1;
    } else {
      mice[0] = (mice_idx_0 == 1.0);
      mice[1] = (mice_idx_1 == 1.0);
      if (b_all(mice) && (peynir_flag == 1)) {
        exitg1 = 1;
      }
    }
  } while (exitg1 == 0);
}

//////////////////////////////////////////////////////////

int main() {
  initialize();
  srand((unsigned int)time(NULL));  
  initialize();
  mdp_run_run();
  stop(1);
}
