/*
 * Copyright (C) 2021 EPFL-REHAssist (Rehabilitation and Assistive Robotics Group).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "haptic_controller.h"
#include "communication.h"
#include "drivers/adc.h"
#include "drivers/incr_encoder.h"
#include "drivers/hall.h"
#include "drivers/callback_timers.h"
#include "lib/utils.h"
#include "torque_regulator.h"
#include "drivers/led.h"

#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 350 // Default control loop period [us].
#define TWO_DIV_INTEGRATION true

volatile uint32_t  hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_encoderPaddleAngle; // Paddle angle measured by the incremental encoder [deg].
volatile float32_t hapt_motorTorque; // Motor torque [N.m].
volatile float32_t stiffness; //spring [N.m/deg]
volatile float32_t damping; //spring [N.m/deg]
volatile float32_t rest_angle; //resting position [deg]
volatile float32_t est_speed; //estimate of the speed [deg/s]
volatile float32_t est_speed_hall; //estimate of the speed from hall [deg/s]
volatile float32_t est_acc; //estimate of the speed [deg/s^2]
volatile float32_t est_acc_hall; //estimate of the speed from hall [deg/s^2]
volatile float32_t est_angle_from_hall; //[deg]
volatile float32_t filtered_speed_hall; //filtered hall speed [deg/s]
volatile float32_t filtered_acc_hall; //filtered hall acceleration [deg/s^2]

volatile float32_t prev_angle = 0.0f;
volatile float32_t prev_angle_hall = 0.0f;
volatile float32_t prev2_angle = 0.0f;
volatile float32_t prev2_angle_hall = 0.0f;

volatile float32_t prev_est_speed = 0.0f;
volatile float32_t prev_est_speed_hall = 0.0f;
volatile float32_t prev2_est_speed = 0.0f;
volatile float32_t prev2_est_speed_hall = 0.0f;

volatile float32_t cutoff_freq; //cutoff frequency for the low pass filter

volatile float32_t k_p = 0.0f;
volatile float32_t k_d = 0.0f;
volatile float32_t k_i = 0.0f;
volatile float32_t ref_pos = 0.0f;

volatile float32_t error = 0.0f;
volatile float32_t integral = 0.0f;
volatile float32_t derivative = 0.0f;
volatile float32_t prev_derivative = 0.0f;
volatile float32_t prev_error = 0.0f;

volatile float32_t set_PID = 0.0f;
volatile float32_t start_dry_test = 0.0f;

float32_t friction_torque = 0.0f; // [N.m].
float32_t step = 0.0001f; // [-].
float32_t counter = 0.0f; // [-].

void hapt_Update(void);

/**
 * @brief Initializes the haptic controller.
 */
void hapt_Init(void)
{
    hapt_timestamp = 0;
    hapt_motorTorque = 0.0f;
    stiffness = 0.0f;
    damping = 0.0f;
    rest_angle = 0.0f;
    prev_angle = 0.0f;
    prev_angle_hall = 0.0f;
    est_angle_from_hall = 0.0f;
    filtered_speed_hall = 0.0f;
    filtered_acc_hall = 0.0f;
    cutoff_freq = 100.0f;


    // initialize PID controller
    k_p = 0.02f;
    k_i = 0.05;
    k_d = 0.0025f;

    // Make the timers call the update function periodically.
    cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

    // Share some variables with the computer.
    comm_monitorUint32Func("timestep [us]", cbt_GetHapticControllerPeriod,
                          cbt_SetHapticControllerPeriod);
    comm_monitorFloat("motor_torque [N.m]", (float32_t*)&hapt_motorTorque, READWRITE);
    comm_monitorFloat("encoder_paddle_pos [deg]", (float32_t*)&hapt_encoderPaddleAngle, READONLY);
    comm_monitorFloat("speed estimate [deg/s]", (float32_t*)&est_speed, READONLY);
    comm_monitorFloat("speed estimate hall [deg/s]", (float32_t*)&est_speed_hall, READONLY);
    comm_monitorFloat("acc estimate [deg/s^2]", (float32_t*)&est_acc, READONLY);
    comm_monitorFloat("acc estimate hall [deg/s^2]", (float32_t*)&est_acc_hall, READONLY);
    comm_monitorFloat("hall_voltage [V]", (float32_t*)&hapt_hallVoltage, READONLY);
    comm_monitorFloat("est angle hall [deg]", (float32_t*)&est_angle_from_hall, READONLY);
    comm_monitorFloat("filtered hall speed [deg/s]", (float32_t*)&filtered_speed_hall, READONLY);
    comm_monitorFloat("filtered hall acc [deg/s^2]", (float32_t*)&filtered_acc_hall, READONLY);
    comm_monitorFloat("cutoff freq [Hz]", (float32_t*)&cutoff_freq, READWRITE);
    comm_monitorFloat("stiffness [N.m/deg]", (float32_t*)&stiffness, WRITEONLY);
    comm_monitorFloat("damping [N.m/deg]", (float32_t*)&damping, WRITEONLY);
    comm_monitorFloat("rest position [deg]", (float32_t*)&rest_angle, WRITEONLY);
    comm_monitorFloat("ref_pos [deg]", (float32_t*)&ref_pos, READWRITE);

    comm_monitorFloat("k_p", (float32_t*)&k_p, WRITEONLY);
    comm_monitorFloat("k_d", (float32_t*)&k_d, WRITEONLY);
    comm_monitorFloat("k_i", (float32_t*)&k_i, WRITEONLY);

    comm_monitorFloat("set PID 0/1", (float32_t*)&set_PID, WRITEONLY);
    comm_monitorFloat("start dry test 0/1", (float32_t*)&start_dry_test, WRITEONLY);

    comm_monitorFloat("error", (float32_t*)&error, READONLY);
    comm_monitorFloat("integral", (float32_t*)&integral, READONLY);
    comm_monitorFloat("derivative", (float32_t*)&derivative, READONLY);
}

/**
 * @brief Updates the haptic controller state.
 */
void hapt_Update()
{
  float32_t motorShaftAngle; // [deg].

  // Compute the dt (uncomment if you need it).
  float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f; // [s].

  // Increment the timestamp.
  hapt_timestamp += cbt_GetHapticControllerPeriod();

  // Get the Hall sensor voltage.
  hapt_hallVoltage = hall_GetVoltage();

  // Get the encoder position.
  motorShaftAngle = enc_GetPosition();
  hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;

  // Angle estimation from the Hall sensor
  float32_t slope = 53.4149;
  float32_t intercept = -133.5939;

  est_angle_from_hall = slope * hapt_hallVoltage + intercept;

  // Compute the motor torque, and apply it.
  // hapt_motorTorque = 0.0f;
  if (stiffness != 0)
  {
    hapt_motorTorque = -stiffness * (hapt_encoderPaddleAngle - rest_angle) * 0.01;
  }

  // speed estimation from optical sensor and Hall sensor
  if (TWO_DIV_INTEGRATION)
  {
    est_speed = (prev2_angle - hapt_encoderPaddleAngle) / (2 * dt);
    est_speed_hall = (prev2_angle_hall - est_angle_from_hall) / (2 * dt);
    // filtered_speed_hall = low_Pass_Filter(est_speed_hall, filtered_speed_hall, 2*dt, 10);
  }
  else
  {
    est_speed = (prev_angle - hapt_encoderPaddleAngle) / dt;
    est_speed_hall = (prev_angle_hall - est_angle_from_hall) / dt;
  }

  filtered_speed_hall = low_Pass_Filter(est_speed_hall, filtered_speed_hall, dt, cutoff_freq);

  // acceleration estimation from optical sensor and Hall sensor
  if (TWO_DIV_INTEGRATION)
  {
    est_acc = (prev2_est_speed - est_speed) / (2 * dt);
    est_acc_hall = (prev2_est_speed_hall - est_speed_hall) / (2 * dt);
    // filtered_acc_hall = low_Pass_Filter(est_acc_hall, filtered_acc_hall, 2*dt, 10);
  }
  else
  {
    est_acc = (prev_est_speed - est_speed) / dt;
    est_acc_hall = (prev_est_speed_hall - est_speed_hall) / dt;
  }

  filtered_acc_hall = low_Pass_Filter(est_acc_hall, filtered_acc_hall, dt, cutoff_freq);

  if (damping != 0)
  {
    if (hapt_encoderPaddleAngle - rest_angle <= 10 && hapt_encoderPaddleAngle - rest_angle >= -10)
    {
      hapt_motorTorque = hapt_motorTorque + damping * est_speed * 0.01;
    }
  }

  float32_t filtered_enc_pos = low_Pass_Filter(hapt_encoderPaddleAngle, prev_angle, dt, cutoff_freq);
  float32_t filtered_hall_pos = low_Pass_Filter(est_angle_from_hall, prev_angle_hall, dt, cutoff_freq);

  if (start_dry_test == 1 && counter >= 500)
  {
    counter = 0;
    if (hapt_encoderPaddleAngle >= -0.5 && hapt_encoderPaddleAngle <= 0.5)
    {
    	hapt_motorTorque = friction_torque;
    	friction_torque -= step;
    }
    else
    {
      for (int i = 0; i < 1000; i++)
      {
        start_dry_test = 0;
        friction_torque = 0;
      }
      hapt_motorTorque = 0.0f;
    }
  } else if (start_dry_test == 1) {
    counter += 1;
  }

  if (hapt_encoderPaddleAngle >= -0.1 && hapt_encoderPaddleAngle <= 0.1)
  {
    led_Set(0, 1.0);
  }else {
	led_Set(0, 0.0);
  }



  // PID controller
  if (set_PID == 1)
  {
    // hall encoder
    // error = ref_pos - est_angle_from_hall;
    // error = ref_pos - filtered_hall_pos;
    // incremental encoder
    error = ref_pos - hapt_encoderPaddleAngle;
    // error = ref_pos - filtered_enc_pos;
    integral = integral + error * dt;
    derivative = (error - prev_error) / dt;

    // derivative = low_Pass_Filter(derivative, prev_derivative, dt, cutoff_freq);

    // if (error >= -0.2 && error <= 0.2){
    //   hapt_motorTorque = 0;
    // } else {
    hapt_motorTorque = k_p * error + k_i * integral + k_d * derivative;

    if (hapt_motorTorque > 0.03)
    {
      hapt_motorTorque = 0.03;
    }
    else if (hapt_motorTorque < -0.03)
    {
      hapt_motorTorque = -0.03;
    }

    prev_error = error;
    prev_derivative = derivative;
  }

  torq_SetTorque(hapt_motorTorque);
  hapt_hallVoltage = hall_GetVoltage();

  // previous positions from optical and hall sensor
  prev2_angle = prev_angle;
  prev_angle = hapt_encoderPaddleAngle;

  prev2_angle_hall = prev_angle_hall;
  prev_angle_hall = est_angle_from_hall;

  // previous speed from optical and hall sensor
  prev2_est_speed = prev_est_speed;
  prev_est_speed = est_speed;

  prev2_est_speed_hall = prev_est_speed_hall;
  prev_est_speed_hall = est_speed_hall;
}

float32_t low_Pass_Filter(float32_t input, float32_t prev_output, float32_t T, float32_t cutoff_freq)
{
  float32_t alpha = T / (T + 1 / (2 * M_PI * cutoff_freq));
  return alpha * input + (1 - alpha) * prev_output;
}
