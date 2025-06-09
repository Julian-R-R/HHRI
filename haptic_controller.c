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
#include "drivers/adc.h"
#include <stdio.h>

// ========================== DEFINES & CONSTANTS ==========================
/* Control loop period and test mode */
#ifdef TEST_MODE_10000
volatile uint32_t DEFAULT_HAPTIC_CONTROLLER_PERIOD = 10000; // Default control loop period [us].
#else
volatile uint32_t DEFAULT_HAPTIC_CONTROLLER_PERIOD = 350; // Default control loop period [us].
#endif

#define TWO_DIV_INTEGRATION true

/* Oscillation detection parameters */
#define OSC_CENTER 15.0f
#define OSC_MARGIN 0.2f
#define OSC_THRESHOLD_MIN (OSC_CENTER - OSC_MARGIN)  // 14.5
#define OSC_THRESHOLD_MAX (OSC_CENTER + OSC_MARGIN)  // 15.5
#define OSC_DETECTION_PERIOD_US 1000000 // 1 second in microseconds
#define OSC_MAX_CROSSES 10

volatile float32_t small_threshold = 10.0f; // Threshold for speed detection [deg/s].

/* Test and torque parameters */
#define TEST_TIME_INTERVAL 500000 // 0.5 second in microseconds
#define AUTOMATIC_MANUAL_TORQUE 0.005f // [N.m].

// ========================== GLOBAL VARIABLES =============================

/* Oscillation detection state */
volatile uint32_t zero_cross_count = 0;
volatile uint32_t osc_start_time = 0;
volatile bool last_below_threshold = false;
volatile bool detected_oscillation = false;
volatile float32_t prev_speed = 0.0f;

/* Haptic controller state */
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

/* Previous values for estimation */
volatile float32_t prev_angle = 0.0f;
volatile float32_t prev_angle_hall = 0.0f;
volatile float32_t prev2_angle = 0.0f;
volatile float32_t prev2_angle_hall = 0.0f;

volatile float32_t prev_est_speed = 0.0f;
volatile float32_t prev_est_speed_hall = 0.0f;
volatile float32_t prev2_est_speed = 0.0f;
volatile float32_t prev2_est_speed_hall = 0.0f;

volatile float32_t cutoff_freq; //cutoff frequency for the low pass filter

/* PID controller variables */
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

/* Friction and test variables */
float32_t friction_torque = 0.0f; // [N.m].
float32_t step = 0.0001f; // [-].
float32_t counter = 0.0f; // [-].

float32_t start_t = 0.0f; // [s].

/* Utility function */
float32_t sign(float32_t x) {
    return (x > 0) - (x < 0);
}

// ========================== COMPENSATION VARIABLES =======================

/* Friction and gravity compensation variables */
volatile uint32_t dry_friction_comp = 0; // Toggle for dry friction (0: off, 1: on)
volatile uint32_t viscous_friction_comp = 0; // Toggle for viscous friction (0: off, 1: on)
volatile uint32_t gravity_comp = 0; // Toggle for gravity compensation (0: off, 1: on)
volatile float32_t center_of_mass_dist = 0.0251f; // [m].
volatile float32_t paddle_mass = 0.077f; // [kg].
volatile float32_t dry_coeff = 0.0f; // [N.m.s/deg].
volatile float32_t dry_comp = 0.0f; // [N.m].
volatile float32_t visc_comp = 0.0f; // [N.m].
volatile float32_t grav_comp = 0.0f; // [N.m].
volatile float32_t sin_angle = 0.0f; // [-].

// ========================== VIRTUAL WALL VARIABLES =======================

volatile float32_t virtual_wall = 0.0f;    
volatile float32_t virtual_wall_automatic = 0.0f;    
volatile float32_t wall_stiffness = 0.001f; // Stiffness of the virtual wall [N.m/deg].
volatile float32_t wall_damping = 0.0f; // Damping of the virtual wall [N.m/deg/s].
volatile float32_t wall_filter = 0.0f; // Filter for the virtual wall [Hz].

float32_t start_wait_time = 0.0f; // [us].
float32_t wait_time_test = 0.0f; // [us].
bool PID_stabilized = false; // Flag to indicate if PID is stabilized

// ========================== EMG VARIABLES ================================

volatile uint32_t set_EMG_torque = 0; // Set EMG value [V].
volatile uint32_t set_EMG_pos = 0; // Set EMG value [V].

volatile float32_t emg_value = 0.0f; // EMG value [V].
volatile float32_t emg_value_prev = 0.0f; // Previous EMG value [V].
volatile float32_t emg_value_filt = 0.0f; // Filtered EMG value [V].

volatile int start_time_emg = 0; // Time for EMG value [us].
volatile uint32_t time_threshold = 1000; // Time for threshold [us].

volatile float32_t emg_min = 100.0f; // Minimum EMG value [V].
volatile float32_t emg_max = -100.0f; // Maximum EMG value [V].
volatile float32_t emg_diff = 0.0f; // EMG threshold [V].
volatile float32_t emg_offset = 0.002f; // EMG offset [V].

volatile uint32_t emg_count = 0; // EMG count [V].
volatile uint32_t emg_count_2 = 0; // EMG count [V].
volatile uint32_t emg_window = 100; // EMG average count [V].
volatile float32_t emg_buff[250] = {0.0f}; // EMG average [V].
volatile float32_t emg_buff_2[250] = {0.0f}; // EMG average [V].
volatile float32_t emg_mean = 0.0f; // EMG average [V].

volatile uint32_t emg_precise = 0;
volatile float32_t emg_hold_value = 0.0f; // EMG hold value [V].
volatile float32_t emg_hold_thres_rise = 0.00105f; // EMG hold position [deg].
volatile float32_t emg_hold_thres_fall = 0.0005f; // EMG hold position [deg].
volatile uint32_t emg_hold_time_window = 250000;
volatile uint32_t emg_hold_count = 0; // EMG hold count [V].
volatile uint32_t emg_hold_start_time = 0;
volatile uint32_t emg_hold_active = 0; // EMG hold active [V].
volatile float32_t emg_hold_sum = 0.0f; // EMG torque [N.m].

// ========================== FUNCTION DECLARATIONS ========================

void hapt_Update(void);

// ========================== INITIALIZATION ===============================

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
    cutoff_freq = 15.0f;

    dry_friction_comp = 0;
    viscous_friction_comp = 0;
    gravity_comp = 0;

    // initialize PID controller
    k_p = 0.008f;
    k_i = 0.005;
    k_d = 0.00015f;

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
    comm_monitorFloat("k_i", (float32_t*)&k_i, WRITEONLY);
    comm_monitorFloat("k_d", (float32_t*)&k_d, WRITEONLY);

    comm_monitorFloat("set PID 0/1", (float32_t*)&set_PID, READWRITE);
    comm_monitorFloat("start dry test 0/1", (float32_t*)&start_dry_test, READWRITE);
    comm_monitorFloat("friction torque [N.m]", (float32_t*)&friction_torque, READONLY);

    // comm_monitorFloat("error", (float32_t*)&error, READONLY);
    // comm_monitorFloat("integral", (float32_t*)&integral, READONLY);
    // comm_monitorFloat("derivative", (float32_t*)&derivative, READONLY);

    comm_monitorUint32("Dry friction compensation", &dry_friction_comp, READWRITE);
    comm_monitorUint32("Viscous friction compensation", &viscous_friction_comp, READWRITE);
    comm_monitorUint32("Gravity compensation", &gravity_comp, READWRITE);

    comm_monitorFloat("Virtual wall", &virtual_wall, WRITEONLY);
    comm_monitorFloat("Virtual wall automatic", &virtual_wall_automatic, WRITEONLY);
    comm_monitorFloat("Wall stiffness [N.m/deg]", &wall_stiffness, READWRITE);
    comm_monitorFloat("Wall damping [N.m/deg/s]", &wall_damping, READWRITE);
    comm_monitorUint32("zero_cross_count", &zero_cross_count, READONLY);
    comm_monitorFloat("Wall filter [Hz]", &wall_filter, READWRITE);

    comm_monitorUint32("set EMG torque", &set_EMG_torque, READWRITE);
    comm_monitorUint32("set EMG position", &set_EMG_pos, READWRITE);
    comm_monitorFloat("EMG value [V]", &emg_value, READONLY);
    comm_monitorFloat("Filtered EMG value [V]", &emg_value_filt, READONLY);
    comm_monitorUint32("EMG time threshold [us]", &time_threshold, READWRITE);
    comm_monitorFloat("EMG diff [V]", &emg_diff, READONLY);
    comm_monitorFloat("EMG min [V]", &emg_min, READONLY);
    comm_monitorFloat("EMG max [V]", &emg_max, READONLY);
    comm_monitorFloat("EMG offset [V]", &emg_offset, READWRITE);

    comm_monitorUint32("EMG window", &emg_window, READWRITE);
    comm_monitorFloat("EMG mean [V]", &emg_mean, READONLY);

    comm_monitorUint32("EMG precise", &emg_precise, READWRITE);
    comm_monitorUint32("EMG hold active", &emg_hold_active, READWRITE);
    comm_monitorFloat("EMG hold value [V]", &emg_hold_value, READWRITE);
    comm_monitorFloat("EMG thres rise [V]", &emg_hold_thres_rise, READWRITE);
    comm_monitorFloat("EMG thres fall [V]", &emg_hold_thres_fall, READWRITE);
}

/**
* @brief Updates the haptic controller state.
*/
void hapt_Update()
{
  hapt_motorTorque = 0.0f; // Reset the motor torque.

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

  // ================== Friction and gravity compensation ====================

  // Dry friction compensation // todo filter speed todo tune dry comp and speed hysteresis
  dry_comp = 0.0f;
  if (dry_friction_comp) {
    if (est_speed > 10.0f) {
        dry_comp = -0.0013; // Apply negative torque for positive velocity
    } else if (est_speed < -10.0f) {
        dry_comp = 0.0009; // Apply positive torque for negative velocity
    }
  } 

  // Viscous friction compensation TODO filter velocity
  float32_t angular_velocity_rad = est_speed * (M_PI / 180.0f); // Convert angular velocity to radians per second
  if (viscous_friction_comp == 1){
    visc_comp = -3e-6f * angular_velocity_rad;
  }else{
    visc_comp = 0.0f;
  }

  // Gravity compensation
  if (gravity_comp == 1){
    sin_angle = sin(hapt_encoderPaddleAngle * M_PI / 180.0f); // Convert angle to radians
    grav_comp = center_of_mass_dist * paddle_mass * 9.81 * sin_angle / 15.0f;
  }else{
    grav_comp = 0.0f;
  }

  // Apply the compensations to the motor torque
  hapt_motorTorque += dry_comp * 0.6 + visc_comp + grav_comp; // Add viscous and gravity compensation to the motor torque

  // ================== virtual wall ====================

  if (virtual_wall) {
    float32_t wall_torque = virtualWall(hapt_encoderPaddleAngle, dt);
    hapt_motorTorque += wall_torque; // Add virtual wall 
  }

  // =================== Automatic friction test =====================

  if (fabs(start_dry_test) && !set_PID)
  {
    if (fabs(hapt_encoderPaddleAngle) <= 0.5)
    {
      hapt_motorTorque += friction_torque;

      if (wait_time_test + TEST_TIME_INTERVAL < hapt_timestamp) { // 1 second
        wait_time_test = hapt_timestamp; // Reset start time for the way
        friction_torque += step * sign(start_dry_test); // Increase friction torque
      }
    }
    else
    {
      friction_torque = 0;
      set_PID = 1.0f;
    }
  }

  // ================= Virtual wall automatic test ===================

  bool crossing_detected = hapt_DetectOscillation(est_speed, hapt_timestamp);

  if(fabs(virtual_wall_automatic) && !set_PID) { // Apply virtual wall only if set_PID is 0
    if (!crossing_detected && fabs(hapt_encoderPaddleAngle) <= 20.0f)
    {
      float32_t wall_torque = virtualWall(hapt_encoderPaddleAngle, dt);
      hapt_motorTorque += wall_torque; // Add virtual wall 

      if (wait_time_test + TEST_TIME_INTERVAL < hapt_timestamp) 
      {
        wait_time_test = hapt_timestamp; // Reset start time for the way
        wall_stiffness += step * 100.0f; // Increase wall stiffness
      }
    }
    else
    {
      set_PID = 1.0f;
      wall_stiffness = 0.001f; // Reset wall stiffness
      osc_start_time = hapt_timestamp;
      last_below_threshold = false;
      detected_oscillation = false;

      #ifdef TEST_MODE_10000
      if (DEFAULT_HAPTIC_CONTROLLER_PERIOD > 350) {
        DEFAULT_HAPTIC_CONTROLLER_PERIOD = 350; // Reset zero-cross count
        cbt_SetHapticControllerPeriod(DEFAULT_HAPTIC_CONTROLLER_PERIOD);
      }
      #endif
    }
  }

  hapt_motorTorque += AUTOMATIC_MANUAL_TORQUE * (-1 * sign(virtual_wall_automatic));
  
  hapt_hallVoltage = hall_GetVoltage();

  // ================= EMG processing =================

  emg_value = adc_GetChannelVoltage(ADC_CHANNEL_6);
  emg_value_filt = low_Pass_Filter(emg_value, emg_value_filt, dt, cutoff_freq); // Apply low-pass filter to EMG value.
  emg_value_prev = emg_value_filt;

  if (start_time_emg + time_threshold < hapt_timestamp) { // 1 ms
    emg_diff = emg_max - emg_min; // Compute EMG threshold
    start_time_emg = hapt_timestamp; // Reset start time for the way
    emg_min = 2.0f; // Reset minimum EMG value
    emg_max = -2.0f; // Reset maximum EMG value
  } else {
    if (emg_value_filt < emg_min) {
      emg_min = emg_value_filt; // Update minimum EMG value
    }
    if (emg_value_filt > emg_max) {
      emg_max = emg_value_filt; // Update maximum EMG value
    }
  }

  if (emg_count < emg_window) {
    emg_buff[emg_count] = emg_diff; // Store EMG value in the buffer
  } else {
    emg_buff[emg_count % emg_window] = emg_diff; // Store EMG value in the buffer
  }

  emg_count++;

  float32_t emg_sum = 0.0f;
  for (uint32_t i = 0; i < ( emg_count < emg_window ? emg_count : emg_window); i++) {
    emg_sum += emg_buff[i]; // Compute the sum of EMG values in the buffer
  }
  emg_mean = emg_sum / emg_window; // Compute the mean of EMG values in the buffer


  if (emg_count_2 < emg_window) {
    emg_buff_2[emg_count_2] = emg_mean; // Store EMG value in the buffer
  } else {
    emg_buff_2[emg_count_2 % emg_window] = emg_mean; // Store EMG value in the buffer
  }

  float32_t emg_sum_2 = 0.0f;
  for (uint32_t i = 0; i < ( emg_count_2 < emg_window ? emg_count_2 : emg_window); i++) {
    emg_sum_2 += emg_buff_2[i]; // Compute the sum of EMG values in the buffer
  }
  emg_mean = emg_sum_2 / emg_window; // Compute the mean of EMG values in the buffer

  emg_count_2++;

  // ================= EMG precision position =================

if (emg_precise == 1) {
    if (emg_mean >= emg_hold_thres_rise && !emg_hold_active) { // Check if EMG value is above the threshold and hold is not active
      emg_hold_start_time = hapt_timestamp; // Reset start time for the hold position
      emg_hold_active = 1; // Set the flag to indicate that EMG hold is active
      emg_hold_count = 0; // Reset the hold count 
    }
    if (hapt_timestamp - emg_hold_start_time <= emg_hold_time_window && emg_hold_active) { // Check if the hold time is reached
      emg_hold_sum += emg_mean; // Add the EMG mean value to the hold sum
      emg_hold_count++; // Increment the hold count
    } else {
      if (emg_hold_active) { // If EMG hold was active
        emg_hold_value = emg_hold_sum / emg_hold_count; // Compute the average EMG value during the hold
      }
    }
    if (emg_mean <= emg_hold_thres_fall && emg_hold_active) { // Check if EMG value is below the threshold and hold is active
      emg_hold_active = 0; // Reset the flag to indicate that EMG hold is not active
      emg_hold_sum = 0.0f; // Reset the hold sum
      emg_hold_count = 0; // Reset the hold count
      emg_hold_value = 0.0f; // Reset the hold value
    }
  }


  float32_t emg_torque = 0.0f; // Compute the EMG difference
  // hapt_motorTorque = 0.0f; // Reset the motor torque.
  if (set_EMG_torque == 1) {
    emg_torque = (emg_mean-emg_offset) * 2.0f; // Apply EMG value to the motor torque.
  } else if (set_EMG_pos == 1) {
    ref_pos = (emg_mean) * 8000.0f; // 8000: biceps / 15000: forearm
    if (ref_pos > 35.0f) {
      ref_pos = 35.0f; // Limit the reference position to 15 degrees
    } else if (ref_pos < -35.0f) {
      ref_pos = -35.0f; // Limit the reference position to -15 degrees
    }
    set_PID = 1.0f; // Set PID controller
  } else if (emg_precise == 1) {
	  ref_pos = (emg_hold_value) * 10000.0f; // 12000: biceps / 18000: forarm
    if (ref_pos > 35.0f) {
      ref_pos = 35.0f; // Limit the reference position to 15 degrees
    } else if (ref_pos < -35.0f) {
      ref_pos = -35.0f; // Limit the reference position to -15 degrees
    }
	  set_PID = 1.0f; // Reset PID controller
  } else {
    emg_torque = 0.0f; // Reset EMG torque
    set_PID = 0.0f; // Reset PID controller
  }


  hapt_motorTorque += emg_torque;

  // ====================== PID controller ====================
  if (set_PID)
  {
    #ifdef TEST_MODE_10000
    if (DEFAULT_HAPTIC_CONTROLLER_PERIOD > 350) {
      DEFAULT_HAPTIC_CONTROLLER_PERIOD = 350; // Reset zero-cross count
      cbt_SetHapticControllerPeriod(DEFAULT_HAPTIC_CONTROLLER_PERIOD);
    }
    #endif

    if (fabs(virtual_wall_automatic) == 1){
      ref_pos = -(15-0.5) * sign(virtual_wall_automatic);
    }
    error = ref_pos - hapt_encoderPaddleAngle;

    integral = integral + error * dt;
    derivative = (error - prev_error) / dt;

    hapt_motorTorque = k_p * error + k_i * integral + k_d * derivative;

    if (hapt_motorTorque > 0.028)
    {
      hapt_motorTorque = 0.028;
    }
    else if (hapt_motorTorque < -0.028)
    {
      hapt_motorTorque = -0.028;
    }

    prev_error = error;
    prev_derivative = derivative;

    if (fabs(start_dry_test) && !PID_stabilized && (hapt_encoderPaddleAngle >= -0.05 && hapt_encoderPaddleAngle <= 0.05) )
    {
      start_wait_time = hapt_timestamp; // Reset start time for the way
      PID_stabilized = true; // Set the flag to indicate that PID is stabilized
    }

    float32_t thresh_angle = hapt_encoderPaddleAngle + (15-0.5) * sign(virtual_wall_automatic);

    if (fabs(virtual_wall_automatic) && (thresh_angle >= -0.1  && thresh_angle <= 0.1) && !PID_stabilized)
    {
      start_wait_time = hapt_timestamp; // Reset start time for the way
      PID_stabilized = true; // Set the flag to indicate that PID is stabilized
    }

    if (PID_stabilized && start_wait_time + 1000000 < hapt_timestamp) { // 1 second
      PID_stabilized = false; // Reset the flag
      hapt_motorTorque = 0.0f;
      set_PID = 0.0f;

      #ifdef TEST_MODE_10000
      DEFAULT_HAPTIC_CONTROLLER_PERIOD = 10000;
      cbt_SetHapticControllerPeriod(DEFAULT_HAPTIC_CONTROLLER_PERIOD);
      #endif

      if (fabs(virtual_wall_automatic)){
        wall_damping += step*125.0f;

        if (wall_damping >= 0.5f) {
          virtual_wall_automatic = 0.0f; // Reset virtual wall
        }
      }
    }
  }
  
  
  if (hapt_motorTorque > 0.028)
  {
    hapt_motorTorque = 0.028;
  }
  else if (hapt_motorTorque < -0.028)
  {
    hapt_motorTorque = -0.028;
  }

  torq_SetTorque(hapt_motorTorque);


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

  // ============ LED indicators ============

  if (hapt_encoderPaddleAngle >= -0.1 && hapt_encoderPaddleAngle <= 0.1)
    led_Set(0, 1.0);
  else 
    led_Set(0, 0.0);

  if (hapt_encoderPaddleAngle <= -14.9)
    led_Set(1, 1.0);
  else 
    led_Set(1, 0.0);

  if (crossing_detected) 
      led_Set(3, 1.0); // Example: light up LED if detected
  else
      led_Set(3, 0.0);
}

// ========================== FILTERS ======================================

/**
 * @brief Simple first-order low-pass filter.
 * @param input New input value.
 * @param prev_output Previous output value.
 * @param T Sample time [s].
 * @param cutoff_freq Cutoff frequency [Hz].
 * @return Filtered value.
 */
float32_t low_Pass_Filter(float32_t input, float32_t prev_output, float32_t T, float32_t cutoff_freq)
{
  float32_t alpha = T / (T + 1 / (2 * M_PI * cutoff_freq));
  return alpha * input + (1 - alpha) * prev_output;
}

// ========================== VIRTUAL WALL ================================

/**
 * @brief Applies a virtual wall at positions 15 and -15 degrees.
 * @param paddle_angle Current paddle angle [deg].
 * @return Torque to apply for the virtual wall [N.m].
 */
float32_t virtualWall(float32_t paddle_angle, float32_t dt)
{
    float32_t wall_torque = 0.0f;
    if (wall_filter != 0.0f){
      // paddle_angle = low_Pass_Filter(paddle_angle, prev_angle, 0.0001f, wall_filter); // Apply low-pass filter to paddle angle.
      est_speed = low_Pass_Filter(est_speed, prev_est_speed, dt, wall_filter); // Apply low-pass filter to estimated speed.
    }

    if (paddle_angle > 15.0f) {
        wall_torque = -wall_stiffness * (paddle_angle - 15.0f) + wall_damping * est_speed * 0.01;; // Push back toward 15 degrees.
    } else if (paddle_angle < -15.0f) {
        wall_torque = -wall_stiffness * (paddle_angle + 15.0f) + wall_damping * est_speed * 0.01;; // Push back toward -15 degrees.
    }

    return wall_torque;
}

// ========================== OSCILLATION DETECTION ========================

/**
 * @brief Detects oscillations based on zero-crossings of speed.
 * @param current_speed Current speed [deg/s].
 * @param timestamp_us Current timestamp [us].
 * @return True if oscillation detected, false otherwise.
 */
bool hapt_DetectOscillation(float32_t current_speed, uint32_t timestamp_us) {
  bool zero_crossed = sign(prev_speed) != sign(current_speed);  // Sign change
  if (zero_crossed && fabs(current_speed-prev_speed) > small_threshold) {
      zero_cross_count++;
  }
  prev_speed = current_speed;

  if (timestamp_us - osc_start_time >= OSC_DETECTION_PERIOD_US) {
      detected_oscillation = (zero_cross_count > OSC_MAX_CROSSES);
      zero_cross_count = 0;
      osc_start_time = timestamp_us;
  }

  return detected_oscillation;
}

