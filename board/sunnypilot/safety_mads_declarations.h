/**
 * The MIT License
 *
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Last updated: July 29, 2024
 */

#pragma once

// ===============================
// Type Definitions and Enums
// ===============================

typedef enum __attribute__((packed)) {
  MADS_BUTTON_UNAVAILABLE = -1,  ///< Button state cannot be determined
  MADS_BUTTON_NOT_PRESSED = 0,   ///< Button is not pressed
  MADS_BUTTON_PRESSED = 1        ///< Button is pressed
} ButtonState;

typedef enum __attribute__((packed)) {
  MADS_EDGE_NO_CHANGE = 0,  ///< No state change detected
  MADS_EDGE_RISING = 1,     ///< State changed from false to true
  MADS_EDGE_FALLING = 2     ///< State changed from true to false
} EdgeTransition;

typedef enum __attribute__((packed)) {
  MADS_DISENGAGE_REASON_NONE = 0,                         ///< No disengagement
  MADS_DISENGAGE_REASON_BRAKE = 1,                        ///< Brake pedal pressed
  MADS_DISENGAGE_REASON_LAG = 2,                          ///< System lag detected
  MADS_DISENGAGE_REASON_BUTTON = 4,                       ///< User button press
  MADS_DISENGAGE_REASON_ACC_MAIN_OFF = 8,                 ///< ACC system turned off
  MADS_DISENGAGE_REASON_NON_PCM_ACC_MAIN_DESYNC = 16,     ///< ACC sync error
  MADS_DISENGAGE_REASON_HEARTBEAT_ENGAGED_MISMATCH = 32,  ///< Heartbeat mismatch
} DisengageReason;

// ===============================
// Constants and Defines
// ===============================

#define ALT_EXP_ENABLE_MADS 1024
#define ALT_EXP_DISENGAGE_LATERAL_ON_BRAKE 2048

#define MISMATCH_DEFAULT_THRESHOLD 25

// ===============================
// Data Structures
// ===============================

typedef struct {
  DisengageReason active_reason;    // The reason that actually disengaged controls
  DisengageReason pending_reasons;  // All conditions that would've prevented engagement while controls were disengaged
} DisengageState;

typedef struct {
  ButtonState current;
  ButtonState last;
  EdgeTransition transition;
} ButtonStateTracking;

typedef struct {
  EdgeTransition transition;
  bool current : 1;
  bool previous : 1;
} BinaryStateTracking;

typedef struct {
  bool is_vehicle_moving : 1;

  ButtonStateTracking mads_button;
  BinaryStateTracking acc_main;
  BinaryStateTracking op_controls_allowed;
  BinaryStateTracking braking;

  DisengageState current_disengage;

  bool system_enabled : 1;
  bool disengage_lateral_on_brake : 1;
  bool controls_requested_lat : 1;
  bool controls_allowed_lat : 1;
} MADSState;

// ===============================
// Global Variables
// ===============================

extern ButtonState mads_button_press;
extern MADSState m_mads_state;

// state for mads controls_allowed_lat timeout logic
extern bool heartbeat_engaged_mads;
extern uint32_t heartbeat_engaged_mads_mismatches;

// ===============================
// External Function Declarations (kept as needed)
// ===============================

extern void mads_set_system_state(bool enabled, bool disengage_lateral_on_brake);
extern void mads_set_alternative_experience(const int *mode);
extern void mads_state_update(bool op_vehicle_moving, bool op_acc_main, bool op_allowed, bool is_braking);
extern void mads_exit_controls(DisengageReason reason);
extern bool mads_is_lateral_control_allowed_by_mads(void);
extern void mads_heartbeat_engaged_check(void);