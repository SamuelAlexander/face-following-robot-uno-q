// SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
//
// SPDX-License-Identifier: MPL-2.0

// -----------------------------------------------------------------------
// MCU Sketch: Continuous-rotation servo control via Bridge RPC
//
// Two servos are driven via the Arduino Servo library on the UNO Q's
// STM32 MCU.  The MPU (Python) calls "set_wheel_pwm" over the Bridge
// and sends direct wheel pulse commands (microseconds).
// -----------------------------------------------------------------------

#include <Arduino_RouterBridge.h>
#include <Servo.h>

namespace {

// --- Pin assignments (change here if you re-wire) --------------------
const int kLeftServoPin  = 3;   // D3
const int kRightServoPin = 6;   // D6

// Standard continuous-rotation servo pulse range in microseconds.
const uint32_t kStopPulseUs = 1500;
const uint32_t kMinPulseUs  = 1000;
const uint32_t kMaxPulseUs  = 2000;

// Flip this if left wheel runs backward when commanded forward.
const bool kInvertLeftWheel  = false;
// Flip this if right wheel runs backward when commanded forward.
const bool kInvertRightWheel = true;

Servo leftServo;
Servo rightServo;

uint32_t clamp_pulse_us(int pulse_us) {
  if (pulse_us < static_cast<int>(kMinPulseUs)) return kMinPulseUs;
  if (pulse_us > static_cast<int>(kMaxPulseUs)) return kMaxPulseUs;
  return static_cast<uint32_t>(pulse_us);
}

uint32_t apply_invert(uint32_t pulse_us, bool invert) {
  if (!invert) return pulse_us;
  return (kStopPulseUs * 2U) - pulse_us;
}

}  // namespace

// --- Bridge RPC handler ----------------------------------------------
// Called from Python: Bridge.call("set_wheel_pwm", left_us, right_us)
bool set_wheel_pwm(int left_us, int right_us) {
  uint32_t left  = clamp_pulse_us(left_us);
  uint32_t right = clamp_pulse_us(right_us);

  left  = apply_invert(left,  kInvertLeftWheel);
  right = apply_invert(right, kInvertRightWheel);

  leftServo.writeMicroseconds(left);
  rightServo.writeMicroseconds(right);
  return true;
}

void setup() {
  Bridge.begin();

  leftServo.attach(kLeftServoPin);
  rightServo.attach(kRightServoPin);

  // Start both servos in the stopped position.
  leftServo.writeMicroseconds(kStopPulseUs);
  rightServo.writeMicroseconds(kStopPulseUs);

  // Expose the RPC so the MPU can control the servos.
  Bridge.provide_safe("set_wheel_pwm", set_wheel_pwm);
}

void loop() {
  // Nothing to do — servo state is updated via Bridge RPC callbacks.
}
