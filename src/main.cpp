#include <Arduino.h>

#include "pins.h"
#include "config.h"
#include "axis.h"
#include "limits.h"
#include "homing.h"
#include "commands.h"
#include "system_state.h"

// Axis instances
Axis tiptilt(MOTOR_A_STEP, MOTOR_A_DIR, MOTOR_A_EN, LIM_MIN_A, LIM_MAX_A);
Axis azimuth(MOTOR_B_STEP, MOTOR_B_DIR, MOTOR_B_EN, LIM_MIN_B, LIM_MAX_B);

SystemState sys;   // NEW

static bool tiptiltWasMoving = false;
static bool azimuthWasMoving = false;

// Limits state
LimitsState lim;

// Serial line buffer
static String line;

void setup()
{
  Serial.begin(115200);

  axisInitPins(tiptilt);
  axisInitPins(azimuth);

  setEnable(tiptilt, false);
  setEnable(azimuth, false);

  // Default motion params (tune)
  tiptilt.stepper.setMaxSpeed(1000);
  tiptilt.stepper.setAcceleration(100);

  azimuth.stepper.setMaxSpeed(600);
  azimuth.stepper.setAcceleration(100);

  // Helps TB6600 clones that need wider STEP pulses
  // tiptilt.stepper.setMinPulseWidth(MIN_PULSE_WIDTH_US);
  // azimuth.stepper.setMinPulseWidth(MIN_PULSE_WIDTH_US);

  systemStateInit(sys); // NEW

  Serial.println("READY");
}

void loop()
{
  // Auto re-enable limits if timer elapsed
  limitsAutoReenableTick(lim);

  // Update instantaneous block states
  updateLimitBlocks(tiptilt, lim);
  updateLimitBlocks(azimuth, lim);

  // Enforce immediate stop if limits enabled and a limit asserts during normal motion
  enforceHardwareStops(tiptilt, lim);
  enforceHardwareStops(azimuth, lim);

  // Run motion / homing (same logic as your original)
  if (tiptilt.hs == HomeState::IDLE || tiptilt.hs == HomeState::DONE || tiptilt.hs == HomeState::ERROR)
    tiptilt.stepper.run();
  else
    updateHoming(tiptilt);

  if (azimuth.hs == HomeState::IDLE || azimuth.hs == HomeState::DONE || azimuth.hs == HomeState::ERROR)
    azimuth.stepper.run();
  else
    updateHoming(azimuth);

  bool movingA = (tiptilt.stepper.distanceToGo() != 0);
  bool movingB = (azimuth.stepper.distanceToGo() != 0);

  if (tiptiltWasMoving && !movingA) {
    bumpSeq(sys);
    Serial.print(F("EVENT move_done a pos="));
    Serial.println(tiptilt.stepper.currentPosition());
  }
  if (azimuthWasMoving && !movingB) {
    bumpSeq(sys);
    Serial.print(F("EVENT move_done b pos="));
    Serial.println(azimuth.stepper.currentPosition());
  }

  azimuthWasMoving = movingB;
  tiptiltWasMoving = movingA;

  // Read commands (same behavior as your original)
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n')
    {
      handleCmd(line, sys, lim, tiptilt, azimuth);
      line = "";
    } 
    else 
    {
      if (line.length() < 120) line += c;
    }
  }
}