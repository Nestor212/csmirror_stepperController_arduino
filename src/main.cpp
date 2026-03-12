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

static unsigned long lastTipTiltReport = 0;
static unsigned long lastAzimuthReport = 0;
static const unsigned long REPORT_INTERVAL_MS = 1000; // ms

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
  tiptilt.stepper.setMinPulseWidth(MIN_PULSE_WIDTH_US);
  azimuth.stepper.setMinPulseWidth(MIN_PULSE_WIDTH_US);

  systemStateInit(sys); // NEW

  Serial.println("READY");
}

void loop()
{
  // ---- Global housekeeping ----
  limitsAutoReenableTick(lim);

  // ---- Tip/Tilt axis service ----
  if (!(tiptilt.hs == HomeState::IDLE ||
        tiptilt.hs == HomeState::DONE ||
        tiptilt.hs == HomeState::ERROR))
  {
    // Homing owns this axis this iteration.
    updateHoming(tiptilt);
  }
  else
  {
    updateLimitBlocks(tiptilt, lim);
    enforceHardwareStops(tiptilt, lim);

    if (tiptilt.cmd_stopRequested && !tiptilt.stepper.isRunning())
    {
      tiptilt.stepper.setAcceleration(tiptilt.accel);
      tiptilt.cmd_stopRequested = false;
    }
    tiptilt.stepper.run();
  }

  // ---- Azimuth axis service ----
  if (!(azimuth.hs == HomeState::IDLE ||
        azimuth.hs == HomeState::DONE ||
        azimuth.hs == HomeState::ERROR))
  {
    // Homing owns this axis this iteration.
    updateHoming(azimuth);
  }
  else
  {
    updateLimitBlocks(azimuth, lim);
    enforceHardwareStops(azimuth, lim);

    if (azimuth.cmd_stopRequested && !azimuth.stepper.isRunning())
    {
      azimuth.stepper.setAcceleration(azimuth.accel);
      azimuth.cmd_stopRequested = false;
    }
    azimuth.stepper.run();
  }

  // ---- Motion state ----
  bool movingA = tiptilt.stepper.isRunning();
  bool movingB = azimuth.stepper.isRunning();

  unsigned long now = millis();

  // Reset each report timer when motion starts so it waits a full 5 s before printing.
  if (movingA && !tiptiltWasMoving)
    lastTipTiltReport = now;

  if (movingB && !azimuthWasMoving)
    lastAzimuthReport = now;

  // Periodic progress reporting while moving.
  if (movingA && (now - lastTipTiltReport >= REPORT_INTERVAL_MS))
  {
    lastTipTiltReport = now;
    Serial.print(F("tip/tilt pos: "));
    Serial.println(tiptilt.stepper.currentPosition());
  }

  if (movingB && (now - lastAzimuthReport >= REPORT_INTERVAL_MS))
  {
    lastAzimuthReport = now;
    Serial.print(F("azimuth pos: "));
    Serial.println(azimuth.stepper.currentPosition());
  }

  // One-time move completion event when motion transitions from moving -> stopped.
  if (tiptiltWasMoving && !movingA)
  {
    bumpSeq(sys);
    Serial.print(F("EVENT move_done a pos="));
    Serial.println(tiptilt.stepper.currentPosition());
  }

  if (azimuthWasMoving && !movingB)
  {
    bumpSeq(sys);
    Serial.print(F("EVENT move_done b pos="));
    Serial.println(azimuth.stepper.currentPosition());
  }

  tiptiltWasMoving = movingA;
  azimuthWasMoving = movingB;

  // ---- Read commands ----
  while (Serial.available())
  {
    char c = (char)Serial.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      handleCmd(line, sys, lim, tiptilt, azimuth);
      line = "";
    }
    else if (line.length() < 120)
    {
      line += c;
    }
  }
}