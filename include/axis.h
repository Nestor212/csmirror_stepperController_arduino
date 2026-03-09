#pragma once
#include <Arduino.h>
#include <AccelStepper.h>

// -------------------- AXIS STRUCT --------------------
enum class HomeState : uint8_t {
  IDLE,
  SEEK_LOWER_FAST,
  BACKOFF_FROM_LOWER,
  SEEK_LOWER_SLOW,
  CLEAR_LOWER_FINAL,
  SET_ZERO,

  SEEK_UPPER_FAST,
  BACKOFF_FROM_UPPER,
  SEEK_UPPER_SLOW,
  CLEAR_UPPER_FINAL,
  SET_MAX,

  MOVE_TO_MID,
  DONE,
  ERROR
};

struct Axis {
  AccelStepper stepper;
  uint8_t enPin;
  uint8_t limLoPin;
  uint8_t limHiPin;

  HomeState hs = HomeState::IDLE;
  uint32_t homeStartMs;
  bool enabled = false;
  bool homed = false;
  bool posValid = false;
  
  long maxPos = 0;
  uint32_t t_ms = 0;

  // Direction block based on active photodetectors (when limits enabled)
  //  0 = none
  // -1 = lower active -> block negative direction
  // +1 = upper active -> block positive direction
  //  2 = both active -> block all (faulty/rare)
  int8_t blockDir = 0;

  Axis(uint8_t stepPin, uint8_t dirPin, uint8_t en, uint8_t lo, uint8_t hi)
  : stepper(AccelStepper::DRIVER, stepPin, dirPin),
    enPin(en), limLoPin(lo), limHiPin(hi) {}
};

// Setup pins for enable + photodetectors
void axisInitPins(Axis& ax);

// Enable/disable motor driver
void setEnable(Axis& ax, bool on);

// Stop axis immediately (cancel target)
void stopAxis(Axis& ax);