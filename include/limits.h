#pragma once
#include <Arduino.h>
#include "axis.h"

struct LimitsState {
  bool enabled = true;
  uint32_t disableUntilMs = 0;
};

struct Axis;
struct LimitsState;

enum class TargetBlockReason : uint8_t {
  OK = 0,
  PhotodetectorBlockBoth,
  PhotodetectorBlockNegative,
  PhotodetectorBlockPositive,
  OutOfBoundsLow,
  OutOfBoundsHigh,
  NotHomedBoundsUnknown,   // optional if you want a specific message
};

struct TargetBlockInfo {
  TargetBlockReason reason = TargetBlockReason::OK;
  long cur = 0;
  long target = 0;
  long maxPos = 0;
  int8_t blockDir = 0;
  bool homed = false;
  bool limitsEnabled = false;
};

// Read photodetector and interpret as "triggered"
bool limitTriggered(uint8_t pin);

// Update Axis::blockDir based on instantaneous photodetector states
void updateLimitBlocks(Axis& ax, const LimitsState& lim);

// During normal motion (not homing), if a limit asserts while moving in that direction, stop immediately.
void enforceHardwareStops(Axis& ax, const LimitsState& lim);

// Returns true if allowed. If false, fills info->reason/details (if info != nullptr).
bool allowTarget(Axis& ax, const LimitsState& lim, long target, TargetBlockInfo* info);

// Handle auto re-enable timer in loop
void limitsAutoReenableTick(LimitsState& lim);