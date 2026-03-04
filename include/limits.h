#pragma once
#include <Arduino.h>
#include "axis.h"

struct LimitsState {
  bool enabled = true;
  uint32_t disableUntilMs = 0;
};

// Read photodetector and interpret as "triggered"
bool limitTriggered(uint8_t pin);

// Update Axis::blockDir based on instantaneous photodetector states
void updateLimitBlocks(Axis& ax, const LimitsState& lim);

// During normal motion (not homing), if a limit asserts while moving in that direction, stop immediately.
void enforceHardwareStops(Axis& ax, const LimitsState& lim);

// Decide whether a commanded target is allowed given bounds + active limit blocks.
bool allowTarget(Axis& ax, const LimitsState& lim, long target);

// Handle auto re-enable timer in loop
void limitsAutoReenableTick(LimitsState& lim);