#include "limits.h"
#include "config.h"
#include "axis.h"

bool limitTriggered(uint8_t pin)
{
  int v = digitalRead(pin);
  return LIMIT_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void updateLimitBlocks(Axis& ax, const LimitsState& lim)
{
  if (!lim.enabled)
  {
    ax.blockDir = 0;
    return;
  }

  bool lo = limitTriggered(ax.limLoPin);
  bool hi = limitTriggered(ax.limHiPin);

  if (lo && !hi) ax.blockDir = -1;
  else if (hi && !lo) ax.blockDir = +1;
  else if (!lo && !hi) ax.blockDir = 0;
  else ax.blockDir = 2; // both triggered (rare) -> block all
}

void enforceHardwareStops(Axis& ax, const LimitsState& lim)
{
  if (!lim.enabled) return;

  // Don't interfere with homing state machine (it relies on hitting limits)
  if (!(ax.hs == HomeState::IDLE || ax.hs == HomeState::DONE || ax.hs == HomeState::ERROR)) return;

  long dist = ax.stepper.distanceToGo();
  if (dist == 0) return;

  if (ax.blockDir == 2)
  {
    stopAxis(ax);
    return;
  }

  int moveDir = (dist > 0) ? +1 : -1;
  if (ax.blockDir != 0 && ax.blockDir == moveDir)
  {
    stopAxis(ax);
  }
}

bool allowTarget(Axis& ax, const LimitsState& lim, long target)
{
  if (!lim.enabled) return true;

  // Block direction if photodetector active
  long cur = ax.stepper.currentPosition();
  if (ax.blockDir == 2) return false;
  if (ax.blockDir == -1 && target < cur) return false;
  if (ax.blockDir == +1 && target > cur) return false;

  // Software bounds if homed
  if (ax.homed)
  {
    if (target < 0 || target > ax.maxPos) return false;
  }
  return true;
}

void limitsAutoReenableTick(LimitsState& lim)
{
  // Auto re-enable limits if timer elapsed
  if (!lim.enabled && LIMITS_TIMEOUT_MS > 0)
  {
    if ((int32_t)(millis() - lim.disableUntilMs) >= 0)
    {
      lim.enabled = true;
      Serial.println("Limits auto re-enabled.");
    }
  }
}