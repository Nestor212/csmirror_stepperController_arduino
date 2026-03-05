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

enum class BlockReason : uint8_t {
  OK = 0,
  LimitsDisabled,
  HardBlockBoth,          // blockDir == 2
  HardBlockNegative,      // blockDir == -1 and target < cur
  HardBlockPositive,      // blockDir == +1 and target > cur
  OutOfBoundsLow,         // homed && target < 0
  OutOfBoundsHigh         // homed && target > maxPos
};

struct BlockInfo {
  BlockReason reason = BlockReason::OK;
  long cur = 0;
  long target = 0;
  long maxPos = 0;
  int8_t blockDir = 0;
  bool homed = false;
  bool limitsEnabled = false;
};

bool allowTarget(Axis& ax, const LimitsState& lim, long target, TargetBlockInfo* info)
{
  long cur = ax.stepper.currentPosition();

  if (info) {
    info->cur = cur;
    info->target = target;
    info->maxPos = ax.maxPos;
    info->blockDir = ax.blockDir;
    info->homed = ax.homed;
    info->limitsEnabled = lim.enabled;
    info->reason = TargetBlockReason::OK;
  }

  if (!lim.enabled) {
    // limits disabled => always allow
    return true;
  }

  // Block direction if photodetector active
  if (ax.blockDir == 2) {
    if (info) info->reason = TargetBlockReason::PhotodetectorBlockBoth;
    return false;
  }
  if (ax.blockDir == -1 && target < cur) {
    if (info) info->reason = TargetBlockReason::PhotodetectorBlockNegative;
    return false;
  }
  if (ax.blockDir == +1 && target > cur) {
    if (info) info->reason = TargetBlockReason::PhotodetectorBlockPositive;
    return false;
  }

  // Software bounds if homed
  if (ax.homed) {
    if (target < 0) {
      if (info) info->reason = TargetBlockReason::OutOfBoundsLow;
      return false;
    }
    if (target > ax.maxPos) {
      if (info) info->reason = TargetBlockReason::OutOfBoundsHigh;
      return false;
    }
  } 
  else 
  {

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