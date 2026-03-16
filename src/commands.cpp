#include "commands.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <avr/wdt.h>

#include "config.h"
#include "homing.h"
#include "limits.h"
#include "axis.h"

// -------------------- small helpers --------------------

static int split_tokens(char* s, char* out[], int max_tokens)
{
  // Splits on whitespace in-place. Returns token count.
  int n = 0;
  while (*s && n < max_tokens) {
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
    if (!*s) break;

    out[n++] = s;

    while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
    if (*s) *s++ = '\0';
  }
  return n;
}

static bool streq(const char* a, const char* b) { return strcmp(a, b) == 0; }

static bool parse_int(const char* s, int& out)
{
  if (!s || !*s) return false;
  char* end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s) return false;
  out = (int)v;
  return true;
}

static bool parse_long(const char* s, long& out)
{
  if (!s || !*s) return false;
  char* end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s) return false;
  out = v;
  return true;
}

// AVR-friendly float parse (Uno doesn't have strtof)
static bool parse_float(const char* s, float& out)
{
  if (!s || !*s) return false;
  out = (float)atof(s);
  return true;
}

static Axis* axisFromId(char id, Axis& tiptilt, Axis& azimuth)
{
  if (id == 'a') return &tiptilt;
  if (id == 'b') return &azimuth;
  return nullptr;
}

static void printMoveBlocked(char axisName, const TargetBlockInfo& bi)
{
  Serial.print(F("ERR: Move blocked. axis="));
  Serial.print(axisName);  // or ax.name / whatever you have
  Serial.print(F(" cur=")); Serial.print(bi.cur);
  Serial.print(F(" target=")); Serial.print(bi.target);

  switch (bi.reason) 
  {
    case TargetBlockReason::PhotodetectorBlockBoth:
      Serial.println(F(" (photodetector active: both directions blocked)"));
      break;

    case TargetBlockReason::PhotodetectorBlockNegative:
      Serial.println(F(" (photodetector active: negative direction blocked)"));
      break;

    case TargetBlockReason::PhotodetectorBlockPositive:
      Serial.println(F(" (photodetector active: positive direction blocked)"));
      break;

    case TargetBlockReason::OutOfBoundsLow:
      Serial.println(F(" (out of bounds: target < 0)"));
      break;

    case TargetBlockReason::OutOfBoundsHigh:
      Serial.print(F(" (out of bounds: target > maxPos="));
      Serial.print(bi.maxPos);
      Serial.println(F(")"));
      break;

    default:
      Serial.println(F(" (unknown reason)"));
      break;
  }
  return;
}

// -------------------- command classification --------------------

enum class CmdClass : uint8_t { UNKNOWN, GLOBAL, AXIS };

static CmdClass classify_cmd(const char* action)
{
  // Global commands (accept underscore + non-underscore aliases)
  if (streq(action, "status") || streq(action, "status_long") ||
      streq(action, "pos") || streq(action, "position") ||
      streq(action, "enable_limits") ||
      streq(action, "disable_limits") ||
      streq(action, "enable_all") || streq(action, "enableall") ||
      streq(action, "disable_all") || streq(action, "disableall") ||
      streq(action, "stop_all") || streq(action, "stopall") || streq(action, "stop") ||
      streq(action, "estop") ||
      streq(action, "reset") ||
      streq(action, "reboot")) {
    return CmdClass::GLOBAL;
  }

  // Per-axis commands
  if (streq(action, "enable") ||
      streq(action, "disable") ||
      streq(action, "setaxis") ||
      streq(action, "stop") ||
      streq(action, "home") ||
      streq(action, "zero") ||
      streq(action, "move") ||
      streq(action, "moveto")) {
    return CmdClass::AXIS;
  }

  return CmdClass::UNKNOWN;
}

// -------------------- command handlers --------------------
static void cmd_enable_limits(LimitsState& lim)
{
  lim.enabled = true;
  Serial.println("Limit switches enabled.");
  return;
}

static void cmd_disable_limits(LimitsState& lim)
{
  lim.enabled = false;
  if (LIMITS_TIMEOUT_MS > 0) 
  {
    lim.disableUntilMs = millis() + LIMITS_TIMEOUT_MS;
    Serial.print("Limit switches disabled for ");
    Serial.print(LIMITS_TIMEOUT_MS);
    Serial.println(" ms.");
  } 
  else 
  {
    Serial.println("Limit switches disabled.");
  }
  return;
}

static void cmd_enableall(Axis& tiptilt, Axis& azimuth)
{
  setEnable(tiptilt, true);
  setEnable(azimuth, true);
  Serial.println("All motors enabled.");
  return;
}

static void cmd_disableall(Axis& tiptilt, Axis& azimuth)
{
  setEnable(tiptilt, false);
  setEnable(azimuth, false);
  Serial.println("All motors disabled.");
  return;
}

static void cmd_stopall(Axis& tiptilt, Axis& azimuth)
{
  stopAxis(tiptilt);
  stopAxis(azimuth);
  Serial.println("All motors stopped.");
  return;
}

static void cmd_estop(Axis& tiptilt, Axis& azimuth)
{
  emergencyStopAxis(tiptilt);
  emergencyStopAxis(azimuth);
  Serial.println("All motors emergency stopped & disabled.");
  return;
}

static void cmd_enable_axis(Axis* ax, char axis_id)
{
  if (!ax) { Serial.println("ERR"); return; }
  setEnable(*ax, true);
  Serial.print("Motor ");
  Serial.print((char)toupper(axis_id));
  Serial.println(" enabled.");
  return;
}

static void cmd_disable_axis(Axis* ax, char axis_id)
{
  if (!ax) { Serial.println("ERR"); return; }
  setEnable(*ax, false);
  Serial.print("Motor ");
  Serial.print((char)toupper(axis_id));
  Serial.println(" disabled.");
  return;
}

static void cmd_stop_axis(Axis* ax, char axis_id)
{
  if (!ax) { Serial.println("ERR"); return; }
  stopAxis(*ax);
  Serial.print("Motor ");
  Serial.print((char)toupper(axis_id));
  Serial.println(" stopped.");
  return;
}

static void cmd_home_axis(Axis* ax, char axis_id, const LimitsState& lim)
{
  if (!ax) { Serial.println("ERR"); return; }

  if (!lim.enabled) {
    Serial.print("ERR:Motor ");
    Serial.print((char)toupper(axis_id));
    Serial.println(" limits are disabled. Cannot home.");
    return;
  }
  startHoming(*ax);
  return;
}

static void cmd_set_axis(SystemState& sys, Axis* ax, char axis_id, int ntok, char* tok[])
{
  // Expected format:
  // setaxis <axis> <pos> <max>

  if (!ax) {
    Serial.println(F("ERR:Invalid axis."));
    return;
  }

  if (!(ntok == 4)) {
    Serial.println(F("ERR:Expected: setaxis <axis> <pos> <max>"));
    return;
  }

  long pos = 0;
  long maxv = 0;
  int valid = 0;

  if (!parse_long(tok[2], pos)) {
    Serial.println(F("ERR:Invalid position."));
    return;
  }

  if (!parse_long(tok[3], maxv)) {
    Serial.println(F("ERR:Invalid max limit."));
    return;
  }

  // ----- apply state -----
  ax->stepper.setCurrentPosition(pos);
  ax->maxPos = maxv;
  
  ax->posValid = true;
  ax->homed = true;
  ax->hs = HomeState::DONE;

  // bump system sequence so Pi knows something changed
  bumpSeq(sys);

  Serial.print(F("AXIS "));
  Serial.print((char)toupper(axis_id));
  Serial.print(F(" SET pos="));
  Serial.print(pos);
  Serial.print(F(" max="));
  Serial.print(maxv);
  return;
}

// Robust MOVE parser:
// move <axis> <dir> <steps> [speed] [accel]
static void cmd_move(SystemState& sys, Axis* ax, char axis_id, const LimitsState& lim, int ntok, char* tok[])
{
  // move <axis> <dir> <steps> [speed] [accel]
  if (!ax) { Serial.println(F("ERR")); return; }

  //check if position is valid (homed) before allowing move or moveto
  if (!ax->posValid || !ax->homed) {
    Serial.print(F("ERR:Motor "));
    Serial.print((char)toupper(axis_id));
    Serial.println(F(" position invalid. Home or sync state first."));
    return;
  }

  if (ntok < 4) {
    Serial.println(F("ERR:Invalid MOVE command format."));
    Serial.println(F("Expected: move <axis> <dir> <steps> [speed] [accel]"));
    return;
  }

  int dir = 0;
  long steps = 0;
  if (!parse_int(tok[2], dir) || !parse_long(tok[3], steps)) {
    Serial.println(F("ERR:Invalid MOVE args (dir/steps)."));
    return;
  }

  float speed = ax->stepper.maxSpeed();
  float accel = 0.0f;

  if (ntok >= 5) {
    if (!parse_float(tok[4], speed)) { Serial.println(F("ERR:Invalid speed.")); return; }
  }
  if (ntok >= 6) {
    if (!parse_float(tok[5], accel)) { Serial.println(F("ERR:Invalid accel.")); return; }
  }

  if (speed > MOTOR_MAX_SPEED) 
  {
    speed = float(MOTOR_MAX_SPEED);
    Serial.println(F("WARN: Speed capped to MOTOR_MAX_SPEED = 1500 sps."));
  }

  if (dir != 0 && dir != 1) {
    Serial.println(F("ERR:dir must be 0 or 1"));
    return;
  }

  if (!ax->enabled) {
    Serial.print(F("ERR:Motor "));
    Serial.print((char)toupper(axis_id));
    Serial.println(F(" is disabled."));
    return;
  }

  if (speed < 1.0f) speed = 1.0f;
  ax->stepper.setMaxSpeed(speed);

  // keep your behavior: accel < 1 => huge accel (effectively constant speed)
  if (accel < 1.0f) accel = 1000000.0f;
  ax->stepper.setAcceleration(accel);
  ax->accel = accel;

  long cur = ax->stepper.currentPosition();
  long delta = (dir == 0) ? -steps : +steps;
  long target = cur + delta;

  updateLimitBlocks(*ax, lim);

  TargetBlockInfo bi;
  if (!allowTarget(*ax, lim, target, &bi)) {
    printMoveBlocked(axis_id, bi);
    return;
  }

  ax->stepper.moveTo(target);
  bumpSeq(sys); // NEW: authoritative state changed (accepted a move)
  return;
}

// Robust MOVETO parser:
// moveto <axis> <pos> [speed] [accel]
static void cmd_moveto(SystemState& sys, Axis* ax, char axis_id, const LimitsState& lim, int ntok, char* tok[])
{
  // moveto <axis> <pos> [speed] [accel]
  if (!ax) { Serial.println(F("ERR")); return; }

  //check if position is valid (homed) before allowing move or moveto
  if (!ax->posValid || !ax->homed) 
  {
    Serial.print(F("ERR:Motor "));
    Serial.print((char)toupper(axis_id));
    Serial.println(F(" position invalid. Home or sync state first."));
    return;
  }

  if (ntok < 3) 
  {
    Serial.println(F("ERR:Invalid MOVETO command format."));
    Serial.println(F("Expected: moveto <axis> <pos> [speed] [accel]"));
    return;
  }

  long pos = 0;
  if (!parse_long(tok[2], pos)) 
  {
    Serial.println(F("ERR:Invalid position."));
    return;
  }

  float speed = ax->stepper.maxSpeed();
  float accel = 0.0f;

  if (ntok >= 4) 
  {
    if (!parse_float(tok[3], speed)) 
    { 
      Serial.println(F("ERR:Invalid speed.")); return; 
    }
  }
  if (ntok >= 5) 
  {
    if (!parse_float(tok[4], accel)) 
    { 
      Serial.println(F("ERR:Invalid accel.")); return; 
    }
  }

  if (speed > MOTOR_MAX_SPEED) 
  {
    speed = float(MOTOR_MAX_SPEED);
    Serial.println(F("WARN: Speed capped to MOTOR_MAX_SPEED = 1500 sps."));
  }

  if (!ax->enabled) 
  {
    Serial.print(F("ERR:Motor "));
    Serial.print((char)toupper(axis_id));
    Serial.println(F(" is disabled."));
    return;
  }

  if (speed < 1.0f) 
  {
    speed = 1.0f;
  }
  ax->stepper.setMaxSpeed(100.0f);

  if (accel < 1.0f) 
  {
    accel = 1000000.0f;
  }
  ax->stepper.setAcceleration(accel);
  ax->accel = accel;

  updateLimitBlocks(*ax, lim);
  TargetBlockInfo bi;
  if (!allowTarget(*ax, lim, pos, &bi)) 
  {
    printMoveBlocked(axis_id, bi);
    return;
  }
  Serial.print("Moving to position: ");
  ax->stepper.moveTo(long(pos));
  Serial.println(ax->stepper.targetPosition());
  bumpSeq(sys); // NEW
  return;
}

// -------------------- public API --------------------
void printStatus(const SystemState& sys, const LimitsState& lim, Axis& tiptilt, Axis& azimuth)
{
  auto printAx = [](const char* name, char axis_id, Axis& ax) 
  {
    Serial.print(axis_id);
    Serial.print(F(" "));
    Serial.print(ax.enabled ? 1 : 0);
    Serial.print(F(" "));
    Serial.print(ax.stepper.currentPosition());
    Serial.print(F(" "));
    Serial.print(ax.maxPos);
    Serial.println();
  };

  printAx("Tip/Tilt", 'a', tiptilt);
  printAx("Azimuth",  'b', azimuth);
  return;
}

void printStatus_long(const SystemState& sys, const LimitsState& lim, Axis& tiptilt, Axis& azimuth)
{
  if (tiptilt.stepper.isRunning() || azimuth.stepper.isRunning()) return;

  Serial.print(F("boot_id="));
  Serial.println(sys.boot_id);

  Serial.print(F("limitsEnabled="));
  Serial.println(lim.enabled ? 1 : 0);

  auto printAx = [](const char* name, char axis_id, Axis& ax) 
  {
    Serial.print(name);
    Serial.print(F(" id="));
    Serial.print(axis_id);
    Serial.print(F(" en="));
    Serial.print(ax.enabled ? 1 : 0);
    Serial.print(F(" pos="));
    Serial.print(ax.stepper.currentPosition());
    Serial.print(F(" max="));
    Serial.print(ax.maxPos);
    Serial.print(F(" homed="));
    Serial.print(ax.homed ? 1 : 0);
    Serial.print(F(" valid="));
    Serial.print(ax.posValid ? 1 : 0);
    Serial.print(F(" lolim="));
    Serial.print(limitTriggered(ax.limLoPin) ? 1 : 0);
    Serial.print(F(" hilim="));
    Serial.print(limitTriggered(ax.limHiPin) ? 1 : 0);
    Serial.println();
  };

  printAx("Tip/Tilt", 'a', tiptilt);
  printAx("Azimuth",  'b', azimuth);
  return;
}

void printPosStatus(const SystemState& sys, const LimitsState& lim, Axis& tiptilt, Axis& azimuth)
{
  // if (tiptilt.stepper.isRunning() || azimuth.stepper.isRunning()) return;
  Serial.print(tiptilt.stepper.currentPosition());
  Serial.print(" ");
  Serial.println(azimuth.stepper.currentPosition());
  return;
}

void handleCmd(String s, SystemState& sys, LimitsState& lim, Axis& tiptilt, Axis& azimuth)
{
  s.trim();
  if (!s.length()) return;
  s.toLowerCase();

  char buf[160];
  s.toCharArray(buf, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';

  char* tok[8] = {0};
  int ntok = split_tokens(buf, tok, 8);
  if (ntok <= 0) return;

  const char* action = tok[0];
  CmdClass c = classify_cmd(action);

  if (c == CmdClass::UNKNOWN) {
    Serial.print(F("ERR:Unknown command: "));
    Serial.println(action);
    return;
  }

  // ---- global ----
  if (c == CmdClass::GLOBAL && ntok == 1) 
  {
    if (streq(action, "status_long")) { printStatus_long(sys, lim, tiptilt, azimuth); return; }
    if (streq(action, "status")) { printStatus(sys, lim, tiptilt, azimuth); return; }
    if (streq(action, "position") || streq(action, "pos")) { printPosStatus(sys, lim, tiptilt, azimuth); return; }
    if (streq(action, "enable_limits"))  { cmd_enable_limits(lim); bumpSeq(sys); return; }
    if (streq(action, "disable_limits")) { cmd_disable_limits(lim); bumpSeq(sys); return; }

    if (streq(action, "enable_all") || streq(action, "enableall"))  { cmd_enableall(tiptilt, azimuth);  bumpSeq(sys); return; }
    if (streq(action, "disable_all") || streq(action, "disableall")){ cmd_disableall(tiptilt, azimuth); bumpSeq(sys); return; }
    if (streq(action, "stop_all") || streq(action, "stopall") || streq(action, "stop"))      { cmd_stopall(tiptilt, azimuth);    bumpSeq(sys); return; }
    if (streq(action, "estop")) { cmd_estop(tiptilt, azimuth);    bumpSeq(sys); return; }
    if (streq(action, "reset") || streq(action, "reboot")) { Serial.println(F("Resetting controller.")); delay(50); wdt_enable(WDTO_15MS); while (1) {}  }

    Serial.print(F("ERR:Unhandled global command: "));
    Serial.println(action);
    return;
  }

  // ---- axis ----
  if (ntok < 2 || !tok[1] || !tok[1][0]) 
  {
    Serial.print(F("ERR:Axis required (a/b) for command: "));
    Serial.println(action);
    return;
  }

  char axis_id = tok[1][0];
  Axis* ax = axisFromId(axis_id, tiptilt, azimuth);
  if (!ax) 
  {
    Serial.println(F("ERR:Invalid axis (use a or b)."));
    return;
  }

  updateLimitBlocks(*ax, lim);

  if (streq(action, "enable"))  { cmd_enable_axis(ax, axis_id);  bumpSeq(sys); return; }
  if (streq(action, "disable")) { cmd_disable_axis(ax, axis_id); bumpSeq(sys); return; }
  if (streq(action, "stop"))    { cmd_stop_axis(ax, axis_id);    bumpSeq(sys); return; }
  if (streq(action, "home") || streq(action, "zero")) { cmd_home_axis(ax, axis_id, lim); bumpSeq(sys); return; }
  
  if (streq(action, "setaxis")) { cmd_set_axis(sys, ax, axis_id, ntok, tok); return; }
  if (streq(action, "move"))   { cmd_move(sys, ax, axis_id, lim, ntok, tok); return; }
  if (streq(action, "moveto")) { cmd_moveto(sys, ax, axis_id, lim, ntok, tok); return; }

  Serial.print(F("ERR:Unhandled axis command: "));
  Serial.println(action);
  return;
}