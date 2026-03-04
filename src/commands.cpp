#include "commands.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

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
    // skip leading whitespace
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
    if (!*s) break;

    out[n++] = s;

    // advance to end of token
    while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;

    // terminate token
    if (*s) *s++ = '\0';
  }
  return n;
}

static bool streq(const char* a, const char* b) { return strcmp(a, b) == 0; }

// safe-ish parse helpers (no exceptions)
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

static bool parse_float(const char* s, float& out)
{
  if (!s || !*s) return false;

  out = atof(s);   // AVR-safe
  return true;
}

// Map 'a'/'b' to axis pointer
static Axis* axisFromId(char id, Axis& tiptilt, Axis& azimuth)
{
  if (id == 'a') return &tiptilt;
  if (id == 'b') return &azimuth;
  return nullptr;
}

// -------------------- command handlers --------------------

static void cmd_status(const LimitsState& lim, Axis& tiptilt, Axis& azimuth)
{
  printStatus(lim, tiptilt, azimuth);
}

static void cmd_enable_limits(LimitsState& lim)
{
  lim.enabled = true;
  Serial.println("Limit switches enabled.");
  Serial.println("OK");
}

static void cmd_disable_limits(LimitsState& lim)
{
  lim.enabled = false;
  if (LIMITS_TIMEOUT_MS > 0) {
    lim.disableUntilMs = millis() + LIMITS_TIMEOUT_MS;
    Serial.print("Limit switches disabled for ");
    Serial.print(LIMITS_TIMEOUT_MS);
    Serial.println(" ms.");
  } else {
    Serial.println("Limit switches disabled.");
  }
  Serial.println("OK");
}

static void cmd_enableall(Axis& tiptilt, Axis& azimuth)
{
  setEnable(tiptilt, true);
  setEnable(azimuth, true);
  Serial.println("All motors enabled.");
  Serial.println("OK");
}

static void cmd_disableall(Axis& tiptilt, Axis& azimuth)
{
  setEnable(tiptilt, false);
  setEnable(azimuth, false);
  Serial.println("All motors disabled.");
  Serial.println("OK");
}

static void cmd_stopall(Axis& tiptilt, Axis& azimuth)
{
  stopAxis(tiptilt);
  stopAxis(azimuth);
  Serial.println("All motors stopped.");
  Serial.println("OK");
}

static void cmd_enable_axis(Axis* ax, char motor_id)
{
  if (!ax) { Serial.println("ERR"); return; }
  setEnable(*ax, true);
  Serial.print("Motor ");
  Serial.print((char)toupper(motor_id));
  Serial.println(" enabled.");
  Serial.println("OK");
}

static void cmd_disable_axis(Axis* ax, char motor_id)
{
  if (!ax) { Serial.println("ERR"); return; }
  setEnable(*ax, false);
  Serial.print("Motor ");
  Serial.print((char)toupper(motor_id));
  Serial.println(" disabled.");
  Serial.println("OK");
}

static void cmd_stop_axis(Axis* ax, char motor_id)
{
  if (!ax) { Serial.println("ERR"); return; }
  stopAxis(*ax);
  Serial.print("Motor ");
  Serial.print((char)toupper(motor_id));
  Serial.println(" stopped.");
  Serial.println("OK");
}

static void cmd_home_axis(Axis* ax, char motor_id, const LimitsState& lim)
{
  if (!ax) { Serial.println("ERR"); return; }

  if (!lim.enabled) {
    Serial.print("ERR:Motor ");
    Serial.print((char)toupper(motor_id));
    Serial.println(" limits are disabled. Cannot home.");
    return;
  }

  startHoming(*ax);
  Serial.println("OK");
}

// Robust MOVE parser:
// move <axis> <dir> <steps> [speed] [accel]
static void cmd_move(Axis* ax, char motor_id, const LimitsState& lim, int ntok, char* tok[])
{
  if (!ax) { Serial.println("ERR"); return; }

  // tok[0]=move tok[1]=axis tok[2]=dir tok[3]=steps tok[4]=speed tok[5]=accel
  if (ntok < 4) {
    Serial.println("ERR:Invalid MOVE command format.");
    Serial.println("Expected: move <axis> <dir> <steps> [speed] [accel]");
    return;
  }

  int dir = 0;
  long steps = 0;

  if (!parse_int(tok[2], dir) || !parse_long(tok[3], steps)) {
    Serial.println("ERR:Invalid MOVE args (dir/steps).");
    return;
  }

  float speed = ax->stepper.maxSpeed();
  float accel = 0.0f; // 0 => treated as "constant-ish speed" by your accel fallback

  if (ntok >= 5) {
    if (!parse_float(tok[4], speed)) {
      Serial.println("ERR:Invalid speed.");
      return;
    }
  }
  if (ntok >= 6) {
    if (!parse_float(tok[5], accel)) {
      Serial.println("ERR:Invalid accel.");
      return;
    }
  }

  if (dir != 0 && dir != 1) {
    Serial.println("ERR:dir must be 0 or 1");
    return;
  }

  if (!ax->enabled) {
    Serial.print("ERR:Motor ");
    Serial.print((char)toupper(motor_id));
    Serial.println(" is disabled.");
    return;
  }

  if (speed < 1.0f) speed = 1.0f;
  ax->stepper.setMaxSpeed(speed);

  // keep your existing behavior: accel < 1 => huge accel (approx constant speed)
  if (accel < 1.0f) accel = 1000000.0f;
  ax->stepper.setAcceleration(accel);

  long cur = ax->stepper.currentPosition();
  long delta = (dir == 0) ? -steps : +steps;
  long target = cur + delta;

  updateLimitBlocks(*ax, lim);
  if (!allowTarget(*ax, lim, target)) {
    Serial.println("ERR:Move blocked by limits or bounds.");
    return;
  }

  ax->stepper.moveTo(target);

  Serial.println("OK");
}

// Robust MOVETO parser:
// moveto <axis> <pos> [speed] [accel]
static void cmd_moveto(Axis* ax, char motor_id, const LimitsState& lim, int ntok, char* tok[])
{
  if (!ax) { Serial.println("ERR"); return; }

  // tok[0]=moveto tok[1]=axis tok[2]=pos tok[3]=speed tok[4]=accel
  if (ntok < 3) {
    Serial.println("ERR:Invalid MOVETO command format.");
    Serial.println("Expected: moveto <axis> <pos> [speed] [accel]");
    return;
  }

  long pos = 0;
  if (!parse_long(tok[2], pos)) {
    Serial.println("ERR:Invalid position.");
    return;
  }

  float speed = ax->stepper.maxSpeed();
  float accel = 0.0f;

  if (ntok >= 4) {
    if (!parse_float(tok[3], speed)) {
      Serial.println("ERR:Invalid speed.");
      return;
    }
  }
  if (ntok >= 5) {
    if (!parse_float(tok[4], accel)) {
      Serial.println("ERR:Invalid accel.");
      return;
    }
  }

  if (!ax->enabled) {
    Serial.print("ERR:Motor ");
    Serial.print((char)toupper(motor_id));
    Serial.println(" is disabled.");
    return;
  }

  if (speed < 1.0f) speed = 1.0f;
  ax->stepper.setMaxSpeed(speed);

  if (accel < 1.0f) accel = 1000000.0f;
  ax->stepper.setAcceleration(accel);

  updateLimitBlocks(*ax, lim);
  if (!allowTarget(*ax, lim, pos)) {
    Serial.println("ERR:Move blocked by limits or bounds.");
    return;
  }

  ax->stepper.moveTo(pos);

  Serial.println("OK");
}

// -------------------- public API --------------------

void printStatus(const LimitsState& lim, Axis& tiptilt, Axis& azimuth)
{
  auto printAx = [](const char* name, Axis& ax) {
    Serial.print(name);
    Serial.print(" en=");
    Serial.print(ax.enabled);
    Serial.print(" homed=");
    Serial.print(ax.homed);
    Serial.print(" hs=");
    Serial.print((int)ax.hs);
    Serial.print(" pos=");
    Serial.print(ax.stepper.currentPosition());
    Serial.print(" tgt=");
    Serial.print(ax.stepper.targetPosition());
    Serial.print(" max=");
    Serial.print(ax.maxPos);
    Serial.print(" lo=");
    Serial.print(limitTriggered(ax.limLoPin));
    Serial.print(" hi=");
    Serial.print(limitTriggered(ax.limHiPin));
    Serial.print(" block=");
    Serial.print((int)ax.blockDir);
    Serial.println();
  };

  Serial.print("limitsEnabled=");
  Serial.println(lim.enabled ? 1 : 0);

  printAx("Tip/Tilt", tiptilt);
  printAx("Azimuth", azimuth);
}

void handleCmd(String s, LimitsState& lim, Axis& tiptilt, Axis& azimuth)
{
  s.trim();
  if (!s.length()) return;
  s.toLowerCase();

  // copy into mutable buffer for tokenization
  char buf[160];
  s.toCharArray(buf, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';

  char* tok[8] = {0};
  int ntok = split_tokens(buf, tok, 8);
  if (ntok <= 0) return;

  // tok[0] is the action
  const char* action = tok[0];

  // Global commands
  if (streq(action, "status")) { cmd_status(lim, tiptilt, azimuth); return; }
  if (streq(action, "enable_limits")) { cmd_enable_limits(lim); return; }
  if (streq(action, "disable_limits")) { cmd_disable_limits(lim); return; }
  if (streq(action, "enableall")) { cmd_enableall(tiptilt, azimuth); return; }
  if (streq(action, "disableall")) { cmd_disableall(tiptilt, azimuth); return; }
  if (streq(action, "stopall")) { cmd_stopall(tiptilt, azimuth); return; }

  // Per-axis commands require tok[1] = axis id
  if (ntok < 2 || !tok[1] || !tok[1][0]) {
    Serial.println("ERR:Axis required (a/b).");
    return;
  }

  char axis_id = tok[1][0]; // expect 'a' or 'b'
  Axis* ax = axisFromId(axis_id, tiptilt, azimuth);
  if (!ax) {
    Serial.println("ERR:Invalid axis (use a or b).");
    return;
  }

  // Update blocks before acting (so we can reject moves)
  updateLimitBlocks(*ax, lim);

  if (streq(action, "enable")) { cmd_enable_axis(ax, axis_id); return; }
  if (streq(action, "disable")) { cmd_disable_axis(ax, axis_id); return; }
  if (streq(action, "stop")) { cmd_stop_axis(ax, axis_id); return; }
  if (streq(action, "home") || streq(action, "zero")) { cmd_home_axis(ax, axis_id, lim); return; }

  if (streq(action, "move")) { cmd_move(ax, axis_id, lim, ntok, tok); return; }
  if (streq(action, "moveto")) { cmd_moveto(ax, axis_id, lim, ntok, tok); return; }

  Serial.print("ERR:Unknown command: ");
  Serial.println(action);
}