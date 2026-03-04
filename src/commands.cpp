#include "commands.h"
#include <ctype.h>
#include "config.h"
#include "homing.h"
#include "limits.h"
#include "axis.h"

static bool handle_special_command(const String& action, LimitsState& lim, Axis& tiptilt, Axis& azimuth)
{
  if (action == "status")
  {
    printStatus(lim, tiptilt, azimuth);
    return true;
  }

  if (action == "enable_limits")
  {
    lim.enabled = true;
    Serial.println("Limit switches enabled.");
    return true;
  }

  if (action == "disable_limits")
  {
    lim.enabled = false;
    if (LIMITS_TIMEOUT_MS > 0)
    {
      lim.disableUntilMs = millis() + LIMITS_TIMEOUT_MS;
      Serial.print("Limit switches disabled for ");
      Serial.print(LIMITS_TIMEOUT_MS);
      Serial.println(" ms.");
    } else {
      Serial.println("Limit switches disabled.");
    }
    return true;
  }

  if (action == "stopall")
  {
    stopAxis(tiptilt);
    stopAxis(azimuth);
    Serial.println("All motors stopped.");
    return true;
  }

  if (action == "enableall")
  {
    setEnable(tiptilt, true);
    setEnable(azimuth, true);
    Serial.println("All motors enabled.");
    return true;
  }

  if (action == "disableall")
  {
    setEnable(tiptilt, false);
    setEnable(azimuth, false);
    Serial.println("All motors disabled.");
    return true;
  }

  return false;
}

static bool handle_motor_action(Axis* ax, char motor_id, const String& action)
{
  if (!ax) return false;

  if (action == "enable")
  {
    setEnable(*ax, true);
    Serial.print("Motor ");
    Serial.print((char)toupper(motor_id));
    Serial.println(" enabled.");
    return true;
  }

  if (action == "disable")
  {
    setEnable(*ax, false);
    Serial.print("Motor ");
    Serial.print((char)toupper(motor_id));
    Serial.println(" disabled.");
    return true;
  }

  if (action == "stop")
  {
    stopAxis(*ax);
    Serial.print("Motor ");
    Serial.print((char)toupper(motor_id));
    Serial.println(" stopped.");
    return true;
  }

  return false;
}

static void handle_move_command(Axis* ax, char motor_id, const LimitsState& lim, const char* cmd_cstr)
{
  // Format: move <motor> <dir> <steps> <speed> <accel>
  // dir: 0 = negative, 1 = positive
  int dir = 0;
  long steps = 0;
  float speed = 0.0f;
  float accel = 0.0f;

  if (sscanf(cmd_cstr, "%*s %*s %d %ld %f %f", &dir, &steps, &speed, &accel) < 4)
  {
    Serial.println("ERR:Invalid MOVE command format.");
    //print command string for debugging
    Serial.print("Received command: ");
    Serial.println(cmd_cstr);
    //print individual parsed values for debugging
    Serial.print("Parsed dir: ");
    Serial.print(dir);  
    Serial.print(" steps: ");
    Serial.print(steps);
    Serial.print(" speed: ");
    Serial.print(speed);
    Serial.print(" accel: ");    
    Serial.println(accel);
    return;
  }

  if (dir != 0 && dir != 1)
  {
    Serial.println("Invalid direction. Must be 0 or 1.");
    Serial.println("ERR");
    return;
  }

  if (!ax->enabled)
  {
    Serial.print("Motor ");
    Serial.print((char)toupper(motor_id));
    Serial.println(" is disabled. Move not started.");
    Serial.println("ERR");
    return;
  }

  if (speed < 1.0f) speed = 1.0f;
  ax->stepper.setMaxSpeed(speed);

  if (accel < 1.0f) accel = 1000000.0f; // approximate constant speed mode
  ax->stepper.setAcceleration(accel);

  long cur = ax->stepper.currentPosition();
  long delta = (dir == 0) ? -steps : +steps;
  long target = cur + delta;

  // Update blocks right before evaluating command
  updateLimitBlocks(*ax, lim);

  if (!allowTarget(*ax, lim, target))
  {
    Serial.println("Move blocked by limits or bounds.");
    Serial.println("ERR");
    return;
  }

  ax->stepper.moveTo(target);

  Serial.print("Motor ");
  Serial.print((char)toupper(motor_id));
  Serial.print(" moving: dir=");
  Serial.print(dir);
  Serial.print(" steps=");
  Serial.print(steps);
  Serial.print(" speed=");
  Serial.print(speed, 2);
  Serial.print(" accel=");
  Serial.println(accel, 2);
}

static void handle_moveto_command(Axis* ax, char motor_id, const LimitsState& lim, const char* cmd_cstr)
{
  // Format: moveto <motor> <position> <speed> <accel>
  long pos = 0;
  float speed = 0.0f;
  float accel = 0.0f;

  if (sscanf(cmd_cstr, "%*s %*s %ld %f %f", &pos, &speed, &accel) < 3)
  {
    Serial.println("Invalid MOVETO command format.");
    Serial.println("ERR");
    return;
  }

  if (!ax->enabled)
  {
    Serial.print("Motor ");
    Serial.print((char)toupper(motor_id));
    Serial.println(" is disabled. Move not started.");
    Serial.println("ERR");
    return;
  }

  if (speed < 1.0f) speed = 1.0f;
  ax->stepper.setMaxSpeed(speed);

  if (accel < 1.0f) accel = 1000000.0f;
  ax->stepper.setAcceleration(accel);

  updateLimitBlocks(*ax, lim);

  if (!allowTarget(*ax, lim, pos))
  {
    Serial.println("Move blocked by limits or bounds.");
    Serial.println("ERR");
    return;
  }

  ax->stepper.moveTo(pos);

  Serial.print("Motor ");
  Serial.print((char)toupper(motor_id));
  Serial.print(" moving to: pos=");
  Serial.print(pos);
  Serial.print(" speed=");
  Serial.print(speed, 2);
  Serial.print(" accel=");
  Serial.println(accel, 2);

}

static void handle_home_or_zero(Axis* ax, char motor_id, const LimitsState& lim, const String& action)
{
  (void)action;

  if (!lim.enabled)
  {
    Serial.print("Motor ");
    Serial.print((char)toupper(motor_id));
    Serial.println(" limits are disabled. Cannot home/zero.");
    Serial.println("ERR");
    return;
  }

  // You can add a true "zero" routine later; for now both run the full homing sequence.
  startHoming(*ax);

}

void printStatus(const LimitsState& lim, Axis& tiptilt, Axis& azimuth)
{
  auto printAx = [](const char* name, Axis& ax)
  {
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

  char buf[160];
  s.toCharArray(buf, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';

  char action_c[32] = {0};
  char motor_id = '\0';

  int n = sscanf(buf, "%31s %c", action_c, &motor_id);
  if (n < 1)
  {
    Serial.println("Invalid command format.");
    Serial.println("ERR");
    return;
  }

  String action(action_c);

  // Special commands (no motor id)
  if (handle_special_command(action, lim, tiptilt, azimuth)) return;

  if (n < 2)
  {
    Serial.println("Motor ID required (a/b).");
    Serial.println("ERR");
    return;
  }

  Axis* ax = nullptr;
  if (motor_id == 'a') ax = &tiptilt;
  else if (motor_id == 'b') ax = &azimuth;
  else 
  {
    Serial.print("Invalid motor ID: ");
    Serial.println(motor_id);
    Serial.println("ERR");
    return;
  }

  // Update blocks before action checks (so enable/disable/move sees current limit state)
  updateLimitBlocks(*ax, lim);

  if (handle_motor_action(ax, motor_id, action)) return;

  if (action == "home" || action == "zero")
  {
    handle_home_or_zero(ax, motor_id, lim, action);
    return;
  }

  if (action == "move")
  {
    handle_move_command(ax, motor_id, lim, buf);
    return;
  }

  if (action == "moveto")
  {
    handle_moveto_command(ax, motor_id, lim, buf);
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(action);
  Serial.println("ERR");
}