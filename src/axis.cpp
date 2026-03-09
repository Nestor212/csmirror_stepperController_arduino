#include "axis.h"
#include "config.h"

void axisInitPins(Axis& ax)
{
  pinMode(ax.enPin, OUTPUT);
  pinMode(ax.limLoPin, INPUT_PULLUP);
  pinMode(ax.limHiPin, INPUT_PULLUP);
}

void setEnable(Axis& ax, bool on)
{
  ax.enabled = on;
  if (TB6600_ENABLE_ACTIVE_LOW) digitalWrite(ax.enPin, on ? LOW : HIGH);
  else                          digitalWrite(ax.enPin, on ? HIGH : LOW);
}

void stopAxis(Axis& ax)
{
  ax.cmd_stopRequested = true; // for cooperative stop in loop() vs hard stop from limits
  ax.stepper.setAcceleration(2000);   // choose a much higher safe value
  ax.stepper.stop();                  // decelerate quickly
}

void emergencyStopAxis(Axis& ax)
{
  // Cancel any motion intent in the stepper object
  long pos = ax.stepper.currentPosition();
  ax.stepper.setCurrentPosition(pos);   // target = current, speed = 0

  // Position may no longer be trustworthy after torque is removed
  // Enforce re-homing required before allowing moves again
  ax.posValid = false;
  ax.homed = false; 

  // Finally remove motor torque
  setEnable(ax, false);
}