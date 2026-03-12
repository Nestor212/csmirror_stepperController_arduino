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
  ax.stepper.setAcceleration(3000);   // choose a much higher safe value
  ax.stepper.stop();                  // decelerate quickly
}

void emergencyStopAxis(Axis& ax)
{
  ax.stepper.setCurrentPosition(ax.stepper.currentPosition());   // target = current, speed = 0

  // Finally remove motor torque
  setEnable(ax, false);
}