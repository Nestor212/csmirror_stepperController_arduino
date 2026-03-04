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
  // AccelStepper stop() decelerates; also cancel target so it won't resume toward the limit.
  ax.stepper.stop();
  ax.stepper.moveTo(ax.stepper.currentPosition());
}