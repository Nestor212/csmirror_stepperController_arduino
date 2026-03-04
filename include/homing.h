#pragma once
#include <Arduino.h>
#include "axis.h"

// Start homing sequence (lower -> upper -> mid)
void startHoming(Axis& ax);

// Run homing state machine update (call frequently from loop)
void updateHoming(Axis& ax);