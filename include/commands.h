#pragma once
#include <Arduino.h>
#include "axis.h"
#include "limits.h"

// Print system status (same as your original)
void printStatus(const LimitsState& lim, Axis& tiptilt, Axis& azimuth);

// Handle one command line (lowercased + trimmed inside)
void handleCmd(String s, LimitsState& lim, Axis& tiptilt, Axis& azimuth);