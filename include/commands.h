#pragma once
#include <Arduino.h>
#include "axis.h"
#include "limits.h"
#include "system_state.h"

// Print system status
void printStatus(const SystemState& sys, const LimitsState& lim, Axis& tiptilt, Axis& azimuth);

// Handle one command line
void handleCmd(String s, SystemState& sys, LimitsState& lim, Axis& tiptilt, Axis& azimuth);