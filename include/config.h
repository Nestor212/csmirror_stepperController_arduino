#pragma once
#include <Arduino.h>

// -------------------- LIMITS MODE --------------------
// enable/disable bounds + photodetector enforcement
static const uint32_t LIMITS_TIMEOUT_MS = 60000; // 60s safety re-enable (set 0 to disable timer)
constexpr uint32_t HOME_TIMEOUT_MS = 60000; // 60s homing timeout (set 0 to disable)

// -------------------- DRIVER SETTINGS --------------------
static const bool TB6600_ENABLE_ACTIVE_LOW = true; // ENA asserted when LOW

// Homing profile
static const float HOME_FAST_SPEED = 2000.0f;      // steps/sec (tune)
static const float HOME_SLOW_SPEED = 200.0f;       // steps/sec (tune)
static const float HOME_ACCEL      = 4000.0f;      // steps/sec^2 (tune)

static const long  BACKOFF_STEPS      = 100;       // back off from switch after hit
static const long  FINAL_CLEAR_STEPS  = 50;        // move away after final touch
static const long  BIG_TRAVEL         = 1000000L;  // big move used for seek
static const uint16_t DEBOUNCE_MS     = 10;

// Step pulse width for TB6600 clones
static const uint16_t MIN_PULSE_WIDTH_US = 10;