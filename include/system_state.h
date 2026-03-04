#pragma once
#include <Arduino.h>

struct SystemState {
  uint32_t boot_id = 0;
  uint32_t seq = 0;
};

void systemStateInit(SystemState& st);

// Increment seq when Arduino state changes (moves accepted/completed, homing done, etc.)
static inline void bumpSeq(SystemState& st) { st.seq++; }