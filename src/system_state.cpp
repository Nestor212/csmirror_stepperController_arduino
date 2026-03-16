#include "system_state.h"

static uint32_t pseudoRandom32()
{
  // Boot Marker
  // Mix time + floating analog pin noise.
  uint32_t x = (uint32_t)millis();
  x ^= ((uint32_t)analogRead(A0) << 16);
  x ^= ((uint32_t)analogRead(A1) << 0);

  // simple xorshift
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;

  if (x == 0) x = 1; // avoid 0
  return x;
}

void systemStateInit(SystemState& st)
{
  st.boot_id = pseudoRandom32();
  st.seq = 0;
}