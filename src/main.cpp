#include <AccelStepper.h>

// -------------------- PINOUT --------------------
#define MOTOR_A_EN   2
#define MOTOR_A_DIR 3
#define MOTOR_A_STEP  4
#define PHOTODETECTOR_A1 5  // lower
#define PHOTODETECTOR_A2 6  // upper

#define MOTOR_B_EN   8
#define MOTOR_B_DIR 9
#define MOTOR_B_STEP  10
#define PHOTODETECTOR_B1 11  // lower
#define PHOTODETECTOR_B2 12  // upper

// -------------------- SETTINGS --------------------
static const bool TB6600_ENABLE_ACTIVE_LOW = true;   // ENA asserted when LOW (common wiring)
static const bool LIMIT_ACTIVE_LOW = true;           // using INPUT_PULLUP => triggered=LOW

// Homing profile
static const float HOME_FAST_SPEED = 1000.0f;        // steps/sec (tune)
static const float HOME_SLOW_SPEED = 300.0f;         // steps/sec (tune)
static const float HOME_ACCEL      = 500.0f;        // steps/sec^2 (tune)

static const long  BACKOFF_STEPS   = 1000;            // back off from switch after hit
static const long  FINAL_CLEAR_STEPS = 100;           // move away after final touch
static const long  BIG_TRAVEL      = 1000000L;       // big move used for seek
static const uint16_t DEBOUNCE_MS  = 10;

// -------------------- AXIS STRUCT --------------------
enum class HomeState : uint8_t {
  IDLE,
  SEEK_LOWER_FAST,
  BACKOFF_FROM_LOWER,
  SEEK_LOWER_SLOW,
  CLEAR_LOWER_FINAL,
  SET_ZERO,

  SEEK_UPPER_FAST,
  BACKOFF_FROM_UPPER,
  SEEK_UPPER_SLOW,
  CLEAR_UPPER_FINAL,
  SET_MAX,

  MOVE_TO_MID,
  DONE,
  ERROR
};

struct Axis {
  AccelStepper stepper;
  uint8_t enPin;
  uint8_t limLoPin;
  uint8_t limHiPin;

  HomeState hs = HomeState::IDLE;
  bool enabled = false;
  bool homed = false;

  long maxPos = 0;
  uint32_t t_ms = 0;

  Axis(uint8_t stepPin, uint8_t dirPin, uint8_t en, uint8_t lo, uint8_t hi)
  : stepper(AccelStepper::DRIVER, stepPin, dirPin), enPin(en), limLoPin(lo), limHiPin(hi) {}
};

Axis tiptilt(MOTOR_A_STEP, MOTOR_A_DIR, MOTOR_A_EN, PHOTODETECTOR_A1, PHOTODETECTOR_A2);
Axis azimuth(MOTOR_B_STEP, MOTOR_B_DIR, MOTOR_B_EN, PHOTODETECTOR_B1, PHOTODETECTOR_B2);

// -------------------- HELPERS --------------------
static bool limitTriggered(uint8_t pin) {
  int v = digitalRead(pin);
  return LIMIT_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

static void setEnable(Axis& ax, bool on) {
  ax.enabled = on;
  if (TB6600_ENABLE_ACTIVE_LOW) digitalWrite(ax.enPin, on ? LOW : HIGH);
  else                         digitalWrite(ax.enPin, on ? HIGH : LOW);
}

static void startHoming(Axis& ax) {
  if (!ax.enabled) setEnable(ax, true);

  ax.homed = false;
  ax.maxPos = 0;

  ax.stepper.setAcceleration(HOME_ACCEL);
  ax.stepper.setMaxSpeed(HOME_FAST_SPEED);

  // Start by seeking lower limit (negative direction assumed)
  ax.stepper.moveTo(-BIG_TRAVEL);
  ax.hs = HomeState::SEEK_LOWER_FAST;
}

static void stopAxis(Axis& ax) {
  ax.stepper.stop(); // decel stop
}

static void updateHoming(Axis& ax) {
  switch (ax.hs) {
    case HomeState::IDLE:
    case HomeState::DONE:
    case HomeState::ERROR:
      return;

    // -------- Lower limit --------
    case HomeState::SEEK_LOWER_FAST: {
      ax.stepper.run();
      if (limitTriggered(ax.limLoPin)) {
        ax.stepper.stop();
        ax.t_ms = millis();
        ax.hs = HomeState::BACKOFF_FROM_LOWER;
      }
      return;
    }

    case HomeState::BACKOFF_FROM_LOWER: {
      ax.stepper.run(); // let stop/decel complete
      if (millis() - ax.t_ms < DEBOUNCE_MS) return;

      // Back off positive direction
      ax.stepper.setMaxSpeed(HOME_FAST_SPEED);
      ax.stepper.move(BACKOFF_STEPS);
      ax.hs = HomeState::SEEK_LOWER_SLOW;
      return;
    }

    case HomeState::SEEK_LOWER_SLOW: {
      ax.stepper.run();
      if (ax.stepper.distanceToGo() == 0) {
        // After backing off, creep back into the lower switch
        ax.stepper.setMaxSpeed(HOME_SLOW_SPEED);
        ax.stepper.moveTo(-BIG_TRAVEL);
      }
      if (limitTriggered(ax.limLoPin)) {
        ax.stepper.stop();
        ax.t_ms = millis();
        ax.hs = HomeState::CLEAR_LOWER_FINAL;
      }
      return;
    }

    case HomeState::CLEAR_LOWER_FINAL: {
      ax.stepper.run();
      if (millis() - ax.t_ms < DEBOUNCE_MS) return;

      // Move away slightly so we’re not sitting on the edge
      ax.stepper.setMaxSpeed(HOME_SLOW_SPEED);
      ax.stepper.move(FINAL_CLEAR_STEPS);
      ax.hs = HomeState::SET_ZERO;
      return;
    }

    case HomeState::SET_ZERO: {
      ax.stepper.run();
      if (ax.stepper.distanceToGo() != 0) return;

      ax.stepper.setCurrentPosition(0);

      // -------- Upper limit --------
      ax.stepper.setAcceleration(HOME_ACCEL);
      ax.stepper.setMaxSpeed(HOME_FAST_SPEED);
      ax.stepper.moveTo(+BIG_TRAVEL);
      ax.hs = HomeState::SEEK_UPPER_FAST;
      return;
    }

    // -------- Upper limit --------
    case HomeState::SEEK_UPPER_FAST: {
      ax.stepper.run();
      if (limitTriggered(ax.limHiPin)) {
        ax.stepper.stop();
        ax.t_ms = millis();
        ax.hs = HomeState::BACKOFF_FROM_UPPER;
      }
      return;
    }

    case HomeState::BACKOFF_FROM_UPPER: {
      ax.stepper.run();
      if (millis() - ax.t_ms < DEBOUNCE_MS) return;

      // Back off negative direction
      ax.stepper.setMaxSpeed(HOME_FAST_SPEED);
      ax.stepper.move(-BACKOFF_STEPS);
      ax.hs = HomeState::SEEK_UPPER_SLOW;
      return;
    }

    case HomeState::SEEK_UPPER_SLOW: {
      ax.stepper.run();
      if (ax.stepper.distanceToGo() == 0) {
        // After backing off, creep back into the upper switch
        ax.stepper.setMaxSpeed(HOME_SLOW_SPEED);
        ax.stepper.moveTo(+BIG_TRAVEL);
      }
      if (limitTriggered(ax.limHiPin)) {
        ax.stepper.stop();
        ax.t_ms = millis();
        ax.hs = HomeState::CLEAR_UPPER_FINAL;
      }
      return;
    }

    case HomeState::CLEAR_UPPER_FINAL: {
      ax.stepper.run();
      if (millis() - ax.t_ms < DEBOUNCE_MS) return;

      // Move away slightly (negative direction)
      ax.stepper.setMaxSpeed(HOME_SLOW_SPEED);
      ax.stepper.move(-FINAL_CLEAR_STEPS);
      ax.hs = HomeState::SET_MAX;
      return;
    }

    case HomeState::SET_MAX: {
      ax.stepper.run();
      if (ax.stepper.distanceToGo() != 0) return;

      // Current position after clearing upper is "near upper".
      // For max, we want the position at the *upper edge*. We can approximate:
      // maxPos = currentPosition + FINAL_CLEAR_STEPS (since we moved away).
      ax.maxPos = ax.stepper.currentPosition() + FINAL_CLEAR_STEPS;

      // Move to middle
      long mid = ax.maxPos / 2;
      ax.stepper.setMaxSpeed(HOME_FAST_SPEED);
      ax.stepper.moveTo(mid);
      ax.hs = HomeState::MOVE_TO_MID;
      return;
    }

    case HomeState::MOVE_TO_MID: {
      ax.stepper.run();
      if (ax.stepper.distanceToGo() == 0) {
        ax.homed = true;
        ax.hs = HomeState::DONE;
      }
      return;
    }

    default:
      ax.hs = HomeState::ERROR;
      return;
  }
}

// -------------------- SIMPLE USB COMMANDS --------------------
static String line;

static Axis* axisFromToken(const String& tok) {
  if (tok == "A") return &tiptilt;
  if (tok == "B") return &azimuth;
  return nullptr;
}

static void printStatus() {
  auto printAx = [](const char* name, Axis& ax) {
    Serial.print(name);
    Serial.print(" en=");
    Serial.println(ax.enabled);
    Serial.print(" homed=");
    Serial.println(ax.homed);
    Serial.print(" hs=");
    Serial.println((int)ax.hs);
    Serial.print(" pos=");
    Serial.println(ax.stepper.currentPosition());
    Serial.print(" tgt=");
    Serial.println(ax.stepper.targetPosition());
    Serial.print(" max=");
    Serial.println(ax.maxPos);
    Serial.print(" lo=");
    Serial.println(limitTriggered(ax.limLoPin));
    Serial.print(" hi=");
    Serial.println(limitTriggered(ax.limHiPin));
  };

  printAx("Tip/Tilt", tiptilt);
  printAx("Azimuth", azimuth);
}

static void handleCmd(String s) {
  s.trim();
  if (!s.length()) return;

  // tokens: CMD [AXIS|ALL] [arg]
  String cmd, t1, t2;
  int p = s.indexOf(' ');
  cmd = (p < 0) ? s : s.substring(0, p);
  String rest = (p < 0) ? "" : s.substring(p + 1);
  rest.trim();

  int q = rest.indexOf(' ');
  t1 = (q < 0) ? rest : rest.substring(0, q);
  t2 = (q < 0) ? "" : rest.substring(q + 1);
  t2.trim();

  cmd.toUpperCase();
  t1.toUpperCase();

  if (cmd == "ENA") {
    // ENA A|B|ALL 0|1
    bool on = (t2 == "1");
    if (t1 == "ALL") { setEnable(tiptilt, on); setEnable(azimuth, on); }
    else if (Axis* ax = axisFromToken(t1)) setEnable(*ax, on);
    else { Serial.println("ERR axis"); return; }
    Serial.println("ENA CMD:OK");
    return;
  }

  if (cmd == "HOME") {
    // HOME A|B|ALL
    if (t1 == "ALL") { startHoming(tiptilt); startHoming(azimuth); }
    else if (Axis* ax = axisFromToken(t1)) startHoming(*ax);
    else { Serial.println("ERR axis"); return; }
    Serial.println("HOME CMD:OK");
    return;
  }

  if (cmd == "MOVETO") {
    // MOVETO A 12345
    Axis* ax = axisFromToken(t1);
    if (!ax) { Serial.println("ERR axis"); return; }
    long pos = t2.toInt();
    if (!ax->enabled) setEnable(*ax, true);
    // Optional clamp to [0, maxPos] if homed
    if (ax->homed) {
      if (pos < 0) pos = 0;
      if (pos > ax->maxPos) pos = ax->maxPos;
    }
    ax->stepper.moveTo(pos);
    Serial.println("MOVETO CMD:OK");
    return;
  }

  if (cmd == "MOVE") {
    // MOVE A -100
    Axis* ax = axisFromToken(t1);
    if (!ax) { Serial.println("ERR axis"); return; }
    long delta = t2.toInt();
    if (!ax->enabled) setEnable(*ax, true);
    ax->stepper.move(delta);
    Serial.println("MOVE CMD:OK");
    return;
  }

  if (cmd == "STOP") {
    // STOP A|B|ALL
    if (t1 == "ALL") { stopAxis(tiptilt); stopAxis(azimuth); }
    else if (Axis* ax = axisFromToken(t1)) stopAxis(*ax);
    else { Serial.println("ERR axis"); return; }
    Serial.println("STOP CMD:OK");
    return;
  }

  if (cmd == "STATUS") {
    printStatus();
    Serial.println("STATUS CMD:OK");
    return;
  }

  Serial.println("ERR unknown");
}

// -------------------- SETUP/LOOP --------------------
void setup() {
  Serial.begin(115200);

  // Pins
  pinMode(tiptilt.enPin, OUTPUT);
  pinMode(azimuth.enPin, OUTPUT);

  pinMode(tiptilt.limLoPin, INPUT_PULLUP);
  pinMode(tiptilt.limHiPin, INPUT_PULLUP);
  pinMode(azimuth.limLoPin, INPUT_PULLUP);
  pinMode(azimuth.limHiPin, INPUT_PULLUP);

  // Disable motors by default
  setEnable(tiptilt, false);
  setEnable(azimuth, false);

  // Default motion params (you’ll tune)
  tiptilt.stepper.setMaxSpeed(1000);
  tiptilt.stepper.setAcceleration(100);

  azimuth.stepper.setMaxSpeed(600);
  azimuth.stepper.setAcceleration(100);

  Serial.println("READY");
}

void loop() {
  // Run motion / homing
  if (tiptilt.hs == HomeState::IDLE || tiptilt.hs == HomeState::DONE || tiptilt.hs == HomeState::ERROR)
    tiptilt.stepper.run();
  else
    updateHoming(tiptilt);

  if (azimuth.hs == HomeState::IDLE || azimuth.hs == HomeState::DONE || azimuth.hs == HomeState::ERROR)
    azimuth.stepper.run();
  else
    updateHoming(azimuth);

  // Read commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleCmd(line);
      line = "";
    } else {
      if (line.length() < 120) line += c;
    }
  }
}