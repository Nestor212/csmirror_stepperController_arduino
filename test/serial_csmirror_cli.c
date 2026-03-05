// serial_csmirror_cli.c
//
// Minimal serial command interface for the CS Mirror stepper Arduino firmware.
//
// What this program does
// ----------------------
// 1) Opens a serial port (e.g. /dev/ttyACM0) at a given baud.
// 2) Provides an interactive "cmd> " prompt.
//    - Local commands: help, save, sync, quit/exit
//    - Anything else is sent verbatim to the Arduino (with multi-command chaining).
// 3) Implements "save" by sending "status", parsing the response, and writing ./state.json.
// 4) Implements "sync" by reading ./state.json and sending "setaxis ..." ONLY if boot_id changed.
//
// Arduino reset prevention (IMPORTANT)
// ------------------------------------
// Many Arduino USB-serial interfaces reset the MCU when DTR transitions HIGH->LOW.
// Linux often drops DTR on the last close of a serial port due to HUPCL (Hang Up On Close).
//
// This program reduces unwanted resets by:
//   A) Clearing HUPCL on the serial port (prevents kernel "hangup" from dropping DTR on close)
//   B) Forcing DTR asserted (HIGH) and RTS deasserted (LOW) after opening the port
//
// Look for the block marked:  "==== ARDUINO RESET PREVENTION ===="
//
// Build (Fedora):
//   sudo dnf install -y gcc make json-c-devel
//   gcc -O2 -Wall -Wextra -std=c11 serial_csmirror_cli.c -o serial_csmirror_cli -ljson-c
//
// Run:
//   ./serial_csmirror_cli /dev/ttyACM0
//   ./serial_csmirror_cli /dev/ttyACM0 --save
//   ./serial_csmirror_cli /dev/ttyACM0 --sync
//
// Notes:
// - Output parsing expects lines similar to your Arduino "status" printout:
//     boot_id=... seq=... limitsEnabled=...
//     Tip/Tilt id=a ... pos=... tgt=... max=... valid=... homed=... en=...
//     Azimuth  id=b ... pos=... tgt=... max=... valid=... homed=... en=...

#define _GNU_SOURCE
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <json-c/json.h>
#include <libgen.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_BAUD 115200

#define STATE_FILE "/home/oper/projects/csmirror_stepperController_arduino/test/state.json"

// ------------------------------
// Help text
// ------------------------------
static const char *ARDUINO_COMMANDS =
"Arduino Commands\n"
"\n"
"Global:\n"
"  status\n"
"  enable_limits\n"
"  disable_limits\n"
"  enable_all     (alias: enableall)\n"
"  disable_all    (alias: disableall)\n"
"  stop_all       (alias: stopall)\n"
"\n"
"Axis commands:\n"
"  enable <axis>\n"
"  disable <axis>\n"
"  stop <axis>\n"
"  home <axis>\n"
"  zero <axis>\n"
"\n"
"  setaxis <axis> <pos> <max> <valid>\n"
"\n"
"  move <axis> <dir> <steps> [speed] [accel]\n"
"      dir: 0 = negative\n"
"           1 = positive\n"
"\n"
"  moveto <axis> <pos> [speed] [accel]\n"
"\n"
"Chaining commands in this program:\n"
"  Use ':', ';', or '|' to send multiple commands back-to-back.\n"
"\n"
"Example:\n"
"  move a 0 10000 100 100 : move b 1 10000 100 100\n";

static const char *LOCAL_COMMANDS =
"Local Commands (cmd> prompt)\n"
"\n"
"  help\n"
"  save     (send 'status', parse it, update ./state.json)\n"
"  sync     (only restore setaxis from ./state.json if boot_id changed)\n"
"  quit / exit\n";

// ------------------------------
// Time helpers
// ------------------------------
static double monotonic_now_s(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static double realtime_now_s(void) {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

// ------------------------------
// Baud conversion (int -> termios speed_t)
// ------------------------------
static speed_t baud_to_speed(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return 0;
  }
}

// ============================================================================
// ==== ARDUINO RESET PREVENTION ==============================================
// ============================================================================

// Clear the HUPCL flag on the serial port.
// HUPCL = "Hang Up On Close". When set, the kernel will drop modem-control lines
// (including DTR) when the last process closes the port.
// Dropping DTR often triggers an Arduino reset (undesired for control systems).
static int disable_hupcl_fd(int fd) {
  struct termios tio;
  if (tcgetattr(fd, &tio) != 0) return -1;
  tio.c_cflag &= ~HUPCL;
  if (tcsetattr(fd, TCSAFLUSH, &tio) != 0) return -1;
  return 0;
}

// Force DTR/RTS levels using modem control ioctls.
// Keeping DTR HIGH avoids falling-edge resets.
// Clearing RTS is a common "be nice" default; your Python did the same.
static int set_dtr_rts(int fd, bool dtr_on, bool rts_on) {
  int status = 0;
  if (ioctl(fd, TIOCMGET, &status) != 0) return -1;

  if (dtr_on) status |= TIOCM_DTR;
  else        status &= ~TIOCM_DTR;

  if (rts_on) status |= TIOCM_RTS;
  else        status &= ~TIOCM_RTS;

  if (ioctl(fd, TIOCMSET, &status) != 0) return -1;
  return 0;
}

// Open and configure the serial port.
static int open_port(const char *path, int baud) {
  // O_NOCTTY: don't let this port become our controlling terminal
  // O_CLOEXEC: close on exec
  int fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd < 0) return -1;

  // ---- Reset prevention step A: clear HUPCL ----
  // If this fails, we keep going (not always fatal).
  (void)disable_hupcl_fd(fd);

  struct termios tio;
  if (tcgetattr(fd, &tio) != 0) { close(fd); return -1; }

  // Raw mode: no line discipline, no echo, no special char handling.
  cfmakeraw(&tio);

  speed_t spd = baud_to_speed(baud);
  if (!spd) { close(fd); errno = EINVAL; return -1; }
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);

  // 8N1
  tio.c_cflag |= (CLOCAL | CREAD); // ignore modem control, enable receiver
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;

  // Non-block-ish read behavior; we still use select() for timed reads.
  // VTIME is in deciseconds.
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 1; // 0.1 s

  if (tcsetattr(fd, TCSAFLUSH, &tio) != 0) { close(fd); return -1; }

  // ---- Reset prevention step B: keep DTR stable/high, RTS low ----
  (void)set_dtr_rts(fd, true, false);

  // Give USB-serial a moment to settle; then clear any junk in the RX buffer.
  usleep(50 * 1000);
  tcflush(fd, TCIFLUSH);

  return fd;
}

// ============================================================================
// ==== SERIAL I/O HELPERS =====================================================
// ============================================================================

// Write all bytes or fail.
static int write_all(int fd, const void *buf, size_t n) {
  const uint8_t *p = (const uint8_t*)buf;
  size_t off = 0;
  while (off < n) {
    ssize_t w = write(fd, p + off, n - off);
    if (w < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    off += (size_t)w;
  }
  return 0;
}

// Send a command line to Arduino, ensuring it ends with '\n'.
static int send_line(int fd, const char *cmd) {
  size_t len = strlen(cmd);
  if (len == 0) return 0;
  if (cmd[len - 1] == '\n') return write_all(fd, cmd, len);
  if (write_all(fd, cmd, len) != 0) return -1;
  return write_all(fd, "\n", 1);
}

// Read one line (terminated by '\n') with an overall timeout in ms.
// Returns:
//   >0 : number of bytes (line content, newline stripped)
//    0 : timeout/no data
//   -1 : error
static int read_line_timeout(int fd, char *out, size_t out_sz, int timeout_ms) {
  if (out_sz < 2) return -1;
  out[0] = '\0';

  size_t used = 0;
  double end = monotonic_now_s() + (double)timeout_ms / 1000.0;

  while (monotonic_now_s() < end) {
    int remaining_ms = (int)((end - monotonic_now_s()) * 1000.0);
    if (remaining_ms < 0) remaining_ms = 0;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    struct timeval tv;
    tv.tv_sec = remaining_ms / 1000;
    tv.tv_usec = (remaining_ms % 1000) * 1000;

    int r = select(fd + 1, &rfds, NULL, NULL, &tv);
    if (r < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    if (r == 0) return 0;

    char ch;
    ssize_t n = read(fd, &ch, 1);
    if (n < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
      return -1;
    }
    if (n == 0) continue;

    if (ch == '\r') continue;

    if (used + 1 < out_sz) {
      out[used++] = ch;
      out[used] = '\0';
    }

    if (ch == '\n') {
      while (used > 0 && (out[used - 1] == '\n' || out[used - 1] == '\r')) {
        out[--used] = '\0';
      }
      return (int)used;
    }
  }

  return 0;
}

// Drain serial for a fixed time window and print everything as "< ..."
// Also returns captured lines via malloc so we can parse them.
static int drain_window(int fd, double seconds, char ***lines_out, size_t *n_out) {
  size_t cap = 32;
  size_t n = 0;
  char **lines = calloc(cap, sizeof(char*));
  if (!lines) return -1;

  double end = monotonic_now_s() + seconds;
  char buf[1024];

  while (monotonic_now_s() < end) {
    int remaining_ms = (int)((end - monotonic_now_s()) * 1000.0);
    if (remaining_ms < 1) remaining_ms = 1;

    int rc = read_line_timeout(fd, buf, sizeof(buf), remaining_ms);
    if (rc < 0) {
      for (size_t i = 0; i < n; i++) free(lines[i]);
      free(lines);
      return -1;
    }
    if (rc == 0) continue;
    if (buf[0] == '\0') continue;

    printf("< %s\n", buf);

    if (n == cap) {
      cap *= 2;
      char **tmp = realloc(lines, cap * sizeof(char*));
      if (!tmp) {
        for (size_t i = 0; i < n; i++) free(lines[i]);
        free(lines);
        return -1;
      }
      lines = tmp;
    }
    lines[n++] = strdup(buf);
  }

  *lines_out = lines;
  *n_out = n;
  return 0;
}

// ============================================================================
// ==== STATUS PARSING =========================================================
// ============================================================================
//
// We don’t use regex; we just search for substrings like "boot_id=" and parse ints.
//
// Expected fields (based on your Python):
//   boot_id=#### seq=#### limitsEnabled=0/1
// Axis lines contain: id=a or id=b plus pos/max/valid (required).
//

typedef struct {
  int pos;
  int tgt;
  int max;
  int valid;
  int homed;
  int en;
  bool present;
} AxisState;

typedef struct {
  double ts; // real time timestamp (seconds since epoch)
  int boot_id;
  int seq;
  int limitsEnabled;
  AxisState a;
  AxisState b;
} StatusRec;

static bool find_int_after(const char *line, const char *key, int *out) {
  const char *p = strstr(line, key);
  if (!p) return false;
  p += strlen(key);
  while (*p == ' ' || *p == '\t') p++;
  char *end = NULL;
  long v = strtol(p, &end, 10);
  if (end == p) return false;
  *out = (int)v;
  return true;
}

static bool find_axis_id(const char *line, char *axis_out) {
  const char *p = strstr(line, "id=");
  if (!p) return false;
  p += 3;
  if (*p == 'a' || *p == 'b') {
    *axis_out = *p;
    return true;
  }
  return false;
}

static void parse_status_line(StatusRec *rec, const char *line) {
  // Global fields can appear on any line; harmless to try each time.
  (void)find_int_after(line, "boot_id=", &rec->boot_id);
  (void)find_int_after(line, "seq=", &rec->seq);
  (void)find_int_after(line, "limitsEnabled=", &rec->limitsEnabled);

  char axis = 0;
  if (!find_axis_id(line, &axis)) return;

  AxisState *ax = (axis == 'a') ? &rec->a : &rec->b;

  int pos=0, tgt=0, max=0, valid=0, homed=0, en=0;
  bool ok_pos = find_int_after(line, "pos=", &pos);
  bool ok_max = find_int_after(line, "max=", &max);
  bool ok_valid = find_int_after(line, "valid=", &valid);

  // Require at least pos/max/valid like your Python did.
  if (!(ok_pos && ok_max && ok_valid)) return;

  if (!find_int_after(line, "tgt=", &tgt)) tgt = pos;
  if (!find_int_after(line, "homed=", &homed)) homed = 0;
  if (!find_int_after(line, "en=", &en)) en = 0;

  ax->pos = pos;
  ax->tgt = tgt;
  ax->max = max;
  ax->valid = valid;
  ax->homed = homed;
  ax->en = en;
  ax->present = true;
}

// Query status from Arduino and parse it into StatusRec.
static int get_status(int fd, StatusRec *out) {
  memset(out, 0, sizeof(*out));
  out->ts = realtime_now_s();
  out->boot_id = -1;
  out->seq = -1;
  out->limitsEnabled = -1;

  if (send_line(fd, "status") != 0) return -1;

  char **lines = NULL;
  size_t n = 0;
  if (drain_window(fd, 0.7, &lines, &n) != 0) return -1;

  for (size_t i = 0; i < n; i++) {
    parse_status_line(out, lines[i]);
    free(lines[i]);
  }
  free(lines);

  bool have_axis = out->a.present || out->b.present;
  if (out->boot_id < 0 || out->seq < 0 || !have_axis) {
    errno = EPROTO;
    return -1;
  }
  return 0;
}

// ============================================================================
// ==== JSON (state.json) HELPERS =============================================
// ============================================================================
//
// state.json structure matches your Python output:
// {
//   "ts": ..., "boot_id":..., "seq":..., "limitsEnabled":...,
//   "axes": { "a":{...}, "b":{...} }
// }

static json_object *axis_to_json(const AxisState *ax) {
  json_object *o = json_object_new_object();
  json_object_object_add(o, "pos", json_object_new_int(ax->pos));
  json_object_object_add(o, "tgt", json_object_new_int(ax->tgt));
  json_object_object_add(o, "max", json_object_new_int(ax->max));
  json_object_object_add(o, "valid", json_object_new_int(ax->valid));
  json_object_object_add(o, "homed", json_object_new_int(ax->homed));
  json_object_object_add(o, "en", json_object_new_int(ax->en));
  return o;
}

static json_object *status_to_json(const StatusRec *st) {
  json_object *root = json_object_new_object();
  json_object_object_add(root, "ts", json_object_new_double(st->ts));
  json_object_object_add(root, "boot_id", json_object_new_int(st->boot_id));
  json_object_object_add(root, "seq", json_object_new_int(st->seq));
  json_object_object_add(root, "limitsEnabled", json_object_new_int(st->limitsEnabled));

  json_object *axes = json_object_new_object();
  if (st->a.present) json_object_object_add(axes, "a", axis_to_json(&st->a));
  if (st->b.present) json_object_object_add(axes, "b", axis_to_json(&st->b));
  json_object_object_add(root, "axes", axes);

  return root;
}

static int write_json_file(const char *path, json_object *obj) {
  // Older json-c may not support "SORTED" flags, so we just pretty-print.
  const char *txt = json_object_to_json_string_ext(obj, JSON_C_TO_STRING_PRETTY);

  FILE *f = fopen(path, "w");
  if (!f) return -1;
  if (fputs(txt, f) < 0) { fclose(f); return -1; }
  fputc('\n', f);
  fclose(f);
  return 0;
}

static json_object *read_json_file(const char *path) {
  FILE *f = fopen(path, "r");
  if (!f) return NULL;

  if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
  long sz = ftell(f);
  if (sz < 0 || sz > 10 * 1024 * 1024) { fclose(f); return NULL; }
  rewind(f);

  char *buf = malloc((size_t)sz + 1);
  if (!buf) { fclose(f); return NULL; }

  size_t n = fread(buf, 1, (size_t)sz, f);
  buf[n] = '\0';
  fclose(f);

  json_object *obj = json_tokener_parse(buf);
  free(buf);
  return obj;
}

// "Coerce" different JSON types to int (string, bool, double, etc.)
static int json_get_int_default(json_object *obj, const char *key, int def) {
  json_object *v = NULL;
  if (!json_object_object_get_ex(obj, key, &v) || !v) return def;

  if (json_object_is_type(v, json_type_int)) return json_object_get_int(v);
  if (json_object_is_type(v, json_type_double)) return (int)json_object_get_double(v);
  if (json_object_is_type(v, json_type_boolean)) return json_object_get_boolean(v) ? 1 : 0;

  if (json_object_is_type(v, json_type_string)) {
    const char *s = json_object_get_string(v);
    if (!s) return def;
    char *end = NULL;
    long x = strtol(s, &end, 10);
    if (end == s) return def;
    return (int)x;
  }
  return def;
}

static bool load_axis_from_json(json_object *axes, const char *name, AxisState *out) {
  json_object *ax = NULL;
  if (!json_object_object_get_ex(axes, name, &ax) || !ax) return false;

  out->pos   = json_get_int_default(ax, "pos", 0);
  out->tgt   = json_get_int_default(ax, "tgt", out->pos);
  out->max   = json_get_int_default(ax, "max", 0);
  out->valid = json_get_int_default(ax, "valid", 0);
  out->homed = json_get_int_default(ax, "homed", 0);
  out->en    = json_get_int_default(ax, "en", 0);
  out->present = true;
  return true;
}

// ============================================================================
// ==== LOCAL COMMANDS: save / sync ===========================================
// ============================================================================

static void cmd_save(int fd, const char *state_path) {
  StatusRec st;
  if (get_status(fd, &st) != 0) {
    fprintf(stderr, "ERROR: Failed to read/parse status (boot_id/seq/axes missing).\n");
    return;
  }

  json_object *obj = status_to_json(&st);
  if (write_json_file(state_path, obj) != 0) {
    fprintf(stderr, "ERROR: Could not write %s: %s\n", state_path, strerror(errno));
  } else {
    printf("Saved state to %s\n", state_path);
  }
  json_object_put(obj);
}

static void cmd_sync(int fd, const char *state_path) {
  json_object *saved = read_json_file(state_path);
  if (!saved) {
    printf("ERROR: no saved state found at %s. Run 'save' first.\n", state_path);
    return;
  }

  int saved_boot = json_get_int_default(saved, "boot_id", -1);

  // Always read current status so we can compare boot_id.
  StatusRec cur;
  if (get_status(fd, &cur) != 0) {
    printf("ERROR: could not read/parse current status.\n");
    json_object_put(saved);
    return;
  }
  int cur_boot = cur.boot_id;

  if (saved_boot < 0 || cur_boot < 0) {
    printf("ERROR: missing boot_id in saved or current status.\n");
    json_object_put(saved);
    return;
  }

  if (saved_boot == cur_boot) {
    printf("SYNC: boot_id matches (%d). Skipping setaxis restore.\n", cur_boot);
    json_object_put(saved);
    return;
  }

  printf("SYNC: boot_id changed (saved %d -> now %d). Restoring axes via setaxis...\n",
         saved_boot, cur_boot);

  json_object *axes = NULL;
  if (!json_object_object_get_ex(saved, "axes", &axes) || !axes) {
    printf("ERROR: saved state has no 'axes' object.\n");
    json_object_put(saved);
    return;
  }

  AxisState axa = {0}, axb = {0};
  bool ha = load_axis_from_json(axes, "a", &axa);
  bool hb = load_axis_from_json(axes, "b", &axb);

  if (!ha && !hb) {
    printf("ERROR: no axes found in saved state.\n");
    json_object_put(saved);
    return;
  }

  // Send setaxis for each axis present in saved state.
  if (ha) {
    int valid = (axa.valid == 1) ? 1 : 0;
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "setaxis a %d %d %d", axa.pos, axa.max, valid);
    printf("> %s\n", cmd);
    (void)send_line(fd, cmd);

    char **lines=NULL; size_t n=0;
    (void)drain_window(fd, 0.35, &lines, &n);
    for (size_t i=0;i<n;i++) free(lines[i]);
    free(lines);
  } else {
    printf("WARNING: axis 'a' missing in state.json; skipping\n");
  }

  if (hb) {
    int valid = (axb.valid == 1) ? 1 : 0;
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "setaxis b %d %d %d", axb.pos, axb.max, valid);
    printf("> %s\n", cmd);
    (void)send_line(fd, cmd);

    char **lines=NULL; size_t n=0;
    (void)drain_window(fd, 0.35, &lines, &n);
    for (size_t i=0;i<n;i++) free(lines[i]);
    free(lines);
  } else {
    printf("WARNING: axis 'b' missing in state.json; skipping\n");
  }

  // Recommended: save fresh status after restore (mirrors your Python behavior).
  StatusRec st2;
  if (get_status(fd, &st2) == 0) {
    json_object *obj2 = status_to_json(&st2);
    if (write_json_file(state_path, obj2) == 0) {
      printf("Saved state to %s\n", state_path);
    } else {
      printf("WARNING: restore done, but failed to save updated status: %s\n", strerror(errno));
    }
    json_object_put(obj2);
  } else {
    printf("WARNING: restore done, but failed to re-read status.\n");
  }

  json_object_put(saved);
}

// ============================================================================
// ==== COMMAND CHAINING (':', ';', '|') ======================================
// ============================================================================

// Split a line into multiple commands separated by :, ;, |
// Returns malloc'd array of malloc'd strings.
static char **split_multi_cmd(const char *line, size_t *count_out) {
  *count_out = 0;
  if (!line) return NULL;

  size_t cap = 8, n = 0;
  char **out = calloc(cap, sizeof(char*));
  if (!out) return NULL;

  const char *p = line;
  while (*p) {
    while (*p && isspace((unsigned char)*p)) p++;

    const char *start = p;
    while (*p && *p != ':' && *p != ';' && *p != '|') p++;
    const char *end = p;

    while (end > start && isspace((unsigned char)end[-1])) end--;

    if (end > start) {
      size_t len = (size_t)(end - start);
      char *s = malloc(len + 1);
      if (!s) break;
      memcpy(s, start, len);
      s[len] = '\0';

      if (n == cap) {
        cap *= 2;
        char **tmp = realloc(out, cap * sizeof(char*));
        if (!tmp) { free(s); break; }
        out = tmp;
      }
      out[n++] = s;
    }

    if (*p) p++; // skip delimiter
  }

  *count_out = n;
  return out;
}

static void free_split(char **arr, size_t n) {
  if (!arr) return;
  for (size_t i = 0; i < n; i++) free(arr[i]);
  free(arr);
}

// ============================================================================
// ==== REPL (interactive loop) ===============================================
// ============================================================================

static void print_help(void) {
  printf("%s\n%s\n", LOCAL_COMMANDS, ARDUINO_COMMANDS);
}

static void drain_banner(int fd) {
  char **lines = NULL; size_t n = 0;
  (void)drain_window(fd, 0.8, &lines, &n);
  for (size_t i = 0; i < n; i++) free(lines[i]);
  free(lines);
}

static int get_exe_dir(char *out_dir, size_t out_sz, const char *argv0) {
  char tmp[PATH_MAX];
  if (!realpath(argv0, tmp)) {
    if (getcwd(out_dir, out_sz)) return 0;
    return -1;
  }

  char tmp2[PATH_MAX];
  strncpy(tmp2, tmp, sizeof(tmp2)-1);
  tmp2[sizeof(tmp2)-1] = '\0';

  char *d = dirname(tmp2);
  if (!d) return -1;

  strncpy(out_dir, d, out_sz-1);
  out_dir[out_sz-1] = '\0';
  return 0;
}

static void repl(int fd, const char *state_path) {
  drain_banner(fd);

  printf("\nInteractive serial interface\n");
  printf("Type 'help' for commands\n\n");

  char *line = NULL;
  size_t linecap = 0;

  while (1) {
    printf("cmd> ");
    fflush(stdout);

    ssize_t r = getline(&line, &linecap, stdin);
    if (r < 0) { printf("\n"); break; }

    // strip newline
    while (r > 0 && (line[r-1] == '\n' || line[r-1] == '\r')) line[--r] = '\0';
    if (r == 0) continue;

    // Lowercase copy for local command checks
    char low[128];
    snprintf(low, sizeof(low), "%s", line);
    for (size_t i = 0; low[i]; i++) low[i] = (char)tolower((unsigned char)low[i]);

    if (!strcmp(low, "quit") || !strcmp(low, "exit")) break;
    if (!strcmp(low, "help") || !strcmp(low, "?")) { print_help(); continue; }
    if (!strcmp(low, "save")) { cmd_save(fd, state_path); continue; }
    if (!strcmp(low, "sync")) { cmd_sync(fd, state_path); continue; }

    // Otherwise: raw passthrough, with chaining support.
    size_t ncmd = 0;
    char **cmds = split_multi_cmd(line, &ncmd);

    for (size_t i = 0; i < ncmd; i++) {
      printf("> %s\n", cmds[i]);
      (void)send_line(fd, cmds[i]);

      // Drain a bit to show Arduino responses.
      char **lines2 = NULL; size_t n2 = 0;
      (void)drain_window(fd, 0.6, &lines2, &n2);
      for (size_t j = 0; j < n2; j++) free(lines2[j]);
      free(lines2);
    }

    free_split(cmds, ncmd);
  }

  free(line);
}

// ============================================================================
// ==== MAIN / CLI ARGS ========================================================
// ============================================================================

static void usage(const char *prog) {
  fprintf(stderr,
    "Usage:\n"
    "  %s <port> [--baud N] [--save] [--sync]\n"
    "\n"
    "Examples:\n"
    "  %s /dev/ttyACM0\n"
    "  %s /dev/ttyACM0 --save\n"
    "  %s /dev/ttyACM0 --sync\n",
    prog, prog, prog, prog);
}

int main(int argc, char **argv) {
  if (argc < 2) {
    usage(argv[0]);
    return 2;
  }

  const char *port = argv[1];
  int baud = DEFAULT_BAUD;
  bool one_save = false;
  bool one_sync = false;

  // Parse optional flags
  for (int i = 2; i < argc; i++) {
    if (!strcmp(argv[i], "--baud") && i + 1 < argc) {
      baud = atoi(argv[++i]);
    } else if (!strcmp(argv[i], "--save")) {
      one_save = true;
    } else if (!strcmp(argv[i], "--sync")) {
      one_sync = true;
    } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
      usage(argv[0]);
      print_help();
      return 0;
    } else {
      fprintf(stderr, "Unknown arg: %s\n", argv[i]);
      usage(argv[0]);
      return 2;
    }
  }

  // Open serial port (includes reset-prevention steps described above)
  int fd = open_port(port, baud);
  if (fd < 0) {
    fprintf(stderr, "ERROR opening serial port %s: %s\n", port, strerror(errno));
    fprintf(stderr, "Tip (Linux): add user to dialout group, then log out/in:\n");
    fprintf(stderr, "  sudo usermod -aG dialout $USER\n");
    return 1;
  }

  // Determine where to write state.json:
  // We'll default to "same directory as the executable".
  char exe_dir[PATH_MAX];
  if (get_exe_dir(exe_dir, sizeof(exe_dir), argv[0]) != 0) {
    strncpy(exe_dir, ".", sizeof(exe_dir)-1);
    exe_dir[sizeof(exe_dir)-1] = '\0';
  }

  char state_path[] = STATE_FILE;

  // One-shot modes vs interactive REPL
  if (one_save) {
    cmd_save(fd, state_path);
  } else if (one_sync) {
    cmd_sync(fd, state_path);
  } else {
    repl(fd, state_path);
  }

  // Because we cleared HUPCL, Linux should NOT drop DTR automatically on close.
  close(fd);
  return 0;
}