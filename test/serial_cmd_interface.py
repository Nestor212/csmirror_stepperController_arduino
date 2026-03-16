#!/usr/bin/env python3
import argparse
import json
import os
import re
import sys
import time
import termios

import serial

DEFAULT_BAUD = 115200

# -------------------------------------------------------
# Help text (shown with -h and in REPL help)
# -------------------------------------------------------

ARDUINO_COMMANDS = r"""
Arduino Commands

Global:
  status
  enable_limits
  disable_limits
  enable_all     (alias: enableall)
  disable_all    (alias: disableall)
  stop_all       (alias: stopall)

Axis commands:
  enable <axis>
  disable <axis>
  stop <axis>
  home <axis>
  zero <axis>

  setaxis <axis> <pos> <max> <valid>

  move <axis> <dir> <steps> [speed] [accel]
      dir: 0 = negative
           1 = positive

  moveto <axis> <pos> [speed] [accel]

Chaining commands in this script:
  Use ':', ';', or '|' to send multiple commands back-to-back.

Example:
  move a 0 10000 100 100 : move b 1 10000 100 100
"""

LOCAL_COMMANDS = r"""
Local Commands (cmd> prompt)

  help
  save     (send 'status', parse it, update ./state.json)
  sync     (only restore setaxis from ./state.json if boot_id changed)
  quit / exit
"""

HELP_TEXT = LOCAL_COMMANDS + "\n" + ARDUINO_COMMANDS


# -------------------------------------------------------
# STATUS parsing (matches your Arduino printStatus)
# -------------------------------------------------------

BOOT_RE = re.compile(r"\bboot_id=(\d+)\b")
SEQ_RE = re.compile(r"\bseq=(\d+)\b")
LIMITS_RE = re.compile(r"\blimitsEnabled=(\d+)\b")

AXIS_ID_RE = re.compile(r"\bid=([ab])\b")
POS_RE = re.compile(r"\bpos=(-?\d+)\b")
TGT_RE = re.compile(r"\btgt=(-?\d+)\b")
MAX_RE = re.compile(r"\bmax=(-?\d+)\b")
VALID_RE = re.compile(r"\bvalid=(\d+)\b")
HOMED_RE = re.compile(r"\bhomed=(\d+)\b")
EN_RE = re.compile(r"\ben=(\d+)\b")


def parse_status_lines(lines):
    """
    Returns a dict suitable for state.json:
      {
        ts, boot_id, seq, limitsEnabled,
        axes: { a:{pos,tgt,max,valid,homed,en}, b:{...} }
      }
    """
    rec = {
        "ts": time.time(),
        "boot_id": None,
        "seq": None,
        "limitsEnabled": None,
        "axes": {}
    }

    for l in lines:
        m = BOOT_RE.search(l)
        if m:
            rec["boot_id"] = int(m.group(1))

        m = SEQ_RE.search(l)
        if m:
            rec["seq"] = int(m.group(1))

        m = LIMITS_RE.search(l)
        if m:
            rec["limitsEnabled"] = int(m.group(1))

        m = AXIS_ID_RE.search(l)
        if not m:
            continue

        axis = m.group(1)
        mp = POS_RE.search(l)
        mt = TGT_RE.search(l)
        mm = MAX_RE.search(l)
        mv = VALID_RE.search(l)
        mh = HOMED_RE.search(l)
        men = EN_RE.search(l)

        # Require at least pos/max/valid to accept axis record
        if not (mp and mm and mv):
            continue

        rec["axes"][axis] = {
            "pos": int(mp.group(1)),
            "tgt": int(mt.group(1)) if mt else int(mp.group(1)),
            "max": int(mm.group(1)),
            "valid": int(mv.group(1)),
            "homed": int(mh.group(1)) if mh else 0,
            "en": int(men.group(1)) if men else 0,
        }

    return rec


# -------------------------------------------------------
# Serial helpers
# -------------------------------------------------------

def disable_hupcl(path: str):
    """
    Disable the Linux HUPCL (Hang Up On Close) flag for a serial device.

    Why this exists
    ----------------
    Arduino Uno boards automatically reset whenever the DTR signal on the
    USB serial interface transitions from HIGH -> LOW.

    On Linux, when the last process closes a serial port the kernel normally
    performs a "hangup" operation which drops the DTR line.  This behavior is
    controlled by the HUPCL termios flag.

    For Arduino devices this causes an unwanted reset every time a program
    connects to or disconnects from the port.

    Clearing HUPCL prevents the kernel from toggling DTR on close, which
    allows software to reconnect to the device without resetting it.

    This is especially important for control systems where reconnecting to
    the device must not interrupt the running firmware.

    Side effects
    ------------
    • The Arduino will no longer automatically reset when the port closes.
    • Tools that rely on the reset pulse (e.g. Arduino IDE uploads) may need
      HUPCL temporarily re-enabled.
    • If a program crashes, the Arduino will continue running instead of
      resetting.

    References
    ----------
    https://stackoverflow.com/questions/2810939
    https://www.arduino.cc/en/Main/ArduinoBoardUno
    """
    
    with open(path, "rb", buffering=0) as f:
        attrs = termios.tcgetattr(f.fileno())
        attrs[2] = attrs[2] & ~termios.HUPCL
        termios.tcsetattr(f.fileno(), termios.TCSAFLUSH, attrs)

def open_port(port, baud):
    disable_hupcl(port)

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.25,
        write_timeout=0.25,
        rtscts=False,
        dsrdtr=False,
    )

    # Keep DTR stable/high (avoid falling-edge resets later)
    try:
        ser.setDTR(True)
        ser.setRTS(False)
    except Exception:
        pass

    time.sleep(0.05)
    ser.reset_input_buffer()
    return ser


def send_line(ser, cmd):
    if not cmd.endswith("\n"):
        cmd += "\n"
    ser.write(cmd.encode("utf-8", errors="replace"))
    ser.flush()


def drain(ser, seconds=0.5):
    """
    Print incoming lines for a short time window.
    Returns list of decoded lines.
    """
    lines = []
    end = time.time() + seconds
    while time.time() < end:
        line = ser.readline()
        if not line:
            continue
        txt = line.decode("utf-8", errors="replace").rstrip("\r\n")
        if txt:
            print("<", txt)
            lines.append(txt)
    return lines


def get_status(ser, read_s=0.7):
    send_line(ser, "status")
    lines = drain(ser, read_s)
    rec = parse_status_lines(lines)

    # Basic sanity: must have boot_id/seq and at least one axis parsed
    if rec["boot_id"] is None or rec["seq"] is None or not rec["axes"]:
        raise RuntimeError(
            "Failed to parse status output (boot_id/seq/axes missing). "
            "Try increasing read window or check firmware output formatting."
        )
    return rec


# -------------------------------------------------------
# JSON helpers
# -------------------------------------------------------

def load_json(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return None


def save_json(path, obj):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, sort_keys=True)
    print(f"Saved state to {path}")


def coerce_int(v, default=0):
    try:
        if v is None:
            return default
        if isinstance(v, bool):
            return int(v)
        if isinstance(v, (int, float)):
            return int(v)
        return int(float(str(v).strip()))
    except Exception:
        return default


# -------------------------------------------------------
# Multi-command support
# -------------------------------------------------------

def split_multi_cmd(line):
    # separators: :, ;, |
    return [c.strip() for c in re.split(r"[:;|]", line) if c.strip()]


# -------------------------------------------------------
# Local commands: save / sync
# -------------------------------------------------------

def cmd_save(ser, state_path):
    st = get_status(ser)
    save_json(state_path, st)


def cmd_sync(ser, state_path):
    saved = load_json(state_path)
    if not saved:
        print(f"ERROR: no saved state found at {state_path}. Run 'save' first.")
        return

    # Always read current status to get current boot_id
    try:
        cur = get_status(ser)
    except Exception as e:
        print(f"ERROR: could not read/parse current status: {e}")
        return

    saved_boot = saved.get("boot_id", None)
    cur_boot = cur.get("boot_id", None)

    if saved_boot is None or cur_boot is None:
        print("ERROR: missing boot_id in saved or current status; cannot decide sync policy safely.")
        return

    if int(saved_boot) == int(cur_boot):
        print(f"SYNC: boot_id matches ({cur_boot}). Skipping setaxis restore.")
        return

    print(f"SYNC: boot_id changed (saved {saved_boot} -> now {cur_boot}). Restoring axes via setaxis...")

    axes = saved.get("axes", {})
    cmds = []
    for axis in ("a", "b"):
        if axis not in axes:
            print(f"WARNING: axis '{axis}' missing in state.json; skipping")
            continue

        ax = axes[axis]
        pos = coerce_int(ax.get("pos"), 0)
        mx = coerce_int(ax.get("max"), 0)
        valid = coerce_int(ax.get("valid"), 0)
        if valid not in (0, 1):
            valid = 0

        cmds.append(f"setaxis {axis} {pos} {mx}")

    if not cmds:
        print("ERROR: no setaxis commands generated from saved state.")
        return

    for c in cmds:
        print(">", c)
        send_line(ser, c)
        drain(ser, 0.35)

    # Save fresh status after restore
    try:
        st2 = get_status(ser)
        save_json(state_path, st2)
    except Exception as e:
        print(f"WARNING: restore done, but failed to re-read/save status: {e}")


# -------------------------------------------------------
# Interactive shell
# -------------------------------------------------------

def repl(ser, state_path):
    drain(ser, 0.8)

    print("\nInteractive serial interface")
    print("Type 'help' for commands\n")

    while True:
        try:
            line = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        low = line.lower()

        if low in ("quit", "exit"):
            break

        if low in ("help", "?"):
            print(HELP_TEXT)
            continue

        if low == "save":
            cmd_save(ser, state_path)
            continue

        if low == "sync":
            cmd_sync(ser, state_path)
            continue

        # raw passthrough, with chaining support
        cmds = split_multi_cmd(line)
        for c in cmds:
            print(">", c)
            send_line(ser, c)
            drain(ser, 0.6)


# -------------------------------------------------------
# main
# -------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Minimal serial command interface (raw passthrough + save/sync state.json).",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=HELP_TEXT
    )

    parser.add_argument("port", help="Serial port (e.g. /dev/ttyACM0 or COM5)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate (default: 115200)")
    parser.add_argument("--sync", action="store_true", help="Run sync once and exit")
    parser.add_argument("--save", action="store_true", help="Run save once and exit (query status -> write state.json)")

    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    state_path = os.path.join(script_dir, "state.json")

    try:
        ser = open_port(args.port, args.baud)
    except Exception as e:
        print(f"ERROR opening serial port {args.port}: {e}", file=sys.stderr)
        print("Tip (Linux): add user to dialout group, then log out/in:", file=sys.stderr)
        print("  sudo usermod -aG dialout $USER", file=sys.stderr)
        return 1

    try:
        if args.save:
            cmd_save(ser, state_path)
            return 0
        if args.sync:
            cmd_sync(ser, state_path)
            return 0
        repl(ser, state_path)
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())