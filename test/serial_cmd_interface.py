#!/usr/bin/env python3

import argparse
import json
import os
import sys
import time
import re

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

Example:
  move a 0 10000 100 100 : move b 1 10000 100 100
"""

LOCAL_COMMANDS = r"""
Local Commands (cmd> prompt)

  help
  sync
  quit / exit

Multiple commands can be chained using:
  :
  ;
  |

Example:
  move a 0 10000 100 100 : move b 1 10000 100 100
"""

HELP_TEXT = LOCAL_COMMANDS + "\n" + ARDUINO_COMMANDS


# -------------------------------------------------------
# Serial helpers
# -------------------------------------------------------

def open_port(port, baud):
    ser = serial.Serial(port, baud, timeout=0.25)
    time.sleep(1.0)
    return ser


def send_line(ser, cmd):
    if not cmd.endswith("\n"):
        cmd += "\n"
    ser.write(cmd.encode("utf-8", errors="replace"))
    ser.flush()


def drain(ser, seconds=0.5):
    end = time.time() + seconds
    while time.time() < end:
        line = ser.readline()
        if line:
            print("<", line.decode(errors="ignore").strip())


# -------------------------------------------------------
# state.json sync
# -------------------------------------------------------

def load_state(path):
    with open(path, "r") as f:
        return json.load(f)


def sync_axes(ser, state_path):

    if not os.path.exists(state_path):
        print("ERROR: state.json not found:", state_path)
        return

    try:
        state = load_state(state_path)
    except Exception as e:
        print("ERROR reading state.json:", e)
        return

    axes = state.get("axes", {})

    for axis in ("a", "b"):

        if axis not in axes:
            print(f"WARNING: axis '{axis}' not found in state.json")
            continue

        ax = axes[axis]

        pos = ax.get("pos", 0)
        mx = ax.get("max", 0)
        valid = ax.get("valid", 0)

        cmd = f"setaxis {axis} {pos} {mx} {valid}"

        print(">", cmd)

        send_line(ser, cmd)
        drain(ser, 0.3)


# -------------------------------------------------------
# Multi-command support
# -------------------------------------------------------

def split_multi_cmd(cmd):
    return [c.strip() for c in re.split(r'[:;|]', cmd) if c.strip()]


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

        if low == "help":
            print(HELP_TEXT)
            continue

        if low == "sync":
            sync_axes(ser, state_path)
            continue

        cmds = split_multi_cmd(line)

        for c in cmds:
            print(">", c)
            send_line(ser, c)
            drain(ser, 0.35)


# -------------------------------------------------------
# main
# -------------------------------------------------------

def main():

    parser = argparse.ArgumentParser(
        description="Minimal serial command interface for Arduino motion controller",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=HELP_TEXT
    )

    parser.add_argument(
        "port",
        help="Serial port (example: /dev/ttyACM0 or COM5)"
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help="Serial baud rate (default 115200)"
    )

    parser.add_argument(
        "--sync",
        action="store_true",
        help="Send setaxis commands from state.json and exit"
    )

    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    state_path = os.path.join(script_dir, "state.json")

    try:
        ser = open_port(args.port, args.baud)
    except Exception as e:
        print(f"ERROR opening serial port {args.port}: {e}")
        sys.exit(1)

    try:

        if args.sync:
            sync_axes(ser, state_path)
            return

        repl(ser, state_path)

    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()