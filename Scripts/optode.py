#!/usr/bin/env python3
"""Interactive raw serial console for an Aanderaa Optode on COM5.

Usage:
  py -3 Scripts/optode.py
  py -3 Scripts/optode.py --port COM5 --baud 9600

The menu sends common Optode commands and prints the raw device response. Use
"custom" for commands such as "Set Enable Text(Yes)" or "Get ConfigXML".
"""

import argparse
import sys
import time

import serial


DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 9600
SERIAL_TIMEOUT = 0.05
DEFAULT_RESPONSE_TIMEOUT = 2.5
SAMPLE_RESPONSE_TIMEOUT = 5.0
SAMPLE_MIN_WAIT = 2.5
QUIET_MS = 250

WAKE_PREAMBLE = ";;;;;;;;;;\r\n;;;;;;;;;;\r\n"

MENU_COMMANDS = [
    ("do sample", "Take one sample"),
    ("get all", "Read all readable properties"),
    ("get configxml", "Read configuration XML"),
    ("get dataxml", "Read enabled data XML"),
    ("get enable text", "Read Enable Text setting"),
    ("get enable temperature", "Read Enable Temperature setting"),
    ("get enable airsaturation", "Read Enable AirSaturation setting"),
    ("get enable rawdata", "Read Enable Rawdata setting"),
]


def decode_bytes(data):
    return data.decode("utf-8", errors="replace")


def read_until_quiet(ser, quiet_ms=QUIET_MS, timeout=DEFAULT_RESPONSE_TIMEOUT, min_wait=0.0):
    """Read serial bytes until no new data arrives for quiet_ms or timeout."""
    data = bytearray()
    started = time.time()
    deadline = time.time() + timeout
    quiet_deadline = time.time() + (quiet_ms / 1000.0)

    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            data.extend(chunk)
            quiet_deadline = time.time() + (quiet_ms / 1000.0)
        elif time.time() >= quiet_deadline and (time.time() - started) >= min_wait:
            break
        else:
            time.sleep(0.01)

    return bytes(data)


def drain_input(ser):
    data = read_until_quiet(ser, quiet_ms=100, timeout=0.4)
    if data:
        print_raw("Drained pending input", data)


def print_raw(title, data):
    print(f"\n--- {title} ({len(data)} bytes) ---")
    if data:
        print(decode_bytes(data), end="" if data.endswith((b"\n", b"\r")) else "\n")
    else:
        print("[no data]")
    print("--- end ---\n")


def send_text(ser, text, timeout, min_wait=0.0):
    ser.write(text.encode("ascii", errors="replace"))
    ser.flush()
    return read_until_quiet(ser, timeout=timeout, min_wait=min_wait)


def send_command(ser, command, timeout=DEFAULT_RESPONSE_TIMEOUT, wake=False, min_wait=0.0):
    drain_input(ser)

    if wake:
        print("[wake] Sending comment preamble...")
        send_text(ser, WAKE_PREAMBLE, timeout=0.25)
        time.sleep(0.15)
        drain_input(ser)

    print(f"[tx] {command!r}")
    return send_text(ser, command + "\r\n", timeout=timeout, min_wait=min_wait)


def prompt_number(max_value):
    choice = input(f"Select 1-{max_value}, c=custom, w=wake, r=read, q=quit: ").strip()
    return choice.lower()


def print_menu():
    print("\nOptode command menu")
    for idx, (cmd, desc) in enumerate(MENU_COMMANDS, start=1):
        print(f"  {idx}. {cmd:<26} {desc}")
    print("  c. custom command")
    print("  w. wake preamble only")
    print("  r. read/drain without sending")
    print("  q. quit")


def interactive(ser):
    print(f"Opened {ser.port} at {ser.baudrate} baud.")
    print("Responses are printed exactly as decoded from serial bytes.")

    while True:
        print_menu()
        choice = prompt_number(len(MENU_COMMANDS))

        if choice == "q":
            return
        if choice == "r":
            data = read_until_quiet(ser, timeout=DEFAULT_RESPONSE_TIMEOUT)
            print_raw("rx", data)
            continue
        if choice == "w":
            drain_input(ser)
            data = send_text(ser, WAKE_PREAMBLE, timeout=0.5)
            print_raw("wake response", data)
            continue
        if choice == "c":
            cmd = input("Command: ").strip()
            if not cmd:
                continue
            timeout_text = input(f"Timeout seconds [{DEFAULT_RESPONSE_TIMEOUT}]: ").strip()
            try:
                timeout = float(timeout_text) if timeout_text else DEFAULT_RESPONSE_TIMEOUT
            except ValueError:
                print("Invalid timeout.")
                continue
            wake = input("Send wake preamble first? [y/N]: ").strip().lower() == "y"
            data = send_command(ser, cmd, timeout=timeout, wake=wake)
            print_raw(cmd, data)
            continue

        try:
            idx = int(choice) - 1
        except ValueError:
            print("Invalid selection.")
            continue

        if idx < 0 or idx >= len(MENU_COMMANDS):
            print("Invalid selection.")
            continue

        cmd, _desc = MENU_COMMANDS[idx]
        timeout = SAMPLE_RESPONSE_TIMEOUT if cmd == "do sample" else DEFAULT_RESPONSE_TIMEOUT
        min_wait = SAMPLE_MIN_WAIT if cmd == "do sample" else 0.0
        data = send_command(ser, cmd, timeout=timeout, wake=True, min_wait=min_wait)
        print_raw(cmd, data)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Interactive Optode raw command tool")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port, default {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate, default {DEFAULT_BAUD}")
    parser.add_argument("--timeout", type=float, default=SERIAL_TIMEOUT, help="Serial read timeout")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])

    try:
        with serial.Serial(port=args.port, baudrate=args.baud, timeout=args.timeout) as ser:
            interactive(ser)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

