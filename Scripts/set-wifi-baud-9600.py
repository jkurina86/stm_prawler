"""
set-wifi-baud-9600.py - Change an Inventek eS-WiFi module from 115200 to 9600.

The module is expected to be in IWIN AT command mode on COM5 at 115200 baud.
The script:
  1. Opens the module at 115200.
  2. Sends U2=9600 and U0 to activate the new UART baud rate.
  3. Reopens the same port at 9600 and verifies command communication.
  4. Sends Z1 only after 9600 baud communication succeeds.

Usage:
  py -3 Scripts/set-wifi-baud-9600.py
  py -3 Scripts/set-wifi-baud-9600.py --port COM5
"""

import argparse
import sys
import time

import serial


DEFAULT_PORT = "COM5"
OLD_BAUD = 115200
NEW_BAUD = 9600
TIMEOUT = 0.05


def read_until_quiet(ser, quiet_ms=150, timeout=2.0):
    """Read serial data until the stream has been quiet for quiet_ms."""
    data = b""
    deadline = time.time() + timeout
    quiet_deadline = time.time() + (quiet_ms / 1000.0)

    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            data += chunk
            quiet_deadline = time.time() + (quiet_ms / 1000.0)
        elif time.time() >= quiet_deadline:
            break
        else:
            time.sleep(0.02)

    return data.decode("ascii", errors="replace")


def read_until_prompt(ser, timeout=3.0, quiet_ms=150):
    """Read until the IWIN prompt is observed and the stream settles."""
    data = b""
    deadline = time.time() + timeout
    quiet_deadline = None

    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            data += chunk
            if b"> " in data:
                quiet_deadline = time.time() + (quiet_ms / 1000.0)
        elif quiet_deadline is not None and time.time() >= quiet_deadline:
            break
        else:
            time.sleep(0.02)

    return data.decode("ascii", errors="replace")


def send_cmd(ser, cmd, timeout=3.0):
    """Send one CR-terminated IWIN command and return the raw response text."""
    read_until_quiet(ser, quiet_ms=150, timeout=0.5)
    ser.reset_input_buffer()
    ser.write((cmd + "\r").encode("ascii"))
    ser.flush()
    return read_until_prompt(ser, timeout=timeout)


def expect_ok(ser, cmd, timeout=3.0):
    """Send command and fail unless the response contains OK and no ERROR."""
    response = send_cmd(ser, cmd, timeout=timeout)
    if "ERROR" in response or "OK" not in response:
        raise RuntimeError(f"{cmd!r} failed or timed out:\n{response}")
    return response


def open_port(port, baud):
    return serial.Serial(port=port, baudrate=baud, timeout=TIMEOUT)


def verify_uart_settings(ser, baud):
    response = expect_ok(ser, "U?", timeout=3.0)
    if str(baud) not in response:
        raise RuntimeError(f"U? did not report {baud} baud:\n{response}")
    print(f"[{baud}] Module responded and reports {baud} baud.")
    return response


def change_baud(port):
    print(f"Opening {port} at {OLD_BAUD}...")
    with open_port(port, OLD_BAUD) as ser:
        verify_uart_settings(ser, OLD_BAUD)

        print(f"[{OLD_BAUD}] Setting baud rate to {NEW_BAUD} with U2...")
        expect_ok(ser, f"U2={NEW_BAUD}", timeout=3.0)

        print(f"[{OLD_BAUD}] Activating UART settings with U0...")
        ser.write(b"U0\r")
        ser.flush()

        # U0 changes the module baud immediately, so the response may not be
        # readable at the old baud. Give the module a moment before reopening.
        time.sleep(0.75)

    print(f"Reopening {port} at {NEW_BAUD}...")
    with open_port(port, NEW_BAUD) as ser:
        response = verify_uart_settings(ser, NEW_BAUD)
        print(response.strip())

        print(f"[{NEW_BAUD}] Saving current settings to NVRAM with Z1...")
        expect_ok(ser, "Z1", timeout=5.0)

    print(f"Done. {port} is configured for {NEW_BAUD} baud and saved with Z1.")


def main():
    parser = argparse.ArgumentParser(
        description="Set an Inventek eS-WiFi module UART baud rate to 9600."
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port for the eS-WiFi module (default: {DEFAULT_PORT})",
    )
    args = parser.parse_args()

    try:
        change_baud(args.port)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
