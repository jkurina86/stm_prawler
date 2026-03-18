"""
prawler-wifi.py - Connect to STM-PRAWLER board over WiFi via ISM4343 module.

This PC has an ISM4343-WBM-L54 module on COM10. The prawler board runs as
AP (SSID: ISM4343_LINK) with a TCP server on port 5000. This script configures
the local module as a station, connects to the AP, establishes a TCP link,
then provides an interactive send/receive console.

Usage: py -3 prawler-wifi.py [--port COM10]
"""

import serial
import threading
import time
import sys
import re
import argparse

# --- Configuration (must match wifi.h on the prawler board) ---
PORT = "COM10"
BAUD = 115200
TIMEOUT = 2

AP_SSID = "prawler"
AP_SECURITY = 0         # 0 = Open
AP_IP = "192.168.10.1"
TCP_PORT = 5000
# --------------------------------------------------------------

lock = threading.Lock()


def _read_until_prompt(ser, timeout):
    """Read from serial until '> ' prompt or timeout."""
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if b"> " in buf:
                break
        else:
            time.sleep(0.02)
    return buf.decode("ascii", errors="replace")


def send_cmd(ser, cmd, timeout=5.0):
    """Send an AT command and wait for the response."""
    ser.reset_input_buffer()
    ser.write((cmd + "\r").encode())
    return _read_until_prompt(ser, timeout)


def expect_ok(ser, cmd, label="", timeout=5.0):
    """Send command and raise on ERROR response."""
    resp = send_cmd(ser, cmd, timeout=timeout)
    if "ERROR" in resp:
        raise RuntimeError(f"[{label or cmd}] failed: {resp}")
    return resp


def wait_for(ser, keyword, timeout=30.0):
    """Read until keyword appears or timeout."""
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            text = buf.decode("ascii", errors="replace")
            if keyword in text:
                return text
        else:
            time.sleep(0.05)
    text = buf.decode("ascii", errors="replace")
    raise TimeoutError(f"Timed out waiting for '{keyword}'. Got: {text}")


def hard_reset(ser):
    """Software-reset the local ISM4343 module and wait for boot."""
    print("[local] Resetting module...")
    send_cmd(ser, "ZR", timeout=5)
    time.sleep(3)
    ser.reset_input_buffer()
    send_cmd(ser, "", timeout=2)


def setup_station(ser):
    """Configure local module as station and join the prawler AP."""
    hard_reset(ser)

    print(f"[local] Joining AP '{AP_SSID}'...")
    expect_ok(ser, f"C1={AP_SSID}", "Set Network SSID")
    expect_ok(ser, f"C3={AP_SECURITY}", "Set Security Type")
    expect_ok(ser, "C4=1", "Enable DHCP")

    resp = send_cmd(ser, "C0", timeout=20)
    if "ERROR" in resp:
        raise RuntimeError(f"Failed to join AP: {resp}")
    print(f"[local] Joined AP.")

    resp = send_cmd(ser, "C?", timeout=5)
    print(f"[local] Network info:\n{resp}")


def setup_tcp_client(ser):
    """Connect to the prawler board's TCP server."""
    print(f"[local] Connecting to {AP_IP}:{TCP_PORT}...")
    expect_ok(ser, "P0=0", "Set Socket 0")
    expect_ok(ser, "P1=0", "Set Protocol TCP")
    expect_ok(ser, f"P3={AP_IP}", "Set Remote Host IP")
    expect_ok(ser, f"P4={TCP_PORT}", "Set Remote Port")
    expect_ok(ser, "R1=1460", "Set Read Packet Size")
    expect_ok(ser, "R2=1000", "Set Read Timeout")

    resp = send_cmd(ser, "P6=1", timeout=15)
    if "ERROR" in resp:
        raise RuntimeError(f"TCP connect failed: {resp}")
    print("[local] TCP connected to prawler board.")


def _extract_data(raw):
    """Extract message payload from R0 response."""
    m = re.search(r"\r\n(.*?)\r\nOK\r\n", raw, re.DOTALL)
    if not m:
        return None
    data = m.group(1).strip()
    if not data or data == "-1":
        return None
    return data


def send_message(ser, message):
    """Send a message to the prawler board via S3."""
    data = message.encode("ascii")
    length = len(data)
    with lock:
        ser.reset_input_buffer()
        ser.write(f"S3={length}\r".encode())
        time.sleep(0.05)
        ser.write(data)
        _read_until_prompt(ser, timeout=5.0)


def read_message(ser):
    """Poll for an incoming message via R0. Returns data or None."""
    with lock:
        ser.reset_input_buffer()
        ser.write(b"R0\r")
        raw = _read_until_prompt(ser, timeout=3.0)
    return _extract_data(raw)


def receiver_thread(ser, stop_event):
    """Background thread that polls for incoming messages from the prawler."""
    while not stop_event.is_set():
        try:
            msg = read_message(ser)
            if msg:
                print(f"\n  [prawler -> local]: {msg}")
                print(">> ", end="", flush=True)
        except Exception:
            pass
        stop_event.wait(0.5)


def main():
    parser = argparse.ArgumentParser(
        description="Connect to STM-PRAWLER board over WiFi"
    )
    parser.add_argument(
        "--port", default=PORT,
        help=f"Serial port for local ISM4343 module (default: {PORT})"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("  STM-PRAWLER WiFi Client")
    print(f"  Local module: {args.port}")
    print(f"  Target AP:    {AP_SSID} ({AP_IP}:{TCP_PORT})")
    print("=" * 60)
    print()

    try:
        ser = serial.Serial(args.port, BAUD, timeout=TIMEOUT)
        print(f"Opened {args.port}")
    except serial.SerialException as e:
        print(f"Failed to open {args.port}: {e}")
        sys.exit(1)

    stop_event = threading.Event()

    try:
        setup_station(ser)
        time.sleep(1)
        setup_tcp_client(ser)

        print()
        print("=" * 60)
        print("  Connected to prawler board!")
        print()
        print("  Type a message and press Enter to send.")
        print("  Incoming messages are displayed automatically.")
        print("  Type 'quit' to exit.")
        print("=" * 60)
        print()

        rx = threading.Thread(
            target=receiver_thread,
            args=(ser, stop_event),
            daemon=True,
        )
        rx.start()

        while True:
            try:
                user_input = input(">> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not user_input:
                continue
            if user_input.lower() == "quit":
                break

            send_message(ser, user_input)
            print(f"  [local -> prawler]: {user_input}")

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nCleaning up...")
        stop_event.set()
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
