"""
prawler-wifi.py - Connect to STM-PRAWLER board over WiFi via ISM4343 module.

This PC has an ISM4343-WBM-L54 module on COM10. The prawler board runs as
AP (SSID: prawler) with a TCP server on port 5000 in passthrough mode (PX).
This script configures the local module as a station, connects to the AP,
enters passthrough mode, then provides an interactive shell console.

Usage: py -3 prawler-wifi.py [--port COM10]
"""

import serial
import threading
import time
import sys
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


def check_passthrough(ser):
    """Check if the module is already in passthrough mode.

    Sends a bare \\n -- in AT command mode this does nothing (AT commands
    need \\r), but in passthrough the prawler WiFi shell responds with '> '.
    """
    saved_timeout = ser.timeout
    ser.timeout = 1
    ser.reset_input_buffer()
    ser.write(b"\n")
    # Block-read up to 16 bytes with the 1s timeout
    data = ser.read(16)
    ser.timeout = saved_timeout
    if b"> " in data:
        print("[local] Module already in passthrough mode -- skipping setup.")
        return True
    return False


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
    """Connect to the prawler board's TCP server and enter passthrough."""
    print(f"[local] Connecting to {AP_IP}:{TCP_PORT}...")
    expect_ok(ser, "P0=0", "Set Socket 0")
    expect_ok(ser, "P1=0", "Set Protocol TCP")
    expect_ok(ser, f"P3={AP_IP}", "Set Remote Host IP")
    expect_ok(ser, f"P4={TCP_PORT}", "Set Remote Port")
    expect_ok(ser, "S1=1460", "Set Write Packet Size")
    expect_ok(ser, "S2=50", "Set Write Timeout")

    resp = send_cmd(ser, "P6=1", timeout=15)
    if "ERROR" in resp:
        raise RuntimeError(f"TCP connect failed: {resp}")
    print("[local] TCP connected to prawler board.")

    # Enter client passthrough mode -- after this, serial port IS the TCP link
    print("[local] Entering passthrough mode...")
    resp = send_cmd(ser, "PX=1,0", timeout=10)
    if "ERROR" in resp:
        raise RuntimeError(f"PX failed: {resp}")

    # Flush any residual AT response bytes
    time.sleep(0.1)
    ser.reset_input_buffer()

    print("[local] Passthrough active.")


def receiver_thread(ser, stop_event):
    """Background thread that reads raw bytes from the passthrough link."""
    while not stop_event.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)
            if data:
                text = data.decode("ascii", errors="replace")
                sys.stdout.write(text)
                sys.stdout.flush()
        except Exception:
            if not stop_event.is_set():
                time.sleep(0.1)


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
    print("  STM-PRAWLER WiFi Shell")
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
        if not check_passthrough(ser):
            setup_station(ser)
            time.sleep(1)
            setup_tcp_client(ser)

        # Start receiver thread for incoming data
        rx = threading.Thread(
            target=receiver_thread,
            args=(ser, stop_event),
            daemon=True,
        )
        rx.start()

        # Send \n to solicit initial prompt from prawler
        ser.write(b"\n")

        print()
        print("=" * 60)
        print("  Connected to prawler WiFi shell!")
        print()
        print("  Type a command and press Enter.")
        print("  Type 'quit' to exit.")
        print("=" * 60)
        print()

        while True:
            try:
                user_input = input("")
            except (EOFError, KeyboardInterrupt):
                break

            if user_input.strip().lower() == "quit":
                break

            ser.write((user_input + "\r").encode())

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
