"""
prawler-wifi.py - Connect to STM-PRAWLER board over WiFi via ISM4343 module.

This PC has an ISM4343-WBM-L54 module on COM10. The prawler board runs as
AP (SSID: prawler) with a TCP server on port 5000 in passthrough mode (PX).
This script configures the local module as a station, connects to the AP,
enters passthrough mode, then provides an interactive shell console.

The script automatically detects connection loss and reconnects.

Usage: py -3 prawler-wifi.py [--port COM10]
"""

import argparse
import sys
import threading
import time

import serial

# --- Configuration (must match wifi.h on the prawler board) ---
PORT = "COM10"
BAUD = 9600
TIMEOUT = 0.05

AP_SSID = "prawler"
AP_SECURITY = 0  # 0 = Open
AP_IP = "192.168.10.1"
TCP_PORT = 5000
CLIENT_IP = "192.168.10.2"
CLIENT_MASK = "255.255.255.0"

# Resilience settings
KEEPALIVE_MS = 30000  # TCP keep-alive idle timeout (ms)
RECONNECT_DELAY = 5  # Seconds to wait between reconnect attempts
MAX_RECONNECT_ATTEMPTS = 0  # 0 = unlimited
AT_PROMPT_QUIET_MS = 100  # Extra quiet time after AT prompt at 9600 baud
# --------------------------------------------------------------


def _read_until_quiet(ser, quiet_ms=AT_PROMPT_QUIET_MS, timeout=2.0):
    """Drain pending serial bytes until the stream is quiet."""
    buf = b""
    deadline = time.time() + timeout
    quiet_deadline = time.time() + (quiet_ms / 1000.0)
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            quiet_deadline = time.time() + (quiet_ms / 1000.0)
        elif time.time() >= quiet_deadline:
            break
        else:
            time.sleep(0.02)
    return buf.decode("ascii", errors="replace")


def _read_until_prompt(ser, timeout, quiet_ms=AT_PROMPT_QUIET_MS):
    """Read from serial until a prompt is seen and the stream goes quiet."""
    buf = b""
    deadline = time.time() + timeout
    quiet_deadline = None
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if b"> " in buf:
                quiet_deadline = time.time() + (quiet_ms / 1000.0)
        elif quiet_deadline is not None and time.time() >= quiet_deadline:
            break
        else:
            time.sleep(0.02)
    return buf.decode("ascii", errors="replace")


def send_cmd(ser, cmd, timeout=5.0):
    """Send an AT command and wait for the response."""
    _read_until_quiet(ser, quiet_ms=150, timeout=0.5)
    ser.reset_input_buffer()
    ser.write((cmd + "\r").encode())
    return _read_until_prompt(ser, timeout)


def expect_ok(ser, cmd, label="", timeout=5.0):
    """Send command and raise on ERROR response."""
    resp = send_cmd(ser, cmd, timeout=timeout)
    if "ERROR" in resp:
        raise RuntimeError(f"[{label or cmd}] failed: {resp}")
    return resp


def expect_ok_retry(ser, cmd, label="", timeout=5.0):
    """Send an AT command, retrying once after delayed async output settles."""
    resp = send_cmd(ser, cmd, timeout=timeout)
    if "ERROR" not in resp:
        return resp

    delayed = _read_until_quiet(ser, quiet_ms=750, timeout=3)
    resp = send_cmd(ser, cmd, timeout=timeout)
    if "ERROR" in resp:
        raise RuntimeError(f"[{label or cmd}] failed: {delayed}{resp}")
    return resp


def check_passthrough(ser):
    """Check if the module is already in passthrough mode.

    Sends a bare \\n -- in AT command mode this does nothing (AT commands
    need \\r), but in passthrough the prawler WiFi shell responds with '$ '.
    """
    saved_timeout = ser.timeout
    ser.timeout = 0.1
    ser.reset_input_buffer()
    ser.write(b"\n")
    data = b""
    deadline = time.time() + 0.5
    while time.time() < deadline:
        data += ser.read(ser.in_waiting or 1)
        if b"$ " in data or b"> " in data:
            break
    ser.timeout = saved_timeout
    if b"$ " in data or b"> " in data:
        print("[local] Module already in passthrough mode -- skipping setup.")
        return True
    return False


def is_in_at_mode(ser):
    """Check if the local module is in AT command mode (not streaming).

    Sends an empty \\r.  In AT mode the module responds with '> ' prompt.
    In streaming mode the bytes go to the TCP peer and we get nothing back
    or the prawler shell prompt, which is not treated as local AT mode.
    """
    saved_timeout = ser.timeout
    ser.timeout = 0.1
    ser.reset_input_buffer()
    ser.write(b"\r")
    data = b""
    deadline = time.time() + 0.5
    while time.time() < deadline:
        data += ser.read(ser.in_waiting or 1)
        if b"> " in data or b"OK" in data:
            break
    ser.timeout = saved_timeout
    # AT mode gives back \r\n\r\nOK\r\n> or similar with the prompt
    text = data.decode("ascii", errors="replace")
    return "OK" in text or "> " in text


def ensure_at_mode(ser):
    """Return the local module to AT command mode if it is not already there."""
    if not is_in_at_mode(ser):
        hard_reset(ser)


def hard_reset(ser):
    """Software-reset the local ISM4343 module and wait for boot."""
    print("[local] Resetting module...")
    send_cmd(ser, "ZR", timeout=5)
    time.sleep(3)
    ser.reset_input_buffer()
    send_cmd(ser, "", timeout=2)


def setup_station(ser):
    """Configure local module as station and join the prawler AP."""
    ensure_at_mode(ser)
    send_cmd(ser, "CD", timeout=3)

    print(f"[local] Joining AP '{AP_SSID}'...")
    expect_ok(ser, f"C1={AP_SSID}", "Set Network SSID")
    expect_ok(ser, f"C3={AP_SECURITY}", "Set Security Type")
    expect_ok(ser, "C4=0", "Disable DHCP")
    expect_ok(ser, f"C6={CLIENT_IP}", "Set Static IP")
    expect_ok(ser, f"C7={CLIENT_MASK}", "Set Netmask")
    expect_ok(ser, f"C8={AP_IP}", "Set Gateway")

    # The script manages connection state explicitly. Saved auto-join can race
    # the setup sequence after module reset and leave C0 reporting 0.0.0.0.
    expect_ok(ser, "CC=0", "Disable Auto-Join")

    # Disable power save to keep the radio active during idle periods.
    expect_ok(ser, "ZP=1,0", "Disable Power Save")

    resp = send_cmd(ser, "C0", timeout=20)
    resp += _read_until_quiet(ser, quiet_ms=500, timeout=4)
    if "ERROR" in resp and "Already connected" not in resp:
        raise RuntimeError(f"Failed to join AP: {resp}")
    if "0.0.0.0" in resp or CLIENT_IP not in resp:
        raise RuntimeError(f"Joined AP without expected IP {CLIENT_IP}: {resp}")
    print(f"[local] Joined AP.")

    resp = send_cmd(ser, "C?", timeout=5)
    resp += _read_until_quiet(ser, quiet_ms=250, timeout=1)
    print(f"[local] Network info:\n{resp}")


def setup_tcp_client(ser):
    """Connect to the prawler board's TCP server and enter passthrough."""
    print(f"[local] Connecting to {AP_IP}:{TCP_PORT}...")
    expect_ok_retry(ser, "P0=0", "Set Socket 0")
    expect_ok_retry(ser, "P1=0", "Set Protocol TCP")
    expect_ok_retry(ser, f"P3={AP_IP}", "Set Remote Host IP")
    expect_ok_retry(ser, f"P4={TCP_PORT}", "Set Remote Port")
    expect_ok_retry(ser, "S1=1460", "Set Write Packet Size")
    expect_ok_retry(ser, "S2=50", "Set Write Timeout")

    # Enable TCP keep-alive so the client detects a dead server
    expect_ok_retry(ser, f"PK=1,{KEEPALIVE_MS}", "Enable TCP Keep-Alive")

    # Enter client passthrough mode -- after this, serial port IS the TCP link.
    # Do not pre-start the API client with P6=1; the IWIN streaming example
    # starts client streaming directly with PX=1,0.
    print("[local] Entering passthrough mode...")
    resp = send_cmd(ser, "PX=1,0", timeout=15)
    if "ERROR" in resp:
        delayed = _read_until_quiet(ser, quiet_ms=750, timeout=3)
        resp = send_cmd(ser, "PX=1,0", timeout=15)
        if "ERROR" in resp:
            raise RuntimeError(f"PX failed: {delayed}{resp}")

    # Flush any residual AT response bytes
    time.sleep(0.1)
    ser.reset_input_buffer()

    print("[local] Passthrough active.")


def connect(ser):
    """Run the full connection sequence: station join + TCP client setup.

    Returns True on success, False on failure.
    """
    try:
        if not check_passthrough(ser):
            setup_station(ser)
            time.sleep(1)
            setup_tcp_client(ser)
        return True
    except Exception as e:
        print(f"[local] Connection failed: {e}")
        return False


def reconnect(ser):
    """Attempt to re-establish the connection after a drop.

    Resets the module and runs the full setup sequence again.
    Returns True on success, False if all attempts are exhausted.
    """
    attempt = 0
    while MAX_RECONNECT_ATTEMPTS == 0 or attempt < MAX_RECONNECT_ATTEMPTS:
        attempt += 1
        print(f"\n[local] Reconnect attempt {attempt}...")
        print(f"[local] Waiting {RECONNECT_DELAY}s before retry...")
        time.sleep(RECONNECT_DELAY)

        try:
            # Make sure we're back in AT command mode.  If the module
            # already dropped out of streaming on its own (keep-alive
            # failure or TCP close), we should be there.  If not, a
            # hard reset will get us there.
            if not is_in_at_mode(ser):
                hard_reset(ser)

            setup_station(ser)
            time.sleep(1)
            setup_tcp_client(ser)
            print("[local] Reconnected successfully!")
            return True
        except Exception as e:
            print(f"[local] Reconnect attempt {attempt} failed: {e}")

    print("[local] All reconnect attempts exhausted.")
    return False


def detect_disconnect(data):
    """Check if received data indicates the module dropped out of streaming.

    When the TCP connection dies and keep-alive detects it, the module
    exits passthrough and reverts to AT command mode.  We'll start seeing
    AT-mode output like 'ERROR', '[AP ...', 'OK', or the '> ' prompt
    instead of prawler shell data.
    """
    text = data.decode("ascii", errors="replace")
    indicators = ["ERROR", "[AP ", "\r\nOK\r\n"]
    return any(ind in text for ind in indicators)


def receiver_thread(ser, stop_event, disconnect_event):
    """Background thread that reads raw bytes from the passthrough link.

    Sets disconnect_event if the connection appears to have dropped.
    """
    idle_count = 0
    while not stop_event.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)
            if data:
                idle_count = 0
                if detect_disconnect(data):
                    print("\n[local] Connection lost (module exited streaming).")
                    disconnect_event.set()
                    return
                text = data.decode("ascii", errors="replace")
                sys.stdout.write(text)
                sys.stdout.flush()
            else:
                idle_count += 1
        except serial.SerialException:
            if not stop_event.is_set():
                print("\n[local] Serial error -- connection lost.")
                disconnect_event.set()
                return
        except Exception:
            if not stop_event.is_set():
                time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(
        description="Connect to STM-PRAWLER board over WiFi"
    )
    parser.add_argument(
        "--port",
        default=PORT,
        help=f"Serial port for local ISM4343 module (default: {PORT})",
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
        if not connect(ser):
            if not reconnect(ser):
                print("Could not establish initial connection.")
                sys.exit(1)

        while True:
            # Start receiver thread for incoming data
            disconnect_event = threading.Event()
            rx = threading.Thread(
                target=receiver_thread,
                args=(ser, stop_event, disconnect_event),
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

            # Interactive loop -- also watches for disconnect
            session_active = True
            while session_active:
                # Check if receiver thread flagged a disconnect
                if disconnect_event.is_set():
                    session_active = False
                    break

                try:
                    # Use a short timeout so we can check disconnect_event
                    # between input attempts.  On Windows, input() blocks,
                    # so we rely on the receiver thread to flag disconnect.
                    user_input = input("")
                except (EOFError, KeyboardInterrupt):
                    stop_event.set()
                    session_active = False
                    break

                if user_input.strip().lower() == "quit":
                    stop_event.set()
                    session_active = False
                    break

                if disconnect_event.is_set():
                    session_active = False
                    break

                try:
                    ser.write((user_input + "\r").encode())
                except serial.SerialException:
                    print("\n[local] Write failed -- connection lost.")
                    disconnect_event.set()
                    session_active = False
                    break

            # If we got here via disconnect (not quit), try to reconnect
            if stop_event.is_set():
                break

            # Wait for receiver thread to finish
            rx.join(timeout=2)

            print("\n[local] Session ended. Attempting reconnect...")
            if not reconnect(ser):
                print("Could not reconnect. Exiting.")
                break

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
