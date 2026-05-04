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

AP_SSID = "prawler2"
AP_SECURITY = 0  # 0 = Open
AP_IP = "192.168.10.1"
TCP_PORT = 5000
CLIENT_IP = "192.168.10.2"
CLIENT_MASK = "255.255.255.0"

# Resilience settings
KEEPALIVE_MS = 30000  # TCP keep-alive idle timeout (ms)
RECONNECT_DELAY = 0.25  # Small throttle after failed reconnect attempts
JOIN_RETRY_DELAY = 1.0  # Let the module settle before another AP join
MAX_RECONNECT_ATTEMPTS = 0  # 0 = unlimited
AT_PROMPT_QUIET_MS = 50  # Extra quiet time after AT prompt at 9600 baud
CMD_PREDRAIN_QUIET_MS = 25
CMD_PREDRAIN_TIMEOUT = 0.2
RETRY_SETTLE_QUIET_MS = 150
RETRY_SETTLE_TIMEOUT = 0.5
TCP_CMD_TIMEOUT = 0.35
PASSTHROUGH_CMD_TIMEOUT = 0.2
JOIN_CMD_TIMEOUT = 12.0
JOIN_DRAIN_QUIET_MS = 350
JOIN_DRAIN_TIMEOUT = 2.0
JOIN_STATUS_DELAY = 0.75
JOIN_STATUS_TIMEOUT = 4.0
JOIN_STATUS_CMD_TIMEOUT = 1.5
POST_JOIN_DELAY = 0.0
POST_PASSTHROUGH_FLUSH = 0.0
SHELL_PROMPT_READ_WINDOW = 0.1
AT_MODE_PROBE_TIMEOUT = 0.25
RESET_BOOT_TIMEOUT = 3.0
# --------------------------------------------------------------

g_initial_shell_output = ""


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
            time.sleep(0.01)
    return buf.decode("ascii", errors="replace")


def _read_until_prompt(ser, timeout, quiet_ms=AT_PROMPT_QUIET_MS):
    """Read from serial until an AT response terminator goes quiet."""
    buf = b""
    deadline = time.time() + timeout
    quiet_deadline = None
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if b"> " in buf or b"OK" in buf or b"ERROR" in buf:
                quiet_deadline = time.time() + (quiet_ms / 1000.0)
        elif quiet_deadline is not None and time.time() >= quiet_deadline:
            break
        else:
            time.sleep(0.01)
    return buf.decode("ascii", errors="replace")


def send_cmd(ser, cmd, timeout=2.0):
    """Send an AT command and wait for the response."""
    _read_until_quiet(
        ser, quiet_ms=CMD_PREDRAIN_QUIET_MS, timeout=CMD_PREDRAIN_TIMEOUT
    )
    ser.reset_input_buffer()
    ser.write((cmd + "\r").encode())
    return _read_until_prompt(ser, timeout)


def expect_ok(ser, cmd, label="", timeout=2.0):
    """Send command and raise on ERROR response."""
    resp = send_cmd(ser, cmd, timeout=timeout)
    if "ERROR" in resp:
        raise RuntimeError(f"[{label or cmd}] failed: {resp}")
    return resp


def expect_ok_retry(ser, cmd, label="", timeout=2.0):
    """Send an AT command, retrying once after delayed async output settles."""
    resp = send_cmd(ser, cmd, timeout=timeout)
    if "ERROR" not in resp:
        return resp

    delayed = _read_until_quiet(
        ser, quiet_ms=RETRY_SETTLE_QUIET_MS, timeout=RETRY_SETTLE_TIMEOUT
    )
    resp = send_cmd(ser, cmd, timeout=timeout)
    if "ERROR" in resp:
        raise RuntimeError(f"[{label or cmd}] failed: {delayed}{resp}")
    return resp


def compact_response(resp, limit=180):
    """Format AT output for concise retry status lines."""
    text = " ".join(resp.split())
    if not text:
        return "<no response>"
    if len(text) > limit:
        return text[: limit - 3] + "..."
    return text


def response_has_expected_ip(resp):
    """Return True when a module response shows the configured client IP."""
    return CLIENT_IP in resp


def wait_for_join_status(ser, first_resp):
    """Wait briefly for a join attempt to finish before retrying C0."""
    combined = first_resp
    deadline = time.time() + JOIN_STATUS_TIMEOUT

    while time.time() < deadline:
        if response_has_expected_ip(combined):
            return True, combined
        if "ERROR" in combined and "[JOIN" in combined:
            return False, combined

        remaining = deadline - time.time()
        if remaining <= 0:
            break

        time.sleep(min(JOIN_STATUS_DELAY, remaining))
        drained = _read_until_quiet(ser, quiet_ms=200, timeout=0.5)
        if drained:
            combined += drained
            if response_has_expected_ip(combined):
                return True, combined
            if "ERROR" in combined and "[JOIN" in combined:
                return False, combined

        resp = send_cmd(
            ser,
            "C?",
            timeout=min(JOIN_STATUS_CMD_TIMEOUT, max(0.2, deadline - time.time())),
        )
        resp += _read_until_quiet(ser, quiet_ms=200, timeout=0.5)
        if resp:
            combined += resp

    return response_has_expected_ip(combined), combined


def solicit_shell_prompt(ser):
    """Send CRLF until the prawler shell prompt appears."""
    data = b""
    while True:
        ser.write(b"\r\n")
        ser.flush()

        deadline = time.time() + SHELL_PROMPT_READ_WINDOW
        while time.time() < deadline:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                data += chunk
                if b"$ " in data:
                    return data.decode("ascii", errors="replace")
            else:
                time.sleep(0.01)


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
    deadline = time.time() + AT_MODE_PROBE_TIMEOUT
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
    deadline = time.time() + AT_MODE_PROBE_TIMEOUT
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


def wait_for_at_prompt(ser, timeout=RESET_BOOT_TIMEOUT):
    """Poll until the module accepts AT commands after reset."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        ser.reset_input_buffer()
        ser.write(b"\r")
        resp = _read_until_prompt(ser, timeout=0.4)
        if "OK" in resp or "> " in resp:
            return True
        time.sleep(0.05)
    return False


def hard_reset(ser):
    """Software-reset the local ISM4343 module and wait for boot."""
    print("[local] Resetting module...")
    send_cmd(ser, "ZR", timeout=2)
    ser.reset_input_buffer()
    if not wait_for_at_prompt(ser):
        raise RuntimeError("Module did not return to AT prompt after reset")


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

    attempt = 0
    while True:
        attempt += 1
        resp = send_cmd(ser, "C0", timeout=JOIN_CMD_TIMEOUT)
        resp += _read_until_quiet(
            ser, quiet_ms=JOIN_DRAIN_QUIET_MS, timeout=JOIN_DRAIN_TIMEOUT
        )

        if response_has_expected_ip(resp):
            print("[local] Joined AP.")
            break

        joined, status = wait_for_join_status(ser, resp)
        if joined:
            print("[local] Joined AP.")
            break

        if "ERROR" in status and "Already connected" not in status:
            print(
                f"[local] Join attempt {attempt} failed: "
                f"{compact_response(status)}"
            )
        else:
            print(
                f"[local] Join attempt {attempt} missing expected IP "
                f"{CLIENT_IP}: {compact_response(status)}"
            )

        time.sleep(JOIN_RETRY_DELAY)

    resp = send_cmd(ser, "C?", timeout=TCP_CMD_TIMEOUT)
    resp += _read_until_quiet(ser, quiet_ms=100, timeout=0.2)
    if resp.strip():
        print(f"[local] Network info:\n{resp}")


def setup_tcp_client(ser):
    """Connect to the prawler board's TCP server and enter passthrough."""
    global g_initial_shell_output

    print(f"[local] Connecting to {AP_IP}:{TCP_PORT}...")
    expect_ok_retry(ser, "P0=0", "Set Socket 0", timeout=TCP_CMD_TIMEOUT)
    expect_ok_retry(ser, "P1=0", "Set Protocol TCP", timeout=TCP_CMD_TIMEOUT)
    expect_ok_retry(ser, f"P3={AP_IP}", "Set Remote Host IP", timeout=TCP_CMD_TIMEOUT)
    expect_ok_retry(ser, f"P4={TCP_PORT}", "Set Remote Port", timeout=TCP_CMD_TIMEOUT)
    expect_ok_retry(ser, "S1=1460", "Set Write Packet Size", timeout=TCP_CMD_TIMEOUT)
    expect_ok_retry(ser, "S2=50", "Set Write Timeout", timeout=TCP_CMD_TIMEOUT)

    # Enable TCP keep-alive so the client detects a dead server
    expect_ok_retry(
        ser, f"PK=1,{KEEPALIVE_MS}", "Enable TCP Keep-Alive", timeout=TCP_CMD_TIMEOUT
    )

    # Enter client passthrough mode -- after this, serial port IS the TCP link.
    # Do not pre-start the API client with P6=1; the IWIN streaming example
    # starts client streaming directly with PX=1,0.
    print("[local] Entering passthrough mode...")
    resp = send_cmd(ser, "PX=1,0", timeout=PASSTHROUGH_CMD_TIMEOUT)
    if "ERROR" in resp:
        delayed = _read_until_quiet(
            ser, quiet_ms=RETRY_SETTLE_QUIET_MS, timeout=RETRY_SETTLE_TIMEOUT
        )
        resp = send_cmd(ser, "PX=1,0", timeout=PASSTHROUGH_CMD_TIMEOUT)
        if "ERROR" in resp:
            raise RuntimeError(f"PX failed: {delayed}{resp}")

    # Flush residual AT response bytes, then immediately poke the remote shell.
    if POST_PASSTHROUGH_FLUSH > 0:
        time.sleep(POST_PASSTHROUGH_FLUSH)
    ser.reset_input_buffer()
    g_initial_shell_output = solicit_shell_prompt(ser)

    print("[local] Passthrough active.")


def connect(ser):
    """Run the full connection sequence: station join + TCP client setup.

    Returns True on success, False on failure.
    """
    try:
        if not check_passthrough(ser):
            setup_station(ser)
            time.sleep(POST_JOIN_DELAY)
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

        try:
            # Make sure we're back in AT command mode.  If the module
            # already dropped out of streaming on its own (keep-alive
            # failure or TCP close), we should be there.  If not, a
            # hard reset will get us there.
            if not is_in_at_mode(ser):
                hard_reset(ser)

            setup_station(ser)
            time.sleep(POST_JOIN_DELAY)
            setup_tcp_client(ser)
            print("[local] Reconnected successfully!")
            return True
        except Exception as e:
            print(f"[local] Reconnect attempt {attempt} failed: {e}")
            time.sleep(RECONNECT_DELAY)

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
    global g_initial_shell_output

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

            print()
            print("=" * 60)
            print("  Connected to prawler WiFi shell!")
            print()
            print("  Type a command and press Enter.")
            print("  Type 'quit' to exit.")
            print("=" * 60)
            print()

            if g_initial_shell_output:
                sys.stdout.write(g_initial_shell_output)
                sys.stdout.flush()
                g_initial_shell_output = ""
            else:
                ser.write(b"\r\n")
                ser.flush()

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
