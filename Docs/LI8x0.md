# LI-8x0 driver

`Core/Inc/LI8x0.h` and `Core/Src/LI8x0.c` adapt the LI-COR driver in
`/home/joe/Work/EDD-pCO2Sensor/Firmware/pCO2Sensor/src/LI8x0.c` and `.h`
(Matt Casari, NOAA/PMEL/EDD) to the STM32 HAL interfaces used here.

The LI-830/850 protocol reference is the local
[LI-COR Integrator's Guide, publication 977-17191](../../EDD-pCO2Sensor/Documentation/Device%20User%20Manuals/830-850-Integrator%27s-Guide-17191-draft.pdf),
particularly sections 1-7 to 1-13 (queries/calibration), 2-2 to 2-4 (measurement
fields), 2-10 (calibration constants), and A-1 (9600 baud, 8N1, no flow control).
LI-820/840 model tags and capability distinctions follow the reference driver;
those models still need hardware verification.

## Branch integration and shell commands

This branch runs the UART shell on USART1 (`huart1`) and LI-COR on USART2
(`huart2`, PA2/PA3), both at 9600 baud, 8N1. PB1 enables the USART2 RS-232
transceiver. USART3, UART4 and UART5 are not initialized, and their transceivers
remain disabled. The driver is bound with an unknown model at boot; initialization
sends no instrument commands. The instrument must be powered and warmed up externally.

SD/FatFS and the external RTC remain available through their existing commands.
The firmware stays awake for shell work. Recorder, legacy sensors, WiFi and
realtime streaming sources are excluded from the firmware target. Their commands,
profile configuration, sample-rate controls and inactive sleep controls are removed.

Start with:

```text
licor-discover
licor-configure
licor-info
licor
```

If the model is known, `licor-model 830` (or 820/840/850) selects it without sending
a discovery request. Other instrument commands discover automatically if the model
is still unknown. Output configuration is explicit; sampling does not silently
change instrument settings.

| Command | Behavior |
|---|---|
| `licor` | Print one measurement, with only the fields actually reported |
| `licor-raw` | Request one measurement and print its complete XML |
| `licor-info` | Read model, instrument serial number and firmware version |
| `licor-discover` | Send the driver's `<who>` discovery request |
| `licor-model [820\|830\|840\|850]` | Show/select the protocol model locally |
| `licor-configure` | Select polled XML measurement output |
| `licor-query [section]` | Print a top-level XML section, or full state if omitted |
| `licor-set section field [subfield] value` | Set one scalar field and require an ACK |
| `licor-heater on\|off` | Set the heater and require an ACK |
| `licor-resync [milliseconds]` | Drain RX after recovery, with optional 0-5000 ms settling |

Examples: `licor-query cal`, `licor-query cfg`, `licor-set cfg pcomp TRUE`,
`licor-set rs232 raw co2 TRUE`. Field names and values are single shell tokens;
XML markup is rejected. Replies are printed in full, without the shell printf
buffer's 256-byte truncation. `who` reports the board serial number (`PW002`),
while `licor-info` reports the instrument identity.

Commands execute in foreground shell context; no driver calls or printing occur
in the UART ISR. Errors are named and reported to the shell. Failed transactions
are never automatically retried or resynchronized: wait until the prior operation
has ended, or reset the instrument, then explicitly run `licor-resync`. Model
selection does not clear a synchronization fault. A timeout on a setting does not
prove that the instrument left the setting unchanged.

## UART contract

- One driver instance, called only from foreground code, not interrupts.
- Exclusively owned HAL UART configured for 9600 baud, 8 data bits, no parity,
  one stop bit, TX/RX, and no hardware flow control.
- Linked normal-mode RX DMA, with its UART/DMA IRQ handlers enabled. Circular DMA
  is rejected. Direction must be peripheral-to-memory, both transfer widths must
  be byte-sized, peripheral increment disabled, and memory increment enabled.
  These settings are checked at binding and again before RX. TX is a bounded
  blocking HAL transmit; no TX DMA is required.
- RX DMA is armed before TX. Receive framing uses complete model closing tags,
  so a UART idle gap does not end a fragmented XML document.
- No LI-8x0-specific centralized HAL callback dispatch is required. Existing
  callbacks must not start a competing transfer on the chosen UART.
- Set polled XML output with `li8x0_configure()` before routine queries. Streaming
  or stripped output is not the supported transaction mode.
- The driver drains a following ACK or a 100 ms quiet gap after a query payload.
  The protocol has no transaction IDs: a timeout does not prove a setting was
  unapplied, and arbitrarily delayed replies cannot be associated safely with a
  later command. Verify state before retrying calibration or consequential writes.
- Cancellation or a failed/rejected UART transaction latches
  `LI8X0_SYNC_REQUIRED`; subsequent commands send nothing until explicit
  `li8x0_resync(settle_ms)` succeeds. Reinitialization does not clear that latch.
  First establish that the previous sensor operation has ended, or arrange a
  sensor reset/power cycle in the application. Resynchronization only drains
  bytes; it cannot prove that an arbitrarily delayed calibration reply will never
  arrive. Its optional settling interval is 0–5000 ms, followed by a required
  100 ms quiet gap, with an overall deadline of settling time plus 5000 ms.
  Failed drains keep the command lockout. No calibration is automatically retried.

Data requests, setting ACKs and discovery have a 2-second deadline. Larger state
queries have 5 seconds, allowing a longer XML response at 9600 baud. Deadlines
start when RX is armed, before command transmission. A split-phase data request
must be collected within that deadline. RX capacity is 4095 bytes plus a string
terminator; command capacity is 255 bytes plus a terminator. Oversize responses
and commands report `LI8X0_OVERFLOW` rather than silently truncating.

## API

| Operation | API |
|---|---|
| Bind a known model, or prepare discovery | `li8x0_init(huart, model)` |
| Legacy `<who>` discovery, without power control | `li8x0_autodiscover()` |
| Set polled XML output | `li8x0_configure()` |
| Blocking measurement | `li8x0_sample(out)` |
| Split request/collect | `li8x0_fire()`, `li8x0_collect(out)` |
| Cancel an owned pending RX | `li8x0_cancel()` |
| Explicitly discard pending sensor replies after recovery | `li8x0_resync(settle_ms)` |
| Identity/full state | `li8x0_get_info(out)` |
| Read CO2/H2O calibration constants | `li8x0_get_cal(out)` |
| Query another top-level section | `li8x0_query(section, out)` |
| Set a scalar, optionally one nested level | `li8x0_set(section, field, subfield, value)` |
| Heater control | `li8x0_heater(enabled)` |
| Write CO2 coefficients | `li8x0_set_co2_zero(value)`, `li8x0_set_co2_span(value)`, `li8x0_set_co2_span2(value)` |
| Start zero/span with an ISO date | `li8x0_start_zero(date)`, `li8x0_start_span(date, ppm, secondary)` |
| Access the last completed response XML | `li8x0_last_response(length)` |
| Pure bounded parser | `li8x0_parse_response(xml, length, out)` |

Return values distinguish invalid arguments, unbound/unknown-model state, an
active request, timeout, UART failure, overflow, malformed replies, and negative
ACKs/instrument errors, as well as required resynchronization. Only `LI8X0_OK`
is success. Full-state queries require serial number and version in the same
document; an unsolicited measurement cannot complete an identity query.

`li8x0_configure()` sets `outrate=0`, disables echo/tag stripping, enables CO2,
cell temperature, cell pressure, input voltage and raw channels, and enables H2O
fields for LI-840/850. It leaves heater, pressure compensation, pump, alarms,
bench geometry and calibration unchanged. Explicit setters remain available for
those instrument settings. Configuration stops at the first failed ACK and may
therefore be partially applied.

Measurements require a reported CO2 field. Missing optional floats are `NAN`;
`data.valid_fields` identifies every reported field, including integer raw
channels. Raw CO2 is parsed separately from concentration CO2. Invalid numeric
text, nonfinite values, duplicate recognized fields, mismatched tags and
truncated XML are rejected. Input XML is not modified; string case is preserved.
Caller outputs are unchanged on a failed parse or failed transaction.

The typed response contains measurements, identity, ACK/error fields and user
calibration constants/dates. Factory polynomials, auxiliary values, pump and
configuration state can be queried as raw XML; this is not a full typed port of
all legacy diagnostic structs or string-formatting functions.

Calibration commands include the date and zero/span value in the same `<cal>`
element, as required in guide section 1-12. `LI8X0_OK` from a start call means
**acknowledged**, not completed. Supply suitable gas, then read back calibration
constants/dates later; the driver does not perform the legacy 60-second busy wait
or feed the system watchdog itself.

## Verification

The host suite uses deliberately constructed protocol fixtures and a simulated
HAL/DMA transport. It is not sensor data captured from hardware or a UART timing
benchmark. Run it independently of the ARM toolchain:

```sh
cmake -S tests -B build/host-tests -DENABLE_SANITIZERS=ON
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

Tests cover measurement/raw/H2O/identity/calibration parsing, every truncated
sample prefix, byte mutations, malformed numbers/XML, fragmented replies,
ACK-only failures, trailing ACKs, split-phase ownership, timeouts, UART failures,
DMA configuration rejection, stale ACKs across DMA restarts, explicit recovery,
unsolicited measurements before full-state replies, overflow, tick rollover,
dated calibration command construction, and model-specific
configuration without changing heater/pressure compensation/calibration.
Compilation uses warnings as errors and keeps assertions active in optimized
builds. The new source also supports a syntax-only check against the repository's
real STM32 HAL/CMSIS headers.

The shell integration suite also checks USART2 transport ownership, command
registration/removal, retained SD/RTC commands, automatic discovery, measurement
validity masks, complete XML output, nested settings, input validation and explicit
recovery after a timeout. Hardware communication and physical timing still require
a connected instrument; these tests never flash the board.
