#!/usr/bin/env python3
"""Decode Prawler idata CSV or framed idata captures into physical units.

Usage:
    python RecordTranslator.py idata.csv
    python RecordTranslator.py framed_idata.txt
    python RecordTranslator.py idata.csv -o output.csv

Inputs can be either the raw payload CSV or a full framed capture:
    @@@CCCCLLLL\n<payload>

CCCC is the frame CRC and LLLL is the payload byte count. The CRC covers the
four ASCII length bytes followed by exactly LLLL payload bytes. It excludes the
@@@ preamble, CCCC field, and separator newline. The firmware uses a
CRC-16/XMODEM-compatible algorithm with init 0x0000, polynomial 0x1021, and two
final zero-byte augmentations. The check vector "123456789" is 0x31C3.

The short payload headers are fixed by an upstream system and are decoded
positionally. Do not rename them on the wire:
    EP,CD,CT,CC
    EP,CD,CT,CC,OT,O2
    EP,CD,CT,CC,OT,O2,CH,TB,CD

Scaling factors (from realtime_comm.c):
    datetime  uint32  raw Unix epoch (seconds)
    depth     int16   pressure * 100       -> dbar
    temp      int16   temperature * 1000   -> deg C
    cond      int16   conductivity * 10000 -> S/m
    wl_c1     uint16  raw signal           -> counts
    wl_c2     uint16  raw signal           -> counts
    wl_c3     uint16  raw signal           -> counts
    o2        uint16  O2_conc * 100        -> umol/L
    o2temp    uint16  optode_temp * 1000   -> deg C
"""

import argparse
import csv
import io
import sys
from datetime import datetime, timezone
from pathlib import Path


def hex_to_int16(h: str) -> int:
    val = int(h, 16)
    if val >= 0x8000:
        val -= 0x10000
    return val


def hex_to_uint16(h: str) -> int:
    return int(h, 16)


FRAME_PREFIX = b"@@@"
FRAME_HEADER_LEN = 12

POSITIONAL_SHORT_HEADERS = {
    ("ep", "cd", "ct", "cc"): [
        "datetime", "depth", "temp", "cond",
    ],
    ("ep", "cd", "ct", "cc", "ot", "o2"): [
        "datetime", "depth", "temp", "cond", "o2temp", "o2",
    ],
    ("ep", "cd", "ct", "cc", "ot", "o2", "ch", "tb", "cd"): [
        "datetime", "depth", "temp", "cond", "o2temp", "o2",
        "wl_c1", "wl_c2", "wl_c3",
    ],
}

OUTPUT_FIELDS = [
    "Unix_Epoch_UTC", "DateTime_UTC",
    "Depth_dbar", "Temp_degC", "Cond_S_m",
    "Chl_counts", "NTU_counts", "CDOM_counts",
    "O2_umol_L", "O2Temp_degC",
]

INPUT_ALIASES = {
    "datetime": "datetime",
    "depth": "depth",
    "temp": "temp",
    "cond": "cond",
    "chl": "wl_c1",
    "wl_c1": "wl_c1",
    "ntu": "wl_c2",
    "wl_c2": "wl_c2",
    "cdom": "wl_c3",
    "wl_c3": "wl_c3",
    "o2": "o2",
    "o2temp": "o2temp",
}


class FrameError(ValueError):
    pass


def rt_crc16_xmodem_accum(accum: int, ch: int) -> int:
    accum = (accum | ch) & 0xFFFFFFFF

    for _ in range(8):
        accum = (accum << 1) & 0xFFFFFFFF
        if accum & 0x1000000:
            accum = (accum ^ 0x102100) & 0xFFFFFFFF

    return accum


def rt_crc16_xmodem(data: bytes) -> int:
    accum = 0
    for ch in data:
        accum = rt_crc16_xmodem_accum(accum, ch)

    accum = rt_crc16_xmodem_accum(accum, 0)
    accum = rt_crc16_xmodem_accum(accum, 0)
    return (accum >> 8) & 0xFFFF


def parse_framed_payload(raw: bytes) -> tuple[bytes, int]:
    if len(raw) < FRAME_HEADER_LEN:
        raise FrameError("framed capture is shorter than @@@CCCCLLLL\\n")

    if not raw.startswith(FRAME_PREFIX):
        raise FrameError("framed capture does not start with @@@")

    try:
        crc_text = raw[3:7].decode("ascii")
        len_text = raw[7:11].decode("ascii")
    except UnicodeDecodeError as exc:
        raise FrameError("frame CRC/length fields are not ASCII") from exc

    if raw[11:12] != b"\n":
        raise FrameError("expected newline after frame length")

    try:
        expected_crc = int(crc_text, 16)
        payload_len = int(len_text, 16)
    except ValueError as exc:
        raise FrameError("frame CRC/length fields must be hexadecimal") from exc

    payload_start = FRAME_HEADER_LEN
    payload_end = payload_start + payload_len
    if len(raw) < payload_end:
        raise FrameError(
            f"frame declares {payload_len} payload bytes, "
            f"but only {len(raw) - payload_start} are present"
        )

    payload = raw[payload_start:payload_end]
    actual_crc = rt_crc16_xmodem(len_text.encode("ascii") + payload)
    if actual_crc != expected_crc:
        raise FrameError(
            f"CRC mismatch: frame has 0x{expected_crc:04X}, "
            f"computed 0x{actual_crc:04X}"
        )

    return payload, len(raw) - payload_end


def load_payload_text(path: Path) -> str:
    raw = path.read_bytes()

    if raw.startswith(FRAME_PREFIX):
        payload, trailing_len = parse_framed_payload(raw)
        if trailing_len:
            print(
                f"Warning: ignoring {trailing_len} trailing byte(s) after framed payload",
                file=sys.stderr,
            )
    else:
        payload = raw

    try:
        return payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{path} is not valid UTF-8/ASCII text") from exc


def normalize_header(header: list[str]) -> list[str]:
    cols = [col.strip().lower() for col in header]
    positional = POSITIONAL_SHORT_HEADERS.get(tuple(cols))
    if positional is not None:
        return positional
    return [INPUT_ALIASES.get(col, col) for col in cols]


def decode_row(row: list[str], header: list[str]) -> dict:
    decoded = {field: "" for field in OUTPUT_FIELDS}

    for column, value in zip(header, row):
        if column == "datetime":
            ts = int(value, 16)
            decoded["Unix_Epoch_UTC"] = ts
            decoded["DateTime_UTC"] = datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
        elif column == "depth":
            decoded["Depth_dbar"] = hex_to_int16(value) / 100.0
        elif column == "temp":
            decoded["Temp_degC"] = hex_to_int16(value) / 1000.0
        elif column == "cond":
            decoded["Cond_S_m"] = hex_to_int16(value) / 10000.0
        elif column == "wl_c1":
            decoded["Chl_counts"] = hex_to_uint16(value)
        elif column == "wl_c2":
            decoded["NTU_counts"] = hex_to_uint16(value)
        elif column == "wl_c3":
            decoded["CDOM_counts"] = hex_to_uint16(value)
        elif column == "o2":
            decoded["O2_umol_L"] = hex_to_uint16(value) / 100.0
        elif column == "o2temp":
            decoded["O2Temp_degC"] = hex_to_uint16(value) / 1000.0

    return decoded


def main():
    parser = argparse.ArgumentParser(
        description="Decode idata CSV or framed idata captures into RECORDS.csv"
    )
    parser.add_argument("input", help="Path to idata payload CSV or framed capture")
    parser.add_argument("-o", "--output", default=None,
                        help="Output CSV path (default: RECORDS.csv next to input)")
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: {input_path} not found", file=sys.stderr)
        sys.exit(1)

    if args.output:
        output_path = Path(args.output)
    else:
        output_path = input_path.parent / "RECORDS.csv"

    rows_decoded = 0
    try:
        payload_text = load_payload_text(input_path)
    except (FrameError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    with io.StringIO(payload_text, newline="") as fin, \
         open(output_path, "w", newline="") as fout:
        reader = csv.reader(fin)
        try:
            header = normalize_header(next(reader))
        except StopIteration:
            print("Error: input payload is empty", file=sys.stderr)
            sys.exit(1)

        if "datetime" not in header:
            print(f"Warning: unexpected header: {header}", file=sys.stderr)

        writer = csv.DictWriter(fout, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()

        for lineno, row in enumerate(reader, start=2):
            if len(row) < len(header):
                print(f"Warning: skipping short row at line {lineno}: {row}",
                      file=sys.stderr)
                continue
            try:
                decoded = decode_row(row, header)
                writer.writerow(decoded)
                rows_decoded += 1
            except (ValueError, IndexError) as e:
                print(f"Warning: failed to decode line {lineno}: {e}",
                      file=sys.stderr)

    print(f"Decoded {rows_decoded} rows -> {output_path}")


if __name__ == "__main__":
    main()
