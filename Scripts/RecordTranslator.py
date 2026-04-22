#!/usr/bin/env python3
"""Decode a hex-encoded idata.csv from the prawler realtime stream into a
human-readable RECORDS.csv with physical units.

Usage:
    python RecordTranslator.py idata.csv
    python RecordTranslator.py idata.csv -o output.csv

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


LEGACY_SHORT_HEADER = ["ep", "cd", "ct", "cc", "ot", "o2", "ch", "tb", "cd"]
LEGACY_SHORT_COLUMNS = [
    "datetime", "depth", "temp", "cond", "o2temp", "o2", "wl_c1", "wl_c2", "wl_c3"
]

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


def normalize_header(header: list[str]) -> list[str]:
    cols = [col.strip().lower() for col in header]
    if cols == LEGACY_SHORT_HEADER:
        return LEGACY_SHORT_COLUMNS
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
        description="Decode hex-encoded idata.csv into human-readable RECORDS.csv"
    )
    parser.add_argument("input", help="Path to idata.csv")
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
    with open(input_path, newline="") as fin, \
         open(output_path, "w", newline="") as fout:
        reader = csv.reader(fin)
        header = normalize_header(next(reader))
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
