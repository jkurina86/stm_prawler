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
    chl       uint16  raw signal           -> counts
    ntu       uint16  raw signal           -> counts
    cdom      uint16  raw signal           -> counts
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


def decode_row(row: list[str]) -> dict:
    ts = int(row[0], 16)
    depth_raw = hex_to_int16(row[1])
    temp_raw = hex_to_int16(row[2])
    cond_raw = hex_to_int16(row[3])
    chl = hex_to_uint16(row[4])
    ntu = hex_to_uint16(row[5])
    cdom = hex_to_uint16(row[6])
    o2_raw = hex_to_uint16(row[7])
    o2temp_raw = hex_to_uint16(row[8])

    return {
        "Unix_Epoch_UTC": ts,
        "DateTime_UTC": datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S"),
        "Depth_dbar": depth_raw / 100.0,
        "Temp_degC": temp_raw / 1000.0,
        "Cond_S_m": cond_raw / 10000.0,
        "Chl_counts": chl,
        "NTU_counts": ntu,
        "CDOM_counts": cdom,
        "O2_umol_L": o2_raw / 100.0,
        "O2Temp_degC": o2temp_raw / 1000.0,
    }


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

    fieldnames = [
        "Unix_Epoch_UTC", "DateTime_UTC",
        "Depth_dbar", "Temp_degC", "Cond_S_m",
        "Chl_counts", "NTU_counts", "CDOM_counts",
        "O2_umol_L", "O2Temp_degC",
    ]

    rows_decoded = 0
    with open(input_path, newline="") as fin, \
         open(output_path, "w", newline="") as fout:
        reader = csv.reader(fin)
        header = next(reader)
        if header[0].strip().lower() != "datetime":
            print(f"Warning: unexpected header: {header}", file=sys.stderr)

        writer = csv.DictWriter(fout, fieldnames=fieldnames)
        writer.writeheader()

        for lineno, row in enumerate(reader, start=2):
            if len(row) < 9:
                print(f"Warning: skipping short row at line {lineno}: {row}",
                      file=sys.stderr)
                continue
            try:
                decoded = decode_row(row)
                writer.writerow(decoded)
                rows_decoded += 1
            except (ValueError, IndexError) as e:
                print(f"Warning: failed to decode line {lineno}: {e}",
                      file=sys.stderr)

    print(f"Decoded {rows_decoded} rows -> {output_path}")


if __name__ == "__main__":
    main()
