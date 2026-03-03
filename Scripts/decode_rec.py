#!/usr/bin/env python3
"""Decode binary recorder files (rec_NNNN.bin) from STM Prawler.

Usage:
    python decode_rec.py rec_0001.bin [rec_0002.bin ...]
    python decode_rec.py --csv rec_0001.bin > data.csv
    python decode_rec.py --summary rec_0001.bin
"""

import argparse
import datetime
import struct
import sys
from pathlib import Path

# record_data_t layout (ARM Cortex-M4, little-endian, natural alignment):
#   uint32_t  magic          offset  0
#   uint32_t  record_num     offset  4
#   uint32_t  timestamp      offset  8
#   float     ctd.cond       offset 12
#   float     ctd.temp       offset 16
#   float     ctd.press      offset 20
#   uint16_t  opt.product    offset 24
#   uint16_t  opt.serial     offset 26
#   float     opt.o2_conc    offset 28
#   float     opt.temp       offset 32
#   float     opt.cal_phase  offset 36
#   float     opt.tc_phase   offset 40
#   float     opt.c1_rph     offset 44
#   float     opt.c2_rph     offset 48
#   float     opt.c1_amp     offset 52
#   float     opt.c2_amp     offset 56
#   float     opt.raw_temp   offset 60
#   uint16_t  wet.chl_lam    offset 64
#   uint16_t  wet.chl_sig    offset 66
#   uint16_t  wet.ntu_lam    offset 68
#   uint16_t  wet.ntu_sig    offset 70
#   uint16_t  wet.cdom_lam   offset 72
#   uint16_t  wet.cdom_sig   offset 74
#   uint16_t  wet.therm      offset 76
#   2 bytes   padding        offset 78
#   Total: 80 bytes

RECORD_MAGIC = 0xFACEFACE
RECORD_FMT = "<III fff HH fffffffff HHHHHHH xx"
RECORD_SIZE = struct.calcsize(RECORD_FMT)  # 80
GPS_EPOCH = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)

FIELD_NAMES = [
    "magic", "record_num", "timestamp",
    "ctd_cond", "ctd_temp", "ctd_press",
    "opt_product", "opt_serial",
    "opt_o2_conc", "opt_temp", "opt_cal_phase", "opt_tc_phase",
    "opt_c1_rph", "opt_c2_rph", "opt_c1_amp", "opt_c2_amp", "opt_raw_temp",
    "wet_chl_lambda", "wet_chl_signal",
    "wet_ntu_lambda", "wet_ntu_signal",
    "wet_cdom_lambda", "wet_cdom_signal",
    "wet_thermistor",
]

CSV_COLUMNS = [
    "file", "record_num", "timestamp_gps", "datetime_utc",
    "ctd_cond", "ctd_temp", "ctd_press",
    "opt_product", "opt_serial",
    "opt_o2_conc", "opt_temp", "opt_cal_phase", "opt_tc_phase",
    "opt_c1_rph", "opt_c2_rph", "opt_c1_amp", "opt_c2_amp", "opt_raw_temp",
    "wet_chl_lambda", "wet_chl_signal",
    "wet_ntu_lambda", "wet_ntu_signal",
    "wet_cdom_lambda", "wet_cdom_signal",
    "wet_thermistor",
]


def gps_to_utc(gps_seconds):
    """Convert GPS epoch seconds to UTC datetime."""
    if gps_seconds == 0:
        return None
    return GPS_EPOCH + datetime.timedelta(seconds=gps_seconds)


def decode_record(data, offset=0):
    """Decode a single record from bytes. Returns dict or None on bad magic."""
    if offset + RECORD_SIZE > len(data):
        return None

    values = struct.unpack_from(RECORD_FMT, data, offset)
    rec = dict(zip(FIELD_NAMES, values))

    if rec["magic"] != RECORD_MAGIC:
        return None

    rec["datetime_utc"] = gps_to_utc(rec["timestamp"])
    return rec


def decode_file(path):
    """Decode all records from a .bin file."""
    data = Path(path).read_bytes()
    records = []
    offset = 0

    while offset + RECORD_SIZE <= len(data):
        rec = decode_record(data, offset)
        if rec is None:
            # Try to resync by scanning for magic
            found = data.find(struct.pack("<I", RECORD_MAGIC), offset + 1)
            if found < 0:
                break
            offset = found
            continue
        records.append(rec)
        offset += RECORD_SIZE

    return records


def print_records(records, filename):
    """Print records in human-readable format."""
    print(f"=== {filename}: {len(records)} records ===\n")

    for rec in records:
        ts = rec["datetime_utc"]
        ts_str = ts.strftime("%Y-%m-%d %H:%M:%S UTC") if ts else "N/A"

        print(f"Record #{rec['record_num']}  |  {ts_str}  (GPS {rec['timestamp']})")
        print(f"  CTD   C={rec['ctd_cond']:.4f}  T={rec['ctd_temp']:.4f}  P={rec['ctd_press']:.5f}")
        print(f"  Optode {rec['opt_product']}/{rec['opt_serial']}  "
              f"O2={rec['opt_o2_conc']:.3f} uM  T={rec['opt_temp']:.3f} C  "
              f"CalPh={rec['opt_cal_phase']:.3f}  TCPh={rec['opt_tc_phase']:.3f}")
        print(f"         C1RPh={rec['opt_c1_rph']:.3f}  C2RPh={rec['opt_c2_rph']:.3f}  "
              f"C1Amp={rec['opt_c1_amp']:.1f}  C2Amp={rec['opt_c2_amp']:.1f}  "
              f"RawT={rec['opt_raw_temp']:.1f}")
        print(f"  WetLab CHL={rec['wet_chl_signal']}@{rec['wet_chl_lambda']}nm  "
              f"NTU={rec['wet_ntu_signal']}@{rec['wet_ntu_lambda']}nm  "
              f"CDOM={rec['wet_cdom_signal']}@{rec['wet_cdom_lambda']}nm  "
              f"Therm={rec['wet_thermistor']}")
        print()


def print_summary(records, filename):
    """Print a brief summary of the file."""
    if not records:
        print(f"{filename}: empty (0 records)")
        return

    first = records[0]
    last = records[-1]
    t0 = first["datetime_utc"]
    t1 = last["datetime_utc"]

    print(f"{filename}: {len(records)} records")
    print(f"  First: #{first['record_num']}  {t0.strftime('%Y-%m-%d %H:%M:%S') if t0 else 'N/A'}")
    print(f"  Last:  #{last['record_num']}  {t1.strftime('%Y-%m-%d %H:%M:%S') if t1 else 'N/A'}")
    if t0 and t1 and t1 > t0:
        duration = t1 - t0
        print(f"  Duration: {duration}")
        print(f"  Avg interval: {duration.total_seconds() / max(len(records) - 1, 1):.1f}s")


def print_csv(all_records):
    """Print all records as CSV to stdout."""
    print(",".join(CSV_COLUMNS))
    for filename, records in all_records:
        for rec in records:
            ts = rec["datetime_utc"]
            ts_str = ts.strftime("%Y-%m-%dT%H:%M:%SZ") if ts else ""
            row = [
                filename,
                str(rec["record_num"]),
                str(rec["timestamp"]),
                ts_str,
                f"{rec['ctd_cond']:.4f}",
                f"{rec['ctd_temp']:.4f}",
                f"{rec['ctd_press']:.5f}",
                str(rec["opt_product"]),
                str(rec["opt_serial"]),
                f"{rec['opt_o2_conc']:.3f}",
                f"{rec['opt_temp']:.3f}",
                f"{rec['opt_cal_phase']:.3f}",
                f"{rec['opt_tc_phase']:.3f}",
                f"{rec['opt_c1_rph']:.3f}",
                f"{rec['opt_c2_rph']:.3f}",
                f"{rec['opt_c1_amp']:.1f}",
                f"{rec['opt_c2_amp']:.1f}",
                f"{rec['opt_raw_temp']:.1f}",
                str(rec["wet_chl_lambda"]),
                str(rec["wet_chl_signal"]),
                str(rec["wet_ntu_lambda"]),
                str(rec["wet_ntu_signal"]),
                str(rec["wet_cdom_lambda"]),
                str(rec["wet_cdom_signal"]),
                str(rec["wet_thermistor"]),
            ]
            print(",".join(row))


def main():
    parser = argparse.ArgumentParser(description="Decode STM Prawler rec_NNNN.bin files")
    parser.add_argument("files", nargs="+", help="Binary recording files to decode")
    parser.add_argument("--csv", action="store_true", help="Output as CSV")
    parser.add_argument("--summary", action="store_true", help="Print summary only")
    args = parser.parse_args()

    all_records = []
    for path in args.files:
        records = decode_file(path)
        all_records.append((Path(path).name, records))

    if args.csv:
        print_csv(all_records)
    elif args.summary:
        for filename, records in all_records:
            print_summary(records, filename)
    else:
        for filename, records in all_records:
            print_records(records, filename)


if __name__ == "__main__":
    main()
