"""Check yaw fields after production sender -> real FatFs -> unchanged converter."""
import csv
import struct
import sys
from pathlib import Path


def f32(value):
    return struct.unpack("<f", struct.pack("<f", value))[0]


def verify(path):
    count = 0
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            # Matches the explicit transport fixture in test_blackbox.c.
            assert float(row["yaw_target_deg"]) == f32(f32(.6) * f32(57.295779513))
            assert float(row["yaw_error_deg"]) == f32(f32(.1) * f32(57.295779513))
            assert row["yaw_hold_active"] == "1"
            assert row["flight_attitude_epoch"] == "6"
            assert row["bmi088_attitude_epoch"] == "7"
            # Original feedback and rate fields retain their meaning/units.
            # Legacy VOFA base uses 180/pi in float32, not the BMI literal.
            assert float(row["yaw_deg"]) == f32(f32(.5) * f32(180 / f32(3.141592653589793)))
            assert float(row["yaw_rate_target_deg_s"]) == 0
            assert float(row["yaw_control_rate_target_deg_s"]) == 0
            if "new_float" in row:
                assert float(row["new_float"]) == 1.25
                assert row["new_uint32"] == "4294967294"
                assert [row[f"new_array[{i}]"] for i in range(3)] == ["-32768", "0", "32767"]
            count += 1
    print(f"PASS {path}: {count} yaw target/error/hold/epoch records and existing fields")
    return count


if __name__ == "__main__":
    count = sum(verify(path) for path in sys.argv[1:])
    assert count == 2000, count
