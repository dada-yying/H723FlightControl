"""Check new H723 fields through unchanged F407 storage/converter fixtures.

Run after H723 tests/pid_tuning/run.ps1 and F407 tests/run.cmd.
This validates wire/log/CSV data, not hardware SPI or SD timing.
"""
from pathlib import Path
import csv
import struct
import sys
import tempfile
import re

root = Path(__file__).resolve().parents[1]
receiver = root.parent / "STM32F407VET6黑匣子"
sys.path.insert(0, str(receiver / "tools"))
import blackbox_to_csv as converter


def f32(value):
    return struct.unpack("<f", struct.pack("<f", value))[0]


def coefficients(name):
    source = (root / "App/bmi088_calibration.h").read_text("utf-8")
    match = re.search(r"#define " + name + r"\s*\\\s*\{([^}]+)\}", source)
    assert match, name
    return [f32(float(v.strip().rstrip("f"))) for v in match.group(1).split(",")]


gyro_bias = coefficients("BMI088_CALIBRATION_GYRO_BIAS_RAD_S")
accel_bias = coefficients("BMI088_CALIBRATION_ACCEL_BIAS_M_S2")
accel_scale = coefficients("BMI088_CALIBRATION_ACCEL_SCALE")
gyro_temperature = coefficients("BMI088_CALIBRATION_GYRO_TEMP_RAD_S_C")


with tempfile.TemporaryDirectory(prefix="bmi088-cross-") as directory:
    for prefix, extra in (("v3cross_", False), ("v3extra_", True)):
        paths = sorted((receiver / "tests/build").glob(prefix + "integration*.bin"))
        assert paths, "Run the real F407 host storage tests first"
        report = converter.convert(paths, Path(directory) / (prefix + "bmi.csv"))
        assert report["undecoded_records"] == report["bad_schema_fragments"] == 0
        assert report["decoded_records"] > 2000
        count = 0
        for output in report["outputs"]:
            with Path(output).open(encoding="utf-8-sig", newline="") as file:
                for row in csv.DictReader(file):
                    for axis, (gyro, accel) in enumerate(zip((-32768, 1234, 32767), (1000, -2000, 8192))):
                        assert f"bmi088_gyro_raw[{axis}]" not in row and f"bmi088_accel_raw[{axis}]" not in row
                        assert f"bmi088_gyro_calibrated_rad_s[{axis}]" not in row
                        assert f"bmi088_accel_calibrated_m_s2[{axis}]" not in row
                        nominal_gyro = f32(gyro * f32(0.0010652644360316953))
                        nominal_accel = f32(accel * f32(f32(6.0*f32(9.80665))/32768.0))
                        applied_bias = f32(gyro_bias[axis] + f32(gyro_temperature[axis] * -7.0))
                        assert f"bmi088_gyro_rad_s[{axis}]" not in row
                        assert float(row[f"bmi088_gyro_deg_s[{axis}]"]) == f32(f32(nominal_gyro - applied_bias) * f32(57.295779513))
                        assert float(row[f"bmi088_accel_m_s2[{axis}]"]) == f32(f32(nominal_accel - accel_bias[axis]) * accel_scale[axis])
                    assert row["bmi088_calibration_version"] == "2026100302"
                    assert row["bmi088_compensation_in_range"] == "1" and float(row["bmi088_compensation_temperature_c"]) == 33
                    assert float(row["bmi088_temperature_c"]) == 33
                    assert "bmi088_temperature_raw" not in row
                    assert int(row["bmi088_sample_count"]) == 4294967294
                    assert int(row["bmi088_age_ms"]) == 0
                    assert all(row["bmi088_" + name] == "1" for name in ("has_sample", "valid", "initialized"))
                    assert row["bmi088_error"] == "0"
                    assert "fc_timestamp_us" not in row and int(row["sample_time_us"]) > 0
                    for name, rad in (("roll", 0.25), ("pitch", -0.125), ("yaw", 0.5)):
                        assert float(row["bmi088_" + name + "_deg"]) == f32(f32(rad) * f32(57.295779513))
                    assert row["bmi088_attitude_valid"] == "1" and row["bmi088_attitude_epoch"] == "7"
                    assert row["bmi088_attitude_accel_used"] == "1" and row["bmi088_attitude_error"] == "2"
                    for axis, rad in enumerate((.125, -.25, .5)):
                        assert float(row[f"bmi088_gyro_filtered_deg_s[{axis}]"]) == f32(f32(rad) * f32(57.295779513))
                    assert row["bmi088_gyro_filter_valid"] == "1"
                    assert float(row["bmi088_gyro_filter_cutoff_hz"]) == 50
                    assert float(row["bmi088_gyro_filter_sample_hz"]) == 2000
                    assert float(row["yaw_target_deg"]) == f32(f32(.6) * f32(57.295779513))
                    assert float(row["yaw_error_deg"]) == f32(f32(.1) * f32(57.295779513))
                    assert row["flight_attitude_epoch"] == "6" and row["yaw_hold_active"] == "1"
                    if extra:
                        assert row["new_float"] == "1.25" and row["new_uint32"] == "4294967294"
                        assert [row[f"new_array[{i}]"] for i in range(3)] == ["-32768", "0", "32767"]
                    else:
                        assert "new_float" not in row
                    count += 1
        assert count == report["decoded_records"]
        print(f"PASS {prefix}: {count} BMI088 records, raw/units/status/integers/arrays restored by unchanged F407/converter")
