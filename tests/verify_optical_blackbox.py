"""Check new optical fields after production sender -> FatFs -> stock decoder."""
import csv
import math
import sys
from pathlib import Path


def verify(path):
    count = 0
    previous_count = 0
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            rx_count = int(row["of_filter_rx_count"])
            assert rx_count >= previous_count > -1
            previous_count = rx_count
            assert row["of_distance_mm"] == "250"
            assert row["of_flow_x"] == "100" and row["of_flow_y"] == "-200"
            assert int(row["of_seq"]) == (rx_count - 1) % 256
            assert int(row["of_time_ms"]) == (rx_count - 1) * 10
            assert int(row["of_filter_update_count"]) == rx_count
            assert int(row["of_filter_age_ms"]) == int(row["of_age_ms"]) < 10
            assert int(row["of_range_age_ms"]) == int(row["of_age_ms"])
            assert int(row["of_filter_dt_ms"]) == (0 if rx_count == 1 else 10)
            streak = min(rx_count, 3)
            assert int(row["of_filter_accepted_streak"]) == streak
            assert int(row["of_filter_valid"]) == int(streak == 3)
            assert int(row["of_range_valid"]) == 1
            assert int(row["of_filter_flags"]) == (256 if streak < 3 else 0)
            for field, expected in {
                "of_flow_x_filtered": 100, "of_flow_y_filtered": -200,
                "of_range_accepted_m": .25, "of_range_innovation_m": 0,
                "of_velocity_x_uncomp_m_s": .25, "of_velocity_y_uncomp_m_s": -.5,
                "of_velocity_x_filtered_m_s": .25, "of_velocity_y_filtered_m_s": -.5,
            }.items():
                assert math.isclose(float(row[field]), expected, rel_tol=1e-5, abs_tol=1e-6), field
            raw = bytes.fromhex(row["of_raw"])
            assert len(raw) == 27 and sum(raw[:-1]) % 256 == raw[-1]
            assert int.from_bytes(raw[6:10], "little") == int(row["of_time_ms"])
            count += 1
    print(f"PASS {path}: {count} coherent raw/filtered optical records")
    return count


if __name__ == "__main__":
    assert sum(verify(path) for path in sys.argv[1:]) == 2000
