"""Replay the report's captured raw27 frames through the production C receiver.

Requires the *_decoded.npz and *_unique_indices.npy produced by decode_flow.py,
plus NumPy/SciPy. Outputs stay under tests/build; source logs are read-only.
"""
import argparse
import json
import struct
import subprocess
from pathlib import Path

import numpy as np
from scipy.signal import butter, lfilter, lfilter_zi


def verify(data, exe, output):
    results = []
    b, a = butter(2, 5, fs=100)
    zi = lfilter_zi(b, a)
    for source in sorted(data.glob("B*_decoded.npz")):
        name = source.name.removesuffix("_decoded.npz")
        with np.load(source) as log:
            indices = np.load(data / f"{name}_unique_indices.npy")
            raw = log["of_raw"][indices]
            tick = (log["sample_time_us"][indices] // 1000 - log["of_age_ms"][indices]) & 0xffffffff
            x = log["of_flow_x"][indices].astype(float)
            y = log["of_flow_y"][indices].astype(float)
            distance = log["of_distance_mm"][indices].astype(float) * .001
            relative_time = log["t"][indices]
        wire = output / f"{name}_filter_input.bin"
        csv = output / f"{name}_filter_replay.csv"
        with wire.open("wb") as stream:
            for timestamp, frame in zip(tick, raw):
                frame = bytes(frame)
                assert len(frame) == 27 and sum(frame[:-1]) % 256 == frame[-1]
                stream.write(struct.pack("<I", int(timestamp)) + frame)
        subprocess.run([str(exe.resolve()), "--replay", str(wire), str(csv)], check=True)
        replay = np.genfromtxt(csv, delimiter=",", names=True)
        assert len(replay) == len(indices)
        assert np.array_equal(replay["rx_count"], np.arange(1, len(indices) + 1))
        advanced = np.r_[replay["update_count"][0] == 1, np.diff(replay["update_count"]) == 1]
        assert np.all((np.diff(replay["update_count"]) == 0) | (np.diff(replay["update_count"]) == 1))
        actual = np.column_stack([replay[n] for n in ("flow_x", "flow_y", "vx_filtered", "vy_filtered")])
        inputs = np.column_stack((x, y, .01*x*distance, .01*y*distance))
        starts = np.flatnonzero(advanced & (replay["streak"] == 1))
        max_error = np.zeros(4)
        for start in starts:
            end = start + 1
            while end < len(replay) and advanced[end] and replay["streak"][end] != 1:
                end += 1
            for axis in range(4):
                expected, _ = lfilter(b, a, inputs[start:end, axis], zi=zi*inputs[start, axis])
                max_error[axis] = max(max_error[axis], float(np.max(np.abs(actual[start:end, axis] - expected))))
        # Float32 firmware vs independent double-precision SciPy design.
        assert np.all(max_error[:2] < .03), (name, max_error)
        assert np.all(max_error[2:] < .001), (name, max_error)
        for i in np.flatnonzero(~advanced):
            if i:
                assert np.array_equal(actual[i], actual[i-1]), (name, i)
                assert replay["valid"][i] == 0
        assert np.all(replay["streak"][replay["valid"] == 1] == 3)
        rejected_range = (replay["flags"].astype(int) & 16) != 0
        result = dict(file=name, frames=len(indices), updates=int(advanced.sum()),
                      range_jump_rejections=int(rejected_range.sum()),
                      max_reference_error=max_error.tolist())
        if name == "B0000029":
            event = (relative_time > 14.21) & (relative_time < 14.26) & (distance > 1)
            assert event.sum() == 3 and np.all(rejected_range[event])
            assert not np.any(replay["valid"][event])
            assert np.all(replay["range"][event] < .2)
            result["known_spike_ranges_m"] = distance[event].tolist()
            result["known_spike_times_s"] = relative_time[event].tolist()
        results.append(result)
        print(f"PASS {name}: {len(indices)} raw frames, SciPy parity, rejection/hold behavior")
    assert results, f"No decoded logs found in {data}"
    (output / "replay_results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    verify(args.data, args.exe, args.output)
