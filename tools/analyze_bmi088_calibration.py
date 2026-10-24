"""Read six-face SBB3 CSV logs without modifying them; profile in sensor XYZ.

Requires numpy/pandas. Successful SPI reads repeated in the logger are counted
once. Equal raw values from different SPI reads are NOT deduplicated.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

GYRO_SCALE = 0.0010652644360316953
# Historical +/-3g RAW-count logs ONLY, using their original conversion.
# Do not change this constant to the new firmware's +/-6g scale: that would
# reinterpret the evidence used to fit the existing physical bias/scale.
# Current calibrated m/s^2 logs are not input to this historical fitter.
ACCEL_SCALE = 0.0008974358974
G = 9.80665
GYRO = [f"bmi088_gyro_raw[{i}]" for i in range(3)]
ACCEL = [f"bmi088_accel_raw[{i}]" for i in range(3)]
COLUMNS = ["session", "fc_timestamp_us", "bmi088_sample_count",
           "bmi088_sample_time_ms", "bmi088_temperature_c", "bmi088_age_ms",
           "bmi088_has_sample", "bmi088_valid", "bmi088_initialized",
           "bmi088_error"] + GYRO + ACCEL + [f"motor_command_{i}" for i in range(4)]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for data in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(data)
    return digest.hexdigest()


def vector_stats(values):
    return dict(mean=values.mean(axis=0).tolist(),
                std=values.std(axis=0).tolist(),
                minimum=values.min(axis=0).tolist(),
                maximum=values.max(axis=0).tolist())


def load(path):
    frame = pd.read_csv(path, usecols=COLUMNS)
    total = len(frame)
    valid = ((frame.bmi088_valid == 1) & (frame.bmi088_has_sample == 1)
             & (frame.bmi088_initialized == 1) & (frame.bmi088_error == 0)
             & (frame.bmi088_age_ms < 20))
    motor_off = (frame[[f"motor_command_{i}" for i in range(4)]] == 0).all(axis=1)
    finite = np.isfinite(frame.drop(columns="session").to_numpy()).all(axis=1)
    safe = frame.loc[valid & motor_off & finite].copy()
    unique = safe.drop_duplicates(["session", "bmi088_sample_count", "bmi088_sample_time_ms"])
    unique = unique.copy()
    # Logger's 64-bit sender time is monotonic; do not fit receive timestamps.
    unique["time_s"] = (unique.fc_timestamp_us - frame.fc_timestamp_us.iloc[0]) / 1e6
    return unique, dict(rows=total, invalid=int((~valid).sum()),
                       motor_on=int((~motor_off).sum()), nonfinite=int((~finite).sum()),
                       repeated_successful_reads=len(safe) - len(unique),
                       unique_successful_reads=len(unique),
                       duration_s=float((frame.fc_timestamp_us.iloc[-1] - frame.fc_timestamp_us.iloc[0])/1e6))


def profile(path):
    frame, counts = load(path)
    gyro = frame[GYRO].to_numpy() * GYRO_SCALE * 180 / np.pi
    accel = frame[ACCEL].to_numpy() * ACCEL_SCALE
    bins = []
    for second, part in frame.groupby(np.floor(frame.time_s).astype(int), sort=True):
        a = part[ACCEL].to_numpy() * ACCEL_SCALE
        w = part[GYRO].to_numpy() * GYRO_SCALE * 180 / np.pi
        bins.append(dict(second=int(second), n=len(part),
                         temperature_mean=float(part.bmi088_temperature_c.mean()),
                         temperature_min=float(part.bmi088_temperature_c.min()),
                         temperature_max=float(part.bmi088_temperature_c.max()),
                         accel_mean=a.mean(axis=0).tolist(), accel_std=a.std(axis=0).tolist(),
                         gyro_mean=w.mean(axis=0).tolist(), gyro_std=w.std(axis=0).tolist(),
                         gyro_peak=float(np.max(np.abs(w)))))
    schema_path = next(path.parent.glob("*.schema.json"))
    schemas = json.loads(schema_path.read_text("utf-8"))
    if len(schemas) != 1 or schemas[0]["payload_size"] < 178:
        raise ValueError("Expected original raw BMI088 fields in the schema")
    fields = {f["name"]: f for f in schemas[0]["fields"]}
    for name, offset in (("bmi088_gyro_raw", 148), ("bmi088_accel_raw", 154)):
        if fields[name]["offset"] != offset or fields[name]["count"] != 3 or fields[name]["type"] != "i16":
            raise ValueError("Unexpected raw BMI088 field layout")
    report_path = next(p for p in path.parent.glob("*.json") if not p.name.endswith(".schema.json"))
    converted = json.loads(report_path.read_text("utf-8"))
    if (converted["decoded_records"] != counts["rows"] or
            any(converted[k] for k in ["undecoded_records", "bad_schema_fragments", "bad_data"])):
        raise ValueError("Converter report contains undecoded/corrupt data")
    result = dict(source=str(path.resolve()), sha256=sha256(path), counts=counts,
                  source_schema_id=schemas[0]["schema_id"],
                  converter_record_gaps=converted["record_gaps_across_input"],
                  temperature_percentiles=np.percentile(frame.bmi088_temperature_c, [0, 5, 50, 95, 100]).tolist(),
                  gyro_deg_s=vector_stats(gyro), accel_m_s2=vector_stats(accel), seconds=bins)
    print(path.parent.name, counts, "T", result["temperature_percentiles"],
          "A", result["accel_m_s2"]["mean"], "W", result["gyro_deg_s"]["mean"], flush=True)
    return result


def selection(profile):
    """Whole-second selection avoids trimming noise asymmetrically per sample."""
    bins = profile["seconds"]
    run = 0
    ready = None
    previous = None
    for b in bins:
        if (b["n"] >= 400 and 39.5 <= b["temperature_min"]
                and b["temperature_max"] <= 40.5):
            run = run + 1 if previous == b["second"] - 1 else 1
        else:
            run = 0
        previous = b["second"]
        if run >= 60:
            ready = max(180, b["second"] + 1)
            break
    if ready is None:
        raise ValueError("No 60-second stable 40 +/- 0.5 C period")
    movement = {b["second"] for b in bins if b["gyro_peak"] > 2
                or max(b["gyro_std"]) > 0.5 or max(b["accel_std"]) > 0.06}
    guard = {s + d for s in movement for d in range(-2, 3)}
    selected = [b["second"] for b in bins if b["second"] >= ready
                and b["n"] >= 400 and b["second"] not in guard
                and 39.5 <= b["temperature_min"] and b["temperature_max"] <= 40.5]
    if len(selected) < 120:
        raise ValueError("Fewer than 120 stationary seconds after warm-up")
    return ready, sorted(movement), selected


def fit_accel(means):
    """Six-face diagonal ellipsoid: ||scale * (a-bias)|| = g.

    Uses transverse components instead of pretending every face is perfectly
    level. Does not estimate a mounting rotation or cross-axis matrix.
    """
    means = np.asarray(means)
    pairs = {}
    for v in means:
        axis = int(np.argmax(np.abs(v)))
        key = axis, int(np.sign(v[axis]))
        if key in pairs or abs(v[axis]) < 0.95 * G:
            raise ValueError("Six independent +/- XYZ gravity faces required")
        pairs[key] = v
    if len(pairs) != 6:
        raise ValueError("Missing gravity face")
    bias = np.array([(pairs[i, 1][i] + pairs[i, -1][i]) / 2 for i in range(3)])
    scale = np.array([2 * G / (pairs[i, 1][i] - pairs[i, -1][i]) for i in range(3)])
    for _ in range(20):
        delta = means - bias
        residual = np.sum((delta * scale) ** 2, axis=1) / G**2 - 1
        jac = np.column_stack((-2 * delta * scale**2 / G**2,
                               2 * delta**2 * scale / G**2))
        step = np.linalg.solve(jac, -residual)
        bias += step[:3]
        scale += step[3:]
        if np.max(np.abs(step)) < 1e-12:
            break
    else:
        raise ValueError("Accelerometer fit did not converge")
    if np.max(np.abs(bias)) > 0.5 or np.min(scale) < 0.9 or np.max(scale) > 1.1:
        raise ValueError("Implausible offset/scale; inspect six-face recording")
    return bias, scale


def calibrate(profiles):
    faces = []
    samples = []
    for name, p in profiles.items():
        ready, movement, selected = selection(p)
        source = Path(p["source"])
        if sha256(source) != p["sha256"]:
            raise ValueError("Source CSV changed since profiling: " + str(source))
        frame, current_counts = load(source)
        if current_counts != p["counts"]:
            raise ValueError("Profiling/fit source counts differ")
        seconds = np.floor(frame.time_s).astype(int)
        accepted = frame.loc[seconds.isin(selected)].copy()
        split = selected[len(selected) // 2]
        train = accepted.loc[np.floor(accepted.time_s) < split]
        held = accepted.loc[np.floor(accepted.time_s) >= split]
        a = accepted[ACCEL].to_numpy() * ACCEL_SCALE
        w = accepted[GYRO].to_numpy() * GYRO_SCALE
        face = dict(name=name, source=p["source"], sha256=p["sha256"],
                    source_schema_id=p["source_schema_id"],
                    converter_record_gaps=p["converter_record_gaps"],
                    input_counts=p["counts"], warmup_end_s=ready,
                    movement_seconds=movement, accepted_seconds=selected,
                    selected_samples=len(accepted),
                    temperature_mean_c=float(accepted.bmi088_temperature_c.mean()),
                    temperature_range_c=[float(accepted.bmi088_temperature_c.min()),
                                         float(accepted.bmi088_temperature_c.max())],
                    accel_mean_m_s2=a.mean(axis=0).tolist(),
                    gyro_mean_rad_s=w.mean(axis=0).tolist(),
                    gyro_std_deg_s=(w.std(axis=0) * 180 / np.pi).tolist(),
                    train_accel_mean_m_s2=(train[ACCEL].to_numpy()*ACCEL_SCALE).mean(axis=0).tolist(),
                    train_gyro_mean_rad_s=(train[GYRO].to_numpy()*GYRO_SCALE).mean(axis=0).tolist(),
                    held_accel_mean_m_s2=(held[ACCEL].to_numpy()*ACCEL_SCALE).mean(axis=0).tolist(),
                    held_gyro_mean_rad_s=(held[GYRO].to_numpy()*GYRO_SCALE).mean(axis=0).tolist())
        faces.append(face)
        samples.append((a, w, held[ACCEL].to_numpy()*ACCEL_SCALE))
        print("SELECT", name, "warm-up", ready, "samples", len(accepted),
              "motion seconds", movement, flush=True)
    # Equal face weights: the longer face-up recording must not dominate bias.
    gyro_bias = np.mean([f["gyro_mean_rad_s"] for f in faces], axis=0)
    accel_bias, accel_scale = fit_accel([f["accel_mean_m_s2"] for f in faces])
    train_bias, train_scale = fit_accel([f["train_accel_mean_m_s2"] for f in faces])
    train_gyro = np.mean([f["train_gyro_mean_rad_s"] for f in faces], axis=0)
    for face, (a, w, held_a) in zip(faces, samples):
        before = np.linalg.norm(a, axis=1) - G
        after = np.linalg.norm((a - accel_bias) * accel_scale, axis=1) - G
        held_before = np.linalg.norm(held_a, axis=1) - G
        held_after = np.linalg.norm((held_a - train_bias) * train_scale, axis=1) - G
        face["accel_norm_error_before_after_m_s2"] = [float(before.mean()), float(after.mean())]
        face["held_norm_error_before_after_m_s2"] = [float(held_before.mean()), float(held_after.mean())]
        face["gyro_residual_deg_s"] = ((w.mean(axis=0) - gyro_bias)*180/np.pi).tolist()
        face["held_gyro_residual_deg_s"] = ((np.array(face["held_gyro_mean_rad_s"])-train_gyro)*180/np.pi).tolist()
        face["dominant_sensor_axis"] = "XYZ"[int(np.argmax(np.abs(a.mean(axis=0))))]
        face["dominant_sign"] = int(np.sign(a.mean(axis=0)[int(np.argmax(np.abs(a.mean(axis=0))))]))
    result = dict(version=20261003, gravity_m_s2=G, reference_temperature_c=40.0,
                  gyro_bias_rad_s=gyro_bias.tolist(),
                  gyro_bias_deg_s=(gyro_bias*180/np.pi).tolist(),
                  accel_bias_m_s2=accel_bias.tolist(), accel_scale=accel_scale.tolist(),
                  method="equal-face static gyro mean; six-face diagonal gravity-norm fit",
                  selection_rules=dict(minimum_warmup_s=180, temperature_range_c=[39.5,40.5],
                                       stable_temperature_seconds=60, min_bin_samples=400,
                                       max_gyro_peak_deg_s=2, max_gyro_std_deg_s=0.5,
                                       max_accel_std_m_s2=0.06, movement_guard_seconds=2),
                  held_validation=dict(train_accel_bias_m_s2=train_bias.tolist(),
                                       train_accel_scale=train_scale.tolist(),
                                       train_gyro_bias_rad_s=train_gyro.tolist(),
                                       max_abs_accel_norm_mean_error_m_s2=max(abs(f["held_norm_error_before_after_m_s2"][1]) for f in faces),
                                       max_abs_gyro_mean_error_deg_s=float(np.max(np.abs([f["held_gyro_residual_deg_s"] for f in faces])))),
                  faces=faces,
                  limitations=["No gyro scale, temperature curve or mounting/body rotation calibration.",
                               "Static bias/scale correction does not remove sample noise or vibration.",
                               "Later halves validate within these recordings, not an independent power cycle."])
    print("CALIBRATION", json.dumps({k:v for k,v in result.items() if k not in ["faces", "limitations"]}, ensure_ascii=False), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--calibration-output", type=Path)
    parser.add_argument("--reuse-profile", action="store_true")
    args = parser.parse_args()
    paths = sorted(args.logs.glob("*/*.csv"))
    if len(paths) != 6:
        raise ValueError("Expected exactly six CSV files, one per face")
    results = (json.loads(args.output.read_text("utf-8")) if args.reuse_profile
               else {path.parent.name: profile(path) for path in paths})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    if args.calibration_output:
        result = calibrate(results)
        args.calibration_output.parent.mkdir(parents=True, exist_ok=True)
        args.calibration_output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
