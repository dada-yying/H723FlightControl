"""Estimate warm-up compensation from ORIGINAL pre-calibration log profiles.

Not a new sensor calibration experiment. Fits anchored temperature changes;
leave-one-recording-out checks disclose repeatability and missing cold poses.
Original CSV files and the 40 C static-calibration result are never overwritten.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

G = 9.80665
RAD = np.pi / 180


def prepare(profiles, static):
    hot = {f["name"]: f for f in static["faces"]}
    rows = []
    counts = []
    for name, profile in profiles.items():
        seconds = profile["seconds"]
        movement = [b["second"] for b in seconds if b["gyro_peak"] > 2
                    or max(b["gyro_std"]) > .5 or max(b["accel_std"]) > .06]
        guard = {s + d for s in movement for d in range(-2,3)}
        selected = [b for b in seconds if b["n"] >= 400
                    and b["second"] not in guard and b["temperature_mean"] <= 39.5]
        groups = {}
        for b in selected:
            groups.setdefault(int(np.floor(b["temperature_mean"] * 2)), []).append(b)
        face_rows = []
        for group in groups.values():
            weight = np.array([b["n"] for b in group])
            t = np.average([b["temperature_mean"] for b in group], weights=weight)
            w = np.average([b["gyro_mean"] for b in group], weights=weight, axis=0)*RAD
            a = np.average([b["accel_mean"] for b in group], weights=weight, axis=0)
            face_rows.append(dict(face=name, temperature_c=float(t), gyro=w.tolist(), accel=a.tolist(),
                                  hot_gyro=hot[name]["gyro_mean_rad_s"],
                                  hot_accel=hot[name]["accel_mean_m_s2"], seconds=[b["second"] for b in group]))
        if len(face_rows) < 10:
            raise ValueError("Insufficient warm-up bins: " + name)
        rows += face_rows
        counts.append(dict(face=name, bins=len(face_rows), seconds=sum(len(r["seconds"]) for r in face_rows),
                           minimum_temperature_c=min(r["temperature_c"] for r in face_rows)))
    return rows, counts


def weights(rows):
    faces = {r["face"] for r in rows}
    return np.array([1/sum(x["face"] == r["face"] for x in rows) for r in rows])


def least_squares(design, values, weight):
    root = np.sqrt(weight)
    return np.linalg.lstsq(design*root[:,None], values*root, rcond=None)[0]


def fit_gyro(rows, order):
    dt = np.array([r["temperature_c"] - 40 for r in rows])
    design = np.column_stack([dt**(i+1) for i in range(order)])
    response = np.array([np.array(r["gyro"]) - r["hot_gyro"] for r in rows])
    return np.array([least_squares(design, response[:,i], weights(rows)) for i in range(3)])


def gyro_delta(rows, coefficient):
    dt = np.array([r["temperature_c"] - 40 for r in rows])
    return np.column_stack([dt*c[0] + dt*dt*c[1] for c in coefficient])


def fit_accel(rows, bias):
    dt = np.array([r["temperature_c"] - 40 for r in rows])
    hot = np.array([r["hot_accel"] for r in rows])
    response = np.array([r["accel"] for r in rows]) - hot
    result = []
    for axis in range(3):
        design = np.column_stack([dt, dt*(hot[:,axis]-bias[axis])])
        result.append(least_squares(design, response[:,axis], weights(rows)))
    return np.array(result)


def accel_correct(rows, static, thermal):
    dt = np.array([r["temperature_c"] - 40 for r in rows])[:,None]
    a = np.array([r["accel"] for r in rows])
    denominator = 1 + dt*thermal[:,1]
    if np.min(denominator) < .9 or np.max(denominator) > 1.1:
        raise ValueError("Implausible accelerometer temperature sensitivity")
    return (a - static["accel_bias_m_s2"] - dt*thermal[:,0]) / denominator * static["accel_scale"]


def fit_accel_norm(rows, static, mode):
    """Static gravity norm, independent of slight placement-angle changes."""
    coef = np.zeros((3,2))
    if mode == "none":
        return coef
    mask = np.array([[mode in ("bias","both"), mode in ("sensitivity","both")]]*3)
    theta = np.array([r["temperature_c"]-40 for r in rows])
    a = np.array([r["accel"] for r in rows])
    s = np.array(static["accel_scale"])
    for _ in range(12):
        corrected = accel_correct(rows,static,coef)
        norm = np.linalg.norm(corrected,axis=1)
        denominator = 1+theta[:,None]*coef[:,1]
        columns = []
        for axis in range(3):
            columns += [-s[axis]*theta*corrected[:,axis]/(norm*denominator[:,axis]),
                        -theta*corrected[:,axis]**2/(norm*denominator[:,axis])]
        jac = np.column_stack(columns)[:,mask.flatten()]
        step = least_squares(jac,G-norm,weights(rows))
        coef[mask] += step
        if np.max(np.abs(step)) < 1e-12:
            break
    if np.max(np.abs(coef[:,0])) > .01 or np.max(np.abs(coef[:,1])) > .001:
        raise ValueError("Implausible gravity-norm thermal coefficients")
    return coef


def weighted_rms(values, weight):
    return np.sqrt(np.average(values**2, weights=weight, axis=0))


def analyse(profiles, static):
    rows, coverage = prepare(profiles, static)
    bias = np.array(static["accel_bias_m_s2"])
    gyro_bias = np.array(static["gyro_bias_rad_s"])
    w = np.array([r["gyro"] for r in rows]) - gyro_bias
    vector_accel = fit_accel(rows, bias)
    candidates = []
    for order in (1,2):
        coefficient = np.zeros((3,2))
        coefficient[:,:order] = fit_gyro(rows, order)
        held = []
        for face in coverage:
            train = [r for r in rows if r["face"] != face["face"]]
            test = [r for r in rows if r["face"] == face["face"]]
            fitted = np.zeros((3,2))
            fitted[:,:order] = fit_gyro(train, order)
            error = np.array([r["gyro"] for r in test]) - gyro_bias - gyro_delta(test, fitted)
            held.append(np.mean(error**2, axis=0))
        cv = np.sqrt(np.mean(held, axis=0))/RAD
        candidates.append(dict(order=order, coefficient=coefficient.tolist(), cv_rms_deg_s=cv.tolist()))
    # Require >5% cross-log improvement to justify adding a quadratic term.
    chosen = np.array(candidates[0]["coefficient"])
    selected_order = [1,1,1]
    for axis in range(3):
        if candidates[1]["cv_rms_deg_s"][axis] < .95*candidates[0]["cv_rms_deg_s"][axis]:
            chosen[axis] = candidates[1]["coefficient"][axis]
            selected_order[axis] = 2
    accel_candidates = []
    for mode in ("none","bias","sensitivity","both"):
        cv = []
        try:
            fitted = fit_accel_norm(rows,static,mode)
            for face in coverage:
                train = [r for r in rows if r["face"] != face["face"]]
                test = [r for r in rows if r["face"] == face["face"]]
                held_coef = fit_accel_norm(train,static,mode)
                b = np.linalg.norm(accel_correct(test,static,np.zeros((3,2))),axis=1)-G
                a = np.linalg.norm(accel_correct(test,static,held_coef),axis=1)-G
                cv.append(dict(face=face["face"], before_rms_m_s2=float(np.sqrt(np.mean(b*b))),
                               after_rms_m_s2=float(np.sqrt(np.mean(a*a)))))
            cv_rms = float(np.sqrt(np.mean([r["after_rms_m_s2"]**2 for r in cv])))
            accel_candidates.append(dict(mode=mode, coefficient=fitted.tolist(), cv=cv, cv_rms_m_s2=cv_rms))
        except ValueError as error:
            accel_candidates.append(dict(mode=mode,rejected=str(error)))
    chosen_accel = accel_candidates[0]
    for candidate in accel_candidates[1:]:
        if ("rejected" not in candidate and candidate["cv_rms_m_s2"] < .95*chosen_accel["cv_rms_m_s2"]
                and sum(r["after_rms_m_s2"] <= r["before_rms_m_s2"] for r in candidate["cv"]) >= 5):
            chosen_accel = candidate
    thermal_accel = np.array(chosen_accel["coefficient"])
    before_norm = np.linalg.norm(accel_correct(rows,static,np.zeros((3,2))),axis=1)-G
    after_norm = np.linalg.norm(accel_correct(rows,static,thermal_accel),axis=1)-G
    result = dict(version=2026100302, reference_temperature_c=40.0,
                  minimum_temperature_c=min(p["temperature_percentiles"][0] for p in profiles.values()),
                  maximum_temperature_c=max(p["temperature_percentiles"][-1] for p in profiles.values()),
                  all_six_face_minimum_temperature_c=max(p["temperature_percentiles"][0] for p in profiles.values()),
                  gyro_bias_rad_s=static["gyro_bias_rad_s"], accel_bias_m_s2=static["accel_bias_m_s2"],
                  accel_scale=static["accel_scale"], gyro_temperature_coefficients=chosen.tolist(),
                  gyro_selected_orders=selected_order, accel_temperature_coefficients=thermal_accel.tolist(),
                  gyro_candidates=candidates, coverage=coverage,
                  gyro_warmup_rms_before_after_deg_s=[(weighted_rms(w,weights(rows))/RAD).tolist(),
                         (weighted_rms(w-gyro_delta(rows,chosen),weights(rows))/RAD).tolist()],
                  accel_warmup_norm_rms_before_after_m_s2=[float(weighted_rms(before_norm,weights(rows))),
                                                          float(weighted_rms(after_norm,weights(rows)))],
                  accel_cross_log=chosen_accel["cv"], accel_selected_mode=chosen_accel["mode"],
                  accel_candidates=accel_candidates,
                  rejected_fixed_orientation_accel_coefficients=vector_accel.tolist(),
                  rejected_fixed_orientation_accel_norm_rms_m_s2=float(weighted_rms(
                       np.linalg.norm(accel_correct(rows,static,vector_accel),axis=1)-G,weights(rows))),
                  rows=rows,
                  limitations=["Temperature trajectories are heating-only; no cooling hysteresis validation.",
                               "Not all six faces cover the lowest logged temperature.",
                               "Cross-log gyro validation does not use the held-out hot bias to correct predictions.",
                               "Static reference orientation inferred at 40 C; not a calibrated motion table."])
    print(json.dumps({k:v for k,v in result.items() if k != "rows"},ensure_ascii=False,indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", type=Path)
    parser.add_argument("static_calibration", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = analyse(json.loads(args.profile.read_text("utf-8")),
                     json.loads(args.static_calibration.read_text("utf-8")))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
