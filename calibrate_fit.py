"""
Fits gyro (angle) and distance correction constants from logged calibration
data.

Reads calibration_log.csv (written by /calibrate/measured in main.py). Each
row is one test — a turn (test_type in {left, right, uturn}) or a forward/
reverse drive (test_type in {forward, reverse}) — pairing what the system's
sensors reported against what the user measured by hand (protractor or tape
measure).

Angle fit: measured_deg ~= k * gyro_deg + b
Since turnByAngle() integrates gyro_z linearly, k/b correct the raw gyro_z
reading itself: apply as GYRO_GAIN = k in robot_firmware.ino's readImu(). b
(a constant angle offset, not a rate) doesn't map to a bias term directly —
it mostly reflects motor-coast overshoot after stopMotors() fires, which
gain/bias can't fix. Only k is worth feeding back into firmware.

Distance fit: two candidate system estimates are compared against
measured_cm — commanded_cm (naive expected distance from FORWARD_MS timing)
and accel_distance_cm (live double-integrated accelerometer estimate).
Double integration is a known-noisy technique and is very likely to fit worse
than the simple timing-based estimate; this script reports both so you can
see by how much, rather than assuming either is better.

Usage: python calibrate_fit.py
"""

import csv
import os
import sys

import numpy as np

CALIB_LOG_PATH = os.path.join(os.path.dirname(__file__), "calibration_log.csv")
MIN_ROWS = 5


def fit_and_report(label, x, y):
    n = len(x)
    if n < MIN_ROWS:
        print(f"  [{label}] only {n} rows — need at least {MIN_ROWS}, skipping")
        return

    x = np.array(x)
    y = np.array(y)
    k, b = np.polyfit(x, y, 1)
    predicted = k * x + b
    residuals = y - predicted
    rmse = np.sqrt(np.mean(residuals ** 2))
    max_err = np.max(np.abs(residuals))

    print(f"  [{label}] fitted from {n} rows:")
    print(f"    y = {k:.4f} * x + {b:.4f}")
    print(f"    RMSE = {rmse:.2f}, max residual = {max_err:.2f}")
    if abs(k - 1.0) > 0.3:
        print(f"    WARNING: gain far from 1.0 — check for a units/logging bug")
    return k, b


def main():
    if not os.path.exists(CALIB_LOG_PATH):
        print(f"No calibration log found at {CALIB_LOG_PATH}")
        print("Log some tests via the dashboard's Calibration panel first.")
        sys.exit(1)

    turn_gyro, turn_measured = [], []
    fwd_commanded, fwd_accel, fwd_measured = [], [], []

    with open(CALIB_LOG_PATH, newline="") as f:
        for row in csv.DictReader(f):
            test_type = row.get("test_type", "")
            if test_type in ("left", "right", "uturn"):
                if row.get("gyro_deg") and row.get("measured_deg"):
                    turn_gyro.append(float(row["gyro_deg"]))
                    turn_measured.append(float(row["measured_deg"]))
            elif test_type in ("forward", "reverse"):
                if row.get("measured_cm"):
                    measured = float(row["measured_cm"])
                    if row.get("commanded_cm"):
                        fwd_commanded.append(float(row["commanded_cm"]))
                        fwd_measured.append(measured)
                    if row.get("accel_distance_cm"):
                        fwd_accel.append(float(row["accel_distance_cm"]))

    print("Angle (turns): measured_deg ~= k * gyro_deg + b")
    angle_fit = fit_and_report("gyro_deg -> measured_deg", turn_gyro, turn_measured)
    if angle_fit:
        k, _ = angle_fit
        print(f"  Paste into robot_firmware.ino: const float GYRO_GAIN = {k:.4f}f;")

    print()
    print("Distance (forward/reverse): measured_cm ~= k * x + b")
    fit_and_report("commanded_cm -> measured_cm (timing estimate)", fwd_commanded, fwd_measured)
    if len(fwd_accel) == len(fwd_measured) and fwd_accel:
        fit_and_report("accel_distance_cm -> measured_cm (accel estimate)", fwd_accel, fwd_measured)
    print("  Compare the two distance fits' RMSE — the accel-based double")
    print("  integration is expected to be noisier; if its RMSE is much worse,")
    print("  it's not worth using and the timing-based estimate is the better")
    print("  fallback signal.")


if __name__ == "__main__":
    main()
