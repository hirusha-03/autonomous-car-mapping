"""
Fits a gyro gain/bias correction from logged turn-calibration data.

Reads calibration_log.csv (written by /calibrate/measured in main.py — each row
pairs a commanded turn angle, what the gyro measured for that turn, and the
real angle measured by hand with a protractor), and fits:

    measured_deg ~= k * gyro_deg + b

Since turnByAngle() integrates gyro_z linearly, the same k/b correct the raw
gyro_z reading itself: apply them in robot_firmware.ino's readImu() as
GYRO_GAIN = k and GYRO_BIAS_DPS such that (raw_dps - GYRO_BIAS_DPS) * GYRO_GAIN
approximates the true rate. b (a constant angle offset, not a rate) doesn't
map to a bias term directly — it mostly reflects overshoot from motor
coasting after stopMotors() fires, which gain/bias can't fix. Only k is worth
feeding back into firmware; b is reported for visibility.

Usage: python calibrate_fit.py
"""

import csv
import os
import sys

import numpy as np

CALIB_LOG_PATH = os.path.join(os.path.dirname(__file__), "calibration_log.csv")
MIN_ROWS = 5


def main():
    if not os.path.exists(CALIB_LOG_PATH):
        print(f"No calibration log found at {CALIB_LOG_PATH}")
        print("Log some turns via the dashboard's Gyro Calibration panel first.")
        sys.exit(1)

    gyro_deg = []
    measured_deg = []
    with open(CALIB_LOG_PATH, newline="") as f:
        for row in csv.DictReader(f):
            gyro_deg.append(float(row["gyro_deg"]))
            measured_deg.append(float(row["measured_deg"]))

    n = len(gyro_deg)
    if n < MIN_ROWS:
        print(f"Only {n} logged rows — need at least {MIN_ROWS} for a meaningful fit.")
        sys.exit(1)

    gyro_deg = np.array(gyro_deg)
    measured_deg = np.array(measured_deg)

    k, b = np.polyfit(gyro_deg, measured_deg, 1)

    predicted = k * gyro_deg + b
    residuals = measured_deg - predicted
    rmse = np.sqrt(np.mean(residuals ** 2))
    max_err = np.max(np.abs(residuals))

    print(f"Fitted from {n} samples:")
    print(f"  measured_deg = {k:.4f} * gyro_deg + {b:.4f}")
    print(f"  RMSE = {rmse:.2f} deg, max residual = {max_err:.2f} deg")
    print()
    print("Paste into robot_firmware.ino:")
    print(f"  const float GYRO_GAIN = {k:.4f}f;")
    print("  // note: b (constant offset) is not a rate bias — likely motor")
    print("  // coast-after-stop overshoot, not something GYRO_BIAS_DPS can fix.")
    print(f"  // fitted b = {b:.4f} deg for reference")

    if abs(k - 1.0) > 0.3:
        print()
        print("WARNING: gain is far from 1.0 — double-check the logged data")
        print("before trusting this (could indicate a units/logging bug rather")
        print("than real sensor error).")


if __name__ == "__main__":
    main()
