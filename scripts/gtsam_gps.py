
import gtsam
import numpy as np
from gtsam.symbol_shorthand import X, L # Pose key, Lever arm key

# --- Setup ---
pose_key = X(0)

# GPS Measurement in local ENU frame (meters)
gps_measurement_enu = gtsam.Point3(10.5, 20.2, 5.1)

# Noise model for GPS measurement (e.g., 0.5m horizontal, 1.0m vertical sigma)
gps_sigmas = np.array([0.5, 0.5, 1.0])
gps_noise_model = gtsam.noiseModel.Diagonal.Sigmas(gps_sigmas)

# --- Scenario 1: GPSFactor (Zero Lever Arm) ---
gps_factor_zero_arm = gtsam.GPSFactor(pose_key, gps_measurement_enu, gps_noise_model)
print("Created GPSFactor (zero lever arm):")
gps_factor_zero_arm.print()

# Evaluate error: Error is difference between pose translation and measurement
test_pose1 = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(10.0, 20.0, 5.0))
error1 = gps_factor_zero_arm.evaluateError(test_pose1)
print("\nGPSFactor Error:", error1) # Expected: [0.5, 0.2, 0.1]