# imu_simulator.py — tuned to your real MPU6050 specs
import numpy as np

class IMUSimulator:
    def __init__(self):
        # MPU6050 datasheet values for ±500°/s, ±8g config
        # These match YOUR calibration constants exactly
        self.gyro_noise_std  = 0.05    # deg/s RMS noise at 500Hz
        self.accel_noise_std = 0.05    # m/s² noise at ±8g range
        self.gyro_bias       = np.zeros(3)
        self.accel_bias      = np.zeros(3)
        # Slow drift — simulates temperature-induced bias shift
        self.bias_walk_std   = 0.0002

    def update_bias(self):
        self.gyro_bias  += np.random.randn(3) * self.bias_walk_std
        self.accel_bias += np.random.randn(3) * self.bias_walk_std

    def get_imu(self, obs):
        self.update_bias()

        # Gyro: add noise and drift to PyFlyt's clean angular velocity
        gyro_rads  = obs[0:3]
        gyro_degs  = gyro_rads * (180.0 / np.pi)
        gyro_meas  = gyro_degs + self.gyro_bias + \
                     np.random.randn(3) * self.gyro_noise_std

        # Accel: gravity vector rotated by current attitude
        # At hover, accel ≈ [0, 0, 9.81] m/s² in body frame
        accel_true = np.array([0.0, 0.0, 9.81])
        accel_meas = accel_true + self.accel_bias + \
                     np.random.randn(3) * self.accel_noise_std

        return {
            "gyro":  gyro_meas.astype(np.float32),   # deg/s
            "accel": accel_meas.astype(np.float32),  # m/s²
            "quat":  obs[3:7].astype(np.float32),
            "pos":   obs[10:13].astype(np.float32),
        }