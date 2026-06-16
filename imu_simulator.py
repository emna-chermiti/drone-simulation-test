import numpy as np

class IMUSimulator:
    def __init__(self):
        self.gyro_noise_std  = 0.05    
        self.accel_noise_std = 0.05    
        self.gyro_bias       = np.zeros(3)
        self.accel_bias      = np.zeros(3)
        self.bias_walk_std   = 0.0002

    def update_bias(self):
        self.gyro_bias  += np.random.randn(3) * self.bias_walk_std
        self.accel_bias += np.random.randn(3) * self.bias_walk_std
    
    def get_imu(self, obs):
        self.update_bias()
        qx, qy, qz, qw = obs[3:7]

        r00 = 1.0 - 2.0 * (qy**2 + qz**2)
        r01 = 2.0 * (qx*qy + qz*qw)
        r02 = 2.0 * (qx*qz - qy*qw)

        r10 = 2.0 * (qx*qy - qz*qw)
        r11 = 1.0 - 2.0 * (qx**2 + qz**2)
        r12 = 2.0 * (qy*qz + qx*qw)

        r20 = 2.0 * (qx*qz + qy*qw)
        r21 = 2.0 * (qy*qz - qx*qw)
        r22 = 1.0 - 2.0 * (qx**2 + qy**2)

        g_world = 9.81
        accel_true = np.array([
            r20 * g_world,
            r21 * g_world,
            r22 * g_world
        ])
        gyro_rads  = obs[0:3]
        gyro_degs  = gyro_rads * (180.0 / np.pi)
        gyro_meas  = gyro_degs + self.gyro_bias + \
                     np.random.randn(3) * self.gyro_noise_std
        accel_meas = accel_true + self.accel_bias + \
                     np.random.randn(3) * self.accel_noise_std

        return {
            "gyro":  gyro_meas.astype(np.float32),   # deg/s
            "accel": accel_meas.astype(np.float32),  # m/s²
            "quat":  obs[3:7].astype(np.float32),
            "pos":   obs[10:13].astype(np.float32),
        }