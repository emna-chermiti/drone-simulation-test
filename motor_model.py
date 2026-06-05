# motor_model.py
import numpy as np

class MotorModel:
    def __init__(self, params):
        self.tau           = params["motor_tau"]
        self.dt            = 1.0 / 40.0
        self.actual_speeds = np.zeros(4)

    def step(self, commanded):
        alpha = self.dt / self.tau
        self.actual_speeds += (commanded - self.actual_speeds) * alpha
        return self.actual_speeds.copy()

    def reset(self):
        self.actual_speeds = np.zeros(4)