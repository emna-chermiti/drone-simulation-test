# test_fusion.py — Phase 2: verify YOUR sensfusion6 tracks sim ground truth
import ctypes, numpy as np, gymnasium
import PyFlyt.gym_envs
from imu_simulator import IMUSimulator

lib = ctypes.CDLL(r"C:\Users\rober\Desktop\simulation exaple\sim_bridge.dll")

class SimInput(ctypes.Structure):
    _fields_ = [("obs",       ctypes.c_float * 16),
                ("accel_ms2", ctypes.c_float * 3)]

class SimAttitude(ctypes.Structure):
    _fields_ = [("roll",  ctypes.c_float),
                ("pitch", ctypes.c_float),
                ("yaw",   ctypes.c_float)]

lib.sim_init.restype           = None
lib.sim_init.argtypes          = []
lib.sim_reset.restype          = None
lib.sim_reset.argtypes         = []
lib.sim_update_fusion.restype  = None
lib.sim_update_fusion.argtypes = [ctypes.POINTER(SimInput),
                                   ctypes.POINTER(SimAttitude)]
lib.sim_init()

imu_sim = IMUSimulator()
env     = gymnasium.make("PyFlyt/QuadX-Hover-v4",
                          render_mode="human", agent_hz=40)
obs, _  = env.reset()
lib.sim_reset()

sim_in = SimInput()
att    = SimAttitude()

print(f"{'Step':>5} | {'Fus Roll':>9} | {'Fus Pitch':>10} | "
      f"{'GT Roll':>8} | {'GT Pitch':>9} | {'Diff R':>6} | {'Diff P':>6}")
print("-" * 70)

for step in range(3000):
    imu_data = imu_sim.get_imu(obs)

    # Fill SimInput from PyFlyt obs
    for i in range(min(len(obs), 16)):
        sim_in.obs[i] = float(obs[i])
    sim_in.accel_ms2[0] = float(imu_data["accel"][0])
    sim_in.accel_ms2[1] = float(imu_data["accel"][1])
    sim_in.accel_ms2[2] = float(imu_data["accel"][2])

    lib.sim_update_fusion(ctypes.byref(sim_in), ctypes.byref(att))

    if step % 40 == 0:
        # Ground truth from PyFlyt quaternion
        q = obs[3:7]  # xyzw
        # Roll
        sin_r   = 2.0*(q[3]*q[0] + q[1]*q[2])
        cos_r   = 1.0 - 2.0*(q[0]**2 + q[1]**2)
        gt_roll = np.degrees(np.arctan2(sin_r, cos_r))
        # Pitch
        sin_p    = 2.0*(q[3]*q[1] - q[2]*q[0])
        gt_pitch = np.degrees(np.arcsin(np.clip(sin_p, -1, 1)))

        dr = abs(att.roll  - gt_roll)
        dp = abs(att.pitch - gt_pitch)
        print(f"{step:5d} | {att.roll:9.2f}° | {att.pitch:10.2f}° | "
              f"{gt_roll:8.2f}° | {gt_pitch:9.2f}° | {dr:6.2f}° | {dp:6.2f}°")

    # Small random movements to stress-test fusion tracking
    action = np.clip(env.action_space.sample() * 0.15 + 0.55, 0, 1)
    obs, _, terminated, truncated, _ = env.step(action)
    if terminated or truncated:
        obs, _ = env.reset()
        lib.sim_reset()

env.close()