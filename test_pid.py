# test_pid.py — Phase 3: verify PID corrections are correct
# Artificially tilts the drone and checks corrections fight the tilt
import ctypes, numpy as np, gymnasium
import PyFlyt.gym_envs
from imu_simulator import IMUSimulator

lib = ctypes.CDLL(r"C:\Users\rober\Desktop\drone-simulation-test\sim_bridge.dll")

class SimInput(ctypes.Structure):
    _fields_ = [("obs",       ctypes.c_float * 16),
                ("accel_ms2", ctypes.c_float * 3)]

class SimAttitude(ctypes.Structure):
    _fields_ = [("roll",  ctypes.c_float),
                ("pitch", ctypes.c_float),
                ("yaw",   ctypes.c_float)]

class SimMotors(ctypes.Structure):
    _fields_ = [("m", ctypes.c_float * 4)]

lib.sim_init.restype  = None; lib.sim_init.argtypes  = []
lib.sim_reset.restype = None; lib.sim_reset.argtypes = []
lib.sim_update_full.restype  = None
lib.sim_update_full.argtypes = [
    ctypes.POINTER(SimInput),
    ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_uint8,
    ctypes.POINTER(SimMotors),
    ctypes.POINTER(SimAttitude),
]
lib.sim_init()

imu_sim = IMUSimulator()
env     = gymnasium.make("PyFlyt/QuadX-Hover-v4",
                         render_mode="human", agent_hz=40)
obs, _  = env.reset()
lib.sim_reset()

sim_in = SimInput(); att = SimAttitude(); motors = SimMotors()

BASE_THRUST = 200   # your HOVER_THRUST constant

print(f"{'Step':>5} | {'Roll':>7} | {'Pitch':>7} | "
      f"{'M1(FL)':>7} | {'M2(FR)':>7} | {'M3(RR)':>7} | {'M4(RL)':>7} | Check")
print("-" * 80)

for step in range(3000):
    imu_data = imu_sim.get_imu(obs)
    for i in range(min(len(obs), 16)):
        sim_in.obs[i] = float(obs[i])
    sim_in.accel_ms2[0] = float(imu_data["accel"][0])
    sim_in.accel_ms2[1] = float(imu_data["accel"][1])
    sim_in.accel_ms2[2] = float(imu_data["accel"][2])

    lib.sim_update_full(ctypes.byref(sim_in),
                        ctypes.c_float(0.0),   # target roll  = level
                        ctypes.c_float(0.0),   # target pitch = level
                        ctypes.c_float(0.0),   # target yaw rate = 0
                        ctypes.c_uint8(BASE_THRUST),
                        ctypes.byref(motors),
                        ctypes.byref(att))

    m = list(motors.m)

    if step % 40 == 0:
        # Sanity checks on motor mixing signs
        # When rolled right (att.roll > 0): M1+M4 (left) should be > M2+M3 (right)
        left  = m[0] + m[3]
        right = m[1] + m[2]
        if att.roll > 2.0:
            check = "✅ L>R" if left > right else "❌ WRONG SIGN"
        elif att.roll < -2.0:
            check = "✅ R>L" if right > left else "❌ WRONG SIGN"
        else:
            check = "  level"

        print(f"{step:5d} | {att.roll:7.2f}° | {att.pitch:7.2f}° | "
              f"{m[0]:7.3f} | {m[1]:7.3f} | {m[2]:7.3f} | {m[3]:7.3f} | {check}")

    # Run the physics step using a correctly formatted numpy array layout
    motor_array = np.array(m, dtype=np.float32)
    obs, reward, terminated, truncated, _ = env.step(motor_array)
    
    if terminated or truncated:
        print(f"--- Environment flagged a boundary hit at step {step}, but keeping engine alive! ---")

env.close()