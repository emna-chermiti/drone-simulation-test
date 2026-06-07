import ctypes, numpy as np, gymnasium
import PyFlyt.gym_envs
from imu_simulator import IMUSimulator
from motor_model    import MotorModel
from drone_params   import DRONE_PARAMS

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

imu_sim   = IMUSimulator()
motor_lag = MotorModel(DRONE_PARAMS)

def make_env():
    return gymnasium.make(
        "PyFlyt/QuadX-Hover-v4",
        render_mode=None,
        flight_dome_size=500.0,
        agent_hz=40,
        max_duration_seconds=120
    )

def safe_close(e):
    try: e.close()
    except: pass

def fresh_start():
    e = make_env()
    o, _ = e.reset()
    lib.sim_reset()
    motor_lag.reset()
    return e, o

HZ = 40

# Mission: (label, duration_seconds, final_target_altitude)
MISSION = [
    ("TAKEOFF",  8,  1.5),
    ("HOVER",   10,  1.5),
    ("DESCEND",  8,  0.3),
    ("LAND",     5,  0.0),
    ("IDLE",     3,  0.0),
]

# Build step list with RAMPED targets
# Takeoff and descend ramp gradually instead of jumping
mission_steps = []
for label, duration, final_alt in MISSION:
    steps = duration * HZ
    if label in ("TAKEOFF", "DESCEND", "LAND"):
        # Get previous altitude
        prev_alt = mission_steps[-1][1] if mission_steps else 0.0
        for i in range(steps):
            # Linear ramp from prev_alt to final_alt
            t = i / steps
            alt = prev_alt + (final_alt - prev_alt) * t
            mission_steps.append((label, alt))
    else:
        mission_steps.extend([(label, final_alt)] * steps)

TOTAL_STEPS = len(mission_steps)
BASE_THRUST  = 80

env, obs = fresh_start()
sim_in = SimInput(); att = SimAttitude(); motors = SimMotors()

print(f"Mission: {[(m[0], m[1], m[2]) for m in MISSION]}")
print(f"Total steps: {TOTAL_STEPS} ({TOTAL_STEPS/HZ:.0f}s) | Ramped takeoff/descend")
print(f"{'Step':>5} | {'Phase':<8} | {'PosZ':>6} | {'TgtZ':>6} | "
      f"{'Roll':>7} | {'Pitch':>7} | {'Thrust':>7}")
print("-" * 75)

prev_phase = ""

for step in range(TOTAL_STEPS):
    phase_label, target_z = mission_steps[step]

    imu_data = imu_sim.get_imu(obs)
    for i in range(min(len(obs), 16)):
        sim_in.obs[i] = float(obs[i])

    # Override target altitude with our ramped mission value
    sim_in.obs[13] = float(target_z)

    sim_in.accel_ms2[0] = float(imu_data["accel"][0])
    sim_in.accel_ms2[1] = float(imu_data["accel"][1])
    sim_in.accel_ms2[2] = float(imu_data["accel"][2])

    lib.sim_update_full(ctypes.byref(sim_in),
                        ctypes.c_float(0.0),
                        ctypes.c_float(0.0),
                        ctypes.c_float(0.0),
                        ctypes.c_uint8(BASE_THRUST),
                        ctypes.byref(motors),
                        ctypes.byref(att))

    commanded = np.array(list(motors.m), dtype=np.float32)
    actual    = motor_lag.step(commanded)
    actual    = np.clip(actual, 0.0, 1.0)

    # Cut motors when on ground during LAND/IDLE
    if phase_label in ("LAND", "IDLE") and obs[10] <= 0.05:
        actual = np.zeros(4, dtype=np.float32)

    try:
        obs, reward, terminated, truncated, _ = env.step(actual)
    except Exception:
        safe_close(env)
        env, obs = fresh_start()
        continue

    # Ignore termination during flight phases — dome is large enough
    # Only reset on actual physics crash (pos_z deeply negative)
    if terminated or truncated:
        if obs[10] < -1.0:
            print(f"  [CRASH] at step {step}, pos_z={obs[10]:.2f}")
            safe_close(env)
            env, obs = fresh_start()

    if step % HZ == 0:
        pos_z  = obs[10]
        vel_z  = obs[9]
        thrust = BASE_THRUST + 6.0*(target_z-pos_z) - 5.0*vel_z
        thrust = max(30, min(200, thrust))
        print(f"{step:5d} | {phase_label:<8} | {pos_z:6.3f} | {target_z:6.3f} | "
              f"{att.roll:7.2f}° | {att.pitch:7.2f}° | {thrust:7.1f}")

    if phase_label != prev_phase:
        print(f"  >>> Phase: {phase_label}")
        prev_phase = phase_label

safe_close(env)
print("\nMission complete.")