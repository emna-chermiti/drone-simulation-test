import ctypes
import numpy as np
import gymnasium
import PyFlyt.gym_envs
from imu_simulator import IMUSimulator
from motor_model    import MotorModel
from drone_params   import DRONE_PARAMS

lib = ctypes.CDLL(r"C:\Users\rober\Desktop\drone-simulation-test\sim_bridge_v2.dll")

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
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_uint8,
    ctypes.POINTER(SimMotors),
    ctypes.POINTER(SimAttitude),
]
lib.sim_update_highlevel.restype  = None
lib.sim_update_highlevel.argtypes = [
    ctypes.POINTER(SimInput),
    ctypes.c_float,  # target_yaw_rate
    ctypes.c_float,  # roll_cmd
    ctypes.c_float,  # pitch_cmd
    ctypes.c_uint8,  # base_thrust 
    ctypes.POINTER(SimMotors),
    ctypes.POINTER(SimAttitude),
]
lib.sim_init()

imu_sim   = IMUSimulator()
motor_lag = MotorModel(DRONE_PARAMS)

def make_env():
    return gymnasium.make(
        "PyFlyt/QuadX-Hover-v4",
        render_mode="human",
        flight_dome_size=500.0,
        agent_hz=40,
        max_duration_seconds=120,
        angle_representation="euler",
    )

def safe_close(e):
    try: e.close()
    except: pass

def fresh_start():
    e = make_env()
    # Spawn the drone on the ground. We set start_pos directly on the
    # unwrapped env so begin_reset picks it up. Y=0.05 puts the body just
    # above the floor — the cf2x drone body extends ~5cm below its origin,
    # so 0.05 is the minimum safe height.
    e.unwrapped.start_pos = np.array([[0.0, 0.05, 0.0]])
    o, _ = e.reset()
    lib.sim_reset()
    motor_lag.reset()
    return e, o


HZ = 40
MISSION_PROFILE = [
    ("TAKEOFF", 3.0,  1.2),   # climb from start (~0.95m) up to 1.2m
    ("HOVER",   1.5,  1.2),   # hold at 1.2m for 1.5 seconds
    ("LAND",    3.0,  0.05),  # descend back toward ground
]

TOTAL_STEPS = int(sum(item[1] for item in MISSION_PROFILE) * HZ)
BASE_THRUST = 92

env, obs = fresh_start()
sim_in = SimInput(); att = SimAttitude(); motors = SimMotors()

print(f"Mission Steps: {TOTAL_STEPS} ({TOTAL_STEPS/HZ}s) | Testing C++ V2 Hardware Code Loop")
print(f"{'Step':>5} | {'Phase':<8} | {'PosZ':>6} | {'TgtZ':>6} | {'Roll':>7} | {'Pitch':>7} | {'TrueThrust':>10}")
print("-" * 75)

for step in range(TOTAL_STEPS):
    current_time = step / HZ
    

    elapsed = 0.0
    phase_label = "IDLE"
    target_z = 0.0

    for name, duration, target_alt in MISSION_PROFILE:
        if current_time <= (elapsed + duration):
            phase_label = name
            target_z = target_alt
            break
        elapsed += duration

    imu_data = imu_sim.get_imu(obs)
    for i in range(min(len(obs), 16)):
        sim_in.obs[i] = float(obs[i])
        
    sim_in.obs[13] = float(target_z)
    sim_in.accel_ms2[0] = float(imu_data["accel"][0])
    sim_in.accel_ms2[1] = float(imu_data["accel"][1])
    sim_in.accel_ms2[2] = float(imu_data["accel"][2])


    lib.sim_update_highlevel(ctypes.byref(sim_in),
                             ctypes.c_float(0.0),  # yaw rate
                             ctypes.c_float(0.0),  # roll_cmd
                             ctypes.c_float(0.0),  # pitch_cmd
                             ctypes.c_uint8(BASE_THRUST),
                             ctypes.byref(motors),
                             ctypes.byref(att))

    action = np.array([0.0, 0.0, 0.0, float(motors.m[0])], dtype=np.float32)

    try:
        obs, reward, terminated, truncated, _ = env.step(action)
    except Exception:
        safe_close(env)
        env, obs = fresh_start()
        continue

    if step % HZ == 0:

        engine_load_percentage = float(motors.m[0]) * 100.0
        print(f"{step:5d} | {phase_label:<8} | {obs[11]:6.3f} | {target_z:6.3f} | "
              f"{att.roll:7.2f}° | {att.pitch:7.2f}° | {engine_load_percentage:8.1f}%")

safe_close(env)
print("\nFlight sequence complete.")