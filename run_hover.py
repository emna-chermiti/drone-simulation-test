# run_hover.py — Phase 4: full YOUR code pipeline, tune BASE_THRUST
import ctypes, numpy as np, gymnasium
import PyFlyt.gym_envs
from imu_simulator import IMUSimulator
from motor_model    import MotorModel
from drone_params   import DRONE_PARAMS

lib = ctypes.CDLL(r"C:\Users\rober\Desktop\simulation exaple\sim_bridge.dll")

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

env = gymnasium.make("PyFlyt/QuadX-Hover-v4",
                     render_mode="human",
                     flight_dome_size=5.0,
                     agent_hz=40)
obs, _ = env.reset()
lib.sim_reset()
motor_lag.reset()

sim_in = SimInput(); att = SimAttitude(); motors = SimMotors()

# ── Tune this value until drone lifts and hovers ───────────────────────────
# Stop increasing when it hovers around target altitude without climbing away
BASE_THRUST = 40

print(f"Running with BASE_THRUST={BASE_THRUST}")
print(f"{'Step':>5} | {'PosZ':>6} | {'TgtZ':>6} | "
      f"{'Roll':>6} | {'Pitch':>6} | {'Motors':>25}")
print("-" * 75)

for step in range(10000):
    imu_data = imu_sim.get_imu(obs)
    for i in range(min(len(obs), 16)):
        sim_in.obs[i] = float(obs[i])
    sim_in.accel_ms2[0] = float(imu_data["accel"][0])
    sim_in.accel_ms2[1] = float(imu_data["accel"][1])
    sim_in.accel_ms2[2] = float(imu_data["accel"][2])

    lib.sim_update_full(ctypes.byref(sim_in),
                        ctypes.c_float(0.0),  # hover level
                        ctypes.c_float(0.0),
                        ctypes.c_float(0.0),
                        ctypes.c_uint8(BASE_THRUST),
                        ctypes.byref(motors),
                        ctypes.byref(att))

    # ESC lag model
    commanded = np.array(list(motors.m), dtype=np.float32)
    actual    = motor_lag.step(commanded)

    obs, reward, terminated, truncated, _ = env.step(actual)

    if step % 40 == 0:
        pos_z = obs[10]; tgt_z = obs[13]
        print(f"{step:5d} | {pos_z:6.3f} | {tgt_z:6.3f} | "
              f"{att.roll:6.2f}° | {att.pitch:6.2f}° | "
              f"{np.round(actual, 2)}")

    if terminated or truncated:
        print(f"=== RESET at step {step} ===")
        obs, _ = env.reset()
        lib.sim_reset()
        motor_lag.reset()

env.close()