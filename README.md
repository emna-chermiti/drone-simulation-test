# Drone Simulation Test

A Python + C++ simulation of a small brushed quadrotor, used to **prototype and
debug the flight-control logic that will eventually run on the real ESP32
firmware** (the AESS-DRONE project).

The simulation uses [PyFlyt](https://pyflyt.readthedocs.io/) on top of
[PyBullet](https://pybullet.org/) as the physics engine, and a compiled
`sim_bridge_v2.dll` that reuses the **same C++ classes as the real firmware**:
sensor fusion, PID controller, and motor mixer. The only thing that changes
between sim and hardware is the `IMUData` source (synthetic obs from PyFlyt vs
real MPU-6050 over I2C).

## Why this exists

The real drone is a **35 g brushed quad** running on an ESP32-S3. Flashing
firmware to test a tuning change is slow, and bugs that look fine in C++ can
make the real drone flip on takeoff (which is both expensive and dangerous).

This repo gives a faster loop:

1. Change a gain, mixer sign, or filter coefficient in the C++ here.
2. Rebuild the DLL with one g++ command (see below).
3. Run `run_hover.py` — opens a PyBullet window showing the drone taking off,
   hovering, and landing.
4. If it looks right, copy the same `.cpp` file to the firmware project and
   flash the ESP32.

The C++ that runs in the sim is the same algorithm as the firmware; gains and
mixer signs are tuned per platform (sim drone is ~500 g, real drone is 35 g,
so the same numbers don't work for both).

## What the simulation does

A short autonomous mission:

  1. **TAKEOFF** (0-4 s) — climb from spawn altitude (~0.95 m) to target 1.5 m.
  2. **HOVER** (4-8 s) — hold 1.5 m for four seconds with attitude inside ±0.05°.
  3. **LAND** (8-12 s) — descend to ~0.05 m.

The C++ bridge receives the obs from PyFlyt each step, runs sensor fusion +
altitude PD, and outputs a thrust command in [0, 0.8] (PyFlyt's high-level
action space). PyFlyt's built-in attitude controllers handle roll, pitch, and
yaw stabilisation. The `sim_update_full` path (per-motor) is still compiled
in for hardware-in-the-loop testing, but is unstable at 40 Hz and is not used
by `run_hover.py`.

## Quick start

```bash
# from the project root, in a git-bash / MSYS shell
cd "your_project_path"

# 1. Build the C++ bridge into a DLL
/c/msys64/mingw64/bin/g++.exe -O2 -shared -fPIC -o sim_bridge_v2.dll \
    compat/arduino_stub.cpp \
    sim_bridge.cpp \
    flight/sensfusion6.cpp \
    flight/controller_pid.cpp \
    flight/pid_advanced.cpp \
    power/power_distrib.cpp \
    -I. -I./compat -I./flight -I./power -I./system

# 2. Run the mission (uses the system Python that has PyFlyt installed)
python run_hover.py
```

A PyBullet window opens showing the drone. Console output looks like:

```
 step | Phase    |   PosZ |   TgtZ |    Roll |   Pitch | TrueThrust
--------------------------------------------------------------------------
    0 | TAKEOFF  |  0.946 |  1.500 |   -0.01° |   -0.01° |     47.6%
   40 | TAKEOFF  |  1.343 |  1.500 |   -0.04° |    0.01° |     29.5%
   80 | TAKEOFF  |  1.477 |  1.500 |    0.02° |    0.00° |     29.6%
  200 | HOVER    |  1.509 |  1.500 |    0.04° |   -0.03° |     30.0%
  320 | HOVER    |  1.498 |  1.500 |   -0.01° |    0.02° |     30.1%
  440 | LAND     |  0.081 |  0.050 |   -0.02° |   -0.00° |     29.4%
```

If the rebuild step is skipped after a C++ change, the running sim uses the
old DLL — verify the timestamp with `ls -la sim_bridge_v2.dll` if behaviour
looks stale.

## File-by-file guide

### Python — the sim harness

  `drone_params.py`
    Physical parameters of the simulated drone: mass, arm length, motor lag
    time constant, max thrust, motor coefficients. The real drone uses
    `mass_kg = 0.035` (35 g) — the sim ignores this and uses PyFlyt's
    internal 500 g cf2x model. The motor lag τ is what actually matters:
    it shapes how quickly the throttle command reaches the props.

  `imu_simulator.py`
    Generates a synthetic IMU reading from PyFlyt's obs each step. Rotates
    world gravity `[0, 0, 9.81]` into the body frame using the obs
    quaternion, adds configurable bias (random walk) and Gaussian noise on
    accel and gyro. This is what the C++ IMU code "sees" in the sim, in
    place of the real MPU-6050.

  `motor_model.py`
    First-order lowpass on the commanded throttle, modelling the brushed
    motor + ESC ramp-up. The previous value `τ = 0.015 s` was below the
    sim's `dt = 0.025 s`, which made the filter unstable (it amplified
    commands instead of smoothing them). Set to `0.06 s` now.

  `run_hover.py`
    The main mission script. Three phases (TAKEOFF, HOVER, LAND) targeting
    a fixed altitude. Uses `flight_mode=0` (high-level action space) and
    calls `sim_update_highlevel` — the C++ does the altitude PD and PyFlyt
    does the attitude stabilisation. Logs pos/altitude/roll/pitch/thrust
    once per second.

  `obs_inspector.py`
    Pre-existing diagnostic that prints what PyFlyt returns in `obs` at the
    start of an episode. Useful when chasing layout changes between PyFlyt
    versions.

  `test.py`, `test_fusion.py`, `test_pid.py`, `test_pid2.py`
    Pre-existing smoke tests for the C++ pipeline. `test_pid2.py` is the
    one that runs without the sim (pure mixer sign check). The others
    spin up PyFlyt and exercise different parts of the bridge.

### C++ — shared with the firmware

These are the **same algorithms** that live in `AESS-DRONE/firmware/main/src/`.
Tuning and a few implementation details differ (see "What differs from the
real firmware" below) but the math is shared.

  `imu.h` / `imu.cpp`
    MPU-6050 driver. Wakes the chip, sets ±8 g / ±500°/s ranges, runs a
    500-sample bias calibration, burst-reads 14 bytes per call. The
    health check (`0.3 ≤ |a| ≤ 3.0 G`) catches free-fall, impact, and
    I2C failures. In the sim build, this file is bypassed — `imu_sim.h`
    injects synthetic data instead.

  `imu_sim.h`
    One-function header: pulls gyro (rad/s → deg/s) and accel (m/s² → G)
    from PyFlyt's obs and packs them into the same `IMUData` struct the
    real IMU produces. **This is the only file that knows about the PyFlyt
    obs layout.** The sim-side accel health window is widened to
    `0.05-8.0 G` to avoid false cuts during takeoff (the real IMU keeps
    the tighter `0.3-3.0 G` window).

  `flight/flight_types.h`
    Shared data shapes — `AttitudeDeg`, `AttitudeRateDps`, `Setpoint`,
    `ControlOutput`. No behaviour, just structs.

  `flight/pid_advanced.h` / `pid_advanced.cpp`
    Single-axis PID with anti-windup and output clamping. Two
    integration paths: `update(measured)` (derivative from differencing
    the error) and `updateWithRate(measured, measuredRate)` (derivative
    from the gyro directly, used for roll/pitch to avoid amplifying
    fused-angle noise).

  `flight/controller_pid.h` / `flight/controller_pid.cpp`
    Three-axis wrapper around `PidAdvanced`: one for roll angle, one for
    pitch angle, one for yaw rate. Holds the per-axis gains and limits.
    The sim's gains are `ROLL_KP=2.0, KI=0.5, KD=0.05` — gentler than
    the firmware's `4.5 / 1.5 / 0.08` because the simulated drone has
    more inertia and different motor dynamics.

  `flight/sensfusion6.h` / `flight/sensfusion6.cpp`
    Complementary filter: `α = 0.98` (98 % gyro integration, 2 % accel
    correction) for roll and pitch. Yaw is gyro-only (drifts). The
    `α = 0.98` value is correct for 500 Hz; at 40 Hz the time constant
    is 1.25 s, which is too slow — this is why the per-motor C++ path
    is unstable in the sim. The high-level path avoids this by
    deferring attitude to PyFlyt's controllers.

  `flight/stabilizer.h` / `flight/stabilizer.cpp`
    Top-level control loop for the ESP32 firmware: IMU → fusion → safety
    → PID → mix → `MotorMix`. In the sim, this class is **not** used by
    `run_hover.py` — its work is split between the C++ altitude PD (in
    `sim_bridge.cpp`) and PyFlyt's built-in attitude loop. Kept in the
    tree so the per-motor path is one rebuild away.

  `power/power_distrib.h` / `power/power_distrib.cpp`
    Quad-X mixer. Combines base thrust + three signed PID corrections
    into four motor duty cycles. Features:
      - Hard zero when `thrust = 0` (disarmed).
      - **Priority scaling**: if any motor exceeds `MOTOR_MAX=200`, all
        four are scaled down proportionally to preserve the ratios.
      - Floor at `MOTOR_MIN=30` per motor (brushed motor stall
        protection).

  `system/flight_system.h` / `system/flight_system.cpp`
    Six-phase state machine: `BOOT → READY → ARMING → HOVERING →
    DISARMING → HALTED`. Enforces a 3 s arming delay, a 55° tilt limit,
    and auto-disarm after 15 s of hover. `shouldCutPower()` is the
    safety gate the stabilizer checks before running the PID.

  `motors.h` / `motors.cpp`
    LEDC PWM driver (ESP32 hardware). 20 kHz PWM, 8-bit resolution,
    GPIO 5/6/7/8 for the four motors. Has a soft-arm ramp (`armMotors`
    ramps from 0 to `MOTOR_ARM_TARGET=40` over ~400 ms) to avoid
    inrush brown-outs. In the sim build, the LEDC calls are stubbed
    out by `compat/arduino.h`.

### C++ — the bridge

  `sim_bridge.h` / `sim_bridge.cpp`
    The DLL exposed to Python. Three entry points:
      - `sim_init()` / `sim_reset()` — set up the static
        `SensFusion6`, `ControllerPid`, and `PowerDistribution`
        instances.
      - `sim_update_fusion(in, att_out)` — run only the sensor
        fusion, return the fused attitude.
      - `sim_update_full(in, target_roll, target_pitch,
        target_yaw_rate, base_thrust, motors_out, att_out)` — per-motor
        path. Runs fusion + altitude PD + attitude PID + mixer. Outputs
        four motor duty cycles in 0-255. **Unstable at 40 Hz with the
        current gains** — kept for hardware-in-the-loop testing on a
        real MPU-6050.
      - `sim_update_highlevel(in, target_yaw_rate, roll_cmd,
        pitch_cmd, base_thrust, motors_out, att_out)` — the path
        actually used by `run_hover.py`. Runs fusion + altitude PD
        only, writes the thrust into `motors_out->m[0]` in [0, 0.8]
        for PyFlyt's high-level action space.

### C++ — Windows compatibility shims

  `compat/arduino.h` / `compat/arduino_stub.cpp` / `compat/wire.h`
    Drop-in replacements for the ESP32 Arduino framework so the firmware
    `.cpp` files compile on Windows with MinGW. `micros()`, `millis()`,
    `delay()`, `constrain()`, `Serial`, `ledcWrite()`, `ledcAttach()` are
    all stubbed. The `Serial` impl filters out `[PID]` messages to keep
    sim output readable. This is **why the same `.cpp` files build for
    both ESP32 and the sim** — `#include <Arduino.h>` resolves to the
    real core on hardware and to this stub on Windows.

### Build artifact

  `sim_bridge_v2.dll`
    The compiled output of the C++ side. `run_hover.py` loads it via
    `ctypes.CDLL`. **Must be rebuilt after every C++ change** — the
    Python loader caches the file handle and will silently run old code
    if the DLL timestamp is older than the source.

## What differs from the real firmware

The C++ in this repo is **the same algorithm** as
`AESS-DRONE/firmware/main/src/`, but tuned for the simulation. Specifically:

  - **PID gains** — sim uses `ROLL_KP=2.0, KI=0.5, KD=0.05` (gentler, for
    the 500 g cf2x model). Firmware uses `4.5 / 1.5 / 0.08` (more
    aggressive, for the 35 g real quad).
  - **Motor layout labels** — sim uses FR/BL/FL/BR (matching PyFlyt's
    cf2x prop order). Firmware uses FL/FR/RR/BL.
  - **Loop rate** — sim runs at 40 Hz (PyFlyt's default). Firmware runs
    at 250 Hz or 500 Hz.
  - **No `main.cpp` here.** The firmware's `main.cpp` ties the
    `Stabilizer`, `FlightSystem`, and `IMU` together and reads setpoints
    from a radio receiver. In the sim, `run_hover.py` is the equivalent
    of `main.cpp`.

When porting sim-tuned gains to the firmware, start with the firmware's
default gains and tune from there — do not copy the sim's numbers.

## Known issues and gotchas

  - **Per-motor C++ path is unstable at 40 Hz.** The complementary
    filter's `α = 0.98` gives a 1.25 s time constant at 40 Hz, which is
    too slow. The attitude signal lags, the PID over-corrects, and the
    drone flips within a second. Workaround: use the high-level path
    (defer attitude to PyFlyt). Real fix for the per-motor path: retune
    `α` and the PID gains for 40 Hz, or run the sim at 500 Hz
    (`agent_hz=500`).
  - **Motor `τ < dt` makes the lag filter explode.** If you ever change
    `motor_tau` below ~0.03 s, the first step will multiply the command
    by `dt/τ > 1` and amplify it. Keep `τ ≥ 0.05 s` for any sub-50 Hz
    sim.
  - **`im_sim.h` health window is wider than the firmware's.** The
    sim-side check uses `0.05-8.0 G` because synthetic accel drift can
    briefly dip below 0.3 G. The real `imu.cpp` keeps the tighter
    `0.3-3.0 G` window — don't change it on the firmware side.
  - **Hardcoded absolute path to the DLL.** `run_hover.py` line 9 has
    the DLL path baked in. If you move the project, update that line.

## License and contact

No license file is included. Add one before distributing.

Questions or changes: open an issue or contact the project owner.
