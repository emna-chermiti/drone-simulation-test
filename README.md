# Drone Simulation Test

A compact drone simulation and control testbed. This repository contains a small C/C++ simulation bridge, flight controllers (PID and advanced), sensor fusion, motor/power models, and Python helpers for running experiments and tests.

**Quick summary**
- **Purpose:** Experiment with controllers and sensor fusion in a lightweight simulation.
- **Output:** Builds a platform-native shared library (sim_bridge.dll) used by the Python harnesses.

**Prerequisites**
- A working `g++` toolchain (MinGW-w64 or similar on Windows, or a GNU toolchain on Linux/macOS).
- Python 3.x for the example scripts (`run_hover.py`, `test_fusion.py`, `test_pid.py`).

**Build: Create the DLL**
Run this command (works in a Unix-like shell or MSYS/MinGW environment on Windows):

```sh
g++ -O2 -shared -fPIC -o sim_bridge.dll \
	compat/arduino_stub.cpp \
	sim_bridge.cpp \
	flight/sensfusion6.cpp \
	flight/controller_pid.cpp \
	flight/pid_advanced.cpp \
	power/power_distrib.cpp \
	-I. -I./compat -I./flight -I./power -I./system
```

Notes:
- On native Windows `-fPIC` is ignored; the command is written to be portable across MinGW/Unix-like toolchains.
- If you use an IDE or Visual Studio toolchain, create a shared library / DLL project and add the same source files and include paths.

**Run examples**
- After building `sim_bridge.dll`, run the Python examples:

```sh
python run_hover.py
python test_fusion.py
python test_pid.py
```

These scripts load the DLL and drive simple simulations / unit tests. See the scripts for configurable parameters.

**File overview**
- [drone_params.py](drone_params.py) — simulation parameters and constants.
- [imu_sim.h](imu_sim.h), [imu.cpp](imu.cpp), [imu.h](imu.h) — IMU simulation and interfaces.
- [motor_model.py](motor_model.py), [motors.cpp](motors.cpp), [motors.h](motors.h) — motor models
- [sim_bridge.cpp](sim_bridge.cpp), [sim_bridge.h](sim_bridge.h) — bridge between simulator and control logic (built to `sim_bridge.dll`).
- [flight/controller_pid.cpp](flight/controller_pid.cpp), [flight/controller_pid.h](flight/controller_pid.h) — PID controller implementation.
- [flight/pid_advanced.cpp](flight/pid_advanced.cpp), [flight/pid_advanced.h](flight/pid_advanced.h) — advanced PID examples.
- [flight/sensfusion6.cpp](flight/sensfusion6.cpp), [flight/sensfusion6.h](flight/sensfusion6.h) — sensor fusion routine.
- [power/power_distrib.cpp](power/power_distrib.cpp), [power/power_distrib.h](power/power_distrib.h) — power distribution model.
- `compat/arduino_stub.cpp`, `compat/arduino.h`, `compat/wire.h` — Arduino compatibility shims used by the flight code.

**Tips & troubleshooting**
- Missing headers during link/build: ensure the `-I` include paths match the repository layout (the command above already sets them).
- If the Python scripts cannot load `sim_bridge.dll` on Windows, ensure the DLL is in the same directory as the script or on the `PATH`.
- Use a MinGW/MSYS shell when running the `g++` command on Windows to get the expected behavior for the `-shared` flag.

**Tests**
- Run `python test_fusion.py` and `python test_pid.py` to exercise the sensor fusion and PID controller logic.

**License & contact**
- This repository does not include an explicit license file. Add one if you plan to distribute the code.
- Questions or changes: open an issue or contact the repository owner.

---
Small, focused, and ready to extend — tell me if you want a CI workflow to automate the build/test steps.