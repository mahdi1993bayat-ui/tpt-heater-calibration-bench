# TPT Heater Calibration Bench

A supervisory controller for a **heated calibration plate** used to calibrate temperature-sensitive instruments against known reference temperatures. The plate is heated by **four electrical cartridge heaters** (German *Patronen*) driven through a solid-state relay, and its surface temperature is held to a target via a PID loop. The controller ramps the plate through a configurable sequence of calibration setpoints, waits for each to stabilize inside a tolerance band, then emits a timed analog trigger — used to fire an external instrument (camera, DAQ, or reference sensor) so a calibration sample is captured at each thermally-stable point.

Built in Python with a Tkinter HMI, an embedded Matplotlib live plot, and Tinkerforge I/O. Emergency-stop (NOT-AUS), sensor sanity checks, and delta-safety between two thermocouples are enforced on **raw** readings, independent of any display-side offset.

---

## Overview

**Plant:** A metal calibration plate heated by 4 × cartridge heaters wired to a single SSR channel. Two thermocouples observe the plant:

| Sensor | Location | Role |
|--------|----------|------|
| **Surface** thermocouple | Plate top surface, near the device-under-calibration | Closed-loop control variable (PID input) — the *calibrated* temperature |
| **Patron** thermocouple  | Inside/near the cartridge heaters | Independent safety witness; guards the cartridges against burnout and enforces the Surface↔Patron delta limit |

The control loop drives the cartridge bank via a **PID + relay** actuator with a monoflop fallback for fail-safe deenergization. Once the plate's surface temperature enters the target band and remains within `STABLE_EPSILON` for `STABLE_DURATION_SEC`, the controller drives an **Analog Out (0–12 V)** to the configured signal level for `AO_ACTIVE_DURATION_SEC` — this is the calibration trigger that tells the external instrument "the plate is stable at target X, capture a measurement now". When the trigger window ends the AO returns to its idle level and the controller advances to the next target in the ramp.

The whole run is configurable from a single-row CSV that the operator loads at startup.

---

## Core Features

- **Multi-step calibration ramp** — starts at `FIRST_TARGET`, steps by `STEP_SIZE` for `NUM_STEPS` targets. Each step must reach and hold stability before it is accepted.
- **Discrete PID controller** (via `simple-pid`) driving an Industrial Dual Relay Bricklet. Tuning defaults: `Kp = 5.0`, `Ki = 0.3`, `Kd = 1.0`.
- **Monoflop fail-safe on the relay** — if the host stops sending heartbeats, the relay deenergizes automatically after `MONOFLOP_DURATION_MS` (2 s default). No stuck-on heater on a host crash.
- **Stability detector** — surface temperature must stay within `±STABLE_EPSILON` of the setpoint for a continuous `STABLE_DURATION_SEC` window before the AO trigger fires.
- **Timed Analog Out trigger** with a runtime-switchable **inversion flag** (`AO_INVERT`). Two operating modes are selected by a single top-of-file constant:
  - `AO_INVERT = False` — idle 0 V, trigger pulses to `AO_VOLTAGE_MV` (e.g. 5 V).
  - `AO_INVERT = True`  — idle at `AO_VOLTAGE_MV`, trigger drops to 0 V (for active-low instruments).
- **NOT-AUS emergency stop** on Industrial Dual Analog In v2 — configurable threshold, debounce (`NOTAUS_DEBOUNCE_MS`), NC-wiring inversion (`NOTAUS_INVERT_LOGIC`), and latching behavior (`NOTAUS_LATCH`). Trips force the relay OFF and the AO to its idle level.
- **Safety on raw readings** — global sensor range check (`SAFETY_MIN_TEMP … SAFETY_MAX_TEMP`), Surface↔Patron delta limit (`SAFE_DELTA_C`), and Patron hard limit (`PATRON_LIMIT`) all operate on raw sensor values. Display-side calibration offsets are never allowed into the safety path.
- **Manual Patron→Surface offset calibration** — one-click button records the current delta as a *display-only* offset so the plot reads a consistent surface temperature after re-seating a sensor. Never affects control or safety.
- **Adaptive Y-axis** — the live plot uses an EMA-smoothed, percentile-based Y-range with a minimum span (`YCFG.min_span`) so noise never over-zooms the trace and long ramps still stay in view.
- **Per-segment tolerance bands** recorded in the historical trace, so operators can see when each target was accepted and what the acceptance window was.
- **CSV-driven configuration** — single-row CSV with 8 required columns (`FIRST_TARGET`, `STEP_SIZE`, `NUM_STEPS`, `PATRON_LIMIT`, `STABLE_EPSILON`, `STABLE_DURATION_SEC`, `AO_ACTIVE_DURATION_SEC`, `AO_VOLTAGE_MV`). Invalid values are guarded: `FIRST_TARGET` outside the safety band silently reverts to the default with a warning dialog.
- **Robust Tinkerforge connect** — retry-with-backoff around `ipcon.connect`, so brickd hiccups at startup don't kill the app.
- **Graceful shutdown** — closing the window, pressing Stop, or hitting NOT-AUS all force the relay off and the AO to its idle level.

---

## Hardware Requirements

- Metal calibration plate with **4 × electrical cartridge heaters** (parallel/series wired to a single SSR channel — total wattage sized to the plate mass and target ramp rate).
- **Solid-state relay** switching the mains-side cartridge bank, driven by the Industrial Dual Relay Bricklet.
- Tinkerforge Master Brick reachable via `brickd` at `localhost:4223` (host/port at top of source).
- **2 × Thermocouple Bricklet v2** — UIDs `2f1V` (Surface, mounted on the plate top face) and `29Me` (Patron, mounted at/inside the cartridge heaters).
- **1 × Industrial Dual Relay Bricklet** — UID `2avn`. Drives the heater SSR.
- **1 × Analog Out Bricklet v3** (0–12 V) — UID `JdS`. Calibration-trigger output to the external instrument.
- **1 × Industrial Dual Analog In v2 Bricklet** — UID `YaT`. NOT-AUS input (channel `NOTAUS_CHANNEL = 0`), typically wired to an NC emergency-stop button (`NOTAUS_INVERT_LOGIC = True`).
- External instrument to be triggered by the AO signal (calibration camera, reference thermometer, DAQ, etc.).

UIDs, PID gains, safety limits, and NOT-AUS configuration all live at the top of `src/tpt_heater_controller.py` — edit them once to match your bench.

---

## Installation

Requires Python 3.9+ and a running Tinkerforge `brickd` daemon.

```bash
git clone https://github.com/mahdi1993bayat-ui/tpt-heater-calibration-bench.git
cd tpt-heater-calibration-bench

python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux / macOS
source .venv/bin/activate

pip install -r requirements.txt
```

Install and start `brickd` from the Tinkerforge downloads page so the controller can reach the Master Brick on `localhost:4223`.

---

## Usage

```bash
python src/tpt_heater_controller.py
```

Operator workflow:

1. A welcome dialog appears — click **OK**, then pick a config CSV (e.g. `config/config.example.csv`). Cancel the picker to run with defaults.
2. The main window opens with the live plot (Surface, Patron, target band).
3. Press **Start** to begin the ramp at `FIRST_TARGET`.
4. When the Surface reading stays inside `±STABLE_EPSILON` for `STABLE_DURATION_SEC`, the AO output pulses to the trigger level for `AO_ACTIVE_DURATION_SEC` and the controller moves to the next target.
5. Use **Calibrate Patron→Surface** (any time) to record the current sensor delta as a display-only offset.
6. **Stop** deenergizes the relay and returns AO to idle. **Reset** clears history and returns to step 1. **Exit** closes the app cleanly.

If a NOT-AUS trip occurs, the run halts, the relay opens, AO returns to idle, and (if `NOTAUS_LATCH = True`) the operator must acknowledge and reset before the app will accept another Start.

---

## CSV Configuration

Single-row CSV with these columns (order not important, header names are case-insensitive):

| Column | Meaning | Default |
|--------|---------|---------|
| `FIRST_TARGET` | First target temperature (°C). Must lie in `[SAFETY_MIN_TEMP, SAFETY_MAX_TEMP]` = `[10, 200]`. | 50 |
| `STEP_SIZE` | Δ°C between successive targets in the ramp. | 10 |
| `NUM_STEPS` | Number of targets (≥ 1). | 5 |
| `PATRON_LIMIT` | Hard upper limit on the Patron sensor (°C). Exceeding it stops the run. | 100 |
| `STABLE_EPSILON` | Half-width of the acceptance band (°C). | 1.0 |
| `STABLE_DURATION_SEC` | Seconds Surface must stay in-band before AO fires. | 60 |
| `AO_ACTIVE_DURATION_SEC` | How long the AO trigger is held per target. | 60 |
| `AO_VOLTAGE_MV` | Trigger level in millivolts (clamped to `0 … 12000`). | 5000 |

An example is included at `config/config.example.csv`.

---

## Configuration Reference (source constants)

Frequently touched knobs at the top of `src/tpt_heater_controller.py`:

| Constant | Default | Meaning |
|----------|---------|---------|
| `AO_INVERT` | `True` | Invert AO polarity (see feature list) |
| `SAFETY_MIN_TEMP` / `SAFETY_MAX_TEMP` | 10 / 200 | Global sensor range gate |
| `SAFE_DELTA_C` | 5.0 | Max allowed `|Surface − Patron|` (raw) |
| `Kp, Ki, Kd` | 5.0 / 0.3 / 1.0 | PID gains |
| `MONOFLOP_DURATION_MS` | 2000 | Relay fail-safe timeout |
| `NOTAUS_ENABLED` | `True` | NOT-AUS master switch |
| `NOTAUS_MODE` / `NOTAUS_THRESHOLD_MV` | `above` / 1000 | NOT-AUS trip condition |
| `NOTAUS_DEBOUNCE_MS` | 150 | NOT-AUS debounce window |
| `NOTAUS_INVERT_LOGIC` | `True` | Invert for NC-wired e-stop button |
| `NOTAUS_LATCH` | `True` | Latch trip until manual reset |
| `YCFG` | dict | Adaptive Y-axis window, percentiles, min span |

---

## Repository Layout

```
tpt-heater-calibration-bench/
├── src/
│   └── tpt_heater_controller.py
├── config/
│   └── config.example.csv
├── docs/
├── README.md
├── requirements.txt
├── LICENSE
└── .gitignore
```

---

## License

Released under the MIT License. See `LICENSE`.
