# -*- coding: utf-8 -*-
"""
Heat Controller (Tk + Matplotlib)
---------------------------------
Features:
- CSV config (targets & timings)
- PID + Relay (monoflop fallback)
- Analog Out (AO) after stability window
- Industrial Dual Analog In v2 as NOT-AUS (debounce + latch)
- Manual Patron→Surface display-offset calibration button
- Tkinter Control Panel + embedded Matplotlib plot
- Target tolerance bands are recorded historically per segment
- Dynamic Y-axis with minimum span for readability

Design notes:
- **Safety logic** (range, delta, patron-limit) runs on **RAW sensor readings** only.
- The plot shows Surface RAW and Patron **with** the *display-only* offset.
- FIRST_TARGET is validated against a safety band; if out of range, we warn and revert to default, without changing other values.
- All UI strings and messages are in **English** per your request.

🔧 FIX FOR ANALOG OUT 3.0:
- AO starts at 0V (device off)
- AO activates to 5V only when Surface temp reaches and stays stable in the target band
- AO automatically turns off after AO_ACTIVE_DURATION_SEC
"""

import time, os, csv, sys
import numpy as np
import matplotlib

# -------------------- GUI / TkAgg imports (robust) --------------------
try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
    import tkinter.font as tkfont
    matplotlib.use("TkAgg")  # backend must be set before pyplot
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
except Exception as e:
    print("[ GUI][IMPORT][ERROR] tkinter / TkAgg load failed:", e)
    tk = filedialog = messagebox = ttk = tkfont = None
    FigureCanvasTkAgg = NavigationToolbar2Tk = None

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D

# -------------------- Tinkerforge --------------------
from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_thermocouple_v2 import BrickletThermocoupleV2
from tinkerforge.bricklet_industrial_dual_relay import BrickletIndustrialDualRelay
from tinkerforge.bricklet_analog_out_v3 import BrickletAnalogOutV3
from tinkerforge.bricklet_industrial_dual_analog_in_v2 import BrickletIndustrialDualAnalogInV2  # NOT-AUS
from simple_pid import PID

# -------------------- TF UIDs / Connection --------------------
HOST, PORT = "localhost", 4223
UID_TC_SURFACE = "2f1V"
UID_TC_PATRON  = "29Me"
UID_RELAY      = "2avn"
UID_AO         = "JdS"
UID_IDAI       = "YaT"

# -------------------- CSV headers --------------------
REQUIRED_HEADERS = [
    "FIRST_TARGET","STEP_SIZE","NUM_STEPS","PATRON_LIMIT",
    "STABLE_EPSILON","STABLE_DURATION_SEC","AO_ACTIVE_DURATION_SEC","AO_VOLTAGE_MV"
]

# -------------------- Safety (global) --------------------
SAFETY_MIN_TEMP = 10.0
SAFETY_MAX_TEMP = 200.0
SAFE_DELTA_C    = 5.0    # |Surface - Patron|

# -------------------- Defaults (canonical) --------------------
DEFAULT_FIRST_TARGET = 50.0
DEFAULT_STEP_SIZE = 10.0
DEFAULT_NUM_STEPS = 5
DEFAULT_PATRON_LIMIT = 100.0
DEFAULT_STABLE_EPSILON = 1.0
DEFAULT_STABLE_DURATION_SEC = 60
DEFAULT_AO_ACTIVE_DURATION_SEC = 60
DEFAULT_AO_VOLTAGE_MV = 5000  # 5V for camera ✓

# -------------------- Runtime config (mutable) --------------------
FIRST_TARGET = DEFAULT_FIRST_TARGET
STEP_SIZE = DEFAULT_STEP_SIZE
NUM_STEPS = DEFAULT_NUM_STEPS
PATRON_LIMIT = DEFAULT_PATRON_LIMIT
STABLE_EPSILON = DEFAULT_STABLE_EPSILON
STABLE_DURATION_SEC = DEFAULT_STABLE_DURATION_SEC
AO_ACTIVE_DURATION_SEC = DEFAULT_AO_ACTIVE_DURATION_SEC
AO_VOLTAGE_MV = DEFAULT_AO_VOLTAGE_MV

# AO clamp:
AO_MIN_MV = 0
AO_MAX_MV = 12000

# ============================================================
# 🔀 کلید معکوس‌سازی AO — فقط این خط را عوض کن:
#   False → حالت عادی: صفر ولت، وقتی دما ثابت شد → 5 ولت (سیگنال)
#   True  → معکوس:     5 ولت، وقتی دما ثابت شد → صفر (سیگنال)
AO_INVERT = True
# ============================================================
# ولتاژ حالت "بیکار" و ولتاژ حالت "سیگنال" بر اساس کلید بالا:
def _ao_idle_mv():
    """ولتاژ حالت عادی (وقتی هنوز سیگنال نداده)."""
    return AO_VOLTAGE_MV if AO_INVERT else 0

def _ao_signal_mv():
    """ولتاژ حالت سیگنال (وقتی دما ثابت شد)."""
    return 0 if AO_INVERT else AO_VOLTAGE_MV

# PID / Relay:
Kp, Ki, Kd = 5.0, 0.3, 1.0
MONOFLOP_DURATION_MS = 2000

# Plot / History:
SENSOR_VALID_MIN = -50.0
SENSOR_VALID_MAX = 200.0
USE_FIXED_YLIM = False
FIXED_YLIM = (20, 120)
SHOW_FULL_HISTORY = True
WINDOW_SEC = 120

YCFG = {
    "window_sec": 60,
    "margin_low":  3.0,
    "margin_high": 3.0,
    "ema_alpha":   0.25,
    "change_max":  0.30,
    "min_step":    2.0,
    "use_percentiles": True,
    "p_low":  5,
    "p_high": 95,
    "min_span": 20.0,
}

# NOT-AUS:
NOTAUS_ENABLED = True
NOTAUS_CHANNEL = 0
# Typical NC wiring scenario → invert logic, trip on "above threshold" then inverted
NOTAUS_MODE = "above"
NOTAUS_THRESHOLD_MV = 1000
NOTAUS_DEBOUNCE_MS = 150
NOTAUS_INVERT_LOGIC = True
NOTAUS_LATCH = True

# CSV path dialog:
CSV_PATH_DEFAULT = "config.csv"
USE_TK_DIALOG = True

# -------------------- Easy color-theming for side buttons --------------------
BUTTON_STYLE = {
    "start": dict(bg="#2ecc71", fg="white", activebackground="#27ae60", activeforeground="white"),
    "stop":  dict(bg="#e74c3c", fg="white", activebackground="#c0392b", activeforeground="white"),
    "reset": dict(bg="#3498db", fg="white", activebackground="#2980b9", activeforeground="white"),
    "file":  dict(bg="#bdc3c7", fg="#2c3e50", activebackground="#95a5a6", activeforeground="white"),
    "calib": dict(bg="#1abc9c", fg="white", activebackground="#16a085", activeforeground="white"),
    "relay": dict(bg="#2ecc71", fg="white", activebackground="#27ae60", activeforeground="white"),
    "exit":  dict(bg="#2c3e50", fg="white", activebackground="#1c2833", activeforeground="white"),
}

# -------------------- Startup notice --------------------
def show_startup_notice_and_maybe_prompt_csv():
    """
    Show a single-OK startup info, then ALWAYS open a file chooser.
    If the user cancels the chooser, we continue with safe defaults (return None).
    """
    if messagebox is None or filedialog is None:
        print("[STARTUP] GUI not available — cannot show upload prompt; defaults will be used if CSV is missing.")
        return None

    # 1) Info box with only OK
    root = tk.Tk()
    root.withdraw()
    msg = (
        "Welcome to Heat Controller!\n\n"
        "Please load your configuration CSV now.\n\n"
        "Click OK to choose a CSV file.\n"
        "If you cancel the file chooser, the system will continue with safe defaults."
    )
    try:
        messagebox.showinfo("Load CSV", msg, parent=root)
    except Exception:
        pass
    finally:
        try: root.destroy()
        except: pass

    # 2) Immediately open the file chooser
    try:
        root2 = tk.Tk(); root2.withdraw()
        sel = filedialog.askopenfilename(
            title="Select CSV config",
            filetypes=[("CSV files","*.csv"), ("All files","*.*")]
        )
        root2.destroy()
        if sel:
            print(f"[CONFIG] Using CSV: {sel}")
            return sel
        else:
            print("[CONFIG] CSV selection cancelled — defaults will be used.")
            return None
    except Exception as e:
        print("[CONFIG] File dialog error:", e)
        return None

# -------------------- Helpers --------------------
def _ask_csv_path():
    if os.path.isfile(CSV_PATH_DEFAULT):
        print(f"[CONFIG] Using CSV: {CSV_PATH_DEFAULT}")
        return CSV_PATH_DEFAULT
    if USE_TK_DIALOG and filedialog is not None:
        try:
            _root = tk.Tk(); _root.withdraw()
            sel = filedialog.askopenfilename(
                title="Select CSV config",
                filetypes=[("CSV files","*.csv"), ("All files","*.*")]
            )
            _root.destroy()
            if sel:
                print(f"[CONFIG] Using CSV: {sel}")
                return sel
        except Exception as e:
            print("[CONFIG] File dialog error:", e)
    print("[CONFIG] No CSV selected — defaults will be used.")
    return None

def _float_from_str(s, default):
    try:
        return float(str(s).replace(',', '.').strip())
    except Exception:
        return default

def _int_from_str(s, default):
    try:
        return int(float(str(s).replace(',', '.').strip()))
    except Exception:
        return default

def _safe(fn):
    try:
        return fn()
    except Exception:
        return None

def _sanitize_reading(val):
    """Returns (value, ok) where `ok` means finite and within sensor sanity limits."""
    if SENSOR_VALID_MIN <= val <= SENSOR_VALID_MAX and np.isfinite(val):
        return val, True
    return np.nan, False

# -------------------- CSV loader with FIRST_TARGET safety guard --------------------
def _load_config_from_csv(path):
    """
    Load a single-row config CSV, validating FIRST_TARGET against safety band.
    If FIRST_TARGET is outside [SAFETY_MIN_TEMP, SAFETY_MAX_TEMP], we warn and revert
    FIRST_TARGET to DEFAULT_FIRST_TARGET (others are still loaded).
    """
    global FIRST_TARGET, STEP_SIZE, NUM_STEPS, PATRON_LIMIT
    global STABLE_EPSILON, STABLE_DURATION_SEC, AO_ACTIVE_DURATION_SEC, AO_VOLTAGE_MV

    if not path:
        return

    try:
        with open(path, "r", encoding="utf-8-sig", newline="") as f:
            sample = f.read(2048); f.seek(0)
            try:
                dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
            except Exception:
                class _D: delimiter = ','  # fallback
                dialect = _D()
            reader = csv.DictReader(f, dialect=dialect)
            hdr = [(h or "").strip() for h in (reader.fieldnames or [])]
            norm = [h.upper().strip() for h in hdr]
            miss = [h for h in REQUIRED_HEADERS if h not in norm]
            if miss:
                raise ValueError("Missing columns: " + ", ".join(miss))

            row_raw = next((r for r in reader if any((v or "").strip() for v in r.values())), None)
            if row_raw is None:
                raise ValueError("CSV has no data rows.")

            row = { (k or "").strip().upper(): (v or "").strip() for k, v in row_raw.items() }

            def to_float(s, d):
                try:
                    s = s.replace(",", ".").strip()
                    return float(s) if s != "" else d
                except Exception:
                    return d

            def to_int(s, d):
                try:
                    s = s.replace(",", ".").strip()
                    return int(float(s)) if s != "" else d
                except Exception:
                    return d

            # FIRST_TARGET with safety guard:
            first_candidate = to_float(row.get("FIRST_TARGET", ""), DEFAULT_FIRST_TARGET)
            if not (SAFETY_MIN_TEMP <= first_candidate <= SAFETY_MAX_TEMP):
                print(f"[CONFIG][WARN] CSV FIRST_TARGET={first_candidate}°C is outside the safe range "
                      f"({SAFETY_MIN_TEMP}…{SAFETY_MAX_TEMP}). Reverting to default {DEFAULT_FIRST_TARGET}°C.")
                FIRST_TARGET = DEFAULT_FIRST_TARGET
                if messagebox:
                    _safe(lambda: messagebox.showwarning(
                        "Safety Limit",
                        f"CSV FIRST_TARGET={first_candidate}°C is outside the safe range "
                        f"({SAFETY_MIN_TEMP}…{SAFETY_MAX_TEMP}).\n"
                        f"Reverted to default {DEFAULT_FIRST_TARGET}°C."
                    ))
            else:
                FIRST_TARGET = first_candidate

            STEP_SIZE    = to_float(row.get("STEP_SIZE",""), STEP_SIZE)
            NUM_STEPS    = max(1, to_int(row.get("NUM_STEPS",""), NUM_STEPS))
            PATRON_LIMIT = to_float(row.get("PATRON_LIMIT",""), PATRON_LIMIT)
            STABLE_EPSILON         = to_float(row.get("STABLE_EPSILON",""), STABLE_EPSILON)
            STABLE_DURATION_SEC    = max(1, to_int(row.get("STABLE_DURATION_SEC",""), STABLE_DURATION_SEC))
            AO_ACTIVE_DURATION_SEC = max(1, to_int(row.get("AO_ACTIVE_DURATION_SEC",""), AO_ACTIVE_DURATION_SEC))
            AO_VOLTAGE_MV          = max(AO_MIN_MV, min(AO_MAX_MV, to_int(row.get("AO_VOLTAGE_MV",""), AO_VOLTAGE_MV)))

            print("[CONFIG] Loaded from CSV.")

    except Exception as e:
        print("[CONFIG][ERROR]", e)
        if messagebox:
            _safe(lambda: messagebox.showerror(
                "CSV config error",
                f"{e}\n\nDefaults will be used."
            ))

# -------------------- Init: CSV + targets --------------------
csv_from_prompt = show_startup_notice_and_maybe_prompt_csv()
if csv_from_prompt:
    _load_config_from_csv(csv_from_prompt)
else:
    _load_config_from_csv(_ask_csv_path())

TARGET_STEPS = [FIRST_TARGET + i*STEP_SIZE for i in range(max(1, NUM_STEPS))]
current_step_index = 0
print("[CONFIG] TARGET_STEPS:", TARGET_STEPS)

# -------------------- Tinkerforge device objects --------------------
ipcon = IPConnection()
tc_surface = BrickletThermocoupleV2(UID_TC_SURFACE, ipcon)
tc_patron  = BrickletThermocoupleV2(UID_TC_PATRON,  ipcon)
relay      = BrickletIndustrialDualRelay(UID_RELAY, ipcon)
ao         = BrickletAnalogOutV3(UID_AO, ipcon)
idai       = BrickletIndustrialDualAnalogInV2(UID_IDAI, ipcon)

def safe_connect(ipcon_obj, retries=5, delay=1.0):
    for i in range(retries):
        try:
            ipcon_obj.connect(HOST, PORT)
            print("[TF] Connected.")
            return True
        except Exception as e:
            print(f"[TF] Connect failed ({i+1}/{retries}): {e}")
            time.sleep(delay)
    print("[TF] Will retry in update loop.")
    return False

safe_connect(ipcon)
_safe(lambda: ao.set_output_voltage(_ao_idle_mv()))  # ✓ شروع در حالت عادی (بسته به AO_INVERT)

pid = PID(Kp=Kp, Ki=Ki, Kd=Kd, setpoint=TARGET_STEPS[current_step_index])
pid.output_limits = (0, 1)

# -------------------- Runtime state --------------------
temps_surface, temps_patron, times = [], [], []
relay_states, ao_states = [], []
start_time = time.time()

stable_start_for_ao = None
ao_on_start = None
ao_active = False

relay_state = False
pid_paused  = True
controller_running = False

# NOT-AUS runtime:
notaus_latched = False
notaus_last_state = False
notaus_last_change_ts = 0.0
last_idai_read_mv = None

# Tolerance segments (historical dashed bands)
target_segments = []

# Patron display-offset calibration state:
patron_offset = 0.0         # display-only offset
baseline_temp = None        # Surface at calibration time
baseline_time_hms = None    # "HH:MM:SS"
last_raw_surface = None
last_raw_patron  = None

def _current_tolerance_values():
    sp = TARGET_STEPS[current_step_index]
    return sp - STABLE_EPSILON, sp + STABLE_EPSILON

def _start_new_segment(now_sec):
    low, high = _current_tolerance_values()
    target_segments.append({'start': now_sec, 'end': None, 'low': low, 'high': high})

def _close_current_segment(now_sec):
    if target_segments and target_segments[-1]['end'] is None:
        target_segments[-1]['end'] = now_sec

def _mark_target_change(now_sec):
    _close_current_segment(now_sec)
    _start_new_segment(now_sec)

# -------------------- Figure --------------------
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.21, right=0.96, top=0.95, bottom=0.15)

(line_surface,) = ax.plot([], [], 'r-', linewidth=2.2, label="Surface (raw)")
(line_patron,)  = ax.plot([], [], 'b-', linewidth=2.2, label="Patron (display)")
(line_low,)     = ax.plot([], [], '--', linewidth=1.2, color='g',      label='Tol Low')
(line_high,)    = ax.plot([], [], '--', linewidth=1.2, color='orange', label='Tol High')

ax.set_xlabel("Time (s)")
ax.set_ylabel("Temperature (°C)")

temp_legend = ax.legend(
    handles=[line_surface, line_patron, line_low, line_high],
    loc='upper left', framealpha=0.9, prop={'size': 8}
)
ax.add_artist(temp_legend)

# Secondary axis for Relay/AO bars:
ax_rel = ax.twinx()
ax_rel.set_ylim(0.0, 1.0)
ax_rel.set_yticks([]); ax_rel.set_ylabel("")
ax_rel.set_zorder(0); ax.set_zorder(1); ax.patch.set_visible(False)

bars_legend = ax.legend(
    handles=[
        Line2D([0], [0], color='green',  lw=6, label='Relay ON'),
        Line2D([0], [0], color='red',    lw=6, label='Relay OFF'),
        Line2D([0], [0], color='purple', lw=6, label='AO ON'),
        Line2D([0], [0], color='orange', lw=6, label='AO OFF')
    ],
    loc='upper left', bbox_to_anchor=(0.02, 0.80), framealpha=0.9, borderaxespad=0.0, prop={'size': 8}
)
ax.add_artist(bars_legend)

text_relay = ax.text(0.95, 0.98, '', transform=ax.transAxes,
                     ha='right', va='top', fontsize=12, weight='bold',
                     bbox=dict(boxstyle='round', facecolor='white', edgecolor='gray'))
text_ao = ax.text(0.95, 0.90, '', transform=ax.transAxes,
                  ha='right', va='top', fontsize=12, weight='bold',
                  bbox=dict(boxstyle='round', facecolor='white', edgecolor='gray'))
text_ao_timer = ax.text(0.50, 0.50, '', transform=ax.transAxes,
                        ha='center', va='center', fontsize=12, color='purple',
                        bbox=dict(boxstyle='round', facecolor='white', edgecolor='purple'))
text_warning = ax.text(
    0.5, 0.12, '', transform=ax.transAxes,
    ha='center', va='center', fontsize=13, color='white', weight='bold',
    bbox=dict(boxstyle='round', facecolor='red', edgecolor='darkred', alpha=0.85)
)

tol_artists = []
bar_relay = None
bar_ao = None
safety_marks = []

# -------------------- Relay / Exit helpers --------------------
def relay_set(channel, state):
    try:
        relay.set_selected_value(channel, state)  # v2
    except Exception:
        try:
            if channel == 0: relay.set_value(state, False)
            else:            relay.set_value(False, state)
        except Exception:
            pass

def relay_all_off():
    try:
        relay.set_selected_value(0, False)
        relay.set_selected_value(1, False)
    except Exception:
        _safe(lambda: relay.set_value(False, False))

def toggle_relay(event=None):
    global relay_state, pid_paused
    if not controller_running or notaus_latched:
        print("[RELAY] Ignored (not running / NOT-AUS).")
        return
    relay_state = not relay_state
    relay_set(0, relay_state)
    pid_paused = not relay_state
    print("Relay", "ON" if relay_state else "OFF", "| PID", "paused" if pid_paused else "active")

def quit_program(event=None):
    print("Exit pressed. Shutting down...")
    _safe(lambda: ani.event_source.stop())
    _safe(lambda: ao.set_output_voltage(AO_MIN_MV))
    _safe(relay_all_off)
    _safe(lambda: ipcon.disconnect())
    _safe(lambda: plt.close('all'))
    if tk is not None and tk._default_root is not None:
        try:
            tk._default_root.quit()
            tk._default_root.destroy()
        except Exception:
            pass
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(0)

# -------------------- User actions --------------------
def start_run():
    """Start RUN (no pre-check / no auto-calibration)."""
    global controller_running, pid_paused, stable_start_for_ao, ao_on_start, ao_active
    global target_segments, notaus_latched, relay_state

    if NOTAUS_ENABLED and NOTAUS_LATCH and notaus_latched:
        if messagebox:
            _safe(lambda: messagebox.showwarning("NOT-AUS latched", "NOT-AUS is latched. Press Reset first."))
        print("[NOTAUS] Latch active — press Reset before Start.")
        return

    _safe(relay_all_off); relay_state = False
    _safe(lambda: ao.set_output_voltage(AO_MIN_MV)); ao_active = False

    pid_paused = False
    controller_running = True
    stable_start_for_ao = None
    ao_on_start = None
    _safe(lambda: text_warning.set_text(""))

    now_sec = time.time() - start_time
    _close_current_segment(now_sec)   # <<< NEW: close any open segment before starting
    _start_new_segment(now_sec)       # open fresh tolerance segment

    print("[GUI] Start → RUN")

def stop_run():
    """Stop RUN and put outputs in a safe state."""
    global controller_running, relay_state, ao_active, stable_start_for_ao, ao_on_start, pid_paused
    controller_running = False
    pid_paused = True
    _safe(relay_all_off); relay_state = False
    _safe(lambda: ao.set_output_voltage(AO_MIN_MV)); ao_active = False
    stable_start_for_ao = None; ao_on_start = None
    _close_current_segment(time.time() - start_time)   # <<< NEW
    print("[GUI] Run stopped (safe outputs).")

def soft_reset():
    """Clear NOT-AUS latch, reconnect if needed, and put outputs safe."""
    global controller_running, relay_state, ao_active
    global stable_start_for_ao, ao_on_start, pid_paused, notaus_latched
    controller_running = False
    pid_paused = True
    _safe(relay_all_off); relay_state = False
    _safe(lambda: ao.set_output_voltage(AO_MIN_MV)); ao_active = False
    stable_start_for_ao = None; ao_on_start = None
    notaus_latched = False
    _safe(lambda: text_warning.set_text(""))
    _close_current_segment(time.time() - start_time)   # <<< NEW
    _safe(lambda: ipcon.get_connection_state())
    if ipcon.get_connection_state() != IPConnection.CONNECTION_STATE_CONNECTED:
        safe_connect(ipcon)
    print("[GUI] Soft reset done. (NOT-AUS latch cleared)")

def do_calibration_now():
    """
    Manual display-offset calibration (Patron → Surface).
    Reads RAW once and sets: patron_offset = Surface_raw − Patron_raw.
    - Safety logic continues to use RAW values only.
    - The plot will show Patron with (raw + patron_offset).
    """
    global patron_offset, baseline_temp, baseline_time_hms

    try: s_raw = tc_surface.get_temperature()/100.0
    except: s_raw = float('nan')
    try: p_raw = tc_patron.get_temperature()/100.0
    except: p_raw = float('nan')

    s, ok_s = _sanitize_reading(s_raw)
    p, ok_p = _sanitize_reading(p_raw)
    if not (ok_s and ok_p):
        msg = "Calibration failed:\ninvalid thermocouple reading."
        if messagebox: _safe(lambda: messagebox.showwarning("Offset", msg))
        print("[OFFSET][ERROR]", msg, f"S={s_raw}, P={p_raw}")
        return

    patron_offset = s - p
    baseline_temp = float(s)
    baseline_time_hms = time.strftime("%H:%M:%S")
    print(f"[OFFSET] Calibrated → offset={patron_offset:+.2f}°C | Surface@Calib={baseline_temp:.2f}°C @ {baseline_time_hms}")

# -------------------- GUI --------------------
GUI_ENABLED = (tk is not None) and (ttk is not None) and (FigureCanvasTkAgg is not None)
print("[GUI] Enabled:", GUI_ENABLED)

if GUI_ENABLED:
    class ControlPanel(tk.Tk):
        def __init__(self):
            super().__init__()
            self.title("Heat Controller — GUI + Plot")
            self.geometry("1340x900+40+40")
            self.minsize(1000, 650)
            self.resizable(True, True)

            ctrl = tk.Frame(self)
            plot = tk.Frame(self, bd=1, relief="sunken")
            side = tk.Frame(self)
            ctrl.grid(row=0, column=0, sticky="nsw", padx=8, pady=6)
            plot.grid(row=0, column=1, sticky="nsew", padx=(0,8), pady=6)
            side.grid(row=0, column=2, sticky="nsne", padx=8, pady=6)
            self.grid_rowconfigure(0, weight=1); self.grid_columnconfigure(1, weight=1)

            # Vars (bound to entries)
            self.v_first   = tk.StringVar(self, value=str(FIRST_TARGET))
            self.v_step    = tk.StringVar(self, value=str(STEP_SIZE))
            self.v_nsteps  = tk.StringVar(self, value=str(NUM_STEPS))
            self.v_plimit  = tk.StringVar(self, value=str(PATRON_LIMIT))
            self.v_eps     = tk.StringVar(self, value=str(STABLE_EPSILON))
            self.v_stable  = tk.StringVar(self, value=str(STABLE_DURATION_SEC))
            self.v_ao_dur  = tk.StringVar(self, value=str(AO_ACTIVE_DURATION_SEC))
            self.v_ao_mv   = tk.StringVar(self, value=str(AO_VOLTAGE_MV))
            self.v_kp      = tk.StringVar(self, value=str(Kp))
            self.v_ki      = tk.StringVar(self, value=str(Ki))
            self.v_kd      = tk.StringVar(self, value=str(Kd))
            self.v_fixylim = tk.BooleanVar(self, value=USE_FIXED_YLIM)
            self.v_ymin    = tk.StringVar(self, value=str(FIXED_YLIM[0]))
            self.v_ymax    = tk.StringVar(self, value=str(FIXED_YLIM[1]))
            self.v_monoflop= tk.StringVar(self, value=str(MONOFLOP_DURATION_MS))
            self.v_safe_delta = tk.StringVar(self, value=str(SAFE_DELTA_C))
            self.v_show_full  = tk.BooleanVar(self, value=SHOW_FULL_HISTORY)
            self.v_window_sec = tk.StringVar(self, value=str(WINDOW_SEC))

            # NOT-AUS (Enable + channel toggle)
            self.v_notaus_en  = tk.BooleanVar(self, value=NOTAUS_ENABLED)
            self.v_notaus_ch1 = tk.BooleanVar(self, value=(NOTAUS_CHANNEL == 1))

            # Live status vars
            self.v_tsurf        = tk.StringVar(self, value="--")
            self.v_tpat         = tk.StringVar(self, value="--")   # Patron (display + offset)
            self.v_target       = tk.StringVar(self, value=f"{TARGET_STEPS[current_step_index]:.1f}")
            self.v_stepidx      = tk.StringVar(self, value=f"{current_step_index+1}/{len(TARGET_STEPS)}")
            self.v_relay        = tk.StringVar(self, value="OFF")
            self.v_ao           = tk.StringVar(self, value="OFF")
            self.v_running      = tk.StringVar(self, value=("YES" if controller_running else "NO"))
            self.v_notaus_mv    = tk.StringVar(self, value="--")
            self.v_notaus_state = tk.StringVar(self, value="OK")

            # Calibration info
            self.v_baseline   = tk.StringVar(self, value="--")
            self.v_calib_time = tk.StringVar(self, value="--")
            self.v_offset     = tk.StringVar(self, value="0.0")
            self.v_delta_now  = tk.StringVar(self, value="--")

            # Controls column
            col1 = tk.LabelFrame(ctrl, text="Controls")
            col1.grid(row=0, column=0, sticky="nsew", padx=(0,8), pady=4)
            ctrl.grid_columnconfigure(0, weight=1)

            def row(parent, label, var, row_i, unit="", width=10):
                tk.Label(parent, text=label).grid(row=row_i, column=0, sticky="w", padx=6, pady=3)
                e=tk.Entry(parent, textvariable=var, width=width); e.grid(row=row_i, column=1, sticky="ew", padx=4, pady=3)
                tk.Label(parent, text=unit).grid(row=row_i, column=2, sticky="w")
                parent.grid_columnconfigure(1, weight=1); return e

            r = 0
            # --- NOT-AUS ---
            tk.Label(col1, text="NOT-AUS", font=("TkDefaultFont",10,"bold")).grid(row=r, column=0, columnspan=3, sticky="w", padx=6, pady=(4,4)); r += 1
            tk.Checkbutton(col1, text="Enable NOT-AUS", variable=self.v_notaus_en).grid(row=r, column=0, columnspan=3, sticky="w", padx=6); r += 1
            tk.Checkbutton(col1, text="Use Channel 1 (unchecked → Channel 0)", variable=self.v_notaus_ch1).grid(row=r, column=0, columnspan=3, sticky="w", padx=6, pady=(0,6)); r += 1
            ttk.Separator(col1, orient="horizontal").grid(row=r, column=0, columnspan=3, sticky="ew", padx=6, pady=(0,8)); r += 1

            # --- Target & Safety ---
            tk.Label(col1, text="Target & Safety", font=("TkDefaultFont",10,"bold")).grid(row=r, column=0, columnspan=3, sticky="w", padx=6, pady=(4,4)); r += 1
            row(col1,"FIRST_TARGET",self.v_first,r,"°C"); r+=1
            row(col1,"STEP_SIZE",self.v_step,r,"°C"); r+=1
            row(col1,"NUM_STEPS",self.v_nsteps,r,""); r+=1
            row(col1,"PATRON_LIMIT",self.v_plimit,r,"°C"); r+=1
            row(col1,"STABLE_EPSILON",self.v_eps,r,"°C"); r+=1
            row(col1,"STABLE_DURATION",self.v_stable,r,"s"); r+=1
            row(col1,"AO_ACTIVE_DURATION",self.v_ao_dur,r,"s"); r+=1
            row(col1,"AO_VOLTAGE",self.v_ao_mv,r,"mV"); r+=1
            row(col1,"SAFE_DELTA_C",self.v_safe_delta,r,"°C"); r+=1

            tk.Label(col1, text="PID", font=("TkDefaultFont",10,"bold")).grid(row=r, column=0, columnspan=3, sticky="w", padx=6, pady=(8,4)); r += 1
            row(col1,"Kp",self.v_kp,r); r+=1
            row(col1,"Ki",self.v_ki,r); r+=1
            row(col1,"Kd",self.v_kd,r); r+=1

            tk.Label(col1, text="Relay / Monoflop", font=("TkDefaultFont",10,"bold")).grid(row=r, column=0, columnspan=3, sticky="w", padx=6, pady=(8,4)); r += 1
            row(col1,"MONOFLOP_MS",self.v_monoflop,r,"ms"); r+=1

            # Action row under controls
            frm = tk.Frame(ctrl); frm.grid(row=1, column=0, sticky="ew", padx=6, pady=(6,2))
            ttk.Button(frm, text="Apply Settings", command=self.apply_settings).pack(side="left", padx=3, pady=2)
            ttk.Button(frm, text="Prev Target",    command=self.prev_target).pack(side="left", padx=3, pady=2)
            ttk.Button(frm, text="Next Target",    command=self.next_target).pack(side="left", padx=3, pady=2)

            # Side buttons
            tk.Button(side, text="Start / Run",
                      bg="green", fg="white",
                      activebackground="darkgreen", activeforeground="white",
                      relief="raised", command=start_run).pack(fill="x", pady=(4,6))

            tk.Button(side, text="Stop Run",
                      bg="red", fg="white",
                      activebackground="darkred", activeforeground="white",
                      relief="raised", command=stop_run).pack(fill="x", pady=3)

            tk.Button(side, text="Reset",
                      bg="orange", fg="black",
                      activebackground="darkorange", activeforeground="black",
                      relief="raised", command=soft_reset).pack(fill="x", pady=(10,6))

            ttk.Separator(side, orient="horizontal").pack(fill="x", pady=(0,8))

            tk.Button(side, text="Load CSV…",
                      bg="lightgray", fg="black",
                      activebackground="gray", activeforeground="white",
                      relief="raised", command=self.load_csv).pack(fill="x", pady=3)

            tk.Button(side, text="Save CSV As…",
                      bg="lightgray", fg="black",
                      activebackground="gray", activeforeground="white",
                      relief="raised", command=self.save_csv).pack(fill="x", pady=3)

            ttk.Separator(side, orient="horizontal").pack(fill="x", pady=(12,8))

            # Calibration button
            tk.Button(side, text="Calibrate Offset (Patron → Surface)",
                      bg="blue", fg="white",
                      activebackground="navy", activeforeground="white",
                      relief="raised", command=self.calibrate_patron).pack(fill="x", pady=(0,8))

            # Relay toggle + Exit
            tk.Button(side, text="Relay ON/OFF",
                      bg="brown", fg="white",
                      activebackground="darkgreen", activeforeground="white",
                      relief="raised", command=toggle_relay).pack(fill="x", pady=4)

            tk.Button(side, text="Exit",
                      bg="black", fg="white",
                      activebackground="darkred", activeforeground="white",
                      relief="raised", command=quit_program).pack(fill="x", pady=(0,8))

            # Status boxes
            status_box = tk.LabelFrame(side, text="Live Status"); status_box.pack(fill="x", pady=(0,6))
            def srow(lbl, var):
                r=tk.Frame(status_box); r.pack(fill="x", padx=6, pady=2)
                tk.Label(r, text=lbl, width=20, anchor="w").pack(side="left")
                tk.Label(r, textvariable=var, width=18, anchor="w").pack(side="left")
            srow("Surface (°C):",          self.v_tsurf)
            srow("Patron (°C):",           self.v_tpat)
            srow("Target (°C):",           self.v_target)
            srow("Step:",                  self.v_stepidx)
            srow("Relay:",                 self.v_relay)
            srow("AO:",                    self.v_ao)
            srow("Running:",               self.v_running)
            srow("NOT-AUS (mV):",          self.v_notaus_mv)
            srow("NOT-AUS State:",         self.v_notaus_state)
            ttk.Separator(status_box, orient="horizontal").pack(fill="x", pady=(4,4))
            srow("Surface@Calib (°C):",    self.v_baseline)
            srow("Calib Time:",            self.v_calib_time)
            srow("Offset vs Surface (°C):",self.v_offset)
            srow("Δ now (°C):",            self.v_delta_now)

            # Canvas
            canvas = FigureCanvasTkAgg(fig, master=plot)
            canvas.draw()
            toolbar = NavigationToolbar2Tk(canvas, plot); toolbar.update()
            toolbar.pack(side="top", fill="x")
            canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

            # Shortcuts
            self.bind("<F5>", lambda e: start_run())
            self.bind("<F6>", lambda e: stop_run())
            self.bind("<F7>", lambda e: toggle_relay())
            self.bind("<F8>", lambda e: soft_reset())
            self.bind("<F9>", lambda e: self.calibrate_patron())
            self.bind("<Escape>", lambda e: quit_program())

            self.apply_settings()
            self.after(500, self._refresh_status)
            self.protocol("WM_DELETE_WINDOW", quit_program)

        def _refresh_status(self):
            try:
                if temps_surface:
                    self.v_tsurf.set(f"{temps_surface[-1]:.1f}" if np.isfinite(temps_surface[-1]) else "--")
                if temps_patron:
                    self.v_tpat.set(f"{temps_patron[-1]:.1f}"  if np.isfinite(temps_patron[-1])  else "--")
                self.v_target.set(f"{TARGET_STEPS[current_step_index]:.1f}")
                self.v_stepidx.set(f"{current_step_index+1}/{len(TARGET_STEPS)}")
                self.v_relay.set("ON" if relay_state else "OFF")
                self.v_ao.set("ON" if ao_active else "OFF")
                self.v_running.set("YES" if controller_running else "NO")
                self.v_notaus_mv.set("--" if last_idai_read_mv is None else f"{last_idai_read_mv:.0f}")
                self.v_notaus_state.set("TRIPPED" if notaus_latched else "OK")

                # Calibration display fields
                self.v_offset.set(f"{patron_offset:+.1f}")
                self.v_baseline.set("--" if baseline_temp is None else f"{baseline_temp:.1f}")
                self.v_calib_time.set("--" if baseline_time_hms is None else baseline_time_hms)

                # Δ now = Surface_now - Patron_display_now
                if temps_surface and temps_patron and np.isfinite(temps_surface[-1]) and np.isfinite(temps_patron[-1]):
                    dnow = temps_surface[-1] - temps_patron[-1]
                    self.v_delta_now.set(f"{dnow:+.1f}")
                else:
                    self.v_delta_now.set("--")
            except Exception:
                pass
            finally:
                self.after(500, self._refresh_status)

        def apply_settings(self):
            """
            Read GUI fields into globals and rebuild target steps.
            FIRST_TARGET is validated against safety range; if invalid, show a warning
            and revert to DEFAULT_FIRST_TARGET (other values still apply).
            """
            global FIRST_TARGET, STEP_SIZE, NUM_STEPS, PATRON_LIMIT
            global STABLE_EPSILON, STABLE_DURATION_SEC, AO_ACTIVE_DURATION_SEC, AO_VOLTAGE_MV
            global Kp, Ki, Kd, USE_FIXED_YLIM, FIXED_YLIM, MONOFLOP_DURATION_MS, SAFE_DELTA_C
            global TARGET_STEPS, current_step_index, pid
            global SHOW_FULL_HISTORY, WINDOW_SEC, NOTAUS_ENABLED, NOTAUS_CHANNEL
            global target_segments

            # --- FIRST_TARGET with safety check ---
            user_first = _float_from_str(self.v_first.get(), DEFAULT_FIRST_TARGET)
            if not (SAFETY_MIN_TEMP <= user_first <= SAFETY_MAX_TEMP):
                if messagebox:
                    _safe(lambda: messagebox.showwarning(
                        "Safety Limit",
                        (f"FIRST_TARGET={user_first}°C is outside the safe range "
                         f"({SAFETY_MIN_TEMP}…{SAFETY_MAX_TEMP}).\n"
                         f"Reverted to default {DEFAULT_FIRST_TARGET}°C.")
                    ))
                FIRST_TARGET = DEFAULT_FIRST_TARGET
                self.v_first.set(str(DEFAULT_FIRST_TARGET))
            else:
                FIRST_TARGET = user_first

            # --- The rest ---
            STEP_SIZE   = _float_from_str(self.v_step.get(), STEP_SIZE)
            NUM_STEPS   = max(1, _int_from_str(self.v_nsteps.get(), NUM_STEPS))
            PATRON_LIMIT= _float_from_str(self.v_plimit.get(), PATRON_LIMIT)
            STABLE_EPSILON      = _float_from_str(self.v_eps.get(), STABLE_EPSILON)
            STABLE_DURATION_SEC = max(1, _int_from_str(self.v_stable.get(), STABLE_DURATION_SEC))
            AO_ACTIVE_DURATION_SEC = max(1, _int_from_str(self.v_ao_dur.get(), AO_ACTIVE_DURATION_SEC))
            AO_VOLTAGE_MV = max(AO_MIN_MV, min(AO_MAX_MV, _int_from_str(self.v_ao_mv.get(), AO_VOLTAGE_MV)))
            MONOFLOP_DURATION_MS = max(10, _int_from_str(self.v_monoflop.get(), MONOFLOP_DURATION_MS))
            SAFE_DELTA_C   = max(0.0, _float_from_str(self.v_safe_delta.get(), SAFE_DELTA_C))

            # PID
            Kp = _float_from_str(self.v_kp.get(), Kp)
            Ki = _float_from_str(self.v_ki.get(), Ki)
            Kd = _float_from_str(self.v_kd.get(), Kd)
            pid.tunings = (Kp, Ki, Kd)

            # Rebuild targets and ALWAYS restart from the first step
            TARGET_STEPS = [FIRST_TARGET + i*STEP_SIZE for i in range(max(1, NUM_STEPS))]
            current_step_index = 0  # start from the first target

            # reset AO / stability state because the sequence restarts
            global stable_start_for_ao, ao_on_start, ao_active
            stable_start_for_ao = None
            ao_on_start = None
            ao_active = False

            SHOW_FULL_HISTORY = bool(self.v_show_full.get())
            WINDOW_SEC = max(1, _int_from_str(self.v_window_sec.get(), WINDOW_SEC))

            USE_FIXED_YLIM = bool(self.v_fixylim.get())
            ymin = _float_from_str(self.v_ymin.get(), FIXED_YLIM[0])
            ymax = _float_from_str(self.v_ymax.get(), FIXED_YLIM[1])
            if ymin < ymax:
                FIXED_YLIM = (ymin, ymax)
            if USE_FIXED_YLIM:
                _safe(lambda: ax.set_ylim(*FIXED_YLIM)); _safe(lambda: fig.canvas.draw_idle())

            # NOT-AUS settings from GUI
            NOTAUS_ENABLED = bool(self.v_notaus_en.get())
            NOTAUS_CHANNEL = 1 if bool(self.v_notaus_ch1.get()) else 0

            # Close current tol segment & open a new one (bands may have changed)
            now_sec = time.time() - start_time
            _mark_target_change(now_sec)

            print("[APPLY]",
                  f"TARGET_STEPS={TARGET_STEPS}",
                  f"PID={pid.tunings}",
                  f"SAFE_DELTA_C={SAFE_DELTA_C}",
                  f"MONOFLOP_MS={MONOFLOP_DURATION_MS}",
                  f"NOT-AUS en={NOTAUS_ENABLED} ch={NOTAUS_CHANNEL}",
                  sep="\n")

        def next_target(self):
            global current_step_index, stable_start_for_ao, ao_on_start, ao_active
            if current_step_index < len(TARGET_STEPS)-1:
                current_step_index += 1
                stable_start_for_ao=None; ao_on_start=None; ao_active=False
                _mark_target_change(time.time()-start_time)
                print(f"[GUI] Next target: {TARGET_STEPS[current_step_index]} °C")
            else:
                print("[GUI] Last target reached.")

        def prev_target(self):
            global current_step_index, stable_start_for_ao, ao_on_start, ao_active
            if current_step_index > 0:
                current_step_index -= 1
                stable_start_for_ao=None; ao_on_start=None; ao_active=False
                _mark_target_change(time.time()-start_time)
                print(f"[GUI] Prev target: {TARGET_STEPS[current_step_index]} °C")
            else:
                print("[GUI] Already at first target.")

        def load_csv(self):
            try:
                path = filedialog.askopenfilename(
                    title="Load CSV",
                    filetypes=[("CSV files","*.csv"), ("All files","*.*")]
                )
                if not path:
                    return
                _load_config_from_csv(path)
                # Push loaded values into fields
                self.v_first.set(str(FIRST_TARGET)); self.v_step.set(str(STEP_SIZE)); self.v_nsteps.set(str(NUM_STEPS))
                self.v_plimit.set(str(PATRON_LIMIT)); self.v_eps.set(str(STABLE_EPSILON))
                self.v_stable.set(str(STABLE_DURATION_SEC)); self.v_ao_dur.set(str(AO_ACTIVE_DURATION_SEC))
                self.v_ao_mv.set(str(AO_VOLTAGE_MV))
                print(f"[GUI] Loaded CSV: {path}")
                self.apply_settings()
            except Exception as e:
                if messagebox:
                    _safe(lambda: messagebox.showerror("Load CSV", f"Failed: {e}"))
                print("[GUI][ERROR] Load CSV:", e)

        def save_csv(self):
            try:
                path = filedialog.asksaveasfilename(
                    defaultextension=".csv",
                    title="Save CSV as…",
                    filetypes=[("CSV files","*.csv"), ("All files","*.*")]
                )
            except Exception:
                path = None
            if not path:
                return
            self.apply_settings()
            try:
                with open(path, "w", encoding="utf-8", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(REQUIRED_HEADERS)
                    w.writerow([
                        FIRST_TARGET, STEP_SIZE, NUM_STEPS, PATRON_LIMIT,
                        STABLE_EPSILON, STABLE_DURATION_SEC, AO_ACTIVE_DURATION_SEC, AO_VOLTAGE_MV
                    ])
                print(f"[GUI] Saved CSV: {path}")
            except Exception as e:
                if messagebox:
                    _safe(lambda: messagebox.showerror("Save CSV", f"Failed: {e}"))
                print("[GUI][ERROR] Save CSV:", e)

        def calibrate_patron(self):
            do_calibration_now()

    panel = ControlPanel()

# -------------------- Dynamic y-limit helper --------------------
class YLimiter:
    def __init__(self, ax, ycfg):
        self.ax=ax; self.cfg=ycfg; self.prev_vmin=None; self.prev_vmax=None
    def _clip_change(self, new_min, new_max):
        if self.prev_vmin is None or self.prev_vmax is None: return new_min, new_max
        span=max(1e-6, self.prev_vmax-self.prev_vmin)
        max_delta=max(self.cfg["min_step"], self.cfg["change_max"]*span)
        if new_min > self.prev_vmin + max_delta: new_min = self.prev_vmin + max_delta
        elif new_min < self.prev_vmin - max_delta: new_min = self.prev_vmin - max_delta
        if new_max > self.prev_vmax + max_delta: new_max = self.prev_vmax + max_delta
        elif new_max < self.prev_vmax - max_delta: new_max = self.prev_vmax - max_delta
        if new_max <= new_min: new_max = new_min + max(1.0, 0.1*span)
        return new_min, new_max
    def update_ylim(self, t, s, p):
        if USE_FIXED_YLIM: return
        s=np.array(s,dtype=float); p=np.array(p,dtype=float)
        v=np.concatenate([s[np.isfinite(s)], p[np.isfinite(p)]], dtype=float)
        if v.size==0: return
        if len(t)>=2 and not SHOW_FULL_HISTORY:
            t0=t[-1]-YCFG["window_sec"]; i0=0
            for k in range(len(t)-1,-1,-1):
                if t[k]<t0: i0=k+1; break
            vw=np.concatenate([s[i0:][np.isfinite(s[i0:])], p[i0:][np.isfinite(p[i0:])]], dtype=float)
            if vw.size>0: v=vw
        if self.cfg["use_percentiles"] and v.size>=5:
            vmin_raw=float(np.percentile(v,self.cfg["p_low"])); vmax_raw=float(np.percentile(v,self.cfg["p_high"]))
        else:
            vmin_raw=float(np.min(v)); vmax_raw=float(np.max(v))
        if not np.isfinite(vmin_raw) or not np.isfinite(vmax_raw) or vmin_raw==vmax_raw:
            vmin_raw-=1.0; vmax_raw+=1.0
        vmin_raw-=self.cfg["margin_low"]; vmax_raw+=self.cfg["margin_high"]
        if self.prev_vmin is None:
            vmin_s=vmin_raw; vmax_s=vmax_raw
        else:
            a=self.cfg["ema_alpha"]; vmin_s=(1-a)*self.prev_vmin+a*vmin_raw; vmax_s=(1-a)*self.prev_vmax+a*vmax_raw
        span = vmax_s - vmin_s
        min_span = float(self.cfg.get("min_span", 0.0))
        if span < min_span:
            mid = 0.5*(vmin_s + vmax_s)
            vmin_s = mid - 0.5*min_span
            vmax_s = mid + 0.5*min_span
            _safe(lambda: self.ax.set_ylim(vmin_s, vmax_s))
            self.prev_vmin, self.prev_vmax = vmin_s, vmax_s
            return
        vmin_f, vmax_f = self._clip_change(vmin_s, vmax_s)
        _safe(lambda: self.ax.set_ylim(vmin_f, vmax_f))
        self.prev_vmin, self.prev_vmax = vmin_f, vmax_f

y_limiter = YLimiter(ax, YCFG)

# -------------------- Update loop --------------------
def update(frame):
    global stable_start_for_ao, ao_on_start, ao_active
    global current_step_index, relay_state, pid_paused, controller_running
    global bar_relay, bar_ao, tol_artists
    global notaus_latched, notaus_last_state, notaus_last_change_ts, last_idai_read_mv
    global last_raw_surface, last_raw_patron

    try:
        # Read sensors (RAW)
        try: raw_surface = tc_surface.get_temperature()/100.0
        except: raw_surface = float('nan')
        try: raw_patron  = tc_patron.get_temperature()/100.0
        except: raw_patron  = float('nan')

        now = time.time() - start_time

        # <<< NEW: if run is OFF but a tolerance segment is still open, close it
        if (not controller_running) and target_segments and (target_segments[-1]['end'] is None):
            _close_current_segment(now)

        # NOT-AUS read + debounce
        if NOTAUS_ENABLED:
            try:
                mv = idai.get_voltage(NOTAUS_CHANNEL); last_idai_read_mv = float(mv)
            except Exception:
                last_idai_read_mv = None
            raw_pressed = False
            if last_idai_read_mv is not None:
                raw_pressed = (last_idai_read_mv > NOTAUS_THRESHOLD_MV) if NOTAUS_MODE=="above" else (last_idai_read_mv < NOTAUS_THRESHOLD_MV)
            if NOTAUS_INVERT_LOGIC:
                raw_pressed = (not raw_pressed)
            now_ts = time.time()
            if raw_pressed != notaus_last_state:
                if (now_ts - notaus_last_change_ts)*1000.0 >= NOTAUS_DEBOUNCE_MS:
                    notaus_last_state = raw_pressed; notaus_last_change_ts = now_ts
            if notaus_last_state and not notaus_latched:
                _safe(relay_all_off); relay_state=False
                _safe(lambda: ao.set_output_voltage(AO_MIN_MV)); ao_active=False
                controller_running=False
                safety_marks.append(now)
                text_warning.set_text("NOT-AUS PRESSED\nSystem Halted.\nPress Reset to clear.")
                notaus_latched=True
                _close_current_segment(now)   # <<< NEW: end tolerance segment on trip
                print("[NOTAUS] TRIPPED")

        # Validate RAW readings
        tS, ok_s = _sanitize_reading(raw_surface)
        tP_raw, ok_p = _sanitize_reading(raw_patron)
        last_raw_surface = tS if ok_s else None
        last_raw_patron  = tP_raw if ok_p else None

        any_invalid = (not ok_s) or (not ok_p)

        if any_invalid:
            _safe(relay_all_off); relay_state=False
            _safe(lambda: ao.set_output_voltage(AO_MIN_MV)); ao_active=False
            if not ok_p and ok_s:
                text_warning.set_text("PATRON sensor invalid/disconnected\nPlease check the thermocouple.")
            elif not ok_s and ok_p:
                text_warning.set_text("SURFACE sensor invalid/disconnected\nPlease check the thermocouple.")
            else:
                text_warning.set_text("Both sensors invalid/disconnected\nPlease check all thermocouples.")
        else:
            # Safety checks (RAW)
            out_of_range = (tS < SAFETY_MIN_TEMP or tS > SAFETY_MAX_TEMP or
                            tP_raw < SAFETY_MIN_TEMP or tP_raw > SAFETY_MAX_TEMP)

            # Signed difference for message (keep absolute for trip logic)
            diff_sp = tS - tP_raw          # signed Surface - Patron
            delta   = abs(diff_sp)         # absolute for decision
            delta_trip = delta > SAFE_DELTA_C

            if out_of_range or delta_trip:
                _safe(relay_all_off); relay_state=False
                _safe(lambda: ao.set_output_voltage(AO_MIN_MV)); ao_active=False
                safety_marks.append(now)
                if delta_trip:
                    text_warning.set_text(
                        f"SAFETY TRIP (DELTA)\nSurface-Patron = {diff_sp:+.1f}°C\nLimit = ±{SAFE_DELTA_C:.1f}°C\nRelay OFF."
                    )
                else:
                    text_warning.set_text("SAFETY TRIP (RANGE)\nTemperature out of safe range.\nRelay forced OFF.")
            else:
                # Clear stale warnings related to invalid/thermocouple/safety
                txt = text_warning.get_text() if hasattr(text_warning, "get_text") else ""
                if isinstance(txt, str) and (("invalid" in txt.lower()) or ("thermocouple" in txt.lower()) or txt.startswith("SAFETY TRIP")):
                    text_warning.set_text("")

        # Patron display value = RAW + offset (even if RAW was used for safety)
        tP_disp = (tP_raw + patron_offset) if ok_p else np.nan

        # Append to plot buffers
        temps_surface.append(tS if ok_s else np.nan)
        temps_patron.append(tP_disp)
        times.append(now); relay_states.append(1 if relay_state else 0); ao_states.append(1 if ao_active else 0)

        # Window trim
        if len(times)>=2:
            if SHOW_FULL_HISTORY: i0=0
            else:
                t0=times[-1]-WINDOW_SEC; i0=0
                for k in range(len(times)-1,-1,-1):
                    if times[k]<t0: i0=k+1; break
            t_trim=times[i0:]; s_trim=temps_surface[i0:]; p_trim=temps_patron[i0:]
            r_trim=relay_states[i0:]; a_trim=ao_states[i0:]
        else:
            t_trim, s_trim, p_trim, r_trim, a_trim = times, temps_surface, temps_patron, relay_states, ao_states

        # Update lines + X/Y limits
        if len(t_trim)>=2:
            line_surface.set_data(t_trim, s_trim); line_patron.set_data(t_trim, p_trim)
            ax.set_xlim(t_trim[0], t_trim[-1]+1)
            y_limiter.update_ylim(t_trim, s_trim, p_trim)

        # Redraw tolerance bands (historical)
        if tol_artists:
            for art in tol_artists:
                _safe(art.remove)
        tol_artists.clear()
        if len(times)>=1:
            x0 = 0.0 if SHOW_FULL_HISTORY else max(0.0, times[-1]-WINDOW_SEC)
            x1 = times[-1]+1.0
        else:
            x0,x1=0.0,1.0
        seg_x_end = times[-1] if times else None
        for seg in target_segments:
            if seg_x_end is None: continue
            seg_start=seg['start']; seg_end=seg['end'] if seg['end'] is not None else seg_x_end
            xmin=max(x0,seg_start); xmax=min(x1,seg_end)
            if xmax>xmin:
                l1,=ax.plot([xmin,xmax],[seg['low'], seg['low']], linestyle='--', linewidth=1.2, color='g', alpha=0.9)
                l2,=ax.plot([xmin,xmax],[seg['high'],seg['high']],linestyle='--', linewidth=1.2, color='orange', alpha=0.9)
                tol_artists.extend([l1,l2])

        # Labels for relay/AO
        text_relay.set_text("Relay ON" if relay_state else "Relay OFF")
        text_relay.set_color("green" if relay_state else "red")
        text_ao.set_text("AO ON" if ao_active else "AO OFF")
        text_ao.set_color("purple" if ao_active else "orange")
        if ao_active and ao_on_start is not None:
            rem = AO_ACTIVE_DURATION_SEC - (time.time()-ao_on_start)
            text_ao_timer.set_text(f"AO active: {rem:.0f}s left")
        else:
            text_ao_timer.set_text("")

        # RUN control (SURFACE RAW drives PID/setpoint)
        if controller_running and (not notaus_latched) and (not any_invalid):
            SURFACE_TARGET = TARGET_STEPS[current_step_index]
            pid.setpoint = SURFACE_TARGET
            STABLE_LOW, STABLE_HIGH = _current_tolerance_values()

            # AO window: activate after Surface stays inside tolerance band for STABLE_DURATION_SEC
            # ao_active == True یعنی "سیگنال داده شده" (بسته به AO_INVERT، این یا 5V است یا 0V)
            if STABLE_LOW <= tS <= STABLE_HIGH:
                if stable_start_for_ao is None:
                    stable_start_for_ao = time.time()
                elif not ao_active and (time.time()-stable_start_for_ao >= STABLE_DURATION_SEC):
                    # دما ثابت شد → حالت سیگنال
                    sig_mv = max(AO_MIN_MV, min(int(_ao_signal_mv()), AO_MAX_MV))
                    _safe(lambda: ao.set_output_voltage(sig_mv))
                    ao_active = True
                    ao_on_start = time.time()
                    print(f"[AO] SIGNAL → {sig_mv} mV ({sig_mv/1000:.1f}V)")
            else:
                if not ao_active:
                    stable_start_for_ao=None

            # اگر از بازه خارج شد و هنوز در حالت سیگنال بود → برگرد به حالت عادی
            if ao_active and not (STABLE_LOW <= tS <= STABLE_HIGH):
                idle_mv = max(AO_MIN_MV, min(int(_ao_idle_mv()), AO_MAX_MV))
                _safe(lambda: ao.set_output_voltage(idle_mv))
                ao_active=False; ao_on_start=None; stable_start_for_ao=None
                print(f"[AO] back to idle → {idle_mv} mV (left tolerance band)")

            # بعد از مدت زمان سیگنال → برگرد به حالت عادی و برو مرحله بعد
            if ao_active and (time.time()-ao_on_start >= AO_ACTIVE_DURATION_SEC):
                idle_mv = max(AO_MIN_MV, min(int(_ao_idle_mv()), AO_MAX_MV))
                _safe(lambda: ao.set_output_voltage(idle_mv))
                ao_active=False; ao_on_start=None; stable_start_for_ao=None
                if current_step_index < len(TARGET_STEPS)-1:
                    current_step_index += 1
                    _mark_target_change(now)
                    print(f"Next target: {TARGET_STEPS[current_step_index]} °C")
                else:
                    print("All targets reached.")

            # Relay/PID control + patron limit (RAW Patron)
            if tP_raw >= PATRON_LIMIT:
                _safe(relay_all_off); relay_state=False
                print("Patron overheated → relay OFF")
            else:
                if not pid_paused:
                    pid_output = pid(tS)
                    if pid_output >= 0.5:
                        try:
                            relay.set_monoflop(0, True, MONOFLOP_DURATION_MS)
                            relay_state=True
                        except Exception as e:
                            relay_set(0, True); relay_state=True
                            print("[RELAY][WARN] set_monoflop failed, fallback:", e)
                    else:
                        relay_state=False
                        relay_set(0, False)
                else:
                    relay_set(0, relay_state)

        # Bars repaint (fast & simple)
        global bar_relay, bar_ao
        if bar_relay is not None:
            for p in list(bar_relay.patches): _safe(p.remove)
        if bar_ao is not None:
            for p in list(bar_ao.patches): _safe(p.remove)
        if len(t_trim)>=1:
            x=np.array(t_trim); w=0.8*np.median(np.diff(x)) if len(x)>=2 else 0.8
            colors_relay=['green' if s==1 else 'red' for s in r_trim]
            colors_ao=['purple' if s==1 else 'orange' for s in a_trim]
            bar_relay=ax_rel.bar(x,[0.05]*len(x),width=w,bottom=0.02,align='center',edgecolor='none',alpha=0.9)
            for i,bp in enumerate(bar_relay): bp.set_color(colors_relay[i])
            bar_ao=ax_rel.bar(x,[0.05]*len(x),width=w,bottom=0.08,align='center',edgecolor='none',alpha=0.9)
            for i,bp in enumerate(bar_ao): bp.set_color(colors_ao[i])
            ax_rel.set_xlim(ax.get_xlim())

    except Exception as e:
        print("Update loop error:", e)
        _safe(relay_all_off)
        _safe(lambda: ao.set_output_voltage(AO_MIN_MV))

# -------------------- Run --------------------
ani = animation.FuncAnimation(fig, update, interval=1000, cache_frame_data=False)
if GUI_ENABLED:
    panel.mainloop()
else:
    plt.show()
