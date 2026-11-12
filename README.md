# EMG/IMU tracker: velocity-based curl analysis with an ESP32 + LSM6 IMU

This repo contains two things:

1. **Firmware.** ESP32 DevKit firmware (Arduino framework, PlatformIO) that
   reads an ST LSM6-series 6-axis IMU over I2C with a small register-level
   driver and streams CSV over serial. It comes with a live plotter that
   records tests.
2. **An experiment and its analysis.** The IMU was strapped to a dumbbell for
   six sets of biceps curls taken to failure, three per arm. The analysis
   detects reps, measures lifting velocity and its decline, and tests ways to
   estimate **reps in reserve (RIR)** from velocity, including on sets stopped
   before failure.

**Main findings:** see [`analysis/REPORT.md`](analysis/REPORT.md) for the full
write-up.
- Reps are detected reliably: 9–11 per set, each set ending in a failed attempt.
- Lifting speed falls 58–82 % by the last rep, in the same three-phase shape in
  every set: plateau, gradual decline, then collapse.
- On sets stopped early, velocity alone estimates RIR to within about 0.9 reps
  on average: RIR ≈ 0.44 + 0.039 × (2-rep average speed, °/s) − 0.044 ×
  (velocity loss %).
- Inter-rep rest and more complex models (random forest, boosting) did not
  improve on this simple formula.

## Repository layout

```
platformio.ini            PlatformIO project (ESP32 DevKit)
include/imu.h             driver API + register verification table
src/imu.cpp               driver: detect, reset, configure, burst read, unit conversion
src/main.cpp              setup (detect, calibrate), CSV streaming loop
tools/live_plot.py        live plot of the six axes + test recorder
recordings/               raw IMU data from the curl experiment (CSV + overview PNG)
analysis/REPORT.md        write-up of the curl experiment
analysis/*.py             analysis scripts (see "Analysis" below)
analysis/output/          generated figures and tables
requirements.txt          Python dependencies
```

## Quick start

```sh
# firmware
pio run -t upload                     # build + flash
# Python tools
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python tools/live_plot.py   # live plot + recorder
```

# Firmware

## Wiring (fixed)

| IMU pin | ESP32        |
|---------|--------------|
| SDA     | GPIO 21      |
| SCL     | GPIO 22      |
| VIN     | 3V3          |
| GND     | GND          |
| CS      | 3V3 (I2C mode) |
| SA0     | GND -> address 0x6A |
| INT1    | GPIO 4       |

## Supported chips (detected by WHO_AM_I, reg 0x0F)

| WHO_AM_I | Part                    | Temp sensitivity |
|----------|-------------------------|------------------|
| 0x69     | LSM6DS3                 | 16 LSB/°C        |
| 0x6A     | LSM6DS3TR-C / LSM6DSL   | 256 LSB/°C       |
| 0x6C     | LSM6DSO / LSM6DSOX      | 256 LSB/°C       |

All have 0 LSB = 25 °C. Note that **LSM6DSL and LSM6DS3TR-C are 256 LSB/°C**,
not 16 like the original LSM6DS3. The register-by-register comparison is in the
header comment of `include/imu.h`.

The driver tries 0x6A, then 0x6B. If neither answers with a supported ID, it
prints an I2C bus scan and halts. The error and scan repeat every 5 s, so you
see them even if you open the monitor late.

## Configuration

- Software reset (SW_RESET, polled until it clears), then BOOT (trim reload)
- CTRL3_C: BDU + IF_INC
- Accel 104 Hz, ±4 g (0.122 mg/LSB); gyro 104 Hz, ±500 dps (17.50 mdps/LSB)
- I2C at 400 kHz
- Each sample is one 14-byte burst read from 0x20: temperature plus the
  12-byte gyro+accel block that starts at 0x22.

## Sampling: INT1 data-ready (default) or polling

By default the IMU's INT1 data-ready output (INT1_CTRL 0x0D, accel and gyro
DRDY) drives sampling on GPIO 4. The ISR only sets a flag, and the I2C read
happens in `loop()`. DRDY is latched on every variant, so it stays high until
the outputs are read. `loop()` also treats a high pin level as pending, which
means a missed edge cannot stall the stream. A 30 ms timeout polls STATUS as a
last resort. At startup the firmware checks that INT1 actually toggles. If it
doesn't, it prints `# WARNING: no activity on INT1 ...` before the header and
falls back to polling.

To force polling, add `-DIMU_USE_INT1=0` to `build_flags` in `platformio.ini`.

## Build and flash

```sh
pio run                       # build
pio run -t upload             # flash (auto-detects the port)
pio device monitor            # 115200 baud, Ctrl+C to exit
```

If the upload stalls at `Connecting....`, hold the **BOOT** button on the DevKit
until it starts writing. If no port shows up, install the CP210x or CH340
USB-UART driver for your board. To pick the port yourself, use
`pio run -t upload --upload-port /dev/cu.usbserial-XXXX`.

## Output

```
# LSM6 IMU streamer
# Detected LSM6DS3TR-C / LSM6DSL (WHO_AM_I=0x6A) at I2C address 0x6A
# Config: accel 104 Hz +-4 g, gyro 104 Hz +-500 dps, I2C 400 kHz
# Calibrating gyro bias for 2000 ms - keep the board still...
# Gyro bias (dps): gx=0.4120 gy=-0.8310 gz=0.1500  (n=208 samples)
# Gyro noise sd (dps): gx=0.0710 gy=0.0650 gz=0.0590
# Sampling on INT1 data-ready (GPIO 4)
t_ms,ax,ay,az,gx,gy,gz,temp_c
2843,0.0123,-0.0201,1.0034,0.012,-0.035,0.021,27.31
2853,0.0119,-0.0207,1.0029,-0.044,0.018,0.009,27.31
...
```

Units: `ax..az` in g, `gx..gz` in dps (bias removed), `temp_c` in °C, `t_ms`
from `millis()`. There are about 104 lines per second. Every startup line begins
with `#`. After the header, only CSV lines are printed. To load a capture in
pandas, use `pd.read_csv(f, comment="#")`.

If the board moves during calibration, the firmware prints a
`# WARNING: board was probably moving...` line. Press EN/RESET to calibrate again.

## Sanity check at rest

Lay the board flat and still:

- **|a| = √(ax² + ay² + az²) ≈ 1.00 g.** Anything within about ±0.03 g is fine
  for an uncalibrated part. With the chip face-up, az ≈ +1.0 g and ax, ay ≈ 0.
  Tilting it moves the 1 g between axes, but |a| stays near 1.
- **gx, gy, gz ≈ 0 dps.** After bias removal they should be within about
  ±0.3 dps, with noise of a few hundredths of a dps.
- **temp_c** should be a few degrees above room temperature, because the chip
  and board warm up.

Common faults:

| Symptom | Likely cause |
|---------|--------------|
| \|a\| ≈ 0.5 or 2.0 g | wrong FS/sensitivity pairing |
| gyro reads ±0.3 dps at rest with stable bias | calibration ran while the board was moving |
| temp off by ×16 | wrong temperature scale for the variant |

The live plotter shows |a| and temperature in its status line.

## Live plot

```sh
pip install -r requirements.txt
python tools/live_plot.py                     # auto-detects the port
python tools/live_plot.py /dev/cu.usbserial-0001 --window 10
```

### Recording a test

Use the bar under the plot:

1. Type a **Test** name, for example `tilt-x` or `wrist-flex-1`.
2. Optionally enter a **Duration** in seconds. If you leave it blank, the test
   runs until you stop it.
3. Press **Start test**. The status shows `REC  elapsed  samples`.
4. Press **Stop & save**, or let the duration run out.

Each test writes two files to `recordings/`:

- `YYYY-MM-DD_HHMMSS_<name>.csv` contains every sample, not just what's on
  screen. It starts with `#` metadata lines: test name, start time, sample
  count and rate, units, and the firmware's chip and gyro-bias lines.
- `YYYY-MM-DD_HHMMSS_<name>.png` is an overview plot of the whole test.

To load a recording:

```python
import pandas as pd
df = pd.read_csv("recordings/2026-11-08_151525_tilt-x.csv", comment="#")
```

Closing the window mid-test still saves what was recorded. Use `--out-dir PATH`
to save somewhere else.

Close the serial monitor first, because only one program can hold the port.
Opening the port resets the ESP32, so calibration runs again. Keep the board
still for the first ~3 s.

# Curl experiment and analysis

## Data

`recordings/` holds the raw sets. The IMU was strapped to the dumbbell and
sampled at 103 Hz.

| File | Arm | Set | Notes |
|---|---|---|---|
| `*_test1.csv` | Left | 1 | |
| `*_test2.csv` | Right | – | incomplete, **excluded** from the analysis |
| `*_test2correct.csv` | Right | 1 | replaces test2 |
| `*_test3.csv` | Left | 2 | |
| `*_test4.csv` | Right | 2 | |
| `*_test5.csv` | Left | 3 | |
| `*_test6.csv` | Right | 3 | |

## Analysis

Run the scripts in this order; each writes into `analysis/output/`:

```sh
.venv/bin/python analysis/analyze_curls.py     # rep detection, per-rep velocity, RIR models
.venv/bin/python analysis/early_rep_rir.py     # decline shape, prediction from the first k reps
.venv/bin/python analysis/stop_early_rir.py    # velocity-only RIR for sets stopped early
.venv/bin/python analysis/model_comparison.py  # inter-rep rest, simple vs complex models
.venv/bin/python analysis/plot_model_table.py  # one-figure model summary
```

| Output | What it is |
|---|---|
| `reps.csv` | One row per rep: ROM, concentric/eccentric time, mean/peak velocity, velocity loss, stall time, RIR |
| `trial_*.png` | Per-set angle, angular velocity and vertical velocity with detected reps shaded |
| `velocity_by_rep.png`, `decline_shape.png` | Velocity decline across reps, raw and normalised |
| `rir_predictions.png`, `stop_early_rir.png` | RIR estimates vs actual |
| `model_comparison.png`, `model_table.png` | Comparison of all RIR models |

The results, methods and limitations are in [`analysis/REPORT.md`](analysis/REPORT.md).
