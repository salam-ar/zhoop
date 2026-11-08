# ESP32 + LSM6 IMU streamer

ESP32 DevKit firmware (Arduino framework, PlatformIO) that reads an ST
LSM6-series 6-axis IMU over I2C using a small register-level driver and streams
CSV over serial.

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

## Layout

```
platformio.ini
include/imu.h      driver API + register verification table
src/imu.cpp        driver: detect, reset, configure, burst read, unit conversion
src/main.cpp       setup (detect, calibrate), CSV streaming loop
tools/live_plot.py live plot of the six axes (pyserial + matplotlib)
```

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
pip install pyserial matplotlib
python tools/live_plot.py                     # auto-detects the port
python tools/live_plot.py /dev/cu.usbserial-0001 --window 10
```

Close the serial monitor first, because only one program can hold the port.
Opening the port resets the ESP32, so calibration runs again. Keep the board
still for the first ~3 s.
