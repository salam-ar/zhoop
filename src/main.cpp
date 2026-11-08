// main.cpp - ESP32 + LSM6-series IMU -> CSV over serial.
//
// Startup lines are prefixed with '#'. After the CSV header
//   t_ms,ax,ay,az,gx,gy,gz,temp_c
// only data lines are printed (ax..az in g, gx..gz in dps, temp in degC).
//
// Sampling is driven by the IMU's INT1 data-ready line on GPIO 4 (build with
// -DIMU_USE_INT1=0 to fall back to polling STATUS).
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "imu.h"

namespace {
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;
constexpr uint32_t I2C_HZ = 400000;
constexpr uint32_t BAUD = 115200;
constexpr uint32_t CAL_MS = 2000;  // gyro bias averaging window

#ifndef IMU_USE_INT1
#define IMU_USE_INT1 1
#endif
constexpr int PIN_INT1 = 4;
// If no data-ready edge arrives for this long, check STATUS anyway. Covers a
// disconnected INT1 wire without ever printing mid-stream. 104 Hz = 9.6 ms.
constexpr uint32_t INT_FALLBACK_MS = 30;

lsm6::IMU imu(Wire);
bool useInt1 = false;
volatile bool dataReadyFlag = false;
uint32_t lastSampleMs = 0;

// Minimal ISR: no I2C, no Serial, just set a flag.
[[maybe_unused]] void IRAM_ATTR onImuInt1() { dataReadyFlag = true; }

[[noreturn]] void halt(const char* why) {
  // Repeat periodically so the message is visible even if the serial monitor
  // is opened after boot.
  for (;;) {
    Serial.printf("# FATAL: %s\n", why);
    lsm6::scanBus(Wire, Serial);
    Serial.println("# Halted. Fix the problem and press EN/RESET.");
    delay(5000);
  }
}

void calibrateGyro() {
  Serial.printf("# Calibrating gyro bias for %lu ms - keep the board still...\n",
                (unsigned long)CAL_MS);
  double sum[3] = {0, 0, 0}, sq[3] = {0, 0, 0};
  uint32_t n = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < CAL_MS) {
    if (!imu.dataReady()) continue;
    lsm6::Sample s;
    if (!imu.read(s)) continue;
    const float g[3] = {s.gx, s.gy, s.gz};
    for (int i = 0; i < 3; ++i) { sum[i] += g[i]; sq[i] += (double)g[i] * g[i]; }
    ++n;
  }
  if (n == 0) halt("no samples during gyro calibration");

  float bias[3], sd[3];
  for (int i = 0; i < 3; ++i) {
    bias[i] = sum[i] / n;
    sd[i] = sqrt(fmax(0.0, sq[i] / n - (double)bias[i] * bias[i]));
  }
  imu.setGyroBias(bias[0], bias[1], bias[2]);

  Serial.printf("# Gyro bias (dps): gx=%.4f gy=%.4f gz=%.4f  (n=%lu samples)\n",
                bias[0], bias[1], bias[2], (unsigned long)n);
  Serial.printf("# Gyro noise sd (dps): gx=%.4f gy=%.4f gz=%.4f\n", sd[0], sd[1], sd[2]);
  if (sd[0] > 1.0f || sd[1] > 1.0f || sd[2] > 1.0f)
    Serial.println("# WARNING: board was probably moving during calibration; reset to redo.");
}
}  // namespace

void setup() {
  Serial.begin(BAUD);
  delay(500);  // let USB-UART settle so the first lines aren't lost
  Serial.println();
  Serial.println("# LSM6 IMU streamer");

  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);

  if (!imu.begin()) {
    if (imu.address() != 0 && imu.chip() == lsm6::Chip::Unknown) {
      static char msg[96];
      snprintf(msg, sizeof msg, "device at 0x%02X returned unsupported WHO_AM_I 0x%02X",
               imu.address(), imu.whoAmI());
      halt(msg);
    }
    halt(imu.address() == 0 ? "no LSM6 IMU answered at 0x6A or 0x6B"
                            : "IMU found but reset/configuration failed");
  }

  Serial.printf("# Detected %s (WHO_AM_I=0x%02X) at I2C address 0x%02X\n", imu.chipName(),
                imu.whoAmI(), imu.address());
  Serial.println("# Config: accel 104 Hz +-4 g, gyro 104 Hz +-500 dps, I2C 400 kHz");

  calibrateGyro();

#if IMU_USE_INT1
  pinMode(PIN_INT1, INPUT_PULLDOWN);  // IMU drives push-pull; pulldown if unwired
  if (!imu.enableDataReadyInt1()) halt("failed to configure INT1_CTRL");
  attachInterrupt(digitalPinToInterrupt(PIN_INT1), onImuInt1, RISING);
  // Make sure the line actually toggles before trusting it.
  const uint32_t t0 = millis();
  while (!dataReadyFlag && digitalRead(PIN_INT1) == LOW && millis() - t0 < 100) {
    if (imu.dataReady()) { lsm6::Sample d; imu.read(d); }  // clears latched DRDY -> new edge
  }
  useInt1 = dataReadyFlag || digitalRead(PIN_INT1) == HIGH;
  if (useInt1) {
    Serial.printf("# Sampling on INT1 data-ready (GPIO %d)\n", PIN_INT1);
  } else {
    detachInterrupt(digitalPinToInterrupt(PIN_INT1));
    Serial.printf("# WARNING: no activity on INT1 (GPIO %d); falling back to polling\n", PIN_INT1);
  }
#else
  Serial.println("# Sampling by polling STATUS_REG");
#endif

  Serial.println("t_ms,ax,ay,az,gx,gy,gz,temp_c");
  lastSampleMs = millis();
}

void loop() {
  if (useInt1) {
    // Edge flag from the ISR, or the latched level as a backup in case an edge
    // was missed (DRDY stays high until the outputs are read), or a timeout.
    const bool pending = dataReadyFlag || digitalRead(PIN_INT1) == HIGH ||
                         millis() - lastSampleMs > INT_FALLBACK_MS;
    if (!pending) return;
    dataReadyFlag = false;
  }
  // INT1 is the OR of XL and G data-ready; wait until both are in. At equal
  // ODR they arrive together, so this normally passes on the first read.
  if (!imu.dataReady()) return;
  lsm6::Sample s;
  if (!imu.read(s)) return;  // drop the sample silently; keep the stream clean
  lastSampleMs = millis();
  Serial.printf("%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f\n", (unsigned long)millis(), s.ax, s.ay,
                s.az, s.gx, s.gy, s.gz, s.temp_c);
}
