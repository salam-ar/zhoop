// imu.cpp - see imu.h for the register verification table.
#include "imu.h"

namespace lsm6 {

namespace {
// CTRL1_XL: ODR_XL = 0100 (104 Hz), FS_XL = 10 (+-4 g), low bits 0.
constexpr uint8_t CTRL1_XL_104HZ_4G = (0b0100 << 4) | (0b10 << 2);    // 0x48
// CTRL2_G: ODR_G = 0100 (104 Hz), FS_G = 01 (500 dps), FS_125 = 0, b0 = 0.
constexpr uint8_t CTRL2_G_104HZ_500DPS = (0b0100 << 4) | (0b01 << 2);  // 0x44

constexpr float ACCEL_G_PER_LSB  = 0.122e-3f;  // +-4 g   -> 0.122 mg/LSB
constexpr float GYRO_DPS_PER_LSB = 17.50e-3f;  // +-500 dps -> 17.50 mdps/LSB
constexpr float TEMP_OFFSET_C    = 25.0f;

inline int16_t le16(const uint8_t* p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
}  // namespace

const char* IMU::chipName() const {
  switch (chip_) {
    case Chip::LSM6DS3:        return "LSM6DS3";
    case Chip::LSM6DS3TRC_DSL: return "LSM6DS3TR-C / LSM6DSL";
    case Chip::LSM6DSO:        return "LSM6DSO / LSM6DSOX";
    default:                   return "unknown";
  }
}

bool IMU::writeReg(uint8_t reg, uint8_t val) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  wire_.write(val);
  return wire_.endTransmission() == 0;
}

bool IMU::readRegs(uint8_t reg, uint8_t* buf, size_t len) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  // Repeated start: register pointer write + read form one transaction.
  if (wire_.endTransmission(false) != 0) return false;
  if (wire_.requestFrom(addr_, (uint8_t)len) != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = wire_.read();
  return true;
}

bool IMU::probe(uint8_t addr) {
  addr_ = addr;
  uint8_t id = 0;
  if (!readReg(REG_WHO_AM_I, id)) return false;
  whoami_ = id;
  switch (id) {
    case WHOAMI_LSM6DS3:
      chip_ = Chip::LSM6DS3;
      temp_lsb_per_c_ = 16.0f;
      return true;
    case WHOAMI_LSM6DS3TRC_DSL:
      // NOTE: the LSM6DS3TR-C and LSM6DSL datasheets both specify
      // 256 LSB/degC (Table "Temperature sensor characteristics"), not the
      // 16 LSB/degC of the original LSM6DS3. Using 16 here would read ~16x too
      // far from 25 degC.
      chip_ = Chip::LSM6DS3TRC_DSL;
      temp_lsb_per_c_ = 256.0f;
      return true;
    case WHOAMI_LSM6DSO:
      chip_ = Chip::LSM6DSO;
      temp_lsb_per_c_ = 256.0f;
      return true;
    default:
      chip_ = Chip::Unknown;
      return false;  // something answered, but it isn't a supported LSM6
  }
}

bool IMU::softwareReset() {
  // Power both sensors down first (recommended before SW_RESET on DSO; harmless
  // on the others), then reset. SW_RESET self-clears when complete (~50 us).
  if (!writeReg(REG_CTRL1_XL, 0x00)) return false;
  if (!writeReg(REG_CTRL2_G, 0x00)) return false;
  if (!writeReg(REG_CTRL3_C, CTRL3_SW_RESET)) return false;

  uint32_t t0 = millis();
  uint8_t c3 = CTRL3_SW_RESET;
  do {
    delay(1);
    if (!readReg(REG_CTRL3_C, c3)) continue;  // bus may NAK during reset
  } while ((c3 & CTRL3_SW_RESET) && millis() - t0 < 100);
  if (c3 & CTRL3_SW_RESET) return false;

  // Reload trimming parameters too (BOOT, ~10-15 ms, also self-clearing).
  if (!writeReg(REG_CTRL3_C, CTRL3_BOOT | CTRL3_IF_INC)) return false;
  t0 = millis();
  uint8_t c3b = CTRL3_BOOT;
  do {
    delay(2);
    if (!readReg(REG_CTRL3_C, c3b)) continue;
  } while ((c3b & CTRL3_BOOT) && millis() - t0 < 100);
  return !(c3b & CTRL3_BOOT);
}

bool IMU::configure() {
  // BDU: output regs not updated until both L and H bytes have been read.
  // IF_INC: auto-increment the register address during burst reads.
  if (!writeReg(REG_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC)) return false;
  if (!writeReg(REG_CTRL1_XL, CTRL1_XL_104HZ_4G)) return false;
  if (!writeReg(REG_CTRL2_G, CTRL2_G_104HZ_500DPS)) return false;

  // Read back to catch wiring/bus problems early.
  uint8_t v[3];
  if (!readRegs(REG_CTRL1_XL, v, 3)) return false;
  return v[0] == CTRL1_XL_104HZ_4G && v[1] == CTRL2_G_104HZ_500DPS &&
         (v[2] & (CTRL3_BDU | CTRL3_IF_INC)) == (CTRL3_BDU | CTRL3_IF_INC);
}

bool IMU::begin() {
  if (!probe(ADDR_SA0_LOW) && !probe(ADDR_SA0_HIGH)) {
    addr_ = 0;
    return false;
  }
  if (!softwareReset()) return false;
  if (!configure()) return false;
  // Gyro turn-on time is ~70 ms (DS3) / ~35 ms (DSO); first samples after an
  // ODR change are unreliable. Wait, then flush a few.
  delay(100);
  Sample s;
  for (int i = 0; i < 5; ++i) {
    uint32_t t0 = millis();
    while (!dataReady() && millis() - t0 < 50) {}
    read(s);
  }
  return true;
}

bool IMU::dataReady() {
  uint8_t st = 0;
  if (!readReg(REG_STATUS, st)) return false;
  return (st & (STATUS_XLDA | STATUS_GDA)) == (STATUS_XLDA | STATUS_GDA);
}

bool IMU::read(Sample& out) {
  // One 14-byte burst covering OUT_TEMP_L (0x20) .. OUTZ_H_XL (0x2D):
  //   [0..1] temp, [2..7] gyro X/Y/Z (from 0x22), [8..13] accel X/Y/Z (0x28).
  // This contains the required 12-byte gyro+accel block starting at 0x22 and
  // picks up temperature in the same transaction.
  uint8_t b[14];
  if (!readRegs(REG_OUT_TEMP_L, b, sizeof b)) return false;

  out.temp_c = TEMP_OFFSET_C + le16(&b[0]) / temp_lsb_per_c_;
  out.gx = le16(&b[2])  * GYRO_DPS_PER_LSB - gbias_[0];
  out.gy = le16(&b[4])  * GYRO_DPS_PER_LSB - gbias_[1];
  out.gz = le16(&b[6])  * GYRO_DPS_PER_LSB - gbias_[2];
  out.ax = le16(&b[8])  * ACCEL_G_PER_LSB;
  out.ay = le16(&b[10]) * ACCEL_G_PER_LSB;
  out.az = le16(&b[12]) * ACCEL_G_PER_LSB;
  return true;
}

void scanBus(TwoWire& wire, Print& out) {
  out.println("# I2C bus scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    wire.beginTransmission(a);
    if (wire.endTransmission() == 0) {
      out.printf("#   device at 0x%02X\n", a);
      ++found;
    }
  }
  if (!found) out.println("#   no devices found (check SDA=21, SCL=22, power, GND)");
}

}  // namespace lsm6
