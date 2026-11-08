// imu.h - minimal register-level driver for ST LSM6-series 6-axis IMUs over I2C.
//
// Supported parts (detected at runtime from WHO_AM_I @ 0x0F):
//   0x69  LSM6DS3              (original, "iNEMO" first gen)
//   0x6A  LSM6DS3TR-C / LSM6DSL (indistinguishable via WHO_AM_I; register map
//                               is identical for everything this driver uses)
//   0x6C  LSM6DSO / LSM6DSOX   (likewise indistinguishable; same core map)
//
// Register / bit-field verification (checked against each part's datasheet):
//
//   Register        Addr  DS3  DS3TR-C/DSL  DSO/DSOX   Notes
//   --------------  ----  ---  -----------  --------   -------------------------
//   INT1_CTRL       0x0D   Y       Y           Y       bit0 INT1_DRDY_XL,
//                                                      bit1 INT1_DRDY_G (all parts)
//   WHO_AM_I        0x0F   Y       Y           Y
//   CTRL1_XL        0x10   Y       Y           Y       ODR_XL[7:4], FS_XL[3:2] on all.
//                                                      Low bits differ:
//                                                        DS3:      BW_XL[1:0] (AA filter,
//                                                                  only used if CTRL4_C.
//                                                                  XL_BW_SCAL_ODR=1)
//                                                        TR-C/DSL: LPF1_BW_SEL(b1), BW0_XL(b0)
//                                                        DSO:      LPF2_XL_EN(b1), b0 must be 0
//                                                      We write 0 to the low bits on every
//                                                      part -> default filtering everywhere.
//   CTRL2_G         0x11   Y       Y           Y       ODR_G[7:4], FS_G[3:2], FS_125(b1).
//                                                      b0 must be 0 (TR-C/DSL/DSO).
//                                                      DS3/DSL/TR-C full scale is
//                                                      245 dps at FS_G=00; DSO is 250.
//                                                      FS_G=01 is 500 dps on all parts.
//   CTRL3_C         0x12   Y       Y           Y       BOOT b7, BDU b6, H_LACTIVE b5,
//                                                      PP_OD b4, SIM b3, IF_INC b2,
//                                                      b1 = BLE (DS3/TR-C/DSL) / 0 (DSO),
//                                                      SW_RESET b0. IF_INC is 1 after
//                                                      reset on all parts.
//   STATUS_REG      0x1E   Y       Y           Y       TDA b2, GDA b1, XLDA b0 on all.
//                                                      (DS3 additionally has EV_BOOT b3.)
//   OUT_TEMP_L/H    0x20   Y       Y           Y
//   OUTX_L_G..      0x22   Y       Y           Y       6 bytes, X/Y/Z little endian
//   OUTX_L_XL..     0x28   Y       Y           Y       (called OUTX_L_A on DSO)
//
//   ODR code 0100 = 104 Hz on all parts (accel and gyro, high-performance mode
//   by default). FS_XL encoding is the non-monotonic 00=2g 01=16g 10=4g 11=8g
//   on all parts (DSO: with CTRL8_XL.XL_FS_MODE=0, which is the reset value).
//
//   Sensitivity at the configured ranges (identical on all parts):
//     accel +-4 g    : 0.122 mg/LSB
//     gyro  +-500 dps: 17.50 mdps/LSB
//
//   Temperature sensitivity (0 LSB == 25 degC on all parts):
//     LSM6DS3            : 16 LSB/degC
//     LSM6DS3TR-C/LSM6DSL: 256 LSB/degC   <-- NOT 16; see note in imu.cpp
//     LSM6DSO/LSM6DSOX   : 256 LSB/degC
#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace lsm6 {

// ---- Register map (common subset) -----------------------------------------
constexpr uint8_t REG_INT1_CTRL  = 0x0D;
constexpr uint8_t REG_WHO_AM_I   = 0x0F;
constexpr uint8_t REG_CTRL1_XL   = 0x10;
constexpr uint8_t REG_CTRL2_G    = 0x11;
constexpr uint8_t REG_CTRL3_C    = 0x12;
constexpr uint8_t REG_STATUS     = 0x1E;
constexpr uint8_t REG_OUT_TEMP_L = 0x20;
constexpr uint8_t REG_OUTX_L_G   = 0x22;
constexpr uint8_t REG_OUTX_L_XL  = 0x28;

// CTRL3_C bits
constexpr uint8_t CTRL3_BOOT     = 1 << 7;
constexpr uint8_t CTRL3_BDU      = 1 << 6;
constexpr uint8_t CTRL3_IF_INC   = 1 << 2;
constexpr uint8_t CTRL3_SW_RESET = 1 << 0;

// STATUS_REG bits
constexpr uint8_t STATUS_XLDA = 1 << 0;
constexpr uint8_t STATUS_GDA  = 1 << 1;
constexpr uint8_t STATUS_TDA  = 1 << 2;

// INT1_CTRL bits
constexpr uint8_t INT1_DRDY_XL = 1 << 0;
constexpr uint8_t INT1_DRDY_G  = 1 << 1;

// WHO_AM_I values
constexpr uint8_t WHOAMI_LSM6DS3  = 0x69;
constexpr uint8_t WHOAMI_LSM6DS3TRC_DSL = 0x6A;
constexpr uint8_t WHOAMI_LSM6DSO  = 0x6C;

constexpr uint8_t ADDR_SA0_LOW  = 0x6A;
constexpr uint8_t ADDR_SA0_HIGH = 0x6B;

enum class Chip : uint8_t { Unknown, LSM6DS3, LSM6DS3TRC_DSL, LSM6DSO };

struct Sample {
  float ax, ay, az;   // g
  float gx, gy, gz;   // dps
  float temp_c;       // degC
};

class IMU {
 public:
  explicit IMU(TwoWire& wire = Wire) : wire_(wire) {}

  // Probes 0x6A then 0x6B, identifies the chip, resets and configures it.
  // Returns false if no supported chip responded.
  bool begin();

  Chip chip() const { return chip_; }
  uint8_t address() const { return addr_; }
  uint8_t whoAmI() const { return whoami_; }
  const char* chipName() const;
  // Human-readable reason for the last begin() failure ("" if none).
  const char* lastError() const { return err_; }

  // True when both a new accel and a new gyro sample are available.
  bool dataReady();

  // Burst-reads temp + gyro + accel in one I2C transaction and converts to
  // physical units. Bias (if set) is subtracted from the gyro.
  bool read(Sample& out);

  // Routes accel + gyro data-ready to INT1 (active high, push-pull, latched).
  bool enableDataReadyInt1();

  void setGyroBias(float bx, float by, float bz) { gbias_[0] = bx; gbias_[1] = by; gbias_[2] = bz; }

  // Low-level helpers (public for debugging).
  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t* buf, size_t len);
  bool readReg(uint8_t reg, uint8_t& val) { return readRegs(reg, &val, 1); }

 private:
  bool probe(uint8_t addr);
  bool softwareReset();
  bool configure();

  TwoWire& wire_;
  uint8_t addr_ = 0;
  uint8_t whoami_ = 0;
  Chip chip_ = Chip::Unknown;
  float temp_lsb_per_c_ = 16.0f;
  float gbias_[3] = {0, 0, 0};
  char err_[96] = "";
  bool fail(const char* fmt, ...);
};

// Scans the bus and prints every responding address to `out`.
void scanBus(TwoWire& wire, Print& out);

}  // namespace lsm6
