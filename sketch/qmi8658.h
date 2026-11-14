#pragma once
#include <Arduino.h>

// MPU6000 on the Uno Q SPI header, same sample layout the QMI driver produced.
// SCK D13, MISO D12, MOSI D11, CS D10. INT on D2 is unused.
// Bit-banged mode 3: the Zephyr SPI driver clocks write data as zeros.
// Axes match the MPU port of this firmware.

struct ImuSample {
  uint8_t ok;
  int16_t accRaw[3];
  int16_t gyroRaw[3];
};

struct ImuSnapshot {
  ImuSample sample;
  uint32_t seq, sampleUs, periodUs, workUs, overruns, errors, initAttempts;
};

static ImuSample imu = {};
static uint8_t qmiAddr = 0;
static bool mpuReady = false;
static uint16_t mpuFail = 0;

static const uint8_t MPU_CS = 10;
static const uint8_t MPU_MOSI = 11;
static const uint8_t MPU_MISO = 12;
static const uint8_t MPU_SCK = 13;
static const int16_t GYRO_BIAS_RAW[3] = {0, 0, 0};

static int16_t mpuClamp(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static int16_t mpuSignedBE(const uint8_t *p) {
  const uint16_t u = ((uint16_t)p[0] << 8) | p[1];
  return (int16_t)((u & 0x8000u) ? (int32_t)u - 65536 : u);
}

static int16_t mpuGyro(int32_t raw) {
  // Power-up full scale stays +/-250 dps (131 LSB/dps): the FS_SEL write does
  // not stick. The rest of this firmware expects 32 LSB/dps.
  const int32_t n = raw * 32;
  return mpuClamp((n >= 0 ? n + 65 : n - 65) / 131);
}

static uint8_t mpuShift(uint8_t out) {
  uint8_t in = 0;
  for (int8_t bit = 7; bit >= 0; --bit) {
    digitalWrite(MPU_SCK, LOW);
    digitalWrite(MPU_MOSI, (out >> bit) & 1);
    digitalWrite(MPU_SCK, HIGH);
    in = (uint8_t)((in << 1) | (digitalRead(MPU_MISO) ? 1 : 0));
  }
  return in;
}

static void mpuBytes(uint8_t *buf, uint8_t n) {
  digitalWrite(MPU_CS, LOW);
  for (uint8_t i = 0; i < n; ++i) buf[i] = mpuShift(buf[i]);
  digitalWrite(MPU_CS, HIGH);
}

static bool mpuWrite(uint8_t reg, uint8_t value) {
  uint8_t buf[2] = {(uint8_t)(reg & 0x7F), value};
  mpuBytes(buf, 2);
  return true;
}

static bool mpuRead(uint8_t reg, uint8_t *data, uint8_t size) {
  uint8_t buf[16];
  if (size + 1 > sizeof(buf)) return false;
  buf[0] = (uint8_t)(reg | 0x80);
  for (uint8_t i = 1; i < size + 1; ++i) buf[i] = 0;
  mpuBytes(buf, size + 1);
  if (data) for (uint8_t i = 0; i < size; ++i) data[i] = buf[i + 1];
  return true;
}

static bool initImu() {
  imu = {};
  mpuReady = false;
  qmiAddr = 0;
  mpuFail = 0;
  pinMode(MPU_CS, OUTPUT);
  pinMode(MPU_MOSI, OUTPUT);
  pinMode(MPU_SCK, OUTPUT);
  pinMode(MPU_MISO, INPUT);
  digitalWrite(MPU_CS, HIGH);
  digitalWrite(MPU_SCK, HIGH);

  uint8_t who = 0;
  for (uint8_t attempt = 0; attempt < 10 && who != 0x68; ++attempt) {
    mpuWrite(0x6A, 0x10); // SPI: disable the I2C slave interface
    mpuRead(0x75, &who, 1);
    if (who != 0x68) delay(20);
  }
  if (who != 0x68) {
    mpuFail = who;
    imu.accRaw[0] = who;
    imu.accRaw[2] = 0x6000;
    return false;
  }
  mpuWrite(0x6B, 0x80);
  delay(100);
  mpuWrite(0x6A, 0x10);
  mpuWrite(0x6B, 0x01);
  delay(50);

  static const uint8_t config[][2] = {
    {0x6C, 0x00}, {0x19, 0x04}, {0x1A, 0x02}, {0x1B, 0x10},
    {0x1C, 0x00}, {0x37, 0x00}, {0x38, 0x01},
  };
  for (const auto &entry : config) mpuWrite(entry[0], entry[1]);
  delay(50);
  mpuReady = true;
  qmiAddr = 1;
  return true;
}

static bool updateImu() {
  imu.ok = 0;
  if (!mpuReady) return false;
  uint8_t data[15];
  if (!mpuRead(0x3A, data, sizeof(data))) {
    mpuReady = false;
    qmiAddr = 0;
    return false;
  }
  if (!(data[0] & 0x01) && data[1] == 0 && data[3] == 0 && data[5] == 0) return false;

  const int32_t ax = mpuSignedBE(data + 1);
  const int32_t ay = mpuSignedBE(data + 3);
  const int32_t az = mpuSignedBE(data + 5);
  const int32_t gx = mpuSignedBE(data + 9) - GYRO_BIAS_RAW[0];
  const int32_t gy = mpuSignedBE(data + 11) - GYRO_BIAS_RAW[1];
  const int32_t gz = mpuSignedBE(data + 13) - GYRO_BIAS_RAW[2];
  // On the back, board facing up. 180 deg about X so standing gravity is -Z.
  imu.accRaw[0] = mpuClamp(az);
  imu.accRaw[1] = mpuClamp(-ax);
  imu.accRaw[2] = mpuClamp(ay);
  imu.gyroRaw[0] = mpuGyro(-gz);
  imu.gyroRaw[1] = mpuGyro(gx);
  imu.gyroRaw[2] = mpuGyro(-gy);
  imu.ok = 1;
  return true;
}
