#include "IMU.h"
#include <Wire.h>
#include <math.h>

static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const float   ACCEL_SCALE = 9.80665f / 16384.0f;   

bool IMU::begin(int sdaPin, int sclPin) {
  Wire.begin(sdaPin, sclPin, 400000);

  for (uint8_t addr : {0x68, 0x69}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      imuAddr_ = addr;
      break;
    }
  }
  if (!imuAddr_) return false;

  imuWrite(REG_PWR_MGMT_1, 0x00);   

  lastUpdateUs_ = micros();
  return true;
}

void IMU::imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr_);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void IMU::readAccelRaw(int16_t& ax, int16_t& ay, int16_t& az) {
  Wire.beginTransmission(imuAddr_);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)imuAddr_, 6);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
}

void IMU::update() {
  if (!imuAddr_) return;

  int16_t axRaw, ayRaw, azRaw;
  readAccelRaw(axRaw, ayRaw, azRaw);

  float ax = axRaw / 16384.0f;
  float ay = ayRaw / 16384.0f;
  float az = azRaw / 16384.0f;
  aMag_ = sqrtf(ax * ax + ay * ay + az * az);

  float axialG;
  switch (AXIAL_AXIS) {
    case 0:  axialG = ax; break;
    case 1:  axialG = ay; break;
    default: axialG = az; break;
  }
  float axialAccel = axialG * 9.80665f;

  uint32_t now = micros();
  float dt = (now - lastUpdateUs_) * 1e-6f;
  lastUpdateUs_ = now;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;   

  kalmanPredict(axialAccel, dt);

  slipDetected_ = driveActive_ && (fabsf(aMag_ - 1.0f) < SLIP_ACCEL_THRESHOLD);
}

void IMU::zero() {
  xHat_[0] = 0.0f;
  xHat_[1] = 0.0f;
}

void IMU::matMul3x3(const float A[3][3], const float B[3][3], float R[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      R[i][j] = 0;
      for (int k = 0; k < 3; k++) R[i][j] += A[i][k] * B[k][j];
    }
}

void IMU::matTranspose3x3(const float A[3][3], float R[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) R[j][i] = A[i][j];
}

void IMU::kalmanPredict(float uAccel, float dt) {
  float A[3][3] = {
    {1, dt, -0.5f * dt * dt},
    {0, 1,  -dt},
    {0, 0,  1}
  };

  float pos  = xHat_[0] + xHat_[1] * dt + 0.5f * dt * dt * (uAccel - xHat_[2]);
  float vel  = xHat_[1] + dt * (uAccel - xHat_[2]);
  float bias = xHat_[2];
  xHat_[0] = pos; xHat_[1] = vel; xHat_[2] = bias;

  float Bc[3] = {0.5f * dt * dt, dt, 0};
  float Q[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      Q[i][j] = Bc[i] * Bc[j] * SIGMA_A * SIGMA_A;
  Q[2][2] = SIGMA_BIAS * SIGMA_BIAS;

  float AP[3][3], At[3][3], APAt[3][3];
  matMul3x3(A, kP_, AP);
  matTranspose3x3(A, At);
  matMul3x3(AP, At, APAt);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      kP_[i][j] = APAt[i][j] + Q[i][j];
}

void IMU::kalmanCorrect(float zPos, float noiseStd) {
  float y = zPos - xHat_[0];
  float S = kP_[0][0] + noiseStd * noiseStd;

  float K[3];
  K[0] = kP_[0][0] / S;
  K[1] = kP_[1][0] / S;
  K[2] = kP_[2][0] / S;

  xHat_[0] += K[0] * y;
  xHat_[1] += K[1] * y;
  xHat_[2] += K[2] * y;

  float P0[3] = {kP_[0][0], kP_[0][1], kP_[0][2]};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      kP_[i][j] -= K[i] * P0[j];
}

void IMU::correctOdometry(float positionMeters, float noiseStd) {
  kalmanCorrect(positionMeters, noiseStd);
}

void IMU::correctCamera(float positionMeters, float noiseStd) {
  kalmanCorrect(positionMeters, noiseStd);
}
