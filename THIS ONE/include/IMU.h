#pragma once
#include <Arduino.h>

class IMU {
    
public:
  static constexpr float SIGMA_A            = 0.021f;  
  static constexpr float SIGMA_BIAS         = 1e-5f;   
  static constexpr float ODO_NOISE_STD      = 0.01f;   
  static constexpr float ODO_NOISE_STD_SLIP = 5.0f;    
  static constexpr float CAM_NOISE_STD      = 0.02f;   
  static constexpr int AXIAL_AXIS = 1;
  bool begin(int sdaPin, int sclPin);
  void update();
  void correctOdometry(float positionMeters, float noiseStd = ODO_NOISE_STD);
  void correctCamera(float positionMeters, float noiseStd = CAM_NOISE_STD);
  void zero();
  bool isConnected() const { return imuAddr_ != 0; }
  void setDriveActive(bool active) { driveActive_ = active; }
  bool isSlipping() const { return slipDetected_; }
  float getPosition() const { return xHat_[0]; }
  float getVelocity() const { return xHat_[1]; }
  float getBias()     const { return xHat_[2]; }
  float getAccelMagnitude() const { return aMag_; }   

private:
  void imuWrite(uint8_t reg, uint8_t val);
  void readAccelRaw(int16_t& ax, int16_t& ay, int16_t& az);
  void kalmanPredict(float uAccel, float dt);
  void kalmanCorrect(float zPos, float noiseStd);
  void matMul3x3(const float A[3][3], const float B[3][3], float R[3][3]);
  void matTranspose3x3(const float A[3][3], float R[3][3]);

  uint8_t imuAddr_ = 0;
  uint32_t lastUpdateUs_ = 0;

  float aMag_ = 1.0f;
  bool driveActive_ = false;
  bool slipDetected_ = false;
  static constexpr float SLIP_ACCEL_THRESHOLD = 0.05f;  
  float xHat_[3] = {0.0f, 0.0f, 0.0f};
  float kP_[3][3] = {
    {0.01f, 0.0f,  0.0f},
    {0.0f,  0.01f, 0.0f},
    {0.0f,  0.0f,  0.0025f}
  };
};