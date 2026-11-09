/*
 * ============================================================================
 *  BOTGyro  -  Adaptive, calibrated gyroscope library for the ST ASM330LHH
 * ============================================================================
 *  Wraps the ASM330LHH 6-axis IMU and provides:
 *    - Zero-rate bias calibration with disturbance validation
 *    - Per-axis adaptive Kalman filtering (smooth at rest, snappy in motion)
 *    - A non-blocking state machine (IDLE / CALIBRATING / RUNNING / FAULT)
 *    - Sensor watchdog with automatic re-initialisation
 *    - Live sample-rate and bias/noise diagnostics
 *
 *  Angular rate is reported in degrees per second (dps).
 *
 *  Self-contained: communicates with the ASM330LHH directly over I2C using the
 *  built-in Wire library. No external sensor library required.
 *
 *  Typical use:
 *      BOTGyro gyro;
 *      gyro.begin();         // bring up the sensor
 *      gyro.calibrate();     // hold the board still
 *      ...
 *      if (gyro.update()) {  // call often; true on each fresh sample
 *        float z = gyro.getZ();
 *      }
 *
 *  License: MIT.
 * ============================================================================
 */

#ifndef BOTGYRO_H
#define BOTGYRO_H

#include <Arduino.h>
#include <Wire.h>

#define BOTGYRO_VERSION "2.0.0"

// ASM330LHH 7-bit I2C addresses (SA0/SDO pin: low -> 0x6A, high -> 0x6B)
#define BOTGYRO_ADDR_LOW  0x6A
#define BOTGYRO_ADDR_HIGH 0x6B

// ---------------------------------------------------------------------------
//  BOTGyroKalman
//
//  Adaptive 1-D Kalman filter. Used internally per axis, but reusable on its
//  own for smoothing any noisy scalar. The process-noise term scales with the
//  measurement innovation: small (in-band) deviations are smoothed heavily,
//  while a large deviation relaxes the filter so it tracks quickly.
// ---------------------------------------------------------------------------
class BOTGyroKalman {
public:
  void  configure(float r, float qMin, float qMax, float deadzone, float ramp);
  void  reset(float value = 0.0f);
  float update(float z);
  float value() const { return _x; }

private:
  float _r = 0.10f, _qMin = 0.001f, _qMax = 0.20f, _deadzone = 0.20f, _ramp = 8.0f;
  float _x = 0.0f,  _p = 1.0f, _k = 0.0f;
};

// ---------------------------------------------------------------------------
//  BOTGyro
// ---------------------------------------------------------------------------
class BOTGyro {
public:
  enum class State : uint8_t { Idle, Calibrating, Running, Fault };

  explicit BOTGyro(TwoWire &wire = Wire, uint8_t i2cAddress = BOTGYRO_ADDR_LOW);

  // ---- Setup (call configuration setters BEFORE begin/calibrate) ----
  bool begin();                                   // returns false if sensor missing
  void setODR(float hz)                  { _odr = hz; }
  void setFullScale(int32_t dps)         { _fs = dps; }          // 250/500/1000/2000
  void setI2CClock(uint32_t hz)          { _i2cClock = hz; }
  void setCalibrationSamples(uint16_t n) { _calSamples = n; }
  void setAccelFullScale(int32_t g)      { _accelFs = g; }            // 2/4/8/16
  void setFilterEnabled(bool en)         { _filterEnabled = en; }     // gyro Kalman on/off
  void enableAttitude(bool en = true)    { _attitudeEnabled = en; }   // compute pitch/roll
  void setAttitudeFilter(float alpha)    { _attAlpha = alpha; }       // 0.90-0.99
  void setKalman(float r, float qMin, float qMax, float rampDps, float deadzoneK) {
    _kfR = r; _kfQMin = qMin; _kfQMax = qMax; _kfRamp = rampDps; _kfDeadzoneK = deadzoneK;
  }

  // ---- Calibration ----
  void startCalibration();                        // non-blocking; progressed by update()
  bool calibrate(uint32_t timeoutMs = 30000);     // blocking convenience; true on success
  bool isCalibrated() const { return _calibrated; }

  // ---- Main loop ----
  bool update();                                  // call often; true on each fresh sample
  void resetFilters() { _kx.reset(); _ky.reset(); _kz.reset(); }

  // ---- Filtered angular rate (dps) ----
  float getX() const { return _filt.x; }
  float getY() const { return _filt.y; }
  float getZ() const { return _filt.z; }

  // ---- Raw angular rate, bias removed (dps) ----
  float getRawX() const { return _raw.x; }
  float getRawY() const { return _raw.y; }
  float getRawZ() const { return _raw.z; }

  // ---- Accelerometer (g) ----
  float getAccelX() const { return _accel.x; }
  float getAccelY() const { return _accel.y; }
  float getAccelZ() const { return _accel.z; }

  // ---- Fused attitude (deg) - enable with enableAttitude() ----
  float getPitch() const { return _pitch; }
  float getRoll()  const { return _roll; }

  // ---- Diagnostics ----
  State       getState() const      { return _state; }
  bool        isRunning() const     { return _state == State::Running; }
  bool        isCalibrating() const { return _state == State::Calibrating; }
  bool        isFaulted() const     { return _state == State::Fault; }
  uint16_t    getSampleRate() const { return _sampleRateHz; }     // Hz, measured
  float       getBiasX() const      { return _bias.x / 1000.0f; } // dps
  float       getBiasY() const      { return _bias.y / 1000.0f; }
  float       getBiasZ() const      { return _bias.z / 1000.0f; }
  float       getNoiseX() const     { return _noise.x; }          // dps (std)
  float       getNoiseY() const     { return _noise.y; }
  float       getNoiseZ() const     { return _noise.z; }
  const char* stateString() const;

private:
  struct Vec3 { float x, y, z; };

  bool reinitSensor();
  bool configureSensor();
  bool writeReg(uint8_t reg, uint8_t value);
  uint8_t readReg(uint8_t reg);
  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
  bool gyroDataReady();
  bool readAxes(int32_t out[3]);      // gyro, mdps
  bool readAccel(int32_t out[3]);     // accel, mg
  void enterState(State s);
  void serviceCalibrating();
  bool serviceRunning();
  void serviceFault();
  void updateAttitude();

  TwoWire* _wire;
  uint8_t  _address;
  float    _gyroSens  = 70.0f;     // mdps/LSB  (set from full scale)
  float    _accelSens = 0.061f;    // mg/LSB    (set from full scale)

  // configuration
  float    _odr        = 104.0f;
  int32_t  _fs         = 2000;
  uint32_t _i2cClock   = 400000;
  uint16_t _calSamples = 600;
  int32_t  _accelFs    = 2;     // +/- g
  float    _kfR = 0.10f, _kfQMin = 0.001f, _kfQMax = 0.20f;
  float    _kfRamp = 8.0f, _kfDeadzoneK = 4.0f;
  float    _calMaxNoise = 1.5f, _calMaxRate = 5.0f;
  uint8_t  _calMaxAttempts = 5;
  uint16_t _drdyTimeoutMs  = 250;
  uint16_t _faultRetryMs   = 1000;

  // state
  State _state      = State::Idle;
  bool  _calibrated = false;

  bool     _filterEnabled   = true;     // gyro Kalman filtering on by default
  bool     _attitudeEnabled = false;    // pitch/roll computed only when enabled
  float    _attAlpha = 0.98f;           // complementary filter constant
  float    _pitch = 0.0f, _roll = 0.0f; // deg
  uint32_t _attLastUs = 0;
  bool     _attInit = false;

  BOTGyroKalman _kx, _ky, _kz;
  Vec3 _bias  = {0, 0, 0};   // mdps
  Vec3 _noise = {0, 0, 0};   // dps
  Vec3 _raw   = {0, 0, 0};   // dps
  Vec3 _filt  = {0, 0, 0};   // dps
  Vec3 _accel = {0, 0, 0};   // g

  // calibration accumulators
  uint16_t _calCount   = 0;
  uint8_t  _calAttempt = 0;
  int64_t  _calSum[3]   = {0, 0, 0};
  int64_t  _calSumSq[3] = {0, 0, 0};
  float    _calPeak[3]  = {0, 0, 0};

  // timing / diagnostics
  uint32_t _lastSampleMs = 0;
  uint32_t _lastFaultMs  = 0;
  uint16_t _sampleRateHz = 0;
  uint16_t _rateCounter  = 0;
  uint32_t _rateWindowMs = 0;
};

#endif  // BOTGYRO_H
