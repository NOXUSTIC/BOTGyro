/*
 * BOTGyro.cpp  -  implementation
 * See BOTGyro.h for the public interface and usage notes.
 *
 * This driver talks to the IMU directly over I2C (Wire). No external
 * sensor library is required.
 */

#include "BOTGyro.h"
#include <math.h>

namespace {
  // ---- IMU register map ----
  const uint8_t REG_WHO_AM_I = 0x0F;   // -> 0x6B
  const uint8_t REG_CTRL1_XL = 0x10;   // accelerometer: ODR | FS
  const uint8_t REG_CTRL2_G  = 0x11;   // gyroscope:     ODR | FS
  const uint8_t REG_CTRL3_C  = 0x12;   // BDU / IF_INC / SW_RESET
  const uint8_t REG_STATUS   = 0x1E;   // XLDA(b0) GDA(b1) TDA(b2)
  const uint8_t REG_OUTX_L_G = 0x22;   // gyro  X..Z, 6 bytes
  const uint8_t REG_OUTX_L_A = 0x28;   // accel X..Z, 6 bytes
  const uint8_t WHOAMI_VALUE = 0x6B;

  uint8_t odrToBits(float hz) {
    if (hz >= 6660) return 0x0A;
    if (hz >= 3330) return 0x09;
    if (hz >= 1660) return 0x08;
    if (hz >= 833)  return 0x07;
    if (hz >= 416)  return 0x06;
    if (hz >= 208)  return 0x05;
    if (hz >= 104)  return 0x04;
    if (hz >= 52)   return 0x03;
    if (hz >= 26)   return 0x02;
    if (hz >= 12)   return 0x01;
    return 0x04;                        
  }
}


//  BOTGyroKalman

void BOTGyroKalman::configure(float r, float qMin, float qMax,
                              float deadzone, float ramp) {
  _r = r; _qMin = qMin; _qMax = qMax; _deadzone = deadzone; _ramp = ramp;
}

void BOTGyroKalman::reset(float value) {
  _x = value;
  _p = 1.0f;
}

float BOTGyroKalman::update(float z) {
  const float innovation = z - _x;
  const float magnitude  = fabs(innovation);

  float frac = 0.0f;                              // 0 = still, 1 = full motion
  if (magnitude > _deadzone) {
    frac = (magnitude - _deadzone) / _ramp;
    if (frac > 1.0f) frac = 1.0f;
  }
  const float q = _qMin + (_qMax - _qMin) * frac; // adaptive process noise

  _p += q;                                        // predict
  _k  = _p / (_p + _r);                           // gain
  _x += _k * innovation;                          // correct
  _p  = (1.0f - _k) * _p;                         // covariance update
  return _x;
}


//  BOTGyro

BOTGyro::BOTGyro(TwoWire &wire, uint8_t i2cAddress)
  : _wire(&wire), _address(i2cAddress) {}

// --- low-level I2C ----------------------------------------------------------
bool BOTGyro::writeReg(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  return _wire->endTransmission() == 0;
}

uint8_t BOTGyro::readReg(uint8_t reg) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return 0;     // repeated start
  if (_wire->requestFrom((int)_address, 1) != 1) return 0;
  return _wire->read();
}

bool BOTGyro::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;  // repeated start
  if (_wire->requestFrom((int)_address, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; ++i) buf[i] = _wire->read();
  return true;                                           // IF_INC auto-increments
}

// --- sensor configuration ---------------------------------------------------
bool BOTGyro::begin() {
  _wire->begin();
  _wire->setClock(_i2cClock);
  if (!reinitSensor()) return false;
  enterState(State::Running);     // stream immediately; calibration is optional
  return true;
}

bool BOTGyro::reinitSensor() {
  if (readReg(REG_WHO_AM_I) != WHOAMI_VALUE) return false;
  return configureSensor();
}

bool BOTGyro::configureSensor() {
  writeReg(REG_CTRL3_C, 0x01);          // SW_RESET
  delay(20);
  writeReg(REG_CTRL3_C, 0x44);          // BDU(b6) + IF_INC(b2)

  // gyro full scale -> CTRL2_G bits [3:1], plus sensitivity (mdps/LSB)
  uint8_t gFs;
  switch (_fs) {
    case 125:  gFs = 0x02; _gyroSens = 4.375f; break;   // FS_125 bit
    case 250:  gFs = 0x00; _gyroSens = 8.75f;  break;
    case 500:  gFs = 0x04; _gyroSens = 17.50f; break;
    case 1000: gFs = 0x08; _gyroSens = 35.0f;  break;
    case 2000:
    default:   gFs = 0x0C; _gyroSens = 70.0f;  break;
  }
  const uint8_t odr = odrToBits(_odr);
  if (!writeReg(REG_CTRL2_G, (uint8_t)((odr << 4) | gFs))) return false;

  // accel full scale -> CTRL1_XL bits [3:2], plus sensitivity (mg/LSB)
  uint8_t aFs;
  switch (_accelFs) {
    case 4:  aFs = 0x08; _accelSens = 0.122f; break;
    case 8:  aFs = 0x0C; _accelSens = 0.244f; break;
    case 16: aFs = 0x04; _accelSens = 0.488f; break;
    case 2:
    default: aFs = 0x00; _accelSens = 0.061f; break;
  }
  if (!writeReg(REG_CTRL1_XL, (uint8_t)((odr << 4) | aFs))) return false;

  delay(100);                           // let the gyro settle
  return true;
}

bool BOTGyro::gyroDataReady() {
  return (readReg(REG_STATUS) & 0x02) != 0;   // GDA
}

bool BOTGyro::readAxes(int32_t out[3]) {        // gyro -> mdps
  uint8_t b[6];
  if (!readRegs(REG_OUTX_L_G, b, 6)) return false;
  for (uint8_t i = 0; i < 3; ++i) {
    const int16_t raw = (int16_t)((uint16_t)b[2 * i] | ((uint16_t)b[2 * i + 1] << 8));
    out[i] = (int32_t)(raw * _gyroSens);
  }
  return true;
}

bool BOTGyro::readAccel(int32_t out[3]) {       // accel -> mg
  uint8_t b[6];
  if (!readRegs(REG_OUTX_L_A, b, 6)) return false;
  for (uint8_t i = 0; i < 3; ++i) {
    const int16_t raw = (int16_t)((uint16_t)b[2 * i] | ((uint16_t)b[2 * i + 1] << 8));
    out[i] = (int32_t)(raw * _accelSens);
  }
  return true;
}


//  State machine

void BOTGyro::startCalibration() {
  _calAttempt = 1;
  enterState(State::Calibrating);
}

bool BOTGyro::calibrate(uint32_t timeoutMs) {
  startCalibration();
  const uint32_t start = millis();
  while (_state == State::Calibrating) {
    update();
    if (millis() - start > timeoutMs) {
      _calibrated = false;
      enterState(State::Running);   // give up waiting; keep streaming
      return false;
    }
  }
  return _calibrated;
}

bool BOTGyro::update() {
  switch (_state) {
    case State::Calibrating: serviceCalibrating(); return false;
    case State::Running:     return serviceRunning();
    case State::Fault:       serviceFault();        return false;
    case State::Idle:
    default:                 return false;
  }
}

void BOTGyro::enterState(State s) {
  _state = s;
  const uint32_t now = millis();

  switch (s) {
    case State::Calibrating:
      _calCount = 0;
      for (uint8_t i = 0; i < 3; ++i) {
        _calSum[i] = 0; _calSumSq[i] = 0; _calPeak[i] = 0.0f;
      }
      _lastSampleMs = now;
      break;

    case State::Running:
      _kx.reset(); _ky.reset(); _kz.reset();
      _lastSampleMs = now;
      _rateCounter  = 0;
      _rateWindowMs = now;
      _sampleRateHz = 0;
      _attInit      = false;
      break;

    case State::Fault:
      _lastFaultMs = now;
      break;

    case State::Idle:
      break;
  }
}

void BOTGyro::serviceCalibrating() {
  if (millis() - _lastSampleMs > _drdyTimeoutMs) {  // sensor stopped
    enterState(State::Fault);
    return;
  }
  if (!gyroDataReady()) return;

  int32_t r[3];
  if (!readAxes(r)) {
    enterState(State::Fault);
    return;
  }
  _lastSampleMs = millis();

  for (uint8_t i = 0; i < 3; ++i) {
    _calSum[i]   += r[i];
    _calSumSq[i] += (int64_t)r[i] * r[i];
    const float mag = fabs(r[i] / 1000.0f);
    if (mag > _calPeak[i]) _calPeak[i] = mag;
  }
  if (++_calCount < _calSamples) return;

  // ---- finalise ----
  const float n = (float)_calSamples;
  float biasOut[3], noiseOut[3], peakMax = 0.0f;
  bool ok = true;

  for (uint8_t i = 0; i < 3; ++i) {
    const double mean = (double)_calSum[i] / n;
    double var = (double)_calSumSq[i] / n - mean * mean;
    if (var < 0) var = 0;
    biasOut[i]  = (float)mean;                     // mdps
    noiseOut[i] = (float)(sqrt(var) / 1000.0);     // dps
    if (noiseOut[i] > _calMaxNoise) ok = false;
    if (_calPeak[i] > peakMax) peakMax = _calPeak[i];
  }
  if (peakMax > _calMaxRate) ok = false;

  if (!ok) {                                       // disturbed -> retry or give up
    if (++_calAttempt > _calMaxAttempts) {
      _calibrated = false;
      enterState(State::Running);   // keep streaming uncalibrated data
    } else {
      enterState(State::Calibrating);
    }
    return;
  }

  _bias  = { biasOut[0],  biasOut[1],  biasOut[2]  };
  _noise = { noiseOut[0], noiseOut[1], noiseOut[2] };

  _kx.configure(_kfR, _kfQMin, _kfQMax, _kfDeadzoneK * _noise.x, _kfRamp);
  _ky.configure(_kfR, _kfQMin, _kfQMax, _kfDeadzoneK * _noise.y, _kfRamp);
  _kz.configure(_kfR, _kfQMin, _kfQMax, _kfDeadzoneK * _noise.z, _kfRamp);

  _calibrated = true;
  enterState(State::Running);
}

bool BOTGyro::serviceRunning() {
  if (millis() - _lastSampleMs > _drdyTimeoutMs) {  // sensor stopped
    enterState(State::Fault);
    return false;
  }
  if (!gyroDataReady()) return false;

  int32_t r[3];
  if (!readAxes(r)) {
    enterState(State::Fault);
    return false;
  }
  _lastSampleMs = millis();

  // measured sample rate (samples in the last ~second)
  _rateCounter++;
  const uint32_t now = millis();
  if (now - _rateWindowMs >= 1000) {
    _sampleRateHz = _rateCounter;
    _rateCounter  = 0;
    _rateWindowMs = now;
  }

  int32_t a[3];
  if (readAccel(a)) {                              // mg -> g
    _accel = { a[0] / 1000.0f, a[1] / 1000.0f, a[2] / 1000.0f };
  }

  _raw = { (r[0] - _bias.x) / 1000.0f,
           (r[1] - _bias.y) / 1000.0f,
           (r[2] - _bias.z) / 1000.0f };

  if (_filterEnabled) {
    _filt = { _kx.update(_raw.x),
              _ky.update(_raw.y),
              _kz.update(_raw.z) };
  } else {
    _filt = _raw;
  }

  if (_attitudeEnabled) updateAttitude();

  return true;
}

void BOTGyro::serviceFault() {
  if (millis() - _lastFaultMs < _faultRetryMs) return;
  _lastFaultMs = millis();

  if (reinitSensor()) {
    enterState(State::Running);     // resume streaming; recalibrate if you need to
  }
}

const char* BOTGyro::stateString() const {
  switch (_state) {
    case State::Idle:        return "IDLE";
    case State::Calibrating: return "CALIBRATING";
    case State::Running:     return "RUNNING";
    case State::Fault:       return "FAULT";
  }
  return "?";
}


void BOTGyro::updateAttitude() {
  const float rad2deg = 57.29577951f;
  const float ax = _accel.x, ay = _accel.y, az = _accel.z;
  const float accPitch = atan2(ay, az) * rad2deg;                       // about X
  const float accRoll  = atan2(-ax, sqrt(ay * ay + az * az)) * rad2deg; // about Y

  if (!_attInit) {                       
    _pitch     = accPitch;
    _roll      = accRoll;
    _attLastUs = micros();
    _attInit   = true;
    return;
  }

  const uint32_t now = micros();
  float dt = (now - _attLastUs) * 1e-6f;
  _attLastUs = now;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.0f;   

  _pitch = _attAlpha * (_pitch + _raw.x * dt) + (1.0f - _attAlpha) * accPitch;
  _roll  = _attAlpha * (_roll  + _raw.y * dt) + (1.0f - _attAlpha) * accRoll;
}
