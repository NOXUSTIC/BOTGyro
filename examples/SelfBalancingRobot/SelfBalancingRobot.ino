/*
 * BOTGyro - SelfBalancingRobot
 * ============================================================================
 *  A two-wheel self-balancing robot (inverted pendulum) built on BOTGyro.
 *
 *  HOW IT WORKS
 *    1. BOTGyro gives us the bias-corrected pitch RATE (gyro) and the raw
 *       accelerometer, both from the ASM330LHH.
 *    2. A COMPLEMENTARY FILTER fuses them into a drift-free tilt ANGLE:
 *         angle = A*(angle + gyroRate*dt) + (1-A)*accelAngle
 *       The gyro handles fast motion; the accelerometer (gravity) anchors the
 *       long-term angle so it never drifts. (A gyro alone WOULD drift and fall.)
 *    3. A PID controller drives the wheels to keep the angle at the balance
 *       point. The gyro rate doubles as the PID derivative term.
 *    4. If the robot tips past FALL_LIMIT it has fallen - motors are cut.
 *
 *  HARDWARE (typical)
 *    - Arduino Nano
 *    - ASM330LHH IMU (I2C: SDA->A4, SCL->A5, VDD->3V3, GND->GND)
 *    - L298N (or similar) dual H-bridge + 2 DC gear motors + wheels
 *    - Battery pack for the motors
 *    Mount the IMU firmly on the chassis, ideally near the wheel axle.
 *
 *  >>> THIS IS A STARTING FRAMEWORK. A balancing robot MUST be PID-tuned for
 *  >>> YOUR specific mass, motors, and geometry. See the tuning notes below.
 *
 * No external libraries needed - uses the built-in Wire library.
 * ============================================================================
 */

#include <BOTGyro.h>

// ---------------------------------------------------------------------------
//  Motor driver pins (L298N).  Avoid A4/A5 (I2C) and 0/1 (Serial).
// ---------------------------------------------------------------------------
const uint8_t ENA = 9,  IN1 = 8,  IN2 = 7;    // left  motor  (ENA must be PWM)
const uint8_t ENB = 10, IN3 = 12, IN4 = 11;   // right motor  (ENB must be PWM)

// If a wheel spins the wrong way, flip its sign here (or swap its two wires).
const int MOTOR_L_SIGN = +1;
const int MOTOR_R_SIGN = +1;

// ---------------------------------------------------------------------------
//  Tuning  -- START HERE.  These are GUESSES; expect to tune them.
// ---------------------------------------------------------------------------
float SETPOINT = 0.0f;          // balance angle (deg). Trim so the robot sits still.

float KP = 22.0f;               // proportional : stiffness   (raise until it twitches)
float KI = 120.0f;              // integral     : kills lean   (add to stop slow drift)
float KD = 0.9f;                // derivative    : damping     (add to stop oscillation)

const float ALPHA          = 0.98f;   // complementary filter (gyro trust, 0.95-0.99)
const float FALL_LIMIT     = 45.0f;   // deg from upright -> "fallen", cut motors
const float INTEGRAL_LIMIT = 1.0f;    // anti-windup clamp on the integral
const int   MOTOR_DEADBAND = 0;       // min PWM to overcome stiction (try 10-40)
const int   MAX_PWM        = 255;

// ---------------------------------------------------------------------------
BOTGyro gyro;

float    angle    = 0.0f;       // fused tilt angle (deg)
float    integral = 0.0f;
uint32_t lastUs   = 0;
uint32_t lastTelemetryMs = 0;

void drive(uint8_t en, uint8_t inA, uint8_t inB, int speed);
void setMotors(int output);
void stopMotors();

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  gyro.setODR(208.0f);            // faster loop for a snappier balance
  gyro.setFullScale(500);         // +/-500 dps is plenty for balancing
  gyro.setAccelFullScale(2);      // +/-2 g for tilt sensing

  if (!gyro.begin()) {
    Serial.println(F("IMU not found - check wiring."));
    while (1) { delay(10); }
  }

  Serial.println(F("Hold the robot UPRIGHT and STILL - calibrating..."));
  gyro.calibrate();

  // Seed the angle from the accelerometer so we don't start from a lurch.
  while (!gyro.update()) { ; }
  angle  = atan2(gyro.getAccelY(), gyro.getAccelZ()) * RAD_TO_DEG;
  lastUs = micros();

  Serial.println(F("Balancing. (Plot 'angle' & 'output' to tune.)"));
}

void loop() {
  if (!gyro.update()) return;     // run the control step once per fresh sample

  // ---- timing ----
  const uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt <= 0 || dt > 0.05f) dt = 0.005f;   // guard against glitches

  // ---- angle from accelerometer (gravity) ----
  // Tilt about the IMU X axis -> use Y and Z. If the angle moves the WRONG way
  // when you tip the robot forward/back, swap the axes or negate here.
  const float accelAngle = atan2(gyro.getAccelY(), gyro.getAccelZ()) * RAD_TO_DEG;

  // ---- pitch rate from the gyro (deg/s). Negate if its sign disagrees ----
  const float pitchRate = gyro.getRawX();

  // ---- complementary filter: fuse rate + gravity into a drift-free angle ----
  angle = ALPHA * (angle + pitchRate * dt) + (1.0f - ALPHA) * accelAngle;

  // ---- safety: if we've fallen over, stop and wait to be stood back up ----
  if (fabs(angle) > FALL_LIMIT) {
    stopMotors();
    integral = 0.0f;              // reset so it doesn't wind up while down
    return;
  }

  // ---- PID controller ----
  const float error = angle - SETPOINT;
  integral += error * dt;
  integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);   // anti-windup

  float output = KP * error + KI * integral + KD * pitchRate;
  output = constrain(output, -MAX_PWM, MAX_PWM);

  setMotors((int)output);

  // ---- telemetry for tuning (~50 Hz, Serial Plotter format) ----
  if (millis() - lastTelemetryMs >= 20) {
    lastTelemetryMs = millis();
    Serial.print(F("angle:"));   Serial.print(angle, 2);
    Serial.print(F(",output:")); Serial.println((int)output);
  }
}

// Apply a signed command (-255..255) to both wheels.
void setMotors(int output) {
  int mag = abs(output);
  if (mag > 0 && mag < MOTOR_DEADBAND) mag = MOTOR_DEADBAND;   // beat stiction
  const int cmd = (output >= 0) ? mag : -mag;
  drive(ENA, IN1, IN2, cmd * MOTOR_L_SIGN);
  drive(ENB, IN3, IN4, cmd * MOTOR_R_SIGN);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// One H-bridge channel: sign sets direction, magnitude sets PWM.
void drive(uint8_t en, uint8_t inA, uint8_t inB, int speed) {
  const bool forward = (speed >= 0);
  const int  pwm = constrain(abs(speed), 0, MAX_PWM);
  digitalWrite(inA, forward ? HIGH : LOW);
  digitalWrite(inB, forward ? LOW  : HIGH);
  analogWrite(en, pwm);
}
