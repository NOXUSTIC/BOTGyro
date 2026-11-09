/*
 * BOTGyro - SerialCommander
 * -------------------------
 * Full-featured demo built on the library. The sketch only handles the user
 * interface (commands + output formatting); the sensor, calibration, filtering
 * and fault recovery all live inside BOTGyro.
 *
 * Serial Monitor @ 115200. Type a key and press Enter:
 *     c  recalibrate (hold still)
 *     r  reset filters
 *     m  cycle output mode (human / plotter / CSV)
 *     s  status (state, sample rate, bias, noise)
 *     ?  help
 *
 * No external libraries needed - uses the built-in Wire library.
 */

#include <BOTGyro.h>

BOTGyro gyro;

enum class OutMode : uint8_t { Human, Plotter, Csv };
OutMode  mode = OutMode::Plotter;     // default plays nicely with Serial Plotter

uint32_t lastPrint = 0;
const uint16_t PRINT_INTERVAL_MS = 40;   // ~25 Hz

void printHelp();
void printStatus();
void cycleMode();
void emit();

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\nBOTGyro - SerialCommander"));

  if (!gyro.begin()) {
    Serial.println(F("ASM330LHH not found - check wiring / address / power."));
    while (1) { delay(10); }
  }

  Serial.println(F("Calibrating - keep the sensor still..."));
  if (!gyro.calibrate()) {
    Serial.println(F("Calibration failed; send 'c' to retry."));
  }
  printHelp();
}

void loop() {
  gyro.update();   // progress sensor + filters (non-blocking)

  // --- announce state changes (calibration finished, fault, recovery) ---
  static BOTGyro::State prev = BOTGyro::State::Idle;
  const BOTGyro::State now = gyro.getState();
  if (now != prev) {
    switch (now) {
      case BOTGyro::State::Running:     Serial.println(F("[ok] running"));                 break;
      case BOTGyro::State::Idle:        Serial.println(F("[!] calibration failed - 'c'"));  break;
      case BOTGyro::State::Fault:       Serial.println(F("[!] sensor fault - recovering")); break;
      case BOTGyro::State::Calibrating: Serial.println(F("[..] calibrating"));              break;
    }
    prev = now;
  }

  // --- commands ---
  while (Serial.available()) {
    const char c = Serial.read();
    switch (c) {
      case 'c': case 'C': Serial.println(F("Recalibrating - hold still...")); gyro.startCalibration(); break;
      case 'r': case 'R': gyro.resetFilters(); Serial.println(F("Filters reset.")); break;
      case 'm': case 'M': cycleMode(); break;
      case 's': case 'S': printStatus(); break;
      case '?': case 'h': case 'H': printHelp(); break;
      default: break;
    }
  }

  // --- throttled output (only while running) ---
  if (gyro.isRunning() && millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();
    emit();
  }
}

void emit() {
  switch (mode) {
    case OutMode::Human:
      Serial.print(F("Gyro [dps]  X:")); Serial.print(gyro.getX(), 2);
      Serial.print(F("  Y:"));           Serial.print(gyro.getY(), 2);
      Serial.print(F("  Z:"));           Serial.println(gyro.getZ(), 2);
      break;
    case OutMode::Plotter:
      Serial.print(F("X:"));  Serial.print(gyro.getX(), 2);
      Serial.print(F(",Y:")); Serial.print(gyro.getY(), 2);
      Serial.print(F(",Z:")); Serial.println(gyro.getZ(), 2);
      break;
    case OutMode::Csv:
      Serial.print(millis());      Serial.print(',');
      Serial.print(gyro.getX(), 3); Serial.print(',');
      Serial.print(gyro.getY(), 3); Serial.print(',');
      Serial.println(gyro.getZ(), 3);
      break;
  }
}

void cycleMode() {
  mode = (OutMode)(((uint8_t)mode + 1) % 3);
  Serial.print(F("Output mode: "));
  switch (mode) {
    case OutMode::Human:   Serial.println(F("human")); break;
    case OutMode::Plotter: Serial.println(F("plotter")); break;
    case OutMode::Csv:     Serial.println(F("CSV")); Serial.println(F("t_ms,x_dps,y_dps,z_dps")); break;
  }
}

void printHelp() {
  Serial.println(F("--- commands ---"));
  Serial.println(F("  c : recalibrate (hold still)"));
  Serial.println(F("  r : reset filters"));
  Serial.println(F("  m : cycle output (human/plotter/CSV)"));
  Serial.println(F("  s : status"));
  Serial.println(F("  ? : help"));
}

void printStatus() {
  Serial.println(F("--- status ---"));
  Serial.print(F("  state : ")); Serial.println(gyro.stateString());
  Serial.print(F("  rate  : ")); Serial.print(gyro.getSampleRate()); Serial.println(F(" Hz"));
  Serial.print(F("  bias  : "));
  Serial.print(gyro.getBiasX(), 3); Serial.print(' ');
  Serial.print(gyro.getBiasY(), 3); Serial.print(' ');
  Serial.println(gyro.getBiasZ(), 3);
  Serial.print(F("  noise : "));
  Serial.print(gyro.getNoiseX(), 3); Serial.print(' ');
  Serial.print(gyro.getNoiseY(), 3); Serial.print(' ');
  Serial.println(gyro.getNoiseZ(), 3);
}
