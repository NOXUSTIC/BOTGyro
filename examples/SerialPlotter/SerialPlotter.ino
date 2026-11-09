/*
 * BOTGyro - SerialPlotter
 * -----------------------
 * Streams the filtered X/Y/Z angular rates in the "label:value" format the
 * Arduino Serial Plotter expects, giving three labelled traces.
 *
 * 1. Upload, then open Tools -> Serial Plotter.
 * 2. Set the Plotter's baud rate to 115200.
 * 3. Hold still during calibration, then twist the board to see the traces move.
 *
 * No external libraries needed - uses the built-in Wire library.
 */

#include <BOTGyro.h>

BOTGyro gyro;

uint32_t lastPrint = 0;
const uint16_t PRINT_INTERVAL_MS = 40;   // ~25 Hz

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  if (!gyro.begin()) {
    Serial.println(F("ASM330LHH not found - check wiring / address / power."));
    while (1) { delay(10); }
  }

  Serial.println(F("Calibrating - hold still..."));
  gyro.calibrate();
  Serial.println(F("Open Tools > Serial Plotter (115200)."));
}

void loop() {
  gyro.update();

  if (gyro.isRunning() && millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();
    // No spaces after the colons - required by the Serial Plotter parser.
    Serial.print(F("X:"));  Serial.print(gyro.getX(), 2);
    Serial.print(F(",Y:")); Serial.print(gyro.getY(), 2);
    Serial.print(F(",Z:")); Serial.println(gyro.getZ(), 2);
  }
}
