/*
 * BOTGyro - BasicReading
 */

#include <BOTGyro.h>

BOTGyro gyro;                 // default: Wire, address 0x6A
                             // for 0x6B use: BOTGyro gyro(Wire, BOTGYRO_ADDR_HIGH);

uint32_t lastPrint = 0;
const uint16_t PRINT_INTERVAL_MS = 40;   // ~25 Hz output

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  if (!gyro.begin()) {
    Serial.println(F("ASM330LHH not found - check wiring / address / power."));
    while (1) { delay(10); }
  }

  Serial.println(F("Calibrating - keep the sensor PERFECTLY STILL..."));
  if (gyro.calibrate()) {
    Serial.println(F("Calibration done. Streaming (dps):"));
  } else {
    Serial.println(F("Calibration failed (movement detected). Reset to retry."));
  }
}

void loop() {
  gyro.update();   // keep the filter fed at the full sensor rate

  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();
    Serial.print(F("X: "));   Serial.print(gyro.getX(), 2);
    Serial.print(F("  Y: ")); Serial.print(gyro.getY(), 2);
    Serial.print(F("  Z: ")); Serial.println(gyro.getZ(), 2);
  }
}
