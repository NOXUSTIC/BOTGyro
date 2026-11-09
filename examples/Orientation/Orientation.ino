/*
 * BOTGyro - Orientation
 * ---------------------
 * Shows the library used as a general 6-axis IMU:
 *   - raw accelerometer and gyro are available the moment begin() returns
 *     (no calibration required),
 *   - calibration is OPTIONAL (it improves gyro accuracy / reduces drift),
 *   - enableAttitude() gives fused pitch & roll via a complementary filter.
 *
 * Open the Serial Plotter (115200) and tilt the board to watch pitch/roll.
 *
 * No external libraries needed - uses the built-in Wire library.
 */

#include <BOTGyro.h>

BOTGyro imu;

uint32_t lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  if (!imu.begin()) {
    Serial.println(F("IMU not found - check wiring."));
    while (1) { delay(10); }
  }

  // Optional: calibrate the gyro bias (hold still). Comment this out and you
  // still get data - just with a small uncorrected gyro bias.
  Serial.println(F("Calibrating gyro (optional) - hold still..."));
  imu.calibrate();

  imu.enableAttitude();          // turn on pitch/roll fusion
  // imu.setFilterEnabled(false); // <- uncomment for raw (unfiltered) gyro

  Serial.println(F("Tilt the board. Plotting pitch & roll..."));
}

void loop() {
  imu.update();                  // refresh all sensors + fusion

  if (millis() - lastPrint >= 40) {
    lastPrint = millis();

    // Fused orientation
    Serial.print(F("pitch:")); Serial.print(imu.getPitch(), 1);
    Serial.print(F(",roll:")); Serial.println(imu.getRoll(), 1);

    // Everything else is available too, any time, e.g.:
    //   imu.getAccelX(); imu.getAccelY(); imu.getAccelZ();   // g
    //   imu.getX(); imu.getY(); imu.getZ();                  // filtered gyro, dps
    //   imu.getRawX(); imu.getRawY(); imu.getRawZ();         // raw gyro, dps
  }
}
