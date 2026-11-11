/*
 * BOTGyro - Orientation
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

  Serial.println(F("Calibrating gyro (optional) - hold still..."));
  imu.calibrate();

  imu.enableAttitude();          
  

  Serial.println(F("Tilt the board. Plotting pitch & roll..."));
}

void loop() {
  imu.update();                  

  if (millis() - lastPrint >= 40) {
    lastPrint = millis();

   
    Serial.print(F("pitch:")); Serial.print(imu.getPitch(), 1);
    Serial.print(F(",roll:")); Serial.println(imu.getRoll(), 1);

  }
}
