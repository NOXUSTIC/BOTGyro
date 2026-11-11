# BOTGyro

BOTGyro is a general-purpose driver library for the BOTGyro 6-axis motion sensor, a type of inertial measurement unit (IMU).

The sensor measures two physical quantities:

- **Angular rate** (gyroscope): the speed at which the sensor rotates, expressed in degrees per second (dps).
- **Acceleration** (accelerometer): the force acting on the sensor, including gravity, expressed in g.

From these measurements the library can additionally compute the sensor's **orientation** (pitch and roll), expressed in degrees.

The library is suitable for any application that requires motion sensing, including robotics, unmanned aerial vehicles, camera gimbals, self-balancing platforms, wearable devices, gesture-based controllers, tilt detection, and data logging.

All functionality beyond raw data acquisition is optional. Calling `begin()` provides live sensor data immediately; calibration, filtering, and orientation estimation may be enabled as required.

## Features

- **Complete 6-axis access:** raw acceleration (g) and angular rate (dps), available immediately after `begin()` with no further configuration.
- **Optional bias calibration:** removes the sensor's resting offset. Calibration includes disturbance validation; if the sensor is moved during the procedure, the result is rejected and the procedure is repeated.
- **Optional adaptive Kalman filter:** applies strong smoothing while the sensor is stationary and responds rapidly during motion. The filter may be enabled or disabled at runtime.
- **Optional orientation estimation:** computes pitch and roll using a complementary filter that fuses gyroscope and accelerometer data, producing angles that do not drift over time.
- **Non-blocking operation:** the library does not use `delay()` and does not stall the main program loop.
- **Watchdog and automatic recovery:** a loose connection or power interruption is detected, and the sensor is re-initialised automatically.
- **Diagnostics:** provides the measured sample rate together with per-axis bias and noise estimates.

## Requirements

- A microcontroller board that provides an I2C interface and supports the Arduino `Wire` library. This includes Arduino boards (Nano, Uno, Mega), ESP32 and ESP8266, Raspberry Pi Pico (RP2040), STM32, Teensy, and comparable platforms.
- A BOTGyro 6-axis sensor, either as a bare chip or on a breakout board.
- No external libraries. BOTGyro communicates with the sensor directly through the built-in `Wire` library.

The library was developed and tested on the **Arduino Nano**. Other platforms that implement `Wire` are expected to be compatible. Users who encounter issues on a particular platform are encouraged to submit an issue report.

> **Supply voltage:** The sensor chip operates at **3.3 V**. A bare chip must be powered from 3.3 V. Certain breakout boards include an onboard voltage regulator and level shifter and may accept 5 V; consult the documentation for the specific board before connecting power.

## Installation

**Arduino IDE**

1. Download `BOTGyro.zip`.
2. Select **Sketch → Include Library → Add .ZIP Library…** and choose the downloaded file.
3. Restart the IDE. The examples are then available under **File → Examples → BOTGyro**.

**Manual installation**

Copy the `BOTGyro` folder into the `Arduino/libraries` directory and restart the IDE.

**PlatformIO**

Place the library folder in the project's `lib/` directory, or add the repository URL to the `lib_deps` entry of `platformio.ini`.

## Wiring (I2C)

The sensor communicates over I2C using two signal lines (SDA and SCL) in addition to power and ground. The sensor-side pin names are identical on all platforms; only the board-side pins differ.

| Sensor Pin | Connection |
|------------|------------|
| VDD        | 3.3 V |
| GND        | Ground |
| SCL        | I2C clock pin (SCL) of the board |
| SDA        | I2C data pin (SDA) of the board |
| SA0 / SDO  | Ground: address `0x6A` (default). 3.3 V: address `0x6B`. |

Default I2C pins on common platforms:

| Board | SDA | SCL |
|-------|-----|-----|
| Arduino Nano / Uno | A4 | A5 |
| ESP32 DevKit | GPIO 21 | GPIO 22 |
| Raspberry Pi Pico | GP4 | GP5 |

Default pin assignments vary between board variants. Consult the pinout of the specific board in use.

To use the alternate address `0x6B`, construct the object as follows:

```cpp
BOTGyro gyro(Wire, BOTGYRO_ADDR_HIGH);
```

## Quick Start

```cpp
#include <BOTGyro.h>

BOTGyro gyro;

void setup() {
  Serial.begin(115200);
  if (!gyro.begin()) {
    Serial.println("BOTGyro sensor not found. Check wiring.");
    while (1) {}
  }
  Serial.println("Calibrating - keep the sensor still...");
  gyro.calibrate();
}

void loop() {
  if (gyro.update()) {              // returns true when a new sample is available
    Serial.print(gyro.getX()); Serial.print('\t');
    Serial.print(gyro.getY()); Serial.print('\t');
    Serial.println(gyro.getZ());
  }
}
```

This example prints the angular rate about the X, Y, and Z axes in degrees per second.

## Examples

| Example | Description |
|---------|-------------|
| **BasicReading** | Minimal example that reads the sensor and prints the values. |
| **SerialPlotter** | Output formatted for the Arduino Serial Plotter. |
| **SerialCommander** | Runtime commands (recalibrate, reset, change output format, display status) with human-readable, plotter, and CSV output modes. |
| **SelfBalancingRobot** | A two-wheel self-balancing robot. Tilt angle, obtained by fusing gyroscope and accelerometer data, drives a PID motor controller. |
| **Orientation** | General-purpose usage: raw data and fused pitch and roll, with calibration and filtering presented as optional. |

## API Reference

### Setup

| Method | Description |
|---|---|
| `bool begin()` | Initialises I2C and the sensor. Returns `false` if the sensor is not detected. |
| `void setODR(float hz)` | Sets the output data rate (default `104`). |
| `void setFullScale(int32_t dps)` | Sets the angular-rate range: `250`, `500`, `1000`, or `2000` (default `2000`). A lower range provides finer resolution and reduced jitter but saturates at lower rotation rates. |
| `void setI2CClock(uint32_t hz)` | Sets the I2C clock frequency (default `400000`). Use `100000` where wiring is long or unreliable. |
| `void setCalibrationSamples(uint16_t n)` | Sets the number of samples averaged during calibration (default `600`). |
| `void setKalman(r, qMin, qMax, rampDps, deadzoneK)` | Configures the smoothing filter (see *Filter Tuning*). |

### Calibration

Calibration measures the sensor's resting offset so that it can be subtracted from all subsequent readings. The sensor must remain stationary for the duration of the procedure.

| Method | Description |
|---|---|
| `bool calibrate(uint32_t timeoutMs = 30000)` | Blocking. Returns `true` on success. |
| `void startCalibration()` | Non-blocking. Progress is advanced by calls to `update()`. |
| `bool isCalibrated()` | Indicates whether a valid calibration is currently held. |

### Main Loop

| Method | Description |
|---|---|
| `bool update()` | Must be called frequently. Returns `true` when a new filtered sample is available. |
| `void resetFilters()` | Resets the internal state of the smoothing filter. |

### Angular Rate Data (dps)

- `getX()`, `getY()`, `getZ()`: filtered values.
- `getRawX()`, `getRawY()`, `getRawZ()`: unfiltered values with bias removed.

### Acceleration Data (g)

- `getAccelX()`, `getAccelY()`, `getAccelZ()`: raw acceleration, suitable for tilt calculations.
- `setAccelFullScale(g)`: sets the range to `2`, `4`, `8`, or `16` (default `2`).

### Orientation (degrees)

- `enableAttitude()`: enables fused pitch and roll estimation.
- `getPitch()`, `getRoll()`: return the estimated angles.
- `setAttitudeFilter(alpha)`: sets the complementary-filter constant, in the range `0.90` to `0.99` (default `0.98`). A higher value places greater weight on the gyroscope.

### Options

- `setFilterEnabled(bool)`: enables or disables the Kalman filter (default enabled). When disabled, `getX()`, `getY()`, and `getZ()` return the unfiltered, bias-corrected rate.

### Diagnostics

`getState()`, `isRunning()`, `isCalibrating()`, `isFaulted()`, `stateString()`, `getSampleRate()`, `getBiasX/Y/Z()`, `getNoiseX/Y/Z()`

## Filter Tuning

The filter alternates between two strengths: strong smoothing while the sensor is stationary and rapid response while it is in motion. It is configured with:

```cpp
setKalman(r, qMin, qMax, rampDps, deadzoneK);
```

| Parameter | Description |
|---|---|
| `qMin` | Process noise at rest. A lower value produces a steadier output while stationary. |
| `qMax` | Process noise during motion. A higher value produces a more responsive output during rotation. |
| `rampDps` | Angular rate, in dps, over which the filter transitions from `qMin` to `qMax`. |
| `deadzoneK` | Width of the stationary deadzone, as a multiple of the measured noise standard deviation. |
| `r` | Measurement-noise term (smoothing scale). Typically left at `0.10`. |

Default configuration: `setKalman(0.10, 0.001, 0.20, 8.0, 4.0)`.

Only the ratio of the process-noise values (`q`) to `r` is significant. It is therefore recommended to adjust `qMin` and `qMax` first and to leave the remaining parameters unchanged.

## License

Released under the MIT License. See the `LICENSE` file for details.
