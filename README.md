# BOTGyro

A general-purpose driver for the **STMicroelectronics ASM330LHH** 6-axis IMU on
Arduino. Use it for whatever you need: raw accelerometer + gyroscope data,
bias-calibrated and noise-filtered angular rates, or fused pitch/roll
orientation. Calibration and filtering are **optional** — `begin()` gives you
live data immediately, and you opt into the rest.

Angular rate is reported in **degrees per second (dps)**, acceleration in **g**,
and orientation in **degrees**.

## Features

- **Full 6-axis access** — raw accelerometer (g) and gyroscope (dps), available
  immediately after `begin()` with no required setup.
- **Optional bias calibration** with **disturbance validation** — calibration is
  rejected and retried if the board was moving while it ran.
- **Optional adaptive Kalman filter** per gyro axis — heavy smoothing when still,
  fast response when rotating; toggle it on or off.
- **Optional fused orientation** — pitch & roll from a complementary filter
  (gyro + accelerometer), drift-free.
- **Non-blocking** — never blocks your loop with `delay()`.
- **Sensor watchdog + auto-recovery** — a loose wire or brown-out is detected and
  the sensor is re-initialised automatically.
- **Diagnostics** — live measured sample rate, plus per-axis bias and noise.

## Requirements

- An Arduino board with I2C (developed/tested on the **Arduino Nano**).
- **No external libraries** — BOTGyro talks to the ASM330LHH directly using the
  built-in `Wire` library.

> The ASM330LHH is a 3.3 V part. Power the bare chip from 3.3 V. Breakout boards
> with an onboard regulator/level-shifter may accept 5 V — check your board.

## Installation

1. **Sketch → Include Library → Add .ZIP Library…** and select `BOTGyro.zip`.
   (Or copy the `BOTGyro` folder into your `Arduino/libraries`.)
2. Restart the IDE. Examples appear under **File → Examples → BOTGyro**.

No other libraries are required.

## Wiring (I2C)

| ASM330LHH | Arduino Nano |
|-----------|--------------|
| VDD       | 3.3V         |
| GND       | GND          |
| SCL       | A5           |
| SDA       | A4           |
| SA0 / SDO | GND → addr `0x6A` (default) · 3.3V → addr `0x6B` |

For address `0x6B`, construct with `BOTGyro gyro(Wire, BOTGYRO_ADDR_HIGH);`.

## Quick start

```cpp
#include <BOTGyro.h>

BOTGyro gyro;

void setup() {
  Serial.begin(115200);
  if (!gyro.begin()) {
    Serial.println("ASM330LHH not found.");
    while (1) {}
  }
  Serial.println("Calibrating - keep the sensor still...");
  gyro.calibrate();
}

void loop() {
  if (gyro.update()) {              // true on each fresh sample
    Serial.print(gyro.getX()); Serial.print('\t');
    Serial.print(gyro.getY()); Serial.print('\t');
    Serial.println(gyro.getZ());
  }
}
```

## Examples

- **BasicReading** — minimal read-and-print.
- **SerialPlotter** — output formatted for the Arduino Serial Plotter.
- **SerialCommander** — runtime commands (recalibrate, reset, switch output
  format, status) plus human / plotter / CSV output modes.
- **SelfBalancingRobot** — a two-wheel balancing robot: complementary-filter
  tilt angle (gyro + accelerometer fusion) driving a PID motor controller.
- **Orientation** — general-purpose use: raw data plus fused pitch/roll, with
  calibration and filtering shown as optional.

## API reference

### Setup
| Method | Description |
|---|---|
| `bool begin()` | Initialise I2C and the sensor. Returns `false` if not found. |
| `void setODR(float hz)` | Output data rate (default `104`). |
| `void setFullScale(int32_t dps)` | Range: `250/500/1000/2000` (default `2000`). Lower = finer resolution, less source jitter, but clips sooner. |
| `void setI2CClock(uint32_t hz)` | I2C clock (default `400000`; use `100000` if wiring is marginal). |
| `void setCalibrationSamples(uint16_t n)` | Samples to average (default `600`). |
| `void setKalman(r, qMin, qMax, rampDps, deadzoneK)` | Filter tuning (see below). |

### Calibration
| Method | Description |
|---|---|
| `bool calibrate(uint32_t timeoutMs = 30000)` | Blocking. Hold still; `true` on success. |
| `void startCalibration()` | Non-blocking; progressed by `update()`. |
| `bool isCalibrated()` | Whether a valid calibration is held. |

### Loop
| Method | Description |
|---|---|
| `bool update()` | Call often. Returns `true` on each new filtered sample. |
| `void resetFilters()` | Re-zero the Kalman state. |

### Data (dps)
`getX() getY() getZ()` — filtered · `getRawX() getRawY() getRawZ()` — raw, bias removed.

### Accelerometer (g)
`getAccelX() getAccelY() getAccelZ()` — raw acceleration, e.g. for tilt-angle
fusion. `setAccelFullScale(g)` sets the range (`2/4/8/16`, default `2`).

### Orientation (deg)
`enableAttitude()` turns on fused pitch/roll; then `getPitch()` and `getRoll()`.
`setAttitudeFilter(alpha)` sets the complementary-filter constant (`0.90`–`0.99`,
default `0.98`).

### Options
`setFilterEnabled(bool)` toggles the gyro Kalman filter (default on; when off,
`getX/Y/Z` return the raw bias-corrected rate).

### Diagnostics
`getState()`, `isRunning()`, `isCalibrating()`, `isFaulted()`, `stateString()`,
`getSampleRate()`, `getBiasX/Y/Z()`, `getNoiseX/Y/Z()`.

## Tuning the filter

The filter adapts between two strengths. Tune via `setKalman(r, qMin, qMax, rampDps, deadzoneK)`:

- **`qMin`** — process noise at rest. *Lower = flatter/smoother* when still.
- **`qMax`** — process noise in motion. *Higher = snappier* when rotating.
- **`rampDps`** — how many dps of motion it takes to ramp from `qMin` to `qMax`.
- **`deadzoneK`** — motion deadzone as a multiple of the measured noise std.
- **`r`** — measurement-noise term (smoothing scale); usually left at `0.10`.

Defaults: `setKalman(0.10, 0.001, 0.20, 8.0, 4.0)`. Only the ratio of `q` to `r`
matters, so start by adjusting `qMin`/`qMax`.

## License

MIT — see `LICENSE`.
