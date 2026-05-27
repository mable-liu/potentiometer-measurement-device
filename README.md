# LEGO Block Measurement System

A custom linear measurement device built from first principles — no commercial distance sensor — that measures the length of 2×4 LEGO bricks to ±1.47 mm using a rotary potentiometer, a hinged linkage, and an Arduino Uno.

## The Problem

Design and calibrate a measurement system for LEGO brick dimensions without using an off-the-shelf distance gauge. The system had to demonstrate a full measurement chain — sensing, signal conditioning, calibration, and uncertainty analysis — and meet a ±2 mm spec.

## How It Works

The device converts **angle to length**:

1. The brick sits on a hinged platform with the end-to-be-measured protruding outward.
2. The user pushes the platform up until the brick contacts the underside of a fixed roof.
3. A two-arm linkage couples the platform to a rotary potentiometer, lifting the platform rotates the pot shaft.
4. The Arduino reads the pot voltage via the ADC, converts it to an angle, and applies a calibration curve to output the brick length in mm.

**Measurement chain:** brick length → platform lift → arm rotation → pot voltage → ADC counts → angle → calibrated length

## Build

**Mechanical:** 3D-printed frame, hinged platform, two linkage arms, mechanical stop for repeatable zero.

**Electronics:** Arduino Uno, rotary potentiometer wired as a voltage divider across 5 V, breadboard.

**Cost:** under $20 in parts.

## Calibration & Results

Calibrated against 11 reference lengths measured with vernier calipers, spanning ~42–128 mm. Fit a linear least-squares model with bias correction:


| Metric | Value |
|--------|-------|
| R² | 0.9986 |
| Max absolute deviation | 1.47 mm |
| Deviation range | −1.15 to +1.47 mm |
| Spec | ±2 mm ✓ |


## Signal Processing

The raw ADC readings are noisy and the mechanical linkage has small jitter, so the Arduino code applies a three-stage filter before computing length:

1. **32× oversampling** on each ADC read to reduce quantization noise
2. **5-sample median filter** to reject spikes
3. **Exponential moving average** (α = 0.25) to smooth the remaining variation
4. **Stability gate** — a reading is only flagged as "stable" after the angle stays within ±0.15° for 250 ms, so the user knows when to record

The calibration fit is done on-Arduino at startup using a closed-form linear least-squares solve over the calibration pairs, then de-biased against the mean residual.

## What I'd Do Differently

- Replace cardboard pads with rigid printed mounts — the compliance was a measurable source of variance.
- Try a higher-resolution sensor (encoder or higher-grade pot) — the ADC was not the limiting factor; mechanical play was.
- Fit a second-order calibration model to capture the small nonlinearity visible in the deviation plot.
- Add a clamping or push mechanism to remove user variability in "how firmly the bed is pushed up."

## Files

- `measurement_device.ino` — Arduino sketch
- `overall-device-photo.jpg`, `closeup-device-photo.jpg` — build photos
- `device-sketch.jpg` — mechanical design sketch
- `circuit-schematic-arduino.jpg` — wiring diagram
- `calibration-plot.jpg` — calibration curve
