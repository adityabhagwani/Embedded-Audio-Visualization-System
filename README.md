# Embedded Audio Visualization System

A real-time audio-reactive LED system built on an ESP32-S3. A microphone feeds live audio into an FFT pipeline that drives a 300-LED strip — soldered into a serpentine pattern to form a 15×20 pixel grid — through three switchable visualization modes, all running on dual FreeRTOS cores in parallel.

## Features

- **Beat-synced strobe** — flashes the full grid white in time with detected beats, with a smooth brightness decay after each flash.
- **Radial frequency map** — renders concentric rings on the grid, each colored and brightened by the live energy in the bass, mid, or treble band.
- **Lyric display** — renders short text messages on the grid letter-by-letter using a custom 3×5 pixel bitmap font, advanced by a physical push button.
- **Mode switching** — cycle between the three modes with a physical button or serial command; a 16x2 I2C LCD shows the active mode.
- **Ambient-light-aware brightness** — an LDR sensor scales overall strip brightness to the room's ambient lighting.

## Hardware

| Component | Spec | Pin |
|---|---|---|
| LED strip | WS2812B, 300 LEDs, 15×20 serpentine grid | 10 |
| Microphone | MAX9814 (analog) | ADC 1 |
| Ambient light sensor | LDR (analog) | ADC 5 |
| Lyric advance button | Momentary, INPUT_PULLUP | 6 |
| Mode cycle button | Momentary, INPUT_PULLUP | 42 |
| LCD | 16×2, I2C | SDA 11 / SCL 12 |
| Microcontroller | ESP32-S3 | — |

## Architecture

The system runs two FreeRTOS tasks pinned to separate cores, communicating through a shared `SystemState` struct (`shared_state.h`):

- **Core 0 — `audioTask`**: continuously samples the microphone (1024 samples @ ~20.5kHz), runs an FFT with Hann windowing, splits the spectrum into bass/mid/treble band energy, and runs beat detection on the bass band.
- **Core 1 — `LEDTask`**: reads ambient brightness, handles mode switching, and dispatches to whichever visualization mode is active, rendering to the LED strip and refreshing the LCD as needed.

### Beat detection

Beat detection tracks a running average of bass-band energy (incremental mean, periodically reset so it re-adapts to volume/song changes) and flags a beat when the current bass energy exceeds that average by a tunable ratio — gated by a minimum interval derived from a max-BPM setting, so the detector can't fire faster than a musically plausible tempo allows.

This running-average + BPM-cooldown approach is adapted from [blaz-r/ESP32-music-beat-sync](https://github.com/blaz-r/ESP32-music-beat-sync). We had implemented our own normalized-min/max-range beat detector prior to adopting this approach; it's preserved (unused) in the code as `KICK_THRESHOLD`/`KICK_MULTIPLIER`.

### Coordinate mapping

LED indices follow the strip's physical serpentine wiring. `ledIndex(x, y)` maps a logical Cartesian coordinate (origin at the bottom-left of the grid) to the correct physical LED index, accounting for the alternating left-to-right / right-to-left wiring direction of each row — this lets every visualization mode work in plain (x, y) grid coordinates instead of raw strip indices.

## Controls

- **Mode button** (pin 42): cycles strobe → radial frequency map → lyrics → strobe.
- **Lyric button** (pin 6): advances to the next word (only active in lyrics mode).
- **Serial**: send `0`, `1`, or `2` over the serial monitor to jump directly to a mode.

## Tuning

Key constants near the top of `AudioReactiveLEDs.ino`:

| Constant | Purpose |
|---|---|
| `LOW_FREQ_BIN` / `MID_FREQ_BIN` | Band boundaries (Hz) for bass/mid/treble split |
| `BEAT_MAX_BPM` | Fastest tempo the beat detector will track |
| `BEAT_MAG_RATIO` | How far above the running average bass energy must spike to count as a beat |
| `BEAT_AVG_COUNT_LIMIT` / `BEAT_AVG_COUNT_LOWER` | Controls how quickly the running average re-adapts |
| `BASS_DECAY_FACTOR` | Strobe fade-out speed |

## Build

Built in the Arduino IDE for an **ESP32-S3** board target (Tools → Board → ESP32 Dev Module family — not an AVR or mbed-based board).

Dependencies:
- `arduinoFFT`
- `FastLED`
- `LiquidCrystal_I2C`

## File Structure

```
AudioReactiveLEDs/
├── AudioReactiveLEDs.ino   # main sketch: FFT, beat detection, LED rendering, tasks
└── shared_state.h          # SystemState struct shared between audio and LED cores
```

