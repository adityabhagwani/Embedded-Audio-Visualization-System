# Embedded-Audio-Visualization-System
Real-time audio-reactive LED system on a dual-core ESP32-S3. FFT based beat detection and frequency-band analysis drive a 300-LED serpentine grid through three modes: beat-synced strobe, radial frequency visualization, and pixel-font text display. FreeRTOS splits audio sampling and LED rendering across separate cores, synced via shared state.
