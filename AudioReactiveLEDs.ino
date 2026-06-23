/**
 * @file AudioReactiveLEDs.ino
 * @brief Audio-reactive LED lighting system with three visualization modes.
 *
 * Runs on an ESP32-S3 using the Arduino framework with FreeRTOS dual-core tasking.
 * Core 0 handles audio sampling and FFT processing. Core 1 handles LED animation
 * and LCD updates. Mode selection is handled via interrupt-driven push buttons.
 *
 * Hardware:
 * - WS2812B LED strip (300 LEDs, 15x20 serpentine grid) on pin 10
 * - MAX9814 microphone on ADC pin 1
 * - LDR ambient light sensor on ADC pin 5
 * - Lyric advance button on pin 6
 * - Mode cycle button on pin 42
 * - 16x2 I2C LCD on SDA=11, SCL=12
 */

#include <arduinoFFT.h>
#include <FastLED.h>
#include <LiquidCrystal_I2C.h>
#include "shared_state.h"
#include <Wire.h>

// LEDs, LDR, Button pin defines
#define LED_PIN 10
#define NUM_LEDS 300
#define LDR_PIN 5
#define BUTTON_PIN 6
#define MODE_BUTTON_PIN 42
#define MIC_PIN 1
#define SDA 11
#define SCL 12

volatile SystemState g_state = {0}; /**< Shared state struct between audio and LED cores. Access is lock-free via volatile. */

volatile bool advanceLyric = false; /**< Set true by buttonISR() to signal lyricalDisplay() to advance to the next word. */

volatile uint8_t encoderMode = 0; /**< Tracks current mode index for the mode button ISR. Mirrors g_state.mode. */

volatile uint8_t modeChanged = 0; /**< Flag set by modeButtonISR() to trigger LCD refresh and strip clear in LEDTask. */

const uint16_t SAMPLES = 1024; /**< FFT sample window size. Must be a power of 2. */
const float SAMPLE_RATE = 20480; /**< ADC sampling rate in Hz. Determines frequency resolution and Nyquist limit. */
const float FREQ_BIN = SAMPLE_RATE / SAMPLES; /**< Hz per FFT bin (~20 Hz). */

const uint16_t LOW_FREQ_BIN = 200;  /**< Upper boundary of the bass band in Hz. */
const uint16_t MID_FREQ_BIN = 2000; /**< Upper boundary of the mid band in Hz. Treble is everything above this. */

const float KICK_THRESHOLD = 0.02;    /**< EMA weight for updating the rolling bass average. Lower = slower adaptation. */
const float KICK_MULTIPLIER = 0.75;   /**< Normalized bass level (0-1) required to trigger a beat flag. */
const float BASS_DECAY_FACTOR = 0.95; /**< Per-step brightness multiplier during strobe decay. 0=instant off, 1=no decay. */

const uint16_t BEAT_MAX_BPM = 180;     /**< Fastest tempo the detector will track. Sets the minimum ms between beats. */
const float BEAT_MAG_RATIO = 1.05f;    /**< Bass energy must exceed this multiple of the running average to count as a beat. */
const uint16_t BEAT_AVG_COUNT_LIMIT = 500; /**< Running average sample count ceiling before it resets to BEAT_AVG_COUNT_LOWER. */
const uint16_t BEAT_AVG_COUNT_LOWER = 100; /**< Sample count the running average resets to, so it re-adapts after a song change. */

const uint16_t LED_COLS = 20; /**< Number of LED columns in the physical grid. */
const uint16_t LED_ROWS = 15; /**< Number of LED rows in the physical grid. */
const uint16_t MAX_COL_IDX = LED_COLS - 1; /**< Maximum valid column index. */
const uint16_t MAX_ROW_IDX = LED_ROWS - 1; /**< Maximum valid row index. */

// 3x5 font bitmap. Each character is 5 rows of 3 bits.
// Bit order: MSB is leftmost pixel. 0b100 = left pixel lit, 0b010 = center, 0b001 = right.
const uint8_t font3x5[26][5] = {
  // A
  {0b010, 0b101, 0b111, 0b101, 0b101},
  // B
  {0b110, 0b101, 0b110, 0b101, 0b110},
  // C
  {0b011, 0b100, 0b100, 0b100, 0b011},
  // D
  {0b110, 0b101, 0b101, 0b101, 0b110},
  // E
  {0b111, 0b100, 0b110, 0b100, 0b111},
  // F
  {0b111, 0b100, 0b110, 0b100, 0b100},
  // G
  {0b011, 0b100, 0b101, 0b101, 0b011},
  // H
  {0b101, 0b101, 0b111, 0b101, 0b101},
  // I
  {0b111, 0b010, 0b010, 0b010, 0b111},
  // J
  {0b111, 0b001, 0b001, 0b101, 0b010},
  // K
  {0b101, 0b101, 0b110, 0b101, 0b101},
  // L
  {0b100, 0b100, 0b100, 0b100, 0b111},
  // M
  {0b101, 0b111, 0b111, 0b101, 0b101},
  // N
  {0b101, 0b111, 0b101, 0b101, 0b101},
  // O
  {0b010, 0b101, 0b101, 0b101, 0b010},
  // P
  {0b110, 0b101, 0b110, 0b100, 0b100},
  // Q
  {0b010, 0b101, 0b101, 0b110, 0b011},
  // R
  {0b110, 0b101, 0b110, 0b101, 0b101},
  // S
  {0b011, 0b100, 0b010, 0b001, 0b110},
  // T
  {0b111, 0b010, 0b010, 0b010, 0b010},
  // U
  {0b101, 0b101, 0b101, 0b101, 0b010},
  // V
  {0b101, 0b101, 0b101, 0b010, 0b010},
  // W
  {0b101, 0b101, 0b111, 0b111, 0b101},
  // X
  {0b101, 0b101, 0b010, 0b101, 0b101},
  // Y
  {0b101, 0b101, 0b010, 0b010, 0b010},
  // Z
  {0b111, 0b001, 0b010, 0b100, 0b111},
};

const char* lyrics[] = {"IM", "LOSIN", "IT"}; /**< Array of lyric words displayed in Mode 2. Max 5 chars each. */
const int lyricCount = 3; /**< Number of entries in the lyrics array. */
volatile int currentWord = -1; /**< Index of the currently displayed lyric word. -1 = nothing displayed yet. */

const uint16_t DEBOUNCE_WINDOW = 50; /**< Minimum milliseconds between valid button presses for software debounce. */

// FFT Globals
float vReal[SAMPLES];  // your audio samples go here
float vImag[SAMPLES];  // always zero-filled before each run
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLE_RATE);

// LCD inititialization
LiquidCrystal_I2C lcd(0x27, 16, 2);  // address 0x27, 16 cols, 2 rows


// LED Globals
CRGB leds[NUM_LEDS];

// Setting up button ISR
/**
 * @brief ISR for the lyric advance button on BUTTON_PIN.
 *
 * Debounced with a 200 ms window. Sets advanceLyric = true on a valid press,
 * which is consumed by lyricalDisplay() on the next LEDTask iteration.
 */
void IRAM_ATTR buttonISR() {
  static unsigned long lastPress = 0;
  unsigned long now = millis();
  if (now - lastPress > DEBOUNCE_WINDOW) {
    advanceLyric = true;
    lastPress = now;
  }
}

// Setting up encoder ISR
/**
 * @brief ISR for the mode cycle button on MODE_BUTTON_PIN.
 *
 * Debounced with a 200 ms window. Increments encoderMode modulo 3,
 * mirrors it to g_state.mode, and sets modeChanged = 1 to trigger
 * an LCD refresh and strip clear in LEDTask.
 */
void IRAM_ATTR modeButtonISR() {
  static unsigned long lastPress = 0;
  unsigned long now = millis();
  if (now - lastPress < DEBOUNCE_WINDOW) return;
  lastPress = now;

  encoderMode = (encoderMode + 1) % 3;
  g_state.mode = encoderMode;
  modeChanged = 1;
}


// FFT Functions
// Calculates FFT, isolates band frequencies, and checks for a kickdrum
/**
 * @brief Runs the full FFT pipeline on the current vReal[] buffer.
 *
 * Zeros vImag[], applies a Hann window, computes the forward FFT,
 * converts to magnitude spectrum, then calls calculate_band_energy()
 * and beat_flag_check() in sequence.
 *
 * @param FFT ArduinoFFT instance bound to vReal[] and vImag[].
 */
void calculate_FFT(ArduinoFFT<float> FFT) {
  memset(vImag, 0, sizeof(vImag));
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  
  calculate_band_energy();

  beat_flag_check();
}

// Helper fn for calculating band frequencies
/**
 * @brief Sums FFT magnitude bins into three frequency bands.
 *
 * Iterates over positive-frequency bins and accumulates energy into
 * bass (0–200 Hz), mid (200–2000 Hz), and treble (>2000 Hz) bands.
 * Results are written to g_state.band_energy[0], [1], and [2].
 */
void calculate_band_energy() {
  float LBE = 0, MBE = 0, HBE = 0;
  for (int i = 1; i < SAMPLES / 2; i++) {
      float currFreq = i * FREQ_BIN;
      if      (currFreq <= LOW_FREQ_BIN)  LBE += vReal[i];
      else if (currFreq <= MID_FREQ_BIN)  MBE += vReal[i];
      else                                HBE += vReal[i];
  }
  g_state.band_energy[0] = LBE;
  g_state.band_energy[1] = MBE;
  g_state.band_energy[2] = HBE;
}

// Compares current bass energy to its running average, gated by a max-BPM cooldown
/**
 * @brief Beat detector using a running-average magnitude ratio, gated by a BPM cooldown.
 *
 * Ported from blaz-r/ESP32-music-beat-sync. Maintains a running mean of bass
 * energy via incremental averaging (mean += (x - mean) / count). The sample
 * count is capped at BEAT_AVG_COUNT_LIMIT and reset down to BEAT_AVG_COUNT_LOWER
 * once reached, so the average re-adapts quickly after a song/volume change
 * instead of being dragged down by a long history.
 *
 * A beat is flagged when the current bass energy exceeds the running average
 * by BEAT_MAG_RATIO, AND at least 60000/BEAT_MAX_BPM ms have passed since the
 * last flagged beat. The cooldown is what prevents beat_flag from firing on
 * every high-rate audioTask iteration.
 */
void beat_flag_check() {
  static float bassAvg = 0.0f;
  static uint16_t avgSampleCount = 1;
  static unsigned long lastBeatMs = 0;

  const unsigned long minBeatIntervalMs = 60000UL / BEAT_MAX_BPM;

  float currentBass = g_state.band_energy[0];

  // incremental running mean
  bassAvg += (currentBass - bassAvg) / avgSampleCount;
  avgSampleCount++;
  if (avgSampleCount > BEAT_AVG_COUNT_LIMIT) {
    avgSampleCount = BEAT_AVG_COUNT_LOWER;
  }

  unsigned long now = millis();
  bool aboveThreshold = currentBass > (bassAvg * BEAT_MAG_RATIO);
  bool cooldownElapsed = (now - lastBeatMs) > minBeatIntervalMs;

  if (aboveThreshold && cooldownElapsed) {
    g_state.beat_flag = true;
    lastBeatMs = now;
  } else {
    g_state.beat_flag = false;
  }
}



// LEDs Functions

// flashes a strobe at every detected beat
/**
 * @brief Mode 0 — Strobes all LEDs white on each detected beat.
 *
 * On a valid beat (g_state.beat_flag), clears the flag and enforces a
 * 200 ms minimum refire window. Performs a blocking brightness decay loop
 * from 255 down to 1 using BASS_DECAY_FACTOR, calling FastLED.show() each
 * step with a 4 ms delay. Blanks the strip if no beat is active.
 */
void bassFlash() { // Mode 0
  static unsigned long lastFlash = 0;

  if(g_state.beat_flag) {
    g_state.beat_flag = false;
    if(millis() - lastFlash > 200) {
      lastFlash = millis();
      
      // full blocking decay — smooth and consistent
      uint8_t brightness = 255;
      while(brightness > 1) {
        fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
        FastLED.show();
        brightness = (uint8_t)(brightness * BASS_DECAY_FACTOR);
        delayMicroseconds(4000); // ~8ms per step, control speed here
      }
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
    }
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
  }
}

// creates circles on the grid each glowing with a brightness given by band freequency energy proportion
/**
 * @brief Mode 1 — Displays three concentric frequency band rings on the LED grid.
 *
 * Computes the proportion of total spectral energy in each band and maps
 * each proportion to a brightness and hue. LEDs within radius 3 of center
 * show mid-band color, radius 3–7 shows treble, and beyond 7 shows bass.
 * Hue ranges: bass 0–85, mid 86–170, treble 171–255.
 *
 * @note Returns immediately if totalEnergy is zero to avoid division by zero.
 */
void radialFreqMap() { // Mode 1 
  float cx = 9.0, cy = 7.0;

  float totalEnergy = g_state.band_energy[0] + g_state.band_energy[1] + g_state.band_energy[2];

  if (totalEnergy == 0) return;  // avoid division by zero in silence

  float bassRatio   = g_state.band_energy[0] / totalEnergy;
  float midRatio    = g_state.band_energy[1] / totalEnergy;
  float trebleRatio = g_state.band_energy[2] / totalEnergy;

  uint8_t bassBrightness   = (uint8_t)(bassRatio   * 255);
  uint8_t midBrightness    = (uint8_t)(midRatio    * 255);
  uint8_t trebleBrightness = (uint8_t)(trebleRatio * 255);

  uint8_t bassHue   = (uint8_t)(bassRatio   * 85);
  uint8_t midHue    = (uint8_t)(midRatio    * 85) + 86;
  uint8_t trebleHue = (uint8_t)(trebleRatio * 85) + 171;

  for (int x = 0; x < LED_COLS; x++) {
    for (int y = 0; y < LED_ROWS; y++) {
      float dist = sqrt(pow(x - cx, 2) + pow(y - cy, 2));
      if (dist <= 3) {
        leds[ledIndex(x, y)] = CHSV(midHue, 255, midBrightness);
      } else if (dist <= 7) {
        leds[ledIndex(x, y)] = CHSV(trebleHue, 255, trebleBrightness);
      } else {
        leds[ledIndex(x, y)] = CHSV(bassHue, 255, bassBrightness);
      }
    }
  }
  /*Serial.print("bassRatio: ");   Serial.print(bassRatio);
  Serial.print("  midRatio: ");  Serial.print(midRatio);
  Serial.print("  trebleRatio: "); Serial.println(trebleRatio);
  Serial.print("bassHue: ");     Serial.print(bassHue);
  Serial.print("  midHue: ");    Serial.print(midHue);
  Serial.print("  trebleHue: "); Serial.println(trebleHue);
  Serial.print("bassBrightness: ");    Serial.print(bassBrightness);
  Serial.print("  midBrightness: ");   Serial.print(midBrightness);
  Serial.print("  trebleBrightness: "); Serial.println(trebleBrightness);*/
  //if(g_state.beat_flag) {
    //triggerColorBassFlash();
  //}
  //else { 
    FastLED.show();
  //}
}


// Flashes pretyped lyrics with a push button
/**
 * @brief Mode 2 — Displays lyric words on the LED grid one word per button press.
 *
 * Checks the advanceLyric flag. When set, increments currentWord (wrapping at
 * lyricCount) and calls displayWord() with the next lyric string.
 */
void lyricalDisplay() { // mode 2
  if (advanceLyric) {
    advanceLyric = false;
    currentWord++;
    if (currentWord >= lyricCount) currentWord = 0;
    displayWord(lyrics[currentWord]);
  }
}

// Checks if lyrics are displayable
/**
 * @brief Validates that all lyric strings are within the 5-character display limit.
 *
 * Prints a warning to Serial for any word that would overflow the grid.
 * Call once in setup() during development to catch font overflow early.
 */
void lyricsCheck() {
  for(int i = 0; i < lyricCount; i++) {
    if(strlen(lyrics[i]) > 5) {
      Serial.print("Lyrics contains words that will overflow. Shorten word to 5 chars at index: ");
      Serial.print(i);
      Serial.print("    Word: ");
      Serial.println(lyrics[i]);
      return;
    }
  }
  Serial.println("Lyrics are printable");
  return;
}

// Prints each word
/**
 * @brief Renders a single lyric word centered on the LED grid.
 *
 * Blanks the strip, computes a centered starting x via centerWord(),
 * then calls drawChar() for each character at 4-pixel horizontal intervals.
 * Calls FastLED.show() once after all characters are drawn.
 *
 * @param word Null-terminated uppercase string to display. Max 5 characters.
 */
void displayWord(const char* word) { // Mode 3
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  int startX = centerWord(word);  // calculate starting x to center the word
  
  for (int i = 0; i < strlen(word); i++) {
    drawChar(word[i], startX + i * 4, 5);  // 4 = char width + spacing, y=5 centers vertically
  }
  FastLED.show();
}

// Creates a strobe with the colors already on the board
/**
 * @brief Performs a brightness decay flash using the LEDs' current colors.
 *
 * Does not set any colors — call this after setting leds[] to the desired
 * color. Decays each LED's brightness in-place each step using nscale8().
 * Useful for adding a beat pulse effect on top of an existing animation.
 */
void triggerColorBassFlash() { // Only creates a flash with existing colors. so set set leds before calling.
  uint8_t brightness = 255;
  while(brightness > 1) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i].nscale8(brightness);
    }
    FastLED.show();
    brightness = brightness * BASS_DECAY_FACTOR;
  }
}

// Triggers a white strobe
/**
 * @brief Stateful white strobe with decay, driven by g_state.beat_flag.
 *
 * On a beat, sets flashing = true and brightness = 255. Each subsequent
 * call decays brightness by BASS_DECAY_FACTOR until it falls below 2,
 * then stops. Non-blocking alternative to bassFlash().
 */
void triggerWhiteBassFlash() {
  static bool flashing = false;
  static uint8_t brightness = 0;

  if(g_state.beat_flag) {
    flashing = true;
    brightness = 255;
    g_state.beat_flag = false;
  }

  if(flashing) {
    fill_solid(leds, NUM_LEDS, CRGB(brightness, brightness, brightness));
    FastLED.show();
    brightness = (uint8_t)(brightness * BASS_DECAY_FACTOR);
    if(brightness < 2) {
      flashing = false;
    }
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
  }
}

// Calculates board brightness based on ambient lighting
/**
 * @brief Reads the LDR and returns a brightness value for FastLED.
 *
 * Maps the 12-bit ADC reading from LDR_PIN (0–4095) linearly to
 * a byte (0–255) suitable for FastLED.setBrightness().
 *
 * @return Brightness scalar in range [0, 255].
 */
uint8_t findBrightness() {
  int ldrValue = analogRead(LDR_PIN);  // 0-4095 on ESP32
  return map(ldrValue, 0, 4095, 0, 255);
}

/**
 * @brief Computes the starting x column to horizontally center a word on the grid.
 *
 * Each character occupies 3 pixels wide with 1 pixel spacing (4 total),
 * except the last character which has no trailing space.
 *
 * @param word Null-terminated string to center.
 * @return Starting column index for the first character.
 */
uint8_t centerWord(const char* word) {
  int wordLen = strlen(word) * 4 - 1;
  return (LED_COLS - wordLen) / 2;
}

// Maps serpentine LED structure to a cartesian plane where origin is the bottom left LED. 
// Assumes a cartesian 1st quadrant where x axis runs horizontally and y axis runs vertically.
/**
 * @brief Maps a logical Cartesian (x, y) coordinate to a physical LED index.
 *
 * Assumes a first-quadrant coordinate system with origin at the bottom-left
 * of the grid. Accounts for serpentine wiring: even physical rows run
 * left-to-right, odd rows run right-to-left.
 *
 * @param x Column index, 0 = leftmost.
 * @param y Row index, 0 = bottom row.
 * @return Index into the leds[] array.
 */
int ledIndex(int x, int y) {
  int physicalRow = MAX_ROW_IDX - y;
  if (physicalRow % 2 == 0) {
    return physicalRow * LED_COLS + x;         // even rows: left to right
  } else {
    return physicalRow * LED_COLS + (MAX_COL_IDX - x);  // odd rows: right to left
  }
}

// Prints an individual character on the grid
/**
 * @brief Renders a single uppercase character onto the LED grid using a 3x5 bitmap font.
 *
 * Looks up the character in font3x5[], iterates over its 5 rows and 3 columns,
 * and sets lit pixels to CRGB::White via ledIndex(). Does not call FastLED.show().
 *
 * @param c   Uppercase ASCII character ('A'–'Z').
 * @param startX Left column of the character's bounding box.
 * @param startY Bottom row of the character's bounding box.
 */
void drawChar(char c, int startX, int startY) {
  int charIdx = c - 'A';  // index into font table
  for (int row = 0; row < 5; row++) {
    for (int col = 0; col < 3; col++) {
      bool lit = (font3x5[charIdx][row] >> (2 - col)) & 1;
      if (lit) {
        leds[ledIndex(startX + col, startY + (4 - row))] = CRGB::White;
      }
    }
  }
}


// Audio Collection Function
/**
 * @brief Collects one full sample window from the MAX9814 microphone.
 *
 * Fills vReal[] with SAMPLES ADC readings from MIC_PIN, center-biased by
 * subtracting 2048 to remove the DC offset. Sample timing is controlled by
 * delayMicroseconds() to approximate SAMPLE_RATE Hz.
 */
void collect_audio() {
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = (float)analogRead(MIC_PIN) - 2048;
    delayMicroseconds(1000000 / SAMPLE_RATE);
  }
}

// LCD Function
/**
 * @brief Refreshes the 16x2 LCD with the current mode name.
 *
 * Prints "Mode:" on row 0 and the mode name on row 1.
 * Only called when modeChanged is set to minimize I2C traffic.
 */
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Mode:");
  lcd.setCursor(0, 1);
  switch (g_state.mode) {
    case 0: lcd.print("Bass Flash");    break;
    case 1: lcd.print("Radial Freq");   break;
    case 2: lcd.print("Lyrics");        break;
  }
  
}


// FreeRTOS Tasks
/**
 * @brief FreeRTOS task for LED animation and LCD control. Pinned to core 1.
 *
 * Each iteration: updates FastLED brightness from the LDR, handles mode
 * transition (strip clear + LCD refresh) if modeChanged is set, then
 * dispatches to bassFlash(), radialFreqMap(), or lyricalDisplay() based
 * on g_state.mode.
 *
 * @param pvParameters Unused FreeRTOS task parameter.
 */
void LEDTask(void* pvParameters) {
  while(1) {
    FastLED.setBrightness(findBrightness());
    if(modeChanged) {
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      updateLCD();
      modeChanged = 0;
      if(g_state.mode == 2) currentWord = -1; // reset lyrics
    }
    switch(g_state.mode) {
      case 0: bassFlash(); break;
      case 1: radialFreqMap(); break;
      case 2: lyricalDisplay(); break;
    }
    vTaskDelay(1);
  }
}

/**
 * @brief FreeRTOS task for audio sampling and FFT processing. Pinned to core 0.
 *
 * Each iteration calls collect_audio() to fill vReal[], then calculate_FFT()
 * to update g_state.band_energy[] and g_state.beat_flag.
 *
 * @param pvParameters Unused FreeRTOS task parameter.
 */
void audioTask(void* pvParameters) {
  while (1) {
    collect_audio();
    calculate_FFT(FFT);
    vTaskDelay(1);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  //tartTime = millis();

  Wire.begin(SDA, SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Mode: BASS FLASH");

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(findBrightness());

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  pinMode(LDR_PIN, INPUT);

  pinMode(MODE_BUTTON_PIN,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON_PIN), modeButtonISR, FALLING);

  xTaskCreatePinnedToCore(audioTask, "FFT Task", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(LEDTask, "LED Task", 10000, NULL, 1, NULL, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '0') { g_state.mode = 0; Serial.println("Mode: Bass Flash"); modeChanged = 1;}
    if (c == '1') { g_state.mode = 1; Serial.println("Mode: Radial Freq"); modeChanged = 1;}
    if (c == '2') { g_state.mode = 2; Serial.println("Mode: Lyrics"); modeChanged = 1;}
  }

}


// Testing Functions
void generate_test_signal1(float *buf, int n, int sample_rate) {
  int check = 0;
  if(millis() % 17 == 0) {
      check = 1;
    }
  for (int i = 0; i < n; i++) {
    buf[i]  = 0.8f * sinf(2 * M_PI * 10000.0f  * i / sample_rate);
    buf[i] += 0.4f * sinf(2 * M_PI * 1100.0f * i / sample_rate);
    if(check) {
      buf[i] += 2.0f * sinf(2 * M_PI * 100.0f * i / sample_rate);
    }
  }
}

void generate_test_signal2(float *buf, int n, int sample_rate) {
  for (int i = 0; i < n; i++) {
    buf[i]  = 0.8f * sinf(2 * M_PI * 20000.0f * i / sample_rate);
    buf[i] += 0.7f * sinf(2 * M_PI * 500.0f * i / sample_rate);
    if(millis() % 16 == 0) {
      buf[i] += 2.0f * sinf(2 * M_PI * 100.0f * i / sample_rate);
    }
  }
}

void testMapper() {
  Serial.println("Corner indices:");
  Serial.print("Bottom-left  (0,0):   "); Serial.println(ledIndex(0, 0));
  Serial.print("Top-left     (0,14):  "); Serial.println(ledIndex(0, 14));
  Serial.print("Bottom-right (19,0):  "); Serial.println(ledIndex(19, 0));
  Serial.print("Top-right    (19,14): "); Serial.println(ledIndex(19, 14));
}

