#include "font7x5.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/structs/systick.h"
#include "vsync.pio.h" // Automatically generated from src/vsync.pio
#include <Arduino.h>
#include <FastLED.h>

// GENLOCK WHISKER
// 2025 M.KOHLER

// USES LMH1980 SYNC SEPARATOR TO READ GENLOCK AND DISPLAY FORMAT ON MATRIX LED
// DISPLAY

// ===== Matrix & LED =====
#define LED_PIN 4
#define MATRIX_WIDTH 15
#define MATRIX_HEIGHT 7
#define NUM_LEDS (MATRIX_WIDTH * MATRIX_HEIGHT)

// ===== LMH1980 Pins =====
#define CSYNC_PIN 23 // LMH1980 CSYNC -> GPIO23
#define VSYNC_PIN 24 // LMH1980 VSYNC -> GPIO24 (used for FPS/jitter)
#define HSYNC_PIN 25 // LMH1980 HSYNC -> GPIO25 (optional)
#define HD_DET_PIN                                                             \
  28 // LMH1980 HD detect -> GPIO28 (LOW=tri-level, HIGH=bi-level)

// ===== Measurement config =====
// Reject edges that are too close together to be real frame edges.
// 59.94 Hz period ≈ 16,683 µs; 60 Hz ≈ 16,667 µs. Anything <5 ms is bogus.
static const uint32_t kMinValidFramePeriodUs = 5000;
// If no VSYNC arrives within this interval, consider "NO LOCK".
static const uint32_t kNoLockTimeoutUs = 250000; // 0.25 s

CRGB leds[NUM_LEDS];

// ===== STATISTICS =====
volatile uint32_t vsyncCount = 0; // valid frame-to-frame intervals counted
volatile uint64_t sumPeriodsCycles = 0;   // sum of periods (cycles)
volatile uint64_t sumSqPeriodsCycles = 0; // sum of period^2 (cycles^2)
volatile uint32_t minPeriodCycles = 0xFFFFFFFF;
volatile uint32_t maxPeriodCycles = 0;

volatile uint16_t lastHsyncCount = 0;
volatile uint32_t linesPerFrame = 0;
uint slice_num;

// PIO Globals
PIO pio = pio0;
uint sm = 0;

// ===== UI state =====
float measuredFps = 0.0f;
String syncLabel = "NO LOCK";
uint8_t scrollOffset = 0;

// ===== Matrix mapping: column-wise, top-to-bottom (your wiring) =====
inline int XY(int x, int y) { return x * MATRIX_HEIGHT + y; }

// HELPER TO DRAW CHARACTER TO SCREEN
void drawChar(char c, int xOffset, CRGB color) {
  if (c < 32 || c > 126)
    return;
  const uint8_t *bitmap = font7x5[c - 32];
  for (int x = 0; x < 5; x++) {
    uint8_t col = bitmap[x];
    for (int y = 0; y < 7; y++) {
      if (col & (1 << y)) {
        int xPos = xOffset + x;
        if (xPos >= 0 && xPos < MATRIX_WIDTH)
          leds[XY(xPos, y)] = color;
      }
    }
  }
}

// HELPER TO DISPLAY TEXT ON SCREEN
void showText(const String &text, int offset, CRGB color) {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  for (int i = 0; i < text.length(); i++)
    drawChar(text[i], i * 6 - offset, color);
  FastLED.show();
}

// LOW => tri-level HD; HIGH => bi-level SD (PER PAGE 5 IN DATASHEET)
static inline bool triLevelDetected() { return digitalRead(HD_DET_PIN) == LOW; }

CRGB colorForSync(const String &label) {
  if (label == "TRI-LEVEL")
    return CRGB::Blue;
  if (label == "BI-LEVEL")
    return CRGB::Green;
  return CRGB::Red;
}

void setup() {
  Serial.begin(115200);

  pinMode(CSYNC_PIN, INPUT);
  // VSYNC_PIN is handled by PIO
  pinMode(HD_DET_PIN, INPUT);

  // Setup PWM Counter for HSYNC (GPIO 25)
  gpio_set_function(HSYNC_PIN, GPIO_FUNC_PWM);
  slice_num = pwm_gpio_to_slice_num(HSYNC_PIN);
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv_mode(
      &cfg,
      PWM_DIV_B_FALLING); // Count on falling edge (active low leading edge)
  pwm_config_set_clkdiv(&cfg, 1.0f);
  pwm_config_set_wrap(&cfg, 65535); // Ensure full 16-bit range
  pwm_init(slice_num, &cfg, true);

  // Setup PIO for VSYNC measurement
  uint offset = pio_add_program(pio, &vsync_program);
  sm = pio_claim_unused_sm(pio, true);

  // Configure PIO:
  // - JMP pin: VSYNC_PIN (for conditional jumps)
  // - Input pin: VSYNC_PIN (for wait instructions)
  pio_sm_config c = vsync_program_get_default_config(offset);
  sm_config_set_jmp_pin(&c, VSYNC_PIN);
  sm_config_set_in_pins(&c, VSYNC_PIN);

  // Initialize GPIO for PIO
  pio_gpio_init(pio, VSYNC_PIN);

  // Load config and start
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(32);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  Serial.println("Genlock Analyzer Started (PIO Mode)");
}

void loop() {
  static uint32_t lastCalcMs = 0;
  static uint32_t lastUiMs = 0;
  static uint32_t lastVsyncTimeMs = 0; // For timeout detection

  uint32_t nowMs = millis();

  // --- POLL PIO FIFO ---
  while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
    uint32_t remaining = pio_sm_get_blocking(pio, sm);

    // Calculate period in cycles
    // Formula: Period = (0xFFFFFFFF - remaining) * 2 + 3
    // The +3 is constant overhead (jmp pin + jmp fell + etc)
    uint32_t periodCycles = (0xFFFFFFFF - remaining) * 2 + 3;

    // Read HSYNC counter immediately to correlate with this frame
    uint16_t currentHsyncCount = pwm_get_counter(slice_num);
    uint16_t lines = (uint16_t)(currentHsyncCount - lastHsyncCount);
    lastHsyncCount = currentHsyncCount;

    // Filter invalid short pulses
    const uint32_t kMinCycles = 5000 * (F_CPU / 1000000);
    if (periodCycles >= kMinCycles) {
      vsyncCount++;
      sumPeriodsCycles += periodCycles;
      sumSqPeriodsCycles += (uint64_t)periodCycles * (uint64_t)periodCycles;

      if (periodCycles < minPeriodCycles)
        minPeriodCycles = periodCycles;
      if (periodCycles > maxPeriodCycles)
        maxPeriodCycles = periodCycles;

      linesPerFrame = lines;   // Update global lines count
      lastVsyncTimeMs = nowMs; // Reset timeout
    }
  }

  // 10Hz update rate (100ms)
  if (nowMs - lastCalcMs >= 100) {
    uint32_t count, minP, maxP, lines;
    uint64_t sumP, sumSqP;

    // Snapshot stats
    count = vsyncCount;
    sumP = sumPeriodsCycles;
    sumSqP = sumSqPeriodsCycles;
    minP = (minPeriodCycles == 0xFFFFFFFF) ? 0 : minPeriodCycles;
    maxP = maxPeriodCycles;
    lines = linesPerFrame;

    // Reset stats
    vsyncCount = 0;
    sumPeriodsCycles = 0;
    sumSqPeriodsCycles = 0;
    minPeriodCycles = 0xFFFFFFFF;
    maxPeriodCycles = 0;

    double fps = 0.0;
    double meanPeriodUs = 0.0;
    double rmsJitterUs = 0.0;
    double ppJitterUs = 0.0;
    double cyclesToUs = 1000000.0 / (double)F_CPU;

    if (count >= 1 && sumP > 0) {
      double meanCycles = (double)sumP / (double)count;
      meanPeriodUs = meanCycles * cyclesToUs;
      fps = 1000000.0 / meanPeriodUs;

      // Variance = E[p^2] - (E[p])^2
      double meanSq = (double)sumSqP / (double)count;
      double varCycles = meanSq - (meanCycles * meanCycles);
      if (varCycles < 0)
        varCycles = 0;

      double rmsCycles = sqrt(varCycles);
      rmsJitterUs = rmsCycles * cyclesToUs;

      if (minP > 0 && maxP >= minP) {
        ppJitterUs = (double)(maxP - minP) * cyclesToUs;
      }
    }

    measuredFps = (float)fps;

    // Determine lock status
    // Check if we have received data recently (within 250ms)
    bool locked = (nowMs - lastVsyncTimeMs < 250);

    if (!locked) {
      syncLabel = "NO LOCK";
      measuredFps = 0.0; // Force 0 if no lock
      lines = 0;
    } else {
      syncLabel = triLevelDetected() ? "TRI-LEVEL" : "BI-LEVEL";
    }

    // Jitter in ppm relative to the mean period (if valid)
    double jitterPpm = 0.0;
    if (meanPeriodUs > 0.0) {
      jitterPpm = (rmsJitterUs / meanPeriodUs) * 1e6;
    }

    Serial.print("{");
    Serial.print("\"frames\":");
    Serial.print(count);
    Serial.print(",");
    Serial.print("\"lines\":");
    Serial.print(lines);
    Serial.print(",");
    Serial.print("\"fps\":");
    Serial.print(measuredFps, 3);
    Serial.print(",");
    Serial.print("\"type\":\"");
    Serial.print(syncLabel);
    Serial.print("\",");
    Serial.print("\"hd_det\":");
    Serial.print(digitalRead(HD_DET_PIN));
    Serial.print(",");
    Serial.print("\"period_us_mean\":");
    Serial.print(meanPeriodUs, 4);
    Serial.print(",");
    Serial.print("\"jitter_rms_us\":");
    Serial.print(rmsJitterUs, 4);
    Serial.print(",");
    Serial.print("\"jitter_pp_us\":");
    Serial.print(ppJitterUs, 4);
    Serial.print(",");
    Serial.print("\"jitter_rms_ppm\":");
    Serial.print(jitterPpm, 1);

    Serial.println("}");

    lastCalcMs = nowMs;
  }

  // Update LED text content (1Hz)
  static uint32_t lastTextUpdateMs = 0;
  static String currentText = "   NO LOCK";
  static CRGB currentColor = CRGB::Red;

  if (nowMs - lastTextUpdateMs >= 1000) {
    String text;
    CRGB color = CRGB::Red; // Default NO LOCK color

    if (syncLabel == "NO LOCK") {
      text = "NO LOCK";
    } else {
      // Determine format string (simplified port of JS logic)
      String fmt = "";
      float fps = measuredFps;
      int lines = linesPerFrame;
      bool isHd = (digitalRead(HD_DET_PIN) == LOW); // LOW = Tri-level (HD)

      // Helper for FPS string
      char fpsBuf[16];
      snprintf(fpsBuf, sizeof(fpsBuf), "%.3f", fps);
      String fpsStr = String(fpsBuf);

      if (!isHd) {
        // SD (Bi-level)
        if (abs(lines - 525) < 10 || abs(lines - 262) < 5) {
          fmt = "NTSC";
          color = CRGB::Green;
        } else if (abs(lines - 625) < 10 || abs(lines - 312) < 5) {
          fmt = "PAL";
          color = CRGB::Yellow;
        } else {
          fmt = "SD";
          color = CRGB::White;
        }
      } else {
        // HD (Tri-level)
        if (abs(lines - 750) < 10) {
          fmt = "720p";
          color = CRGB::Blue;
        } else if (abs(lines - 1125) < 10 || abs(lines - 562) < 5) {
          // Distinguish 1080i vs 1080p based on lines per VSYNC
          // 1080i field = ~562 lines. 1080p frame = ~1125 lines.
          if (abs(lines - 562) < 5) {
            fmt = "1080i";
            color = CRGB::Magenta;
          } else {
            fmt = "1080p";
            color = CRGB::Blue;
          }
        } else {
          fmt = "HD";
          color = CRGB::White;
        }
      }

      // Format: [TYPE] [RES] [FPS]Hz
      // e.g. TRI-LEVEL 720p 60Hz
      // Abbreviate LEVEL to LVL for LED display
      String ledLabel = syncLabel;
      ledLabel.replace("LEVEL", "LVL");

      char buf[64];
      snprintf(buf, sizeof(buf), "%s %s %sHz", ledLabel.c_str(), fmt.c_str(),
               fpsStr.c_str());
      text = String(buf);
    }

    // Add padding to start so it scrolls in from the right
    currentText = "   " + text;
    currentColor = color;
    lastTextUpdateMs = nowMs;
  }

  // Update LED Scroll (Animation)
  if (nowMs - lastUiMs >= 80) {
    showText(currentText, scrollOffset, currentColor);
    scrollOffset++;
    int maxOffset = currentText.length() * 6;
    if (scrollOffset > maxOffset)
      scrollOffset = 0;
    lastUiMs = nowMs;
  }
}
