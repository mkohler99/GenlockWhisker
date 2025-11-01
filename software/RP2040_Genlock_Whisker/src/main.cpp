#include <Arduino.h>
#include <FastLED.h>
#include "font7x5.h"

//GENLOCK WHISKER
//2025 M.KOHLER

//USES LMH1980 SYNC SEPARATOR TO READ GENLOCK AND DISPLAY FORMAT ON MATRIX LED DISPLAY

// ===== Matrix & LED =====
#define LED_PIN        4
#define MATRIX_WIDTH   15
#define MATRIX_HEIGHT  7
#define NUM_LEDS       (MATRIX_WIDTH * MATRIX_HEIGHT)

// ===== LMH1980 Pins =====
#define CSYNC_PIN      23   // LMH1980 CSYNC -> GPIO23
#define VSYNC_PIN      24   // LMH1980 VSYNC -> GPIO24 (used for FPS/jitter)
#define HSYNC_PIN      25   // LMH1980 HSYNC -> GPIO25 (optional)
#define HD_DET_PIN     28   // LMH1980 HD detect -> GPIO28 (LOW=tri-level, HIGH=bi-level)

// ===== Measurement config =====
// Reject edges that are too close together to be real frame edges.
// 59.94 Hz period ≈ 16,683 µs; 60 Hz ≈ 16,667 µs. Anything <5 ms is bogus.
static const uint32_t kMinValidFramePeriodUs = 5000;
// If no VSYNC arrives within this interval, consider "NO LOCK".
static const uint32_t kNoLockTimeoutUs = 250000; // 0.25 s

CRGB leds[NUM_LEDS];

// ===== STATISTICS =====
volatile uint32_t lastVsyncMicros = 0;
volatile uint32_t vsyncCount      = 0;   // valid frame-to-frame intervals counted this window
volatile uint64_t sumPeriodsUs    = 0;   // sum of periods (µs)
volatile uint64_t sumSqPeriodsUs2 = 0;   // sum of period^2 (µs^2) for RMS jitter
volatile uint32_t minPeriodUs     = 0xFFFFFFFF;
volatile uint32_t maxPeriodUs     = 0;

// ===== UI state =====
float measuredFps = 0.0f;
String syncLabel  = "NO LOCK";
uint8_t scrollOffset = 0;

// ===== Matrix mapping: column-wise, top-to-bottom (your wiring) =====
inline int XY(int x, int y) {
  return x * MATRIX_HEIGHT + y;
}


//HELPER TO DRAW CHARACTER TO SCREEN
void drawChar(char c, int xOffset, CRGB color) {
  if (c < 32 || c > 126) return;
  const uint8_t* bitmap = font7x5[c - 32];
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


//HELPER TO DISPLAY TEXT ON SCREEN
void showText(const String& text, int offset, CRGB color) {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  for (int i = 0; i < text.length(); i++)
    drawChar(text[i], i * 6 - offset, color);
  FastLED.show();
}

// ===== DETECT VSYNC PULSE =====
void vsyncISR() {
  uint32_t now = micros();
  uint32_t prev = lastVsyncMicros;
  lastVsyncMicros = now;

  if (prev != 0) {
    uint32_t period = now - prev;
    if (period >= kMinValidFramePeriodUs) {
      vsyncCount++;
      sumPeriodsUs    += period;
      sumSqPeriodsUs2 += (uint64_t)period * (uint64_t)period;
      if (period < minPeriodUs) minPeriodUs = period;
      if (period > maxPeriodUs) maxPeriodUs = period;
    }
  }
}

// LOW => tri-level HD; HIGH => bi-level SD (PER PAGE 5 IN DATASHEET)
static inline bool triLevelDetected() {
  return digitalRead(HD_DET_PIN) == LOW;
}

CRGB colorForSync(const String& label) {
  if (label == "TRI-LEVEL") return CRGB::Blue;
  if (label == "BI-LEVEL")  return CRGB::Green;
  return CRGB::Red;
}

void setup() {
  Serial.begin(115200);

  pinMode(CSYNC_PIN, INPUT);
  pinMode(VSYNC_PIN, INPUT);
  pinMode(HSYNC_PIN, INPUT);
  pinMode(HD_DET_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(VSYNC_PIN), vsyncISR, RISING);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(32);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  Serial.println("Genlock Analyzer Started");
}

void loop() {
  static uint32_t lastCalcMs = 0;
  static uint32_t lastUiMs   = 0;

  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  // Once per second: compute FPS and jitter stats
  if (nowMs - lastCalcMs >= 1000) {
    uint32_t count, lastEdgeUs, minP, maxP;
    uint64_t sumP, sumSqP;

    noInterrupts();
    count     = vsyncCount;
    sumP      = sumPeriodsUs;
    sumSqP    = sumSqPeriodsUs2;
    minP      = (minPeriodUs == 0xFFFFFFFF) ? 0 : minPeriodUs;
    maxP      = maxPeriodUs;
    lastEdgeUs = lastVsyncMicros;

    // reset for next window
    vsyncCount      = 0;
    sumPeriodsUs    = 0;
    sumSqPeriodsUs2 = 0;
    minPeriodUs     = 0xFFFFFFFF;
    maxPeriodUs     = 0;
    interrupts();

    double fps = 0.0;
    double meanPeriodUs = 0.0;
    double rmsJitterUs  = 0.0;
    uint32_t ppJitterUs = 0;

    if (count >= 1 && sumP > 0) {
      // 'count' periods were accumulated (i.e., frames-1 if you counted edges, but we add on each valid period)
      meanPeriodUs = (double)sumP / (double)count;
      fps = 1e6 / meanPeriodUs;

      // Variance = E[p^2] - (E[p])^2
      double meanSq = (double)sumSqP / (double)count;
      double meanP  = meanPeriodUs;
      double var    = meanSq - (meanP * meanP);
      if (var < 0) var = 0;   // numerical guard
      rmsJitterUs = sqrt(var);

      if (minP > 0 && maxP >= minP) {
        ppJitterUs = maxP - minP;
      }
    }

    measuredFps = (float)fps;

    bool locked = (lastEdgeUs != 0) && ((uint32_t)(nowUs - lastEdgeUs) < kNoLockTimeoutUs);
    if (!locked || measuredFps < 1.0f) {
      syncLabel = "NO LOCK";
    } else {
      syncLabel = triLevelDetected() ? "TRI-LEVEL" : "BI-LEVEL";
    }

    // Jitter in ppm relative to the mean period (if valid)
    double jitterPpm = 0.0;
    if (meanPeriodUs > 0.0) {
      jitterPpm = (rmsJitterUs / meanPeriodUs) * 1e6;
    }

    Serial.printf(
      "Frames:%lu  FPS:%.3f  Type:%s  HD_DET:%d  "
      "Period_us_mean:%.3f  Jitter_RMS_us:%.3f  Jitter_pp_us:%lu  Jitter_RMS_ppm:%.1f\n",
      (unsigned long)count,
      measuredFps,
      syncLabel.c_str(),
      digitalRead(HD_DET_PIN),
      meanPeriodUs,
      rmsJitterUs,
      (unsigned long)ppJitterUs,
      jitterPpm
    );

    lastCalcMs = nowMs;
  }

  // Update LED text 
  if (nowMs - lastUiMs >= 80) {
    String text;
    if (syncLabel == "NO LOCK") {
      text = "NO LOCK";
    } else {
      char buf[32];
      snprintf(buf, sizeof(buf), "%s %.3f", syncLabel.c_str(), measuredFps);
      text = String(buf);
    }
    showText(text, scrollOffset, colorForSync(syncLabel));
    scrollOffset++;
    int maxOffset = text.length() * 6;
    if (scrollOffset > maxOffset) scrollOffset = 0;
    lastUiMs = nowMs;
  }
}
