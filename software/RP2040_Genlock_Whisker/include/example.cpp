// ======================= main.cpp (RP2040 badge) =======================
#include <Arduino.h>
#include <FastLED.h>
#include <FastLED_NeoMatrix.h>
#include <Adafruit_GFX.h>
#include <IRremote.hpp>
#include <LittleFS.h>
#include <ArduinoJson.h>

extern "C" {
  #include "hardware/watchdog.h"
}

#include "hardware/vreg.h"


// project headers you already have
#include "config.hpp"          // BadgeConfig CFG; loadConfig/saveConfig
#include "serial_cmds.hpp"     // void handleSerial();
#include "protocol.h"          // just the op codes if you keep them
#include "charms565.hpp"
#include "scene_nyan.hpp"
#include "scene_fuse.hpp"
#include "scene_kitty.hpp"
#include "scene_volt.hpp"
#include "scene_hit.hpp"
#include "scene_hitbars.hpp"
#include "scene_tx.hpp"
//#include "scenes.hpp"

//#define SCORE_SNIFFER
//#define LOUD_SERIAL

// ---- Sleep/Wake ----
#ifndef SCORE_SNIFFER
  #ifndef AUTO_SLEEP_MINUTES_DEFAULT
  #define AUTO_SLEEP_MINUTES_DEFAULT 15   // auto-sleep after 30 min of inactivity
  #endif
#endif


#ifdef SCORE_SNIFFER
  #ifndef AUTO_SLEEP_MINUTES_DEFAULT
  #define AUTO_SLEEP_MINUTES_DEFAULT 60   // auto-sleep after 30 min of inactivity
  #endif
#endif


static uint32_t btnLastMs = 0;
static bool prevB1 = true, prevB2 = true, prevB3 = true;

const char BUILD_DATE[] = __DATE__;
const char BUILD_TIME[] = __TIME__;

CRGB leds[NUM_LEDS];

// --- Fast blit: RGB565 (PROGMEM) -> FastLED CRGB in-place ---
static inline void blit565ToLeds(const uint16_t *frame565) {
  for (uint16_t i = 0; i < SCN_PIX; ++i) {
    uint16_t p = pgm_read_word(&frame565[i]);     // read from flash
    // unpack 5/6/5 to 8/8/8
    uint8_t r = ((p >> 11) & 0x1F) * 255 / 31;
    uint8_t g = ((p >> 5)  & 0x3F) * 255 / 63;
    uint8_t b = ( p        & 0x1F) * 255 / 31;
    leds[i] = CRGB(r, g, b);
  }
}

// ----- Fancy text effects -----
enum TextEffect : uint8_t { EFFECT_SOLID=0, EFFECT_RAINBOW=1, EFFECT_FIRE=2, EFFECT_GLITCH=3, EFFECT_ICE=4, EFFECT_MATRIX=5, EFFECT_FIREWORK=6, EFFECT_CHEVRON=7, EFFECT_BUGS=8, };
static TextEffect currentTextEffect = EFFECT_SOLID;
static uint8_t numberOfEffects = 9;

// params/state used by effects
static CRGB textColor = CRGB(255,255,255);  // used by SOLID
static uint8_t rainbowHue = 0;              // used by RAINBOW

FastLED_NeoMatrix* matrix = new FastLED_NeoMatrix(
  leds, WIDTH, HEIGHT,
  NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE
);

// ---------------- Config / state ----------------
BadgeConfig CFG;

// ================== Scoring state + helpers ==================
struct ScoreState {
  uint32_t score = 0;          // running total
  uint32_t hits_total = 0;     // any FIRE seen
  uint32_t fires_total = 0;    // local button-2 fires
  uint32_t sessions_woke = 0;  // WAKE events seen
  uint32_t scenes_triggered = 0;

  uint32_t rolls_5 = 0, rolls_10 = 0, rolls_20 = 0, rolls_40 = 0; // audit
} g_score;

// persistent meta for throttled saves
static bool     g_scoreDirty = false;
static uint32_t g_scoreLastSave = 0;

// Simple, stable RNG roll in [1..sides]
static uint16_t scoreRoll(uint8_t sides) {
  if (sides < 1) sides = 1;
  // randomSeed should be set in setup(); this is inclusive
  return (uint16_t)random(1, (long)sides + 1);
}

static void scoreAddInternal(uint32_t points, const char* why) {
  g_score.score += points;
  g_scoreDirty = true;
  if (why) Serial.printf("[SCORE] +%lu (%s) → %lu\n", (unsigned long)points, why, (unsigned long)g_score.score);
}

// Award “roll X”: add a random 1..X
static void scoreAwardRoll(uint8_t sides, const char* why) {
  uint16_t add = scoreRoll(sides);
  switch (sides) {
    case 5:  g_score.rolls_5++;  break;
    case 10: g_score.rolls_10++; break;
    case 20: g_score.rolls_20++; break;
    case 40: g_score.rolls_40++; break;
  }
  scoreAddInternal(add, why);
}

// Public helpers (used by serial_cmds.cpp too)
uint32_t scoreGet()            { return g_score.score; }
void     scoreSet(uint32_t v)  { g_score.score = v; g_scoreDirty = true; Serial.printf("[SCORE] SET → %lu\n", (unsigned long)v); }
void     scoreAdd(int32_t d)   { if (d >= 0) scoreAddInternal((uint32_t)d, "serial add"); else {
                                   uint32_t sub = (uint32_t)(-d);
                                   g_score.score = (g_score.score > sub) ? (g_score.score - sub) : 0;
                                   g_scoreDirty = true;
                                   Serial.printf("[SCORE] ADD %ld → %lu\n", (long)d, (unsigned long)g_score.score);
                                 } }

// Optional: persist score counters (keep raw stats for long-term download)
static bool scoreSave() {
  File f = LittleFS.open("/score.json", "w");
  if (!f) return false;
  StaticJsonDocument<384> doc;
  doc["score"]            = g_score.score;
  doc["hits_total"]       = g_score.hits_total;
  doc["fires_total"]      = g_score.fires_total;
  doc["sessions_woke"]    = g_score.sessions_woke;
  doc["scenes_triggered"] = g_score.scenes_triggered;
  doc["r5"]  = g_score.rolls_5;
  doc["r10"] = g_score.rolls_10;
  doc["r20"] = g_score.rolls_20;
  doc["r40"] = g_score.rolls_40;
  serializeJson(doc, f);
  f.close();
  g_scoreDirty = false;
  g_scoreLastSave = millis();
  return true;
}

static void scoreLoad() {
  File f = LittleFS.open("/score.json", "r");
  if (!f) return;
  StaticJsonDocument<384> doc;
  if (deserializeJson(doc, f) == DeserializationError::Ok) {
    g_score.score            = doc["score"]           | 0u;
    g_score.hits_total       = doc["hits_total"]      | 0u;
    g_score.fires_total      = doc["fires_total"]     | 0u;
    g_score.sessions_woke    = doc["sessions_woke"]   | 0u;
    g_score.scenes_triggered = doc["scenes_triggered"]| 0u;
    g_score.rolls_5  = doc["r5"]  | 0u;
    g_score.rolls_10 = doc["r10"] | 0u;
    g_score.rolls_20 = doc["r20"] | 0u;
    g_score.rolls_40 = doc["r40"] | 0u;
  }
  f.close();
}

// Call this in loop() to avoid flash spam
static void scoreMaybeAutoSave() {
  if (!g_scoreDirty) return;
  if (millis() - g_scoreLastSave >= 3000) (void)scoreSave();
}



static uint32_t g_lastScoreRespMs = 0;

static inline bool scoreThrottleOk() {
  if (SCORE_RESP_MIN_MS == 0) return true;                 // disabled
  uint32_t now = millis();
  // unsigned subtraction is rollover-safe
  if ((now - g_lastScoreRespMs) < SCORE_RESP_MIN_MS) return false;
  g_lastScoreRespMs = now;
  return true;
}

struct SleepState {
  bool     asleep = false;
  uint32_t lastActivityMs = 0;           // last time we saw activity (awake only)
  uint32_t autoTimeoutMs  = AUTO_SLEEP_MINUTES_DEFAULT * 60UL * 1000UL;
  uint8_t  savedBrightness128 = 7;      // save/restore user brightness
} g_sleep;

static inline void noteActivity() {
  if (!g_sleep.asleep) g_sleep.lastActivityMs = millis();
}

// ----- Score snapshot (for dump/restore without exposing internals)
struct ScoreSnapshot {
  uint32_t score, hits_total, fires_total, sessions_woke, scenes_triggered;
  uint32_t r5, r10, r20, r40;
};

void getScoreSnapshot(ScoreSnapshot &s) {
  s.score = g_score.score;
  s.hits_total = g_score.hits_total;
  s.fires_total = g_score.fires_total;
  s.sessions_woke = g_score.sessions_woke;
  s.scenes_triggered = g_score.scenes_triggered;
  s.r5 = g_score.rolls_5; s.r10 = g_score.rolls_10; s.r20 = g_score.rolls_20; s.r40 = g_score.rolls_40;
}

void setScoreSnapshot(const ScoreSnapshot &s) {
  g_score.score = s.score;
  g_score.hits_total = s.hits_total;
  g_score.fires_total = s.fires_total;
  g_score.sessions_woke = s.sessions_woke;
  g_score.scenes_triggered = s.scenes_triggered;
  g_score.rolls_5 = s.r5; g_score.rolls_10 = s.r10; g_score.rolls_20 = s.r20; g_score.rolls_40 = s.r40;
  g_scoreDirty = true; (void)scoreSave();
}

// Auto-sleep accessors
uint32_t getAutoSleepMinMs()        { return g_sleep.autoTimeoutMs; }
void     setAutoSleepMinMs(uint32_t ms) { g_sleep.autoTimeoutMs = ms; }


static uint16_t readBatteryMilliVoltsOnce() {
  uint32_t acc = 0;
  for (int i = 0; i < BATT_SAMPLES; i++) acc += analogRead(BATTERY_ADC_PIN);
  uint16_t raw = (uint16_t)(acc / BATT_SAMPLES);
  g_batt_raw = raw;

  // RP2040 ADC default resolution is 12-bit → 0..4095
  float v_adc  = (raw / 4095.0f) * ADC_FULL_SCALE_VREF;
  float v_batt = v_adc / BATTERY_DIVIDER_K;       // inverse of divider
  if (v_batt < 0.f)   v_batt = 0.f;
  if (v_batt > 9.9f)  v_batt = 9.9f;              // clamp to spec range

  return (uint16_t)(v_batt * 1000.0f + 0.5f);     // mV
}

static inline void lowBattUpdateFromReading(uint16_t batt_mV) {
  // Hysteresis:
  if (!g_lowBatt && batt_mV <= LOW_BATT_THRESH_MV) {
    g_lowBatt = true;
  } else if (g_lowBatt && batt_mV >= (LOW_BATT_THRESH_MV + LOW_BATT_HYST_MV)) {
    g_lowBatt = false;
  }
  digitalWrite(LOW_BATT_PIN, g_lowBatt ? LOW_BATT_ON : LOW_BATT_OFF);
}

static inline void lowBattLightUpdate(uint16_t batt_mV) {
  const bool on = (batt_mV >= BATT_PRESENT_MIN_MV) && (batt_mV <= LOW_BATT_THRESH_MV);
  digitalWrite(LOW_BATT_PIN, on ? LOW_BATT_ON : LOW_BATT_OFF);
}

static inline void batteryPollTick() {
  uint32_t now = millis();
  if (now - g_battLastMs >= BATT_UPDATE_MS) {
    g_battLastMs = now;
    g_batt_mV = readBatteryMilliVoltsOnce();
    lowBattLightUpdate(g_batt_mV);
  }
}

static inline bool isLowBattery() { return g_lowBatt; }

// ---------------- SIRC-20 helpers ----------------
// flip 32-bit
static inline uint32_t bitrev32(uint32_t x) {
  x = ((x >> 1) & 0x55555555u) | ((x & 0x55555555u) << 1);
  x = ((x >> 2) & 0x33333333u) | ((x & 0x33333333u) << 2);
  x = ((x >> 4) & 0x0F0F0F0Fu) | ((x & 0x0F0F0F0Fu) << 4);
  x = ((x >> 8) & 0x00FF00FFu) | ((x & 0x00FF00FFu) << 8);
  x = (x >> 16) | (x << 16);
  return x;
}
// Take IRremote's MSB-first raw32, return our LSB-first 20-bit word (right-aligned)
static inline uint32_t sirc20FlipToLSB(uint32_t raw32) {
  return bitrev32(raw32) >> (32 - 20);
}
// Unpack: [0..5]=op(6), [6..14]=attacker(9), [15..19]=hi5(5)
static inline void sirc20Unpack(uint32_t wLSB, uint8_t &op, uint16_t &att, uint8_t &hi5) {
  op  =  wLSB        & 0x3F;
  att = (wLSB >> 6)  & 0x1FF;
  hi5 = (wLSB >> 15) & 0x1F;
}
// For value-style ops (14-bit)
static inline uint16_t sirc20Value14(uint32_t wLSB) {
  uint16_t low9 = (wLSB >> 6) & 0x1FF;
  uint16_t hi5  = (wLSB >> 15) & 0x1F;
  return (hi5 << 9) | low9;
}


// Pack our 20-bit Sony word (LSB-first on the wire):
// [0..5]=op, [6..14]=attacker(9b), [15..19]=charm(5b)
static inline uint32_t sirc20Pack(uint8_t op, uint16_t attacker, uint8_t charm) {
  return (uint32_t(op & 0x3F))
       | (uint32_t(attacker & 0x1FF) << 6)
       | (uint32_t(charm & 0x1F) << 15);
}

static void sendSirc20(uint32_t word, const char* tag = nullptr) {
#ifdef IR_TX_PIN
  if (tag) Serial.printf("[IRTX] %s SIRC20=0x%05lX\n", tag, (unsigned long)word);
  else     Serial.printf("[IRTX] SIRC20=0x%05lX\n", (unsigned long)word);
  // 20-bit Sony (library sends LSB-first as needed)
  IrSender.sendSony(word, 20);
#else
  (void)word; (void)tag;
  Serial.println("[IRTX] (no IR_TX_PIN) Fire pressed — anim only");
#endif
}

// Convenience: fire with our own CFG.id and charm (0..31; 0 = no unlock)
static void sendFireBadge(uint8_t charm = 0) {
        IrReceiver.disableIRIn(); 

  uint32_t w = sirc20Pack(/*op=*/0x00, /*attacker=*/CFG.id, /*charm=*/(CFG.userCharmId & 31));
  sendSirc20(w, "FIRE");
  delay(30);
        IrReceiver.enableIRIn(); 

}

// Respond to a SCORE_REQUEST by sending 3 frames: SCORE0/1/2

static void sendScoreTriplet(uint16_t myId, uint16_t score15) {
  // split 15-bit score into 3×5b chunks
  uint8_t p0 =  score15        & 0x1F; // [4:0]
  uint8_t p1 = (score15 >> 5)  & 0x1F; // [9:5]
  uint8_t p2 = (score15 >> 10) & 0x1F; // [14:10]

  // Add small randomized/backoff delay to reduce collisions across badges
  // (also bias by ID to spread even more)
  uint16_t jitter = 10 + (myId & 0x07) * 5 + (uint16_t)random(0, 15);
  delay(jitter);

  const uint16_t interPartMs  = 14;  // gap between 0→1 and 1→2
  const uint16_t interGroupMs = 28;  // gap between repeat groups
  const uint8_t  groups       = 2;   // send triplet twice

  for (uint8_t g = 0; g < groups; ++g) {
    IrSender.sendSony(sirc20Pack(0x07, myId, p0), 20); // SCORE0
    delay(interPartMs);
    IrSender.sendSony(sirc20Pack(0x08, myId, p1), 20); // SCORE1
    delay(interPartMs);
    IrSender.sendSony(sirc20Pack(0x09, myId, p2), 20); // SCORE2
    delay(interGroupMs);
  }
}

// Pack an op with a 14-bit payload (used by SPECIAL_SCENE, etc.)
static inline uint32_t sirc20Value(uint8_t op, uint16_t value14) {
  uint16_t low9 = (value14 & 0x01FF);
  uint8_t  hi5  = (value14 >> 9) & 0x1F;
  return (uint32_t(op & 0x3F))
       | (uint32_t(low9) << 6)
       | (uint32_t(hi5)  << 15);
}


static void playSceneById(uint8_t sid);

#ifdef SCORE_SNIFFER
  #include "uart_link.hpp"
  // ================= Opcodes =================
  static constexpr uint8_t OP_FIRE          = 0x00;
  static constexpr uint8_t OP_SLEEP         = 0x01;
  static constexpr uint8_t OP_WAKE          = 0x02;
  static constexpr uint8_t OP_SET_BRIGHT    = 0x03;
  static constexpr uint8_t OP_SHOW_MESSAGE  = 0x04;
  static constexpr uint8_t OP_SPECIAL_SCENE = 0x05;

  static constexpr uint8_t OP_SCORE_REQUEST = 0x06; // TX by this sketch (periodic)
  static constexpr uint8_t OP_SCORE0        = 0x07; // RX payload = score[4:0]
  static constexpr uint8_t OP_SCORE1        = 0x08; // RX payload = score[9:5]
  static constexpr uint8_t OP_SCORE2        = 0x09; // RX payload = score[14:10]

  // ================= Request pacing / housekeeping =================
  static uint16_t g_reqPeriodMs = 1500; // changeable via UART {"t":"set","req_ms":...}
  static constexpr uint8_t  IR_REPEATS    = 0;
  static constexpr uint16_t IR_GAP_MS     = 8;
  static constexpr uint16_t REQUESTER_ID  = 0;     // attacker/ID field for the request

  static uint32_t g_lastReqMs = 0;

  // ================= Unique badges seen (for heartbeat) =================
  // 512 badges -> 512 bits
  static uint32_t g_seenBits[16] = {0}; // 16 * 32 = 512
  static inline void seenAdd(uint16_t id) {
    if (id >= 512) return;
    g_seenBits[id >> 5] |= (1u << (id & 31));
  }
  static inline uint16_t uniqueBadgeCount() {
    uint16_t n = 0;
    for (int i=0;i<16;i++) n += __builtin_popcount(g_seenBits[i]);
    return n;
  }

    // ================= Triplet assembler =================
  struct Triplet {
    uint16_t id;              // 0..511, 0xFFFF => free
    uint8_t  have;            // bit0=part0, bit1=part1, bit2=part2
    uint16_t score;           // 15-bit assembled
    uint16_t lastComplete;    // last printed score for dedupe
    uint32_t t0;              // last activity
    uint32_t lastPrintMs;     // last full print time
  };

  static constexpr uint8_t  MAX_TRACK = 32;
  static constexpr uint32_t TTL_MS    = 1500; // clear partial if idle > 1.5s
  static constexpr uint32_t DEDUPE_MS = 600;  // suppress duplicate completes

  static Triplet g_trips[MAX_TRACK];

  static void tripsInit() {
    for (auto &t : g_trips) {
      t.id = 0xFFFF; t.have = 0; t.score = 0; t.lastComplete = 0; t.t0 = 0; t.lastPrintMs = 0;
    }
  }
  static Triplet* tripsFind(uint16_t id) {
    for (auto &t : g_trips) if (t.id == id) return &t;
    return nullptr;
  }
  static Triplet* tripsAlloc(uint16_t id) {
    for (auto &t : g_trips) if (t.id == 0xFFFF) {
      t.id = id; t.have = 0; t.score = 0; t.t0 = millis(); t.lastComplete=0; t.lastPrintMs=0;
      return &t;
    }
    // evict oldest or stale
    uint32_t now = millis();
    Triplet* oldest = &g_trips[0];
    for (auto &t : g_trips) {
      if (t.id != 0xFFFF && (now - t.t0) > TTL_MS) {
        t.id = id; t.have = 0; t.score = 0; t.t0 = now; t.lastComplete=0; t.lastPrintMs=0;
        return &t;
      }
      if (t.t0 < oldest->t0) oldest = &t;
    }
    oldest->id = id; oldest->have = 0; oldest->score = 0; oldest->t0 = now; oldest->lastComplete=0; oldest->lastPrintMs=0;
    return oldest;
  }
  static void tripsHousekeep() {
    uint32_t now = millis();
    for (auto &t : g_trips) {
      if (t.id != 0xFFFF && (now - t.t0) > TTL_MS) {
        t.id = 0xFFFF; t.have = 0; t.score = 0; t.lastComplete = 0; t.lastPrintMs = 0;
      }
    }
  }

  static void processScorePart(uint16_t id, uint8_t which /*0,1,2*/, uint8_t payload5) {
    Triplet* tr = tripsFind(id);
    if (!tr) tr = tripsAlloc(id);
    tr->t0 = millis();
    seenAdd(id); // track unique for heartbeat

    // Insert bits into 15-bit score: [4:0], [9:5], [14:10]
    switch (which) {
      case 0: tr->score = (tr->score & ~0x001F) | (payload5 & 0x1F);         tr->have |= 0x01; break;
      case 1: tr->score = (tr->score & ~0x03E0) | ((payload5 & 0x1F) << 5);  tr->have |= 0x02; break;
      case 2: tr->score = (tr->score & ~0x7C00) | ((payload5 & 0x1F) <<10);  tr->have |= 0x04; break;
      default: return;
    }

    Serial.printf("[SCORE] part=%u id=%u bits=0x%02X have=0x%02X\n",
                  which, id, payload5 & 0x1F, tr->have);

    // Notify ESP about the part we just received
    UartLink::sendPart(id, which, (payload5 & 0x1F), tr->have);

    if (tr->have == 0x07) {
      uint32_t now = millis();
      if (tr->lastComplete == tr->score && (now - tr->lastPrintMs) < DEDUPE_MS) {
        tr->have = 0; // suppress duplicate final print from repeats
        return;
      }
      tr->lastComplete = tr->score;
      tr->lastPrintMs  = now;
      if (tr->id < 500){
        Serial.printf("[SCORE] COMPLETE id=%u score=%u\n", tr->id, tr->score);
        //blink some stuff here
        playSceneById(SCENE_TX_ID);
            noteActivity();

        UartLink::sendScore(tr->id, tr->score, 3);
      }
      else 
      {Serial.println("Score Ignored, Control Badge");}

      tr->have = 0; // ready for next triplet
    }
  }


  static void tickRequestTx() {
    //IrReceiver.disableIRIn(); 
  
    uint32_t now = millis();
    if (now - g_lastReqMs < g_reqPeriodMs) return;
    g_lastReqMs = now;

    uint32_t w = sirc20Pack(OP_SCORE_REQUEST, REQUESTER_ID, 0);
    sendSirc20(w, "SCORE_REQUEST");

    //IrReceiver.enableIRIn(); 
  }
#endif

//prototyle for drawcharm565
static void drawCharm565(uint8_t id, int16_t x, int16_t y);


// exact 0..128 -> 0..255
uint8_t map128to255(uint8_t v){ 
  if (v > 20){return (uint16_t(20));}
  else{return (uint16_t(v)*255u)/128u;} 

}


static inline uint16_t badgeColor565() {
  uint8_t r=(CFG.colorRGB>>16)&0xFF, g=(CFG.colorRGB>>8)&0xFF, b=CFG.colorRGB&0xFF;
  return matrix->Color(r,g,b);
}

// ---- Unique attacker set (512 bits) + persistence ----
static uint32_t g_attackerBits[16]; // 16 * 32 = 512 ids

static inline bool statsAddAttacker(uint16_t id) {
  if (id >= 512) return false;
  uint8_t w = id >> 5;                 // 0..15
  uint32_t m = 1u << (id & 31);
  bool was = (g_attackerBits[w] & m);
  g_attackerBits[w] |= m;
  if (!was) {
    // persist lazily; cheap binary dump
    File f = LittleFS.open("/attackers.bin", "w");
    if (f) { f.write((const uint8_t*)g_attackerBits, sizeof(g_attackerBits)); f.close(); }
  }
  return !was;
}

uint16_t statsUniqueCount() {
  uint16_t n = 0;
  for (int i=0;i<16;i++) n += __builtin_popcount(g_attackerBits[i]);
  return n;
}

void statsLoadAttackers() {
  memset(g_attackerBits, 0, sizeof(g_attackerBits));
  File f = LittleFS.open("/attackers.bin", "r");
  if (f) { f.read((uint8_t*)g_attackerBits, sizeof(g_attackerBits)); f.close(); }
}


// ---------------- Simple message system ----------------
static const uint8_t CHAR_W = 6;     // 5x7 font + 1px gap
static const uint16_t SCROLL_MS = 80;

static String buildIdleLine();
static String resolveMessageText(uint8_t id);


static String resolveMessageText(uint8_t id) {
  //if (id == 0) return buildIdleLine();
  switch (id) {
    case 1: return "Fuse CATS: Michael Kohler, Andy Babin, and Ryan Middlemiss 2025";
    case 2: return "Framework New York 2025";
    case 3: return "Share and Enjoy";
    case 4: return "Next session starts in 5 min";
    case 5: return "Next session starts in 10 min";
    default: return String("MSG ") + id;
  }
}

static String buildIdleLine() {
  String s = CFG.name.length() ? CFG.name : "Badge";
  if (CFG.sleepDisabled) {                 // still append Message 2
    String m2 = resolveMessageText(2);     // “Share and enjoy”
    if (m2.length()) { s += "  "; s += m2; }
  }
  return s;
}


struct ScrollState {
  bool     active = false;
  bool     isIdle = true;
  uint8_t  repeats = 0;
  uint16_t color = 0;
  String   text;      // keep for non-split modes / compatibility
  int16_t  x = WIDTH;
  uint16_t w = 0;
  uint16_t contentW = 0;
  uint32_t lastStep = 0;

  String   textL;     // name / left portion
  String   textR;     // battery / right portion
  uint16_t wL = 0;    // pixel widths (CHAR_W * len)
  uint16_t wR = 0;
} g_scroll;

// ---- Text effects mode ----
enum TextMode : uint8_t { TM_SCROLL = 0, TM_BOUNCE = 1, TM_RAINBOW = 2, TM_MAX };
static TextMode g_textMode = TM_SCROLL;

// Per-mode state (bounce + rainbow)
static int16_t boX = 0;
static int8_t  boDir = +1;
static uint8_t rbHueBase = 0;

static inline bool hasUserCharm() {
  return (CFG.userCharmId != 0xFF);  // no unlock check; always show if configured
}

// Draw the current name/message with the selected effect at X = g_scroll.x
static void drawNameWithEffect() {
  const String &storedName = g_scroll.text;  // alias for FX code
  int16_t scrollX = g_scroll.x;              // alias for FX code

  // baseline: top row; you can tweak Y if you want
  int16_t xPos = scrollX;

  switch (currentTextEffect) {
    case EFFECT_SOLID: {
      // Use the user-defined badge color (seeded in startScroll as g_scroll.color)
      matrix->setTextColor(g_scroll.color);
      matrix->setCursor(xPos, 0);
      matrix->print(storedName);
    } break;

    case EFFECT_RAINBOW: {
      int16_t xp = xPos;
      for (uint8_t i = 0; i < storedName.length(); i++) {
        char c = storedName.charAt(i);
        uint8_t hue = (i * 30) + rainbowHue;
        CRGB color = CHSV(hue, 255, 255);
        matrix->setTextColor(matrix->Color(color.r, color.g, color.b));
        matrix->setCursor(xp, 0);
        matrix->print(c);
        xp += CHAR_W; // you defined CHAR_W=6
      }
      rainbowHue += 3;
    } break;

    case EFFECT_FIRE: {
      // fire-ish background flicker
      for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t flicker = random8(120);
        // warm tone; clamp to byte
        int g = (int)random8(100, 180) - (int)flicker/2;
        if (g < 0) g = 0;
        leds[i] = CRGB(127, g/2, 0);
      }
      // white text on top
      matrix->setTextColor(matrix->Color(255, 255, 255));
      matrix->setCursor(xPos, 0);
      matrix->print(storedName);
    } break;

    case EFFECT_GLITCH: {
      bool burst = (random8() < 10);
      int xPosGlitch = scrollX;
      int screenJitterX = burst ? (int)random8(0,3)-1 : 0; // -1..+1
      int screenJitterY = burst ? (int)random8(0,3)-1 : 0;

      for (uint8_t i = 0; i < storedName.length(); i++) {
        char c = storedName.charAt(i);

        int xJitter = (random8() < 50) ? (int)random8(0,3)-1 : 0;
        int yJitter = (random8() < 50) ? (int)random8(0,3)-1 : 0;

        CRGB color;
        if (burst) {
          color = CRGB(random8(), random8(), random8());
        } else if (random8() < 40) {
          color = CRGB(255, random8(80,255), 255);
        } else {
          color = CRGB(180, 200, 255);
        }

        matrix->setTextColor(matrix->Color(color.r, color.g, color.b));
        matrix->setCursor(xPosGlitch + xJitter + screenJitterX, yJitter + screenJitterY);
        matrix->print(c);

        if (random8() < 40) {
          matrix->setCursor(xPosGlitch + xJitter + ((int)random8(0,3)-1),
                            yJitter + ((int)random8(0,3)-1));
          matrix->print(c);
        }

        xPosGlitch += CHAR_W; // advance 1 char cell
      }
    } break;
    case EFFECT_ICE: {
  // UV/blue flame-ish background flicker
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t flicker = random8(120);

    // cool tone; clamp to byte
    int r = (int)random8(0, 24)   - (int)flicker / 8;  if (r < 0) r = 0;       // slight red for purple tint
    int g = (int)random8(0, 8)   - (int)flicker / 6;  if (g < 0) g = 0;       // keep green very low
    int b = (int)random8(120,170) - (int)flicker / 2;  if (b < 0) b = 0;       // dominant blue, flickers darker

    leds[i] = CRGB(r, g, b);
  }

  // cyan text on top
  matrix->setTextColor(matrix->Color(0, 255, 255));
  matrix->setCursor(xPos, 0);
  matrix->print(storedName);
  } break;
  case EFFECT_MATRIX: {
  // "Green matrix rain" with variable tails (min 1, max 7 requested).
  // Wiring: 15 columns × 7 rows, each column top->bottom, left->right.
  const int COLS = 15;
  const int ROWS = 7;

  // Persistent per-column state (self-contained; no external vars/helpers)
  static bool     inited = false;
  static uint8_t  y[COLS];         // head row per column
  static uint8_t  speed[COLS];     // frames-per-step for each column (2..5)
  static uint8_t  tailLen[COLS];   // desired tail length per column (1..7)
  static uint8_t  tick = 0;        // frame counter

  if (!inited) {
    for (int c = 0; c < COLS; c++) {
      y[c] = random8(ROWS);
      speed[c] = random8(2, 6);              // 2..5
      if (speed[c] < 2) speed[c] = 2;
      tailLen[c] = random8(1, 8);            // 1..7 (requested range)
    }
    inited = true;
  }

  // Gentle fade to create motion persistence without helpers
  for (int i = 0; i < NUM_LEDS; i++) {
    int r = leds[i].r - 6;   if (r < 0) r = 0;
    int g = leds[i].g - 14;  if (g < 0) g = 0;   // slightly stronger green fade
    int b = leds[i].b - 6;   if (b < 0) b = 0;
    leds[i] = CRGB(r, g, b);
  }

  // Update heads, occasionally re-randomize speed and tail length
  for (int c = 0; c < COLS; c++) {
    if ((tick % speed[c]) == 0) {
      y[c] = (uint8_t)((y[c] + 1) % ROWS);
      if (y[c] == 0 && random8() < 96) {    // ~37.5% chance to vary column cadence
        speed[c] = random8(2, 6);
        if (speed[c] < 2) speed[c] = 2;
        tailLen[c] = random8(1, 8);         // 1..7 (requested range)
      }
    }
  }

  // Draw heads and tails
  for (int c = 0; c < COLS; c++) {
    // Head brightness: strong but not neon
    int gHead = (int)random8(150, 210);
    int headIdx = c * ROWS + y[c];
    leds[headIdx] = CRGB(0, gHead, 0);

    // Effective tail length cannot exceed ROWS-1 to avoid overlapping the head on a 7-row panel.
    uint8_t Lreq = tailLen[c];              // 1..7
    uint8_t L    = Lreq;
    if (L >= ROWS) L = ROWS - 1;            // clamp to 6 on a 7-row matrix

    // Global attenuation vs. requested tail length (longer overall ⇒ dimmer segments).
    // Map Lreq=1..7 roughly to 0.85..0.35 (percent scaled ×100).
    int atten_num = 85 - 8 * Lreq;          // 85,77,69,61,53,45,37
    if (atten_num < 35) atten_num = 35;

    // Tail segments above the head, wrapping upward; brightness falls with distance.
    for (uint8_t k = 1; k <= L; k++) {
      // Distance falloff: segFactor ≈ (L - k + 1)/(L + 1)
      int seg_num = (int)(L - k + 1) * 100;
      int seg_den = (int)(L + 1);
      int segFactorPct = (seg_num / seg_den);       // 0..100 (approx)

      // Combine: tail = head * atten * segFactor
      int gTail = (gHead * atten_num * segFactorPct) / (100 * 100);

      // Compute wrapped row above head
      int row = y[c] - k;
      while (row < 0) row += ROWS;
      int idx = c * ROWS + row;

      // Additive blend to preserve visibility
      int gExisting = leds[idx].g;
      int gNew = gExisting + gTail;
      if (gNew > 255) gNew = 255;

      leds[idx] = CRGB(0, gNew, 0);
    }
  }

  tick++;

  // White text over the rain
  matrix->setTextColor(matrix->Color(255, 255, 255));
  matrix->setCursor(xPos, 0);
  matrix->print(storedName);
} break;
case EFFECT_FIREWORK: {
  // 15x7 panel, columns wired left->right, each column top->bottom.
  const int COLS = 15;
  const int ROWS = 7;
  const uint8_t MAX_FW = 3;     // keep max 3 on screen, but allow multi-spawn salvos

  // States: 0=idle, 1=ascending, 2=expanding (bright), 3=fading (at max size)
  static bool     inited = false;
  static uint8_t  state[MAX_FW];
  static int8_t   cx[MAX_FW], cy[MAX_FW];   // current center / head
  static int8_t   tgtY[MAX_FW];             // explosion trigger row (upper half 0..3)
  static uint8_t  riseTick[MAX_FW];         // frames-per-step for ascent (1..3)
  static uint8_t  risePhase[MAX_FW];
  static uint8_t  rC[MAX_FW], gC[MAX_FW], bC[MAX_FW]; // base bright color
  static uint8_t  radius[MAX_FW];           // current explosion radius
  static uint8_t  maxRad[MAX_FW];           // target max radius (2..3 for Ø 4..7)
  static uint8_t  fadeAge[MAX_FW];          // frames spent fading
  static uint8_t  tick = 0;

  if (!inited) {
    for (uint8_t i = 0; i < MAX_FW; i++) {
      state[i] = 0; cx[i] = 0; cy[i] = ROWS - 1; tgtY[i] = 0;
      riseTick[i] = 2; risePhase[i] = 0; radius[i] = 0; maxRad[i] = 2; fadeAge[i] = 0;
      rC[i] = gC[i] = bC[i] = 0;
    }
    inited = true;
  }

  // Light global fade so motion looks smooth without killing brightness.
  for (int i = 0; i < NUM_LEDS; i++) {
    int r = leds[i].r - 4;   if (r < 0) r = 0;
    int g = leds[i].g - 4;   if (g < 0) g = 0;
    int b = leds[i].b - 4;   if (b < 0) b = 0;
    leds[i] = CRGB(r, g, b);
  }

  // Inline helpers (lambdas keep this self-contained)
  auto plot = [&](int x, int y, uint8_t rr, uint8_t gg, uint8_t bb) {
    if (x >= 0 && x < COLS && y >= 0 && y < ROWS) {
      leds[x * ROWS + y] = CRGB(rr, gg, bb);
    }
  };

  auto slot_for_spawn = [&]()->int8_t {
    for (uint8_t i = 0; i < MAX_FW; i++) if (state[i] == 0) return (int8_t)i;
    return (int8_t)-1;
  };

  auto spawn_at = [&](uint8_t slot, int col) {
    state[slot] = 1;                         // ascending
    cx[slot] = (int8_t)col;
    cy[slot] = ROWS - 1;                     // start at bottom
    tgtY[slot] = (int8_t)random8(0, (ROWS + 1) / 2);  // 0..3 (upper half)
    riseTick[slot] = (uint8_t)random8(1, 4);          // 1..3
    risePhase[slot] = 0;
    radius[slot] = 0;
    maxRad[slot] = (uint8_t)random8(3, 6);   // radius 2..3 -> diameter 4..7
    fadeAge[slot] = 0;

    // Vivid bright color biased towards one channel
    uint8_t pick = random8(3);
    if (pick == 0) { // red-ish
      rC[slot] = random8(180, 200); gC[slot] = random8(20, 100);  bC[slot] = random8(20, 100);
    } else if (pick == 1) { // green-ish
      rC[slot] = random8(20, 100);  gC[slot] = random8(180, 200); bC[slot] = random8(20, 100);
    } else { // blue/purple-ish
      rC[slot] = random8(20, 100);  gC[slot] = random8(20, 100);  bC[slot] = random8(180, 200);
    }
  };

  auto column_is_clear = [&](int col, int minSep) {
    // keep horizontal spacing from both active fireworks and any columns we plan this frame
    for (uint8_t i = 0; i < MAX_FW; i++) {
      if (state[i] != 0) {
        int d = cx[i] - col; if (d < 0) d = -d;
        if (d < minSep) return false;
      }
    }
    return true;
  };

  // Count active and consider spawning one or a horizontally spaced "salvo"
  uint8_t activeCount = 0;
  for (uint8_t i = 0; i < MAX_FW; i++) if (state[i] != 0) activeCount++;

  if (activeCount < MAX_FW) {
    // 2 types of spawns:
    // - Salvo (2-3 at once, spaced): rare
    // - Single: common, but still enforces spacing from current actives
    bool didSpawn = false;

    // Try a spaced salvo occasionally
    if (random8() < 12) { // ~1/21 chance per frame
      uint8_t freeSlots = MAX_FW - activeCount;
      uint8_t want = (uint8_t)min((int)freeSlots, (int)random8(2, 4)); // try 2..3
      // Preferred anchor columns to spread color: left/center/right
      const int anchors[3] = { 2, 7, 12 };
      uint8_t spawned = 0;
      // First pass: try anchors that are clear
      for (uint8_t a = 0; a < 3 && spawned < want; a++) {
        if (column_is_clear(anchors[a], 4)) {
          int8_t s = slot_for_spawn();
          if (s >= 0) { spawn_at((uint8_t)s, anchors[a]); spawned++; didSpawn = true; }
        }
      }
      // Second pass: try nearby columns around anchors if still need more
      for (uint8_t a = 0; a < 3 && spawned < want; a++) {
        for (int off = 1; off <= 2 && spawned < want; off++) {
          int candL = anchors[a] - off;
          int candR = anchors[a] + off;
          if (candL >= 0 && column_is_clear(candL, 4)) {
            int8_t s = slot_for_spawn(); if (s >= 0) { spawn_at((uint8_t)s, candL); spawned++; didSpawn = true; }
          }
          if (spawned >= want) break;
          if (candR < COLS && column_is_clear(candR, 4)) {
            int8_t s = slot_for_spawn(); if (s >= 0) { spawn_at((uint8_t)s, candR); spawned++; didSpawn = true; }
          }
        }
      }
    }

    // Otherwise, try a single spawn in a column that respects spacing
    if (!didSpawn && random8() < 24) { // ~1/10 chance
      // Try up to N random columns that keep min separation
      for (uint8_t tries = 0; tries < 8; tries++) {
        int col = (int)random8(COLS);
        if (column_is_clear(col, 3)) {
          int8_t s = slot_for_spawn();
          if (s >= 0) { spawn_at((uint8_t)s, col); break; }
        }
      }
    }
  }

  // Update/draw each firework
  for (uint8_t i = 0; i < MAX_FW; i++) {
    if (state[i] == 0) continue;

    if (state[i] == 1) {
      // ASCENT: move up every riseTick frames
      risePhase[i]++;
      if (risePhase[i] >= riseTick[i]) {
        risePhase[i] = 0;
        cy[i] = cy[i] - 1;
        if (cy[i] <= tgtY[i]) {
          state[i] = 2;           // start bright expansion
          radius[i] = 0;
        }
      }

      // Draw bright head + 3-pixel same-color trail downward
      uint8_t headR = (uint8_t)min(255, (int)rC[i] + 60);
      uint8_t headG = (uint8_t)min(255, (int)gC[i] + 60);
      uint8_t headB = (uint8_t)min(255, (int)bC[i] + 60);
      plot(cx[i], cy[i], headR, headG, headB);

      for (int t = 1; t <= 3; t++) {
        int ty = cy[i] + t;
        if (ty >= ROWS) break;
        int scale = (t == 1) ? 70 : (t == 2 ? 45 : 25);
        plot(cx[i], ty,
             (uint8_t)((rC[i] * scale) / 100),
             (uint8_t)((gC[i] * scale) / 100),
             (uint8_t)((bC[i] * scale) / 100));
      }

    } else if (state[i] == 2) {
      // EXPANDING BRIGHT: increase radius to max without dimming
      int R = radius[i];
      int R2 = R * R;
      uint8_t eR = (uint8_t)min(255, (int)rC[i] + 40);
      uint8_t eG = (uint8_t)min(255, (int)gC[i] + 40);
      uint8_t eB = (uint8_t)min(255, (int)bC[i] + 40);

      for (int dx = -R; dx <= R; dx++) {
        for (int dy = -R; dy <= R; dy++) {
          if (dx*dx + dy*dy <= R2) {
            plot(cx[i] + dx, cy[i] + dy, eR, eG, eB);
          }
        }
      }

      if (radius[i] < maxRad[i]) {
        radius[i]++;
      } else {
        state[i] = 3;     // reached max size → start fading
        fadeAge[i] = 0;
      }

    } else {
      // FADING at max radius (gentle gravity)
      int R = maxRad[i];
      int R2 = R * R;

      uint8_t age = fadeAge[i];
      int fadePct = 100 - (int)age * 10;    // 100→0
      if (fadePct < 0) fadePct = 0;

      int fall = ((age & 2) ? 1 : 0);
      int cyFall = cy[i] + fall;

      uint8_t fR = (uint8_t)((rC[i] * fadePct) / 100);
      uint8_t fG = (uint8_t)((gC[i] * fadePct) / 100);
      uint8_t fB = (uint8_t)((bC[i] * fadePct) / 100);

      for (int dx = -R; dx <= R; dx++) {
        for (int dy = -R; dy <= R; dy++) {
          if (dx*dx + dy*dy <= R2) {
            plot(cx[i] + dx, cyFall + dy, fR, fG, fB);
          }
        }
      }

      fadeAge[i]++;
      if (fadeAge[i] >= 10 || cyFall >= ROWS) {
        state[i] = 0;  // finished
      } else {
        cy[i] = cyFall; // keep falling while fading
      }
    }
  }

  tick++;

  // White text on top
  matrix->setTextColor(matrix->Color(255, 255, 255));
  matrix->setCursor(xPos, 0);
  matrix->print(storedName);
} break;
case EFFECT_CHEVRON: {
  // 15x7 matrix, columns wired left->right, each column top->bottom.
  const int COLS = 15;
  const int ROWS = 7;

  // Self-contained animation state
  static uint8_t inited = 0;
  static int8_t  phase  = 0;   // scroll phase
  static uint8_t tick   = 0;   // frame divider

  if (!inited) { inited = 1; phase = 0; tick = 0; }

  // Stripe geometry: 45° bands (bottom-left → top-right)
  const int BAND_W = 3;              // colored stripe width
  const int GAP_W  = 3;              // black gap width
  const int PERIOD = BAND_W + GAP_W; // full on/off period

  // Reverse direction: decrement phase instead of incrementing
  tick++;
  if ((tick & 0x01) == 0) {
    phase = (int8_t)((phase - 1) % PERIOD);
    if (phase < 0) phase += PERIOD;
  }

  // Colors
  const CRGB ON  = CRGB(32, 0, 160);  // dark purple
  const CRGB OFF = CRGB(0, 0, 0);     // black

  // Draw angled bands. Lines at 45° satisfy x - y = const.
  for (int x = 0; x < COLS; x++) {
    for (int y = 0; y < ROWS; y++) {
      int m = (x - y + phase) % PERIOD; if (m < 0) m += PERIOD;
      bool inBand = (m < BAND_W);
      leds[x * ROWS + y] = inBand ? ON : OFF;
    }
  }

  // Bright yellow text on top
  matrix->setTextColor(matrix->Color(255, 255, 0));
  matrix->setCursor(xPos, 0);
  matrix->print(storedName);
} break;
case EFFECT_BUGS: {
  // 15x7 matrix, columns wired left->right, each column top->bottom.
  const int COLS = 15;
  const int ROWS = 7;

  // Slightly brighter dark-purple bugs & trails (still behind CYAN text).
  // Cardinal motion with wraparound; trails fade.
  const uint8_t MAX_BUGS  = 6;
  const uint8_t MAX_TRAIL = 10;

  // Dark purple palette — nudged brighter
  const uint8_t PUR_R = 110;  // was ~90
  const uint8_t PUR_G = 0;
  const uint8_t PUR_B = 170;  // was ~140

  static uint8_t inited = 0;

  // Per-bug state
  static int8_t  bx[MAX_BUGS],  by[MAX_BUGS];        // head position
  static int8_t  dx[MAX_BUGS],  dy[MAX_BUGS];        // direction ∈ {(±1,0),(0,±1)}
  static uint8_t spd[MAX_BUGS], phase[MAX_BUGS];     // frames-per-step (1..3)
  static uint8_t trailLen[MAX_BUGS];                 // 4..10
  static uint8_t histX[MAX_BUGS][MAX_TRAIL];         // newest-first positions
  static uint8_t histY[MAX_BUGS][MAX_TRAIL];
  static uint8_t histSize[MAX_BUGS];                 // 0..trailLen

  if (!inited) {
    inited = 1;
    for (uint8_t i = 0; i < MAX_BUGS; i++) {
      bx[i] = random8(COLS);
      by[i] = random8(ROWS);
      switch (random8(4)) { // random cardinal direction
        case 0: dx[i] =  1; dy[i] =  0; break;
        case 1: dx[i] = -1; dy[i] =  0; break;
        case 2: dx[i] =  0; dy[i] =  1; break;
        default:dx[i] =  0; dy[i] = -1; break;
      }
      spd[i] = (uint8_t)random8(1, 4);   // 1..3 (1 = fastest)
      phase[i] = 0;
      trailLen[i] = (uint8_t)random8(4, 11); // 4..10
      histSize[i] = 0;
      for (uint8_t k = 0; k < MAX_TRAIL; k++) { histX[i][k] = bx[i]; histY[i][k] = by[i]; }
    }
  }

  // Slightly gentler fade so the brighter purple persists a bit more
  for (int i = 0; i < NUM_LEDS; i++) {
    int r = leds[i].r - 5;  if (r < 0) r = 0;
    int g = leds[i].g - 2;  if (g < 0) g = 0;
    int b = leds[i].b - 6;  if (b < 0) b = 0;
    leds[i] = CRGB(r, g, b);
  }

  auto plot = [&](int x, int y, uint8_t rr, uint8_t gg, uint8_t bb) {
    if (x >= 0 && x < COLS && y >= 0 && y < ROWS) {
      leds[x * ROWS + y] = CRGB(rr, gg, bb);
    }
  };

  // Update bugs
  for (uint8_t i = 0; i < MAX_BUGS; i++) {
    // Step timing
    phase[i]++;
    if (phase[i] >= spd[i]) {
      phase[i] = 0;

      // ~1/3 chance to change direction; pick a new cardinal direction
      if (random8() < 85) {
        switch (random8(4)) {
          case 0: dx[i] =  1; dy[i] =  0; break;
          case 1: dx[i] = -1; dy[i] =  0; break;
          case 2: dx[i] =  0; dy[i] =  1; break;
          default:dx[i] =  0; dy[i] = -1; break;
        }
      }

      // Move head with wrapping
      int nx = bx[i] + dx[i];
      int ny = by[i] + dy[i];
      if (nx < 0) nx = COLS - 1; else if (nx >= COLS) nx = 0;
      if (ny < 0) ny = ROWS - 1; else if (ny >= ROWS) ny = 0;
      bx[i] = (int8_t)nx; by[i] = (int8_t)ny;

      // Push new head to history (newest first), keep up to trailLen[i]
      uint8_t keep = (trailLen[i] > MAX_TRAIL) ? MAX_TRAIL : trailLen[i];
      if (histSize[i] < keep) histSize[i]++;
      for (int k = (int)histSize[i] - 1; k > 0; k--) {
        histX[i][k] = histX[i][k - 1];
        histY[i][k] = histY[i][k - 1];
      }
      histX[i][0] = (uint8_t)bx[i];
      histY[i][0] = (uint8_t)by[i];

      // Occasionally vary trail length and speed
      if (random8() < 16) { trailLen[i] = (uint8_t)random8(4, 11); if (histSize[i] > trailLen[i]) histSize[i] = trailLen[i]; }
      if (random8() < 8)  { spd[i] = (uint8_t)random8(1, 4); }
    }

    // Head color: slightly brighter than base, but not overpowering text
    uint8_t hr = (uint8_t)min(200, (int)PUR_R + 30);
    uint8_t hg = PUR_G;
    uint8_t hb = (uint8_t)min(210, (int)PUR_B + 40);
    plot(bx[i], by[i], hr, hg, hb);

    // Trail brightness: allow up to ~90% of base (was 75%) and lift the floor a bit
    for (uint8_t age = 1; age < histSize[i]; age++) {
      int x = (int)histX[i][age];
      int y = (int)histY[i][age];

      int den = (int)histSize[i]; if (den == 0) den = 1;
      int num = (int)(histSize[i] - age) * 90;   // cap 90% of base
      int pct = num / den;                       // ≈ 90→~0
      pct = (pct * 12) / 10;                     // mild mid-tone lift
      if (pct > 90) pct = 90;
      if (pct < 18) pct = 18;                    // faint but visible tail end

      uint8_t r = (uint8_t)((PUR_R * pct) / 100);
      uint8_t g = 0;
      uint8_t b = (uint8_t)((PUR_B * pct) / 100);

      plot(x, y, r, g, b);
    }
  }

  // Cyan text on top
  matrix->setTextColor(matrix->Color(0, 255, 0));
  matrix->setCursor(xPos, 0);
  matrix->print(storedName);
} break;

  }
}

#ifndef TEXT_ICON_GAP
#define TEXT_ICON_GAP 3
#endif
#ifndef ICON_SPACING
#define ICON_SPACING 1
#endif

// How many pixels wide the inline-charm block will be (for contentW/wrap)
static uint16_t inlineCharmsWidth() {
  // In sleepDisabled mode, show ALL charms in-line (as you asked earlier)
  if (CFG.sleepDisabled) {
    const uint8_t n = CHARM_COUNT;
    return n ? (n * CHARM_W + (n - 1) * ICON_SPACING) : 0;
  }

  // Otherwise: show the user's selected charm if set
  if (CFG.userCharmId != 0xFF) return CHARM_W;

  return 0;
}

// Draw the inline-charm block starting at x
static void drawInlineCharms(int16_t x) {
  if (CFG.sleepDisabled) {
    // Draw ALL charms left→right
    for (uint8_t i = 0; i < CHARM_COUNT; ++i) {
      drawCharm565(i, x, 0);
      x += (CHARM_W + ICON_SPACING);
    }
    return;
  }

  // Default: only the user's selected charm (if any)
  if (CFG.userCharmId != 0xFF) {
    drawCharm565(CFG.userCharmId, x, 0);
    return;
  }

}


// Split the idle line into LEFT (name/etc) and RIGHT (battery)
static String buildIdleLeft() {
  return CFG.name.length() ? CFG.name : "Badge";
}

// If your function is named differently, swap getBatteryMilliVolts() below.
static String buildBatteryText() {
  uint16_t mv = readBatteryMilliVoltsOnce();     // <- use your existing function
  char buf[16];
  // "  2.84V" (leading spaces give a small gap)
  snprintf(buf, sizeof(buf), "  %u.%02uV", mv / 1000, (mv % 1000) / 10);
  return String(buf);
}


void startScroll(bool idle, uint8_t msgId = 0, uint8_t reps = 0, uint16_t color = 0) {
  g_scroll.isIdle  = idle;
  g_scroll.repeats = idle ? 0 : max<uint8_t>(1, reps);
  g_scroll.color   = color ? color : badgeColor565();

  matrix->setFont(NULL);
  matrix->setTextSize(1);
  matrix->setTextWrap(false);

  if (idle) {
    g_scroll.textL = buildIdleLeft();   // name/etc
    g_scroll.textR = "";                // <-- remove battery
    g_scroll.text  = g_scroll.textL;    // compatibility
  } else {
    g_scroll.textL = resolveMessageText(msgId);
    g_scroll.textR = "";
    g_scroll.text  = g_scroll.textL;
  }

  g_scroll.wL = g_scroll.textL.length() * CHAR_W;
  g_scroll.wR = 0;                      // no right text
  g_scroll.w  = g_scroll.wL;

  const uint16_t iconBlockW = inlineCharmsWidth();

  // LEFT + (gap if icons) + icons (charms at the end)
  g_scroll.contentW =
      g_scroll.wL
    + (iconBlockW ? TEXT_ICON_GAP : 0)
    + iconBlockW;

  g_scroll.x = WIDTH;
  g_scroll.lastStep = 0;
  g_scroll.active = true;
}

// Perceptual helper: brighten shadows (inverse ~2.2 gamma), clamped to [0,255].
// This is fast enough and avoids a 256-byte LUT in PROGMEM.
static inline uint8_t gamma_lift_approx(uint8_t v) {
  // Map 0..255 -> 0..1
  float x = v / 255.0f;
  // Inverse-gamma-ish curve to *lift* lows but keep highs similar
  // y = x^(1/2.2)  ≈ powf(x, 0.4545f)
  float y = powf(x, 0.4545f);
  int out = (int)(y * 255.0f + 0.5f);
  if (out < 0) out = 0; if (out > 255) out = 255;
  return (uint8_t)out;
}

//Helper for rounding
static inline uint8_t keep_nonzero_floor(uint8_t v, bool was_nonzero_in_src) {
  if (!was_nonzero_in_src) return v; // true black stays black
  return (v == 0) ? 1 : v;           // keep tiny nonzero values from rounding to 0
}


//Helper for Perceptual Gamma
static inline void unpack565_with_perceptual(uint16_t c, uint8_t &r8, uint8_t &g8, uint8_t &b8) {
  uint8_t r5 = (c >> 11) & 0x1F;
  uint8_t g6 = (c >> 5)  & 0x3F;
  uint8_t b5 =  c        & 0x1F;

  // Bit replication 5/6 -> 8
  uint8_t r = (r5 << 3) | (r5 >> 2);
  uint8_t g = (g6 << 2) | (g6 >> 4);
  uint8_t b = (b5 << 3) | (b5 >> 2);

  // Perceptual shadow lift
  r = gamma_lift_approx(r);
  g = gamma_lift_approx(g);
  b = gamma_lift_approx(b);

  // Nonzero floor only if source channel was nonzero in 565
  r = keep_nonzero_floor(r, r5 != 0);
  g = keep_nonzero_floor(g, g6 != 0);
  b = keep_nonzero_floor(b, b5 != 0);

  r8 = r; g8 = g; b8 = b;
}


// Draw charm `id` at (x,y) using RGB565 from PROGMEM
static void drawCharm565(uint8_t id, int16_t x, int16_t y) {
  if (id >= CHARM_COUNT) return;
  const uint16_t* p = charms565[id];

  for (uint8_t yy = 0; yy < CHARM_H; ++yy) {
    for (uint8_t xx = 0; xx < CHARM_W; ++xx) {
      uint16_t c = pgm_read_word(p++);  // RGB565 from PROGMEM

      // Convert with perceptual lift
      uint8_t r8, g8, b8;
      unpack565_with_perceptual(c, r8, g8, b8);

      // Draw as 24-bit color to bypass NeoMatrix's internal 565 path
      matrix->drawPixel(x + xx, y + yy, matrix->Color(r8, g8, b8));
    }
  }
}

static inline void rgb565ToCRGB(uint16_t c, CRGB &rgb) {
  // Extract 5:6:5
  uint8_t r5 = (c >> 11) & 0x1F;
  uint8_t g6 = (c >> 5)  & 0x3F;
  uint8_t b5 =  c        & 0x1F;

  // Bit replication (fills the low bits instead of zeros)
  // r5: 5 -> 8 bits, g6: 6 -> 8 bits, b5: 5 -> 8 bits
  uint8_t r8 = (r5 << 3) | (r5 >> 2);
  uint8_t g8 = (g6 << 2) | (g6 >> 4);
  uint8_t b8 = (b5 << 3) | (b5 >> 2);

  // Perceptual shadow lift so darks don't crush to black
  r8 = gamma_lift_approx(r8);
  g8 = gamma_lift_approx(g8);
  b8 = gamma_lift_approx(b8);

  // If the source channel was nonzero in 565, keep a tiny floor after mapping
  r8 = keep_nonzero_floor(r8, r5 != 0);
  g8 = keep_nonzero_floor(g8, g6 != 0);
  b8 = keep_nonzero_floor(b8, b5 != 0);

  rgb.r = r8; rgb.g = g8; rgb.b = b8;
}


static void renderScrollTick() {
    watchdog_update();
  const uint32_t now = millis();
  if (now - g_scroll.lastStep < SCROLL_MS) return;
  g_scroll.lastStep = now;

  matrix->fillScreen(0);
  matrix->setFont(NULL);
  matrix->setTextSize(1);
  matrix->setTextWrap(false);

  switch (g_textMode) {
case TM_SCROLL: {
  const uint16_t iconBlockW = inlineCharmsWidth();
  const int16_t  xLeft  = g_scroll.x;
  const int16_t  xIcons = xLeft + (int)g_scroll.wL + (iconBlockW ? TEXT_ICON_GAP : 0);

  // Draw LEFT once with effect (effects that repaint background won’t erase icons drawn after)
  {
    String prev = g_scroll.text;
    g_scroll.text = g_scroll.textL;
    drawNameWithEffect();
    g_scroll.text = prev;
  }

  // Draw charms at the end
  if (iconBlockW) {
    drawInlineCharms(xIcons);
  }

  FastLED.show();
  g_scroll.x--;

  // Wrap on (LEFT + gap + icons)
  if (g_scroll.x < -(int)g_scroll.contentW) {
    if (!g_scroll.isIdle) {
      if (--g_scroll.repeats == 0) { startScroll(true); break; }
    }
    g_scroll.x = WIDTH;
  }
} break;

    case TM_BOUNCE: {
      const int16_t minX = (g_scroll.w > WIDTH) ? -((int16_t)g_scroll.w - WIDTH) : 0;
      const int16_t maxX = (g_scroll.w > WIDTH) ? 0 : (WIDTH - (int16_t)g_scroll.w);

      matrix->setTextColor(g_scroll.color);
      matrix->setCursor(boX, 0);
      matrix->print(g_scroll.text);
      FastLED.show();

      boX += boDir;
      if (boX <= minX || boX >= maxX) {
        boDir = -boDir;
        if (boX <= minX && !g_scroll.isIdle && --g_scroll.repeats == 0) {
          startScroll(true);
          return;
        }
      }
    } break;

    case TM_RAINBOW: {
      int16_t x = g_scroll.x;
      for (uint16_t i = 0; i < g_scroll.text.length(); ++i) {
        CHSV hsv(rbHueBase + i * 8, 255, 255);
        CRGB rgb; hsv2rgb_rainbow(hsv, rgb);
        matrix->setTextColor(matrix->Color(rgb.r, rgb.g, rgb.b));
        matrix->setCursor(x, 0);
        matrix->print(g_scroll.text[i]);
        x += CHAR_W;
      }
      FastLED.show();

      rbHueBase++;
      g_scroll.x--;
      if (g_scroll.x < -(int)g_scroll.w) {
        if (!g_scroll.isIdle && --g_scroll.repeats == 0) {
          startScroll(true);
          return;
        }
        g_scroll.x = WIDTH;
      }
    } break;
  }
}

// Piecewise-triangular "breathing" without floats/trig.
// Brightness goes MIN -> MAX -> MIN over SLEEP_PULSE_PERIOD_MS.
static inline uint8_t sleepPulseValue(uint32_t t) {
  const uint32_t T = SLEEP_PULSE_PERIOD_MS;
  const uint32_t half = T >> 1;
  const uint8_t  lo = SLEEP_PULSE_BRIGHT_MIN;
  const uint8_t  hi = SLEEP_PULSE_BRIGHT_MAX;
  if (T == 0 || hi <= lo) return lo;

  t %= T;
  uint32_t up = (t < half) ? t : (T - t);               // 0..half..0
  uint32_t span = (uint32_t)(hi - lo);
  return (uint8_t)(lo + (up * span) / half);
}



// Force a message to start immediately (preempt)
static void onShowMessage(uint8_t msgId, uint8_t scrolls) {
  startScroll(/*idle*/false, msgId, scrolls, 0xffff);
  Serial.printf("[MSG] NOW id=%u scrolls=%u\n", msgId, scrolls);
}


static inline void drawUserCharmOverlay() {
  if (hasUserCharm()) {
    // draw at left; icon is 9x7
    drawCharm565(CFG.userCharmId, /*x=*/0, /*y=*/0);
  }
}

//scene


// ---- Scene player ----
struct ScenePlayer {
  bool active = false;
  const uint16_t (*frames)[WIDTH*HEIGHT] = nullptr; // PROGMEM frames
  uint8_t frameCount = 0;
  uint8_t fps = 8;
  uint8_t idx = 0;
  uint32_t nextMs = 0;
} g_scene;

static inline uint8_t u5to8(uint8_t v){ return (v * 527 + 23) >> 6; }  // 5->8
static inline uint8_t u6to8(uint8_t v){ return (v * 259 + 33) >> 6; }  // 6->8


static void drawSceneFrame(const uint16_t* frame565) {
  matrix->fillScreen(0);
  for (int y=0; y<HEIGHT; ++y){
    for (int x=0; x<WIDTH; ++x){
      uint16_t c = pgm_read_word(&frame565[y*WIDTH + x]);
      CRGB rgb; rgb565ToCRGB(c, rgb);
      matrix->drawPixel(x, y, matrix->Color(rgb.r, rgb.g, rgb.b));
    }
  }
  FastLED.show();
}

static void sceneStart(const uint16_t (*frames)[WIDTH*HEIGHT],
                       uint8_t count, uint8_t fps)
{
  g_scene.frames     = frames;
  g_scene.frameCount = count;
  g_scene.fps        = (fps == 0 ? 8 : fps);
  g_scene.idx        = 0;
  g_scene.nextMs     = 0;
  g_scene.active     = true;

}


// ---- Simple scene queue (up to 8 ids) ----
struct {
  uint8_t buf[8];
  uint8_t head = 0, tail = 0;
  bool    pendingGap = false;
  uint32_t nextStartMs = 0;
  uint16_t gapMs = 0;   // optional pause between scenes
} g_sceneQ;

static inline bool sqEmpty() { return g_sceneQ.head == g_sceneQ.tail; }
static inline void sqClear() { g_sceneQ.head = g_sceneQ.tail = 0; g_sceneQ.pendingGap=false; }
static bool sqPush(uint8_t sid){
  uint8_t nt = (uint8_t)((g_sceneQ.tail + 1) & 7);
  if (nt == g_sceneQ.head) return false;         // full
  g_sceneQ.buf[g_sceneQ.tail] = sid; g_sceneQ.tail = nt; return true;
}
static bool sqPop(uint8_t &sid){
  if (sqEmpty()) return false;
  sid = g_sceneQ.buf[g_sceneQ.head]; g_sceneQ.head = (uint8_t)((g_sceneQ.head + 1) & 7);
  return true;
}



// ---- Play scene by numeric ID (startup & IR reuse) ----
#ifndef STARTUP_SCENE_ID
#define STARTUP_SCENE_ID 2
#endif

static void playSceneById(uint8_t sid) {
  switch (sid) {
    case 1: // nyan
      sceneStart(scene_nyan_frames, SCENE_NYAN_FRAMES, SCENE_NYAN_FPS);
      break;

    case 2: // fuse
      sceneStart(scene_fuse_frames, SCENE_FUSE_FRAMES, SCENE_FUSE_FPS);
      break;

    case 4: // volt
      sceneStart(scene_volt_frames, SCENE_VOLT_FRAMES, SCENE_VOLT_FPS);
      break;

    case 5: // hit
      sceneStart(scene_hit_frames, SCENE_HIT_FRAMES, SCENE_HIT_FPS);
      break;

    case 6: // hit
      sceneStart(scene_hitbars_frames, SCENE_HITBARS_FRAMES, SCENE_HITBARS_FPS);
      break;


    case 7: // tx
      sceneStart(scene_tx_frames, SCENE_TX_FRAMES, SCENE_TX_FPS);
      break;

      default:
      Serial.printf("[SCENE] unknown id %u\n", sid);
      break;
  }
}


static void sceneTick() {
  if (!g_scene.active) return;
  uint32_t now = millis();
  if (now < g_scene.nextMs) return;

  drawSceneFrame(g_scene.frames[g_scene.idx]);
  g_scene.idx++;

  if (g_scene.idx >= g_scene.frameCount) {
    // scene finished
    if (!sqEmpty()) {
      if (g_sceneQ.gapMs) {
        g_sceneQ.pendingGap = true;
        g_sceneQ.nextStartMs = millis() + g_sceneQ.gapMs;
        g_scene.active = false;   // pause until gap elapses
        return;
      } else {
        uint8_t nextId; sqPop(nextId);
        playSceneById(nextId);
        return;
      }
    }
    g_scene.active = false;           // no queued scene → fall back
    return;
  }
  g_scene.nextMs = now + (1000u / g_scene.fps);
}


// Public helpers
void playSceneChain(uint8_t sid1, uint8_t sid2, uint16_t gapMs = 0){
  sqClear(); sqPush(sid1); sqPush(sid2);
  g_sceneQ.gapMs = gapMs;
  if (!g_scene.active) { uint8_t s; if (sqPop(s)) playSceneById(s); }
}

void playScenes(const uint8_t* ids, uint8_t n, uint16_t gapMs = 0){
  sqClear();
  for (uint8_t i=0;i<n && i<8;i++) sqPush(ids[i]);
  g_sceneQ.gapMs = gapMs;
  if (!g_scene.active) { uint8_t s; if (sqPop(s)) playSceneById(s); }
}

static inline void sceneQueueTick(){
  if (g_sceneQ.pendingGap && !g_scene.active && (int32_t)(millis() - g_sceneQ.nextStartMs) >= 0) {
    g_sceneQ.pendingGap = false;
    uint8_t nextId; if (sqPop(nextId)) playSceneById(nextId);
  }
}


// ---------------- Fire animation ----------------
struct FireAnim { bool active=false; uint32_t t0=0; uint16_t dur=420; } g_fire;

static void fireAnimStart() {
  g_fire.active = true; g_fire.t0 = millis();
  tone(MEOW, 1200, 80);
  tone(PURR, 160,  80);
}
static void fireAnimRender() {
  if (!g_fire.active) return;
  uint32_t t = millis() - g_fire.t0;
  if (t >= g_fire.dur) { g_fire.active = false; return; }

  float p = float(t)/float(g_fire.dur);

  matrix->fillScreen(0);
  for (int y=0; y<HEIGHT; ++y) {
    float band = p*1.4f - (float(y)/(HEIGHT-1));
    uint8_t heat = band <= 0.f ? 0 : (uint8_t)min(255.f, band*255.f);
    CRGB col = HeatColor(heat);
    for (int x=0; x<WIDTH; ++x) leds[y*WIDTH + x] = col;
  }
  FastLED.show();
}

//stats

// ---- Stats view state machine ----
struct StatsView {
  bool active = false;
  uint8_t phase = 0;          // 0 = scrolling line, 1 = charms slideshow
  uint32_t t0 = 0;

  // unlocked (or all) list
  uint8_t list[32];
  uint8_t count = 0;
  uint8_t idx = 0;

  // phase-0 scroller state
  // SPLIT: left text + icons + right text
  String  lineL;              // "Score: ###"
  String  lineR;              // "  Charms:N"
  int16_t scX = 0;
  uint16_t scLeftW = 0;
  uint16_t scRightW = 0;
  uint16_t scIconsW = 0;
  uint16_t contentW = 0;
  uint32_t lastStep = 0;
  uint8_t  loops = 0;         // number of completed scroll loops
} g_stats;



static void statsStart() {
  // Build list of charms to show:
  g_stats.count = 0;
  if (CFG.sleepDisabled) {
    for (uint8_t i = 0; i < CHARM_COUNT; i++) g_stats.list[g_stats.count++] = i;
  } else {
    for (uint8_t i = 0; i < 32 && g_stats.count < CHARM_COUNT; i++) {
      if ((CFG.unlockedMask & (1UL << i)) && i < CHARM_COUNT) g_stats.list[g_stats.count++] = i;
    }
  }

  // Split text: LEFT then RIGHT ("  Charms: ") then icons after
  const uint32_t score = g_score.score;
  g_stats.lineL = String("Score: ") + score;     // LEFT
  g_stats.lineR = String("  Charms: ") + g_stats.count;
  g_stats.phase = 0;
  g_stats.idx = 0;
  g_stats.t0 = millis();

  // Geometry
  matrix->setFont(NULL);
  matrix->setTextSize(1);
  matrix->setTextWrap(false);

  g_stats.scLeftW  = g_stats.lineL.length() * CHAR_W;
  g_stats.scRightW = g_stats.lineR.length() * CHAR_W;
  g_stats.scIconsW = g_stats.count ? (g_stats.count * CHARM_W) + ((g_stats.count - 1) * ICON_SPACING) : 0;

  // total: LEFT + gap + RIGHT + (gap + ICONS if any)
  g_stats.contentW =
      g_stats.scLeftW
    + (g_stats.scRightW ? TEXT_ICON_GAP : 0)
    + g_stats.scRightW
    + (g_stats.scIconsW ? TEXT_ICON_GAP : 0)
    + g_stats.scIconsW;

  g_stats.scX = WIDTH;
  g_stats.lastStep = 0;
  g_stats.loops = 0;
  g_stats.active = true;
}

static void statsTick() {
  if (!g_stats.active) return;
  const uint32_t now = millis();

  if (g_stats.phase == 0) {
    if (now - g_stats.lastStep >= SCROLL_MS) {
      g_stats.lastStep = now;
    // inside statsTick(), phase == 0
    matrix->fillScreen(0);
    matrix->setFont(NULL);
    matrix->setTextSize(1);
    matrix->setTextWrap(false);
    matrix->setTextColor(matrix->Color(190, 190, 190));

    // LEFT: "Score: ###"
    matrix->setCursor(g_stats.scX, 0);
    matrix->print(g_stats.lineL);
    int16_t xAfterLeft = g_stats.scX + (int)g_stats.scLeftW;

    // RIGHT: "  Charms: X"
    int16_t xRight = xAfterLeft + (g_stats.scRightW ? TEXT_ICON_GAP : 0);
    matrix->setCursor(xRight, 0);
    matrix->print(g_stats.lineR);
    int16_t xAfterRight = xRight + (int)g_stats.scRightW;

    // ICONS right after the label-with-count
    if (g_stats.scIconsW) {
      int16_t xIcons = xAfterRight + TEXT_ICON_GAP;
      for (uint8_t i = 0; i < g_stats.count; ++i) {
        drawCharm565(g_stats.list[i], xIcons, 0);
        xIcons += CHARM_W + ICON_SPACING;
      }
    }

    FastLED.show();

    // advance + wrap
    g_stats.scX--;
    if (g_stats.scX < -(int)g_stats.contentW) {
      g_stats.scX = WIDTH;
      g_stats.loops++;
      if (g_stats.loops >= 1) { g_stats.phase = 1; g_stats.t0 = now; }
    }
    }
    return;
  }

  // Phase 1 unchanged…
  if (g_stats.phase == 1) {
    if (g_stats.idx < g_stats.count) {
      if (now - g_stats.t0 >= 400) {
        matrix->fillScreen(0);
        drawCharm565(g_stats.list[g_stats.idx], 3, 0);
        FastLED.show();
        g_stats.idx++;
        g_stats.t0 = now;
      }
    } else {
      g_stats.active = false;
    }
  }
}


// Show all charms in a simple strip for verification
void testAllCharms() {
  matrix->fillScreen(0);
  int16_t x = 0;
  for (uint8_t i = 0; i < CHARM_COUNT; ++i) {
    drawCharm565(i, x, 0);
    x += (CHARM_W + 1);
    if (x + CHARM_W > WIDTH) {  // wrap line if needed
      FastLED.show(); delay(600);
      matrix->fillScreen(0); x = 0;
    }
  }
  FastLED.show();
}


struct SceneDef {
  const uint16_t (*frames)[WIDTH*HEIGHT];
  uint8_t count;
  uint8_t fps;
};

// Map numeric IDs to your compiled scenes.
// If you already have SCENE_*_ID constants, feel free to switch on those.
static bool getSceneDef(uint8_t sid, SceneDef &out) {
  switch (sid) {
    case 1: out.frames = scene_nyan_frames;  out.count = SCENE_NYAN_FRAMES;  out.fps = SCENE_NYAN_FPS;  return true;
    case 2: out.frames = scene_fuse_frames;  out.count = SCENE_FUSE_FRAMES;  out.fps = SCENE_FUSE_FPS;  return true;
    case 3: out.frames = scene_kitty_frames; out.count = SCENE_KITTY_FRAMES; out.fps = SCENE_KITTY_FPS; return true;
    case 4: out.frames = scene_volt_frames;  out.count = SCENE_VOLT_FRAMES;  out.fps = SCENE_VOLT_FPS;  return true;
    case 5: out.frames = scene_hit_frames; out.count = SCENE_HIT_FRAMES; out.fps = SCENE_HIT_FPS; return true;
    case 6: out.frames = scene_hitbars_frames;  out.count = SCENE_HITBARS_FRAMES;  out.fps = SCENE_HITBARS_FPS;  return true;
    default: return false;
  }
}

// Draw a specific frame from a scene (bounds-safe)
static void drawSceneFrameIndex(uint8_t sid, uint8_t frameIdx) {
  SceneDef def;
  if (!getSceneDef(sid, def)) {
    Serial.printf("[SLEEP] unknown scene id=%u\n", sid);
    return;
  }
  if (def.count == 0) return;
  uint8_t idx = frameIdx % def.count;              // clamp/wrap safely
  const uint16_t *frame565 = def.frames[idx];

  matrix->fillScreen(0);
  for (int y = 0; y < HEIGHT; ++y) {
    for (int x = 0; x < WIDTH; ++x) {
      uint16_t c = pgm_read_word(&frame565[y*WIDTH + x]);
      CRGB rgb; rgb565ToCRGB(c, rgb);
      matrix->drawPixel(x, y, matrix->Color(rgb.r, rgb.g, rgb.b));
    }
  }
  FastLED.show();
}

static inline uint8_t sleepBreathValue128(uint32_t t) {
  const uint32_t T = SLEEP_PULSE_PERIOD_MS;
  if (!T) return SLEEP_BRIGHT_MIN_128;
  const uint32_t half = T >> 1;
  const uint8_t lo = SLEEP_BRIGHT_MIN_128, hi = SLEEP_BRIGHT_MAX_128;
  if (hi <= lo) return lo;
  t %= T;
  uint32_t up = (t < half) ? t : (T - t); // 0..half..0
  return (uint8_t)(lo + ( (uint32_t)(hi - lo) * up ) / half);
}

static void sleepBreathTick() {
  const uint32_t now = millis();
  if (now - g_sleepBreathLast < 33) return;  // ~30 Hz, cheap
  g_sleepBreathLast = now;

  uint8_t b128 = sleepBreathValue128(now);
  FastLED.setBrightness(map128to255(b128));
  FastLED.show();  // re-latch with new global brightness; pixel data unchanged
}

static inline void sleepEnter() {
    if (CFG.sleepDisabled) { 
    Serial.println("[SLEEP] ignored (disabled)");
    return; 
  }
  if (g_sleep.asleep) return;
  g_sleep.asleep = true;

  // stop visuals
  g_scene.active = false;
  g_fire.active  = false;
  g_stats.active = false;

  // save user brightness (0..128)
  g_sleep.savedBrightness128 = CFG.brightness;

  // Draw your chosen sleep frame once
  drawSceneFrameIndex(SLEEP_SCENE_ID, SLEEP_FRAME_INDEX);

  // Start at min brightness for the breathing effect
  CFG.brightness = SLEEP_BRIGHT_MIN_128;
  FastLED.setBrightness(map128to255(CFG.brightness));
  FastLED.show();

  noTone(MEOW); noTone(PURR);
  Serial.println("[SLEEP] entered (static scene frame)");
}

static inline void sleepExit() {
  if (!g_sleep.asleep) return;
  g_sleep.asleep = false;

  // restore brightness
  FastLED.setBrightness(map128to255(g_sleep.savedBrightness128));
  CFG.brightness = g_sleep.savedBrightness128;
              
  sceneStart(scene_kitty_frames, SCENE_KITTY_FRAMES, SCENE_KITTY_FPS);

  // restart idle scroll
  startScroll(/*idle*/true);

  // reset inactivity timer
  g_sleep.lastActivityMs = millis();

  Serial.println("[SLEEP] exited");
}




static void handleButtons() {
  if (millis() - btnLastMs < 30) return; // debounce
  btnLastMs = millis();

    // If asleep, any button press wakes and consumes the event
  if (g_sleep.asleep) {
    if (!digitalRead(BTN1) || !digitalRead(BTN2) || !digitalRead(BTN3)) {
      sleepExit();
      // consume this scan; avoid immediate extra actions
      prevB1 = digitalRead(BTN1);
      prevB2 = digitalRead(BTN2);
      prevB3 = digitalRead(BTN3);
      return;
    }
  }

  bool b1 = digitalRead(BTN1); // pullup: pressed == LOW
  bool b2 = digitalRead(BTN2);
  bool b3 = digitalRead(BTN3);

  if (!b1 && prevB1) {
    noteActivity();
    // BTN1: Stats view
    statsStart();
    Serial.println("SendStats");
  }

  if (!b2 && prevB2) {
    noteActivity();

    const uint32_t now = millis();
    const bool allowed = ((now - g_lastFireBtnMs) >= FIRE_MIN_MS);
    if (!allowed) {
      Serial.println("Send Fire (throttled)");
      tone(MEOW, 600, 40);
      prevB1 = b1; prevB2 = b2; prevB3 = b3;
      return;
    }
    g_lastFireBtnMs = now;

    // --- NEW priority: BOTH BTN1+BTN3 held -> SPECIAL MESSAGE 1 ---
    if (!b1 && !b3) {
      const uint8_t messageId = 1;
      const uint8_t scrolls   = 1;  // tweak if you want more passes
      const uint16_t value14  = ((messageId & 0x7F) << 7) | (scrolls & 0x7F);
      uint32_t frame = sirc20Value(/*op*/0x04, value14); // SHOW_MESSAGE
      sendSirc20(frame, "SHOW_MESSAGE 1");
      Serial.println("[UI] BTN1+BTN3+FIRE -> SHOW_MESSAGE 1");
      tone(MEOW, 1800, 70);
      tone(PURR, 240,  70);
    }
    // BTN1 held -> SPECIAL_SCENE 1
    else if (!b1) {
      uint32_t frame = sirc20Value(/*op*/0x05, /*sceneId*/1);
      sendSirc20(frame, "SCENE1");
      Serial.println("[UI] BTN1+FIRE -> SPECIAL_SCENE 1");
      tone(MEOW, 1400, 70);
      tone(PURR, 180,  70);
    }
    // BTN3 held -> SPECIAL_SCENE 2
    else if (!b3) {
      uint32_t frame = sirc20Value(/*op*/0x05, /*sceneId*/2);
      sendSirc20(frame, "SCENE2");
      Serial.println("[UI] BTN3+FIRE -> SPECIAL_SCENE 2");
      tone(MEOW, 1600, 70);
      tone(PURR, 220,  70);
    }
    // Plain FIRE
    else {
      sendFireBadge(/*charm=*/0);
      g_score.fires_total++;
      Serial.println("Send Fire");
    }
  }

  if (!b3 && prevB3) {
    noteActivity();
  currentTextEffect = static_cast<TextEffect>((currentTextEffect + 1) % numberOfEffects);
  // restart current text so you see the new effect immediately
  startScroll(g_scroll.isIdle, /*msgId*/0, g_scroll.repeats, g_scroll.color);
  Serial.printf("[UI] TextEffect -> %u\n", (unsigned)currentTextEffect);
    Serial.printf("[UI] TextMode -> %u\n", (unsigned)g_textMode);
  }
    prevB1 = b1; prevB2 = b2; prevB3 = b3;
}




// ---------------- IR handling ----------------
void handleIR() {
  if (!IrReceiver.decode()) return;
  auto &d = IrReceiver.decodedIRData;

  if (d.numberOfBits == 20) {
    uint32_t wLSB = sirc20FlipToLSB(d.decodedRawData);
    uint8_t  op; uint16_t attacker; uint8_t hi5;
    sirc20Unpack(wLSB, op, attacker, hi5);
        

    // --- SLEEP GATE: while asleep, only WAKE is honored ---
    if (g_sleep.asleep) {
      if (op == 0x02) { // WAKE
        Serial.println("[IR] WAKE (from sleep)");
        sleepExit();
      } // else ignore silently
      IrReceiver.resume();
      return;
    }

    // Awake: any valid IR counts as activity
    noteActivity();

    switch (op) {
      case 0x00: { // FIRE
        uint8_t charm = hi5 & 0x1F;
        Serial.printf("[IR] FIRE attacker=%u charm=%u\n", attacker, charm);
        g_score.hits_total++;
        scoreAwardRoll(5, "hit");
        

        // Unique badge ID tiers (only if first time seen)
        bool firstTime = statsAddAttacker(attacker);
        if (firstTime) {
          if (attacker >= 300 && attacker <= 400)      scoreAwardRoll(40, "unique badge 300-400");
          else if (attacker >= 255 && attacker <= 300) scoreAwardRoll(20, "unique badge 255-300");
          else if (attacker >= 1   && attacker <= 254) scoreAwardRoll(10, "unique badge 1-254");
        }

        // Charm unlock → if newly unlocked, Roll 20
        if (charm) {
          uint32_t bit = (1UL << (charm & 31));
          bool was = (CFG.unlockedMask & bit);
          if (!was) {
            CFG.unlockedMask |= bit;
            saveConfig(CFG);
            scoreAwardRoll(20, "unique charm");
          }
        }

        //fireAnimStart();
        playSceneById(6);
      } break;

      case 0x01: { // SLEEP (value14: minutes 0..127)
        uint8_t minutes = sirc20Value14(wLSB) & 0x7F;
        Serial.printf("[IR] SLEEP %u\n", minutes);
        if (minutes != 0) {
          g_sleep.autoTimeoutMs = (uint32_t)minutes * 60UL * 1000UL;
        }
        sleepEnter(); // enter immediately
      } break;

      case 0x02: { // WAKE
        Serial.println("[IR] WAKE");
        sleepExit();
      } break;

      case 0x03: { // SET_BRIGHTNESS (0..128)
        uint8_t b = sirc20Value14(wLSB) & 0x7F;
        Serial.printf("[IR] BRIGHT %u\n", b);
        FastLED.setBrightness(map128to255(b));
        CFG.brightness = b; saveConfig(CFG);
        FastLED.show();
      } break;

      case 0x04: { // SHOW_MESSAGE (value14 = (id<<7)|scrolls)
        uint16_t v = sirc20Value14(wLSB);
        uint8_t scrolls   =  v & 0x7F;
        uint8_t messageId = (v >> 7) & 0x7F;
        if (scrolls == 0) scrolls = 1;
        Serial.printf("[IR] SHOW_MESSAGE id=%u scrolls=%u\n", messageId, scrolls);
        onShowMessage(messageId, scrolls);
      } break;

      case 0x05: { // SPECIAL_SCENE
        uint16_t sid = sirc20Value14(wLSB) & 0x7F;
        g_score.scenes_triggered++;
        scoreAwardRoll(40, "special scene");
        Serial.printf("[IR] SCENE %u\n", sid);

        switch (sid) {
          case SCENE_NYAN_ID:
            sceneStart(scene_nyan_frames, SCENE_NYAN_FRAMES, SCENE_NYAN_FPS);
            break;
          case SCENE_FUSE_ID:
            sceneStart(scene_fuse_frames, SCENE_FUSE_FRAMES, SCENE_FUSE_FPS);
            break;
          case SCENE_KITTY_ID:
            sceneStart(scene_kitty_frames, SCENE_KITTY_FRAMES, SCENE_KITTY_FPS);
            break;

          default:
            Serial.println("[SCENE] unknown id");
            break;
        }
      } break;

      case 0x06: { // SCORE REQUEST (respond with triplet)
        #ifndef SCORE_SNIFFER
          if (!scoreThrottleOk()) {
            Serial.println("[SCORE] throttled");
            break;
          }
          if (SCORE_RESP_JITTER_MS) delay((uint32_t)random(0, SCORE_RESP_JITTER_MS + 1));

          #ifndef SCORE_TX_DIV
          #define SCORE_TX_DIV 1
          #endif
          uint32_t txScore = scoreGet() / SCORE_TX_DIV;
          if (txScore > 32767) txScore = 32767;
          sendScoreTriplet(CFG.id, (uint16_t)txScore);
        #endif

      } break;

      #ifndef SCORE_SNIFFER
        case 0x07: // SCORE_RSP_LO
        case 0x08: // SCORE_RSP_MID
        case 0x09: // SCORE_RSP_HI
      #endif


      #ifdef SCORE_SNIFFER
        case 0x07: processScorePart(attacker, 0, hi5); break;
        case 0x08: processScorePart(attacker, 1, hi5); break;
        case 0x09: processScorePart(attacker, 2, hi5); break;
      #endif
        // (optional: handle assembling others' scores)
        break;

      default:
        #ifdef LOUD_SERIAL
        Serial.printf("[IR] Unknown op=0x%02X (wLSB=0x%05lX)\n", op, (unsigned long)wLSB);
        #endif
        break;
    }
  }

  IrReceiver.resume();
}

// ---------------- Arduino lifecycle ----------------
void setup() {
  delay(500);
    vreg_set_voltage(VREG_VOLTAGE_1_00);  // options: 0_85, 0_90, 0_95, 1_00, 1_05, 1_10, 1_15, 1_20, 1_25, 1_30
    sleep_ms(10);                          // allow voltage to settle
    set_sys_clock_khz(125000, true);       // now safe to overclock

  Serial.begin(115200);
  watchdog_enable(2000, 1);  // 2s watchdog

  pinMode(MEOW, OUTPUT); pinMode(PURR, OUTPUT);
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(LOW_BATT_PIN, OUTPUT);
  digitalWrite(LOW_BATT_PIN, LOW_BATT_OFF);
    // After pinMode(BTN1/2/3, INPUT_PULLUP);
  // If FIRE (BTN2) is held during boot, disable auto-sleep (runtime only)
  // --- BOOT TOGGLE: hold FIRE to flip sleepDisabled ---
  delay(50); // settle
  analogReadResolution(12);
  pinMode(BATTERY_ADC_PIN, INPUT);
  g_batt_mV = readBatteryMilliVoltsOnce();

  // Seed RNG from jitter
  randomSeed( (uint32_t)micros() ^ (uint32_t)analogRead(A0) ^ (uint32_t)millis() );

  #ifdef IR_TX_PIN
    IrSender.begin(IR_TX_PIN);
  #endif

  LittleFS.begin();
  loadConfig(CFG);
  scoreLoad();
  statsLoadAttackers();

  delay(50);
  if (digitalRead(BTN2) == LOW) {
    CFG.sleepDisabled = !CFG.sleepDisabled;
    saveConfig(CFG);
    Serial.printf("[BOOT] sleepDisabled toggled -> %d\n", (int)CFG.sleepDisabled);
  }

  FastLED.addLeds<CHIPSET, PIXEL_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.setBrightness(map128to255(CFG.brightness));
  FastLED.clear(true); FastLED.show();

  IrReceiver.begin(IR_RX_PIN, DISABLE_LED_FEEDBACK); // disable LED feedback on RP2040 boards

  Serial.printf("[IR] RX on pin %d (SIRC-20)\n", IR_RX_PIN);
  g_sleep.lastActivityMs = millis();
  delay(200);
  //testAllCharms();
  // start idle name scroll
    if (CFG.sleepDisabled) {
    const uint16_t Y = matrix->Color(255,255,0);
      startScroll(/*idle*/true);
  
    playSceneChain(4, 2, 0);
    Serial.println("SLEEP DISABLED");

  }
  else{
        playSceneById(STARTUP_SCENE_ID);

    startScroll(/*idle*/true);
  }
  //startScroll(/*idle*/true);
  //playSceneById(STARTUP_SCENE_ID);


  #ifdef SCORE_SNIFFER
    tripsInit();
      // Bring up UART link to ESP32-S3
    UartLink::init(230400);

    // Allow ESP to adjust request interval at runtime:
    UartLink::onCommand([](const String& line){
      // Expect JSON like: {"t":"set","req_ms":800}
      int p = line.indexOf("\"req_ms\"");
      if (p >= 0) {
        p = line.indexOf(':', p);
        if (p >= 0) {
          uint32_t v = (uint32_t) line.substring(p+1).toInt();
          v = constrain(v, 200u, 5000u);
          g_reqPeriodMs = (uint16_t)v;
          UartLink::logf("info", "req_ms set -> %u", g_reqPeriodMs);
        }
      }
    });
    #endif

    Serial.printf("[BOOT] Build %s %s\n", BUILD_DATE, BUILD_TIME);

}

void loop() {
  watchdog_update();  
  handleIR();
  handleButtons();

  if (g_sleep.asleep) {
    // Do not render anything while asleep; still process IR/buttons/serial below.
    sleepBreathTick();
  } else {
    if (g_scene.active) {
      sceneTick();
    } else if (g_fire.active) {
      fireAnimRender();
    } else if (g_stats.active) {
      statsTick();
    } else {
      renderScrollTick();
    }
  }
  sceneQueueTick();
  batteryPollTick();

  handleSerial(); 
  // If sleep is disabled, pin runtime brightness to 20 (0..128 scale), without persisting.
  if (CFG.sleepDisabled && CFG.brightness != 3) {
    CFG.brightness = 13;                            // runtime only
    FastLED.setBrightness(map128to255(13));
    FastLED.show();
  }

  // --- Auto-sleep when awake (gated by persistent CFG.sleepDisabled) ---
  if (!g_sleep.asleep) {
    if (!CFG.sleepDisabled &&
        g_sleep.autoTimeoutMs &&
        (uint32_t)(millis() - g_sleep.lastActivityMs) >= g_sleep.autoTimeoutMs) {
      sleepEnter();
    }
  }

  scoreMaybeAutoSave();  // throttle persistent writes

#ifdef SCORE_SNIFFER
  tickRequestTx();
  tripsHousekeep();
  UartLink::poll();

  // Heartbeat every 3 seconds
  static uint32_t tHb = 0;
  if ((uint32_t)(millis() - tHb) > 3000) {
    UartLink::sendHeartbeat(g_reqPeriodMs, uniqueBadgeCount());
    tHb = millis();
  }
#endif


}
// ======================================================================
