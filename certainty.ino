#include <Arduino.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <Wire.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/time.h"

static const uint8_t NUM_OUTPUTS = 8;
static const uint8_t OUTPUT_PINS[NUM_OUTPUTS] = {27, 28, 29, 0, 3, 4, 2, 1};

static const uint32_t DEFAULT_BPM = 90;
static const uint32_t MIN_BPM = 20;
static const uint32_t MAX_BPM = 320;
static const uint32_t GATE_PULSE_US = 10000;
static const uint32_t START_DELAY_US = 200000;
static const uint32_t HEARTBEAT_MS = 500;
static const uint32_t STATUS_MS = 1000;

static const uint16_t PWM_WRAP = 255;
static const uint8_t PWM_CLKDIV_INT = 12;
static const uint8_t PWM_CLKDIV_FRAC = 3;
static const uint32_t PWM_SAMPLE_RATE_HZ = 4000;
static const int64_t PWM_SAMPLE_PERIOD_US = 1000000 / PWM_SAMPLE_RATE_HZ;

static const uint8_t I2C_ADDRESS = 0x55;
static const uint8_t I2C_SDA_PIN = 6;
static const uint8_t I2C_SCL_PIN = 7;
static const bool ENABLE_I2C_BPM = true;
static const uint32_t I2C_INIT_DELAY_MS = 3000;
static const uint32_t STAGE_ON_MS = 180;
static const uint32_t STAGE_OFF_MS = 220;
static const uint32_t STAGE_GAP_MS = 700;

// XIAO RP2040 maps GPIO6/7 to Wire (Wire0), not Wire1.
static TwoWire &g_i2cBus = Wire;

enum OutputMode : uint8_t {
  OUTPUT_MODE_GATE = 0,
  OUTPUT_MODE_LFO_TRI = 1,
};

struct Ratio {
  uint16_t num;
  uint16_t den;
};

static const Ratio DEFAULT_RATIOS[NUM_OUTPUTS] = {
    {1, 2}, {1, 2}, {1, 1}, {1, 1}, {2, 1}, {2, 1}, {4, 1}, {4, 1}};

static const OutputMode DEFAULT_MODES[NUM_OUTPUTS] = {
    OUTPUT_MODE_GATE, OUTPUT_MODE_LFO_TRI, OUTPUT_MODE_GATE, OUTPUT_MODE_LFO_TRI,
    OUTPUT_MODE_GATE, OUTPUT_MODE_LFO_TRI, OUTPUT_MODE_GATE, OUTPUT_MODE_LFO_TRI};

struct OutputState {
  uint8_t pin;
  Ratio ratio;
  OutputMode mode;

  uint64_t periodUs;
  uint64_t lfoAnchorUs;
  bool ratioPending;
  uint64_t pendingApplyUs;
  Ratio pendingRatio;
  uint64_t nextRiseUs;
  uint64_t nextFallUs;
  bool gateHigh;
  uint32_t gateRises;
  uint32_t gateFalls;

  uint8_t pwmSlice;
  uint8_t pwmChannel;
  uint16_t lastPwmLevel;
  uint32_t lfoUpdates;
};

struct TransportState {
  uint32_t bpm;
  uint64_t beatPeriodUs;
  uint64_t anchorUs;
  uint32_t resetCount;
};

static OutputState g_outputs[NUM_OUTPUTS];
static TransportState g_transport = {};
static alarm_id_t g_gateAlarmId = -1;
static repeating_timer g_pwmTimer = {};
static volatile uint32_t g_gateSchedulerRuns = 0;
static volatile uint32_t g_gateSchedulerMisses = 0;
static volatile uint32_t g_pwmRuns = 0;

static uint32_t g_pwmSliceMask = 0;
static bool g_pwmTimerRunning = false;
static volatile bool g_pendingI2cBpm = false;
static volatile uint16_t g_pendingI2cBpmValue = DEFAULT_BPM;
static volatile bool g_pendingI2cRatio = false;
static volatile uint8_t g_pendingI2cRatioOutput = 0;
static volatile uint8_t g_pendingI2cRatioNum = 1;
static volatile uint8_t g_pendingI2cRatioDen = 1;
static volatile uint32_t g_i2cRxCount = 0;
static volatile uint32_t g_i2cAppliedCount = 0;
static volatile uint32_t g_i2cRatioAppliedCount = 0;
static volatile uint32_t g_i2cErrorCount = 0;
static bool g_i2cEnabled = false;

static char g_cmdBuffer[32];
static uint8_t g_cmdLen = 0;

static int64_t gateAlarmCallback(alarm_id_t id, void *user_data);
static bool pwmSampleCallback(repeating_timer *rt);
static void i2cReceiveHandler(int bytesCount);
static void i2cRequestHandler();
static void processPendingI2cBpm();
static void processPendingI2cRatio();
static bool tryInitI2cBpmReceiver();
static void stageMarker(uint8_t count);
static bool queueRatioChange(uint8_t outIndex, uint8_t num, uint8_t den);

static inline absolute_time_t absoluteFromUs(uint64_t usSinceBoot) {
  absolute_time_t t;
  update_us_since_boot(&t, usSinceBoot);
  return t;
}

static inline uint32_t clampBpm(uint32_t bpm) {
  if (bpm < MIN_BPM) {
    return MIN_BPM;
  }
  if (bpm > MAX_BPM) {
    return MAX_BPM;
  }
  return bpm;
}

static inline uint64_t periodFromRatio(uint64_t beatPeriodUs, Ratio ratio) {
  uint64_t period = (beatPeriodUs * ratio.den + (ratio.num / 2u)) / ratio.num;
  const uint64_t minPeriod = (uint64_t)GATE_PULSE_US + 1000u;
  if (period < minPeriod) {
    return minPeriod;
  }
  return period;
}

static inline uint64_t nextBeatBoundaryAfterLocked(uint64_t nowUs) {
  const uint64_t anchorUs = g_transport.anchorUs;
  const uint64_t beatUs = g_transport.beatPeriodUs;
  if (beatUs == 0) {
    return nowUs + 1;
  }
  if ((int64_t)(nowUs - anchorUs) < 0) {
    return anchorUs;
  }
  const uint64_t elapsedUs = nowUs - anchorUs;
  const uint64_t beatsElapsed = elapsedUs / beatUs;
  return anchorUs + ((beatsElapsed + 1u) * beatUs);
}

static inline uint64_t alignToGlobalPhaseGridAtOrAfterLocked(uint64_t atLeastUs, uint64_t periodUs) {
  const uint64_t anchorUs = g_transport.anchorUs;
  if (periodUs == 0) {
    return atLeastUs;
  }
  if ((int64_t)(atLeastUs - anchorUs) <= 0) {
    return anchorUs;
  }
  const uint64_t elapsedUs = atLeastUs - anchorUs;
  const uint64_t steps = (elapsedUs + periodUs - 1u) / periodUs;
  return anchorUs + (steps * periodUs);
}

static inline void debugLedInit() {
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif
#ifdef PIN_LED_R
  pinMode(PIN_LED_R, OUTPUT);
#endif
#ifdef PIN_LED_G
  pinMode(PIN_LED_G, OUTPUT);
#endif
#ifdef PIN_LED_B
  pinMode(PIN_LED_B, OUTPUT);
#endif
}

static inline void debugLedSet(bool on) {
#ifdef PIN_LED_R
  digitalWrite(PIN_LED_R, on ? HIGH : LOW);
#endif
#ifdef PIN_LED_G
  digitalWrite(PIN_LED_G, on ? HIGH : LOW);
#endif
#ifdef PIN_LED_B
  digitalWrite(PIN_LED_B, on ? HIGH : LOW);
#endif
#if !defined(PIN_LED_R) && !defined(PIN_LED_G) && !defined(PIN_LED_B) && defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
#endif
}

static void stageMarker(uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    debugLedSet(true);
    delay(STAGE_ON_MS);
    debugLedSet(false);
    delay(STAGE_OFF_MS);
  }
  delay(STAGE_GAP_MS);
}

static const char *modeLabel(OutputMode mode) {
  if (mode == OUTPUT_MODE_GATE) {
    return "gate";
  }
  return "tri";
}

static void printRatioLabel(Ratio ratio) {
  if (ratio.num == 1 && ratio.den == 1) {
    Serial.print("1x");
    return;
  }
  if (ratio.num == 1) {
    Serial.print("/");
    Serial.print(ratio.den);
    return;
  }
  if (ratio.den == 1) {
    Serial.print(ratio.num);
    Serial.print("x");
    return;
  }
  Serial.print(ratio.num);
  Serial.print("/");
  Serial.print(ratio.den);
  Serial.print("x");
}

static void configureOutputPinsForModes() {
  g_pwmSliceMask = 0;

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_outputs[i];
    if (out.mode == OUTPUT_MODE_GATE) {
      gpio_init(out.pin);
      gpio_set_dir(out.pin, GPIO_OUT);
      gpio_put(out.pin, 0);
      continue;
    }

    gpio_set_function(out.pin, GPIO_FUNC_PWM);
    out.pwmSlice = (uint8_t)pwm_gpio_to_slice_num(out.pin);
    out.pwmChannel = (uint8_t)pwm_gpio_to_channel(out.pin);
    g_pwmSliceMask |= (1u << out.pwmSlice);
    pwm_set_gpio_level(out.pin, 0);
  }

  for (uint8_t slice = 0; slice < NUM_PWM_SLICES; ++slice) {
    if ((g_pwmSliceMask & (1u << slice)) == 0) {
      continue;
    }
    pwm_set_clkdiv_int_frac(slice, PWM_CLKDIV_INT, PWM_CLKDIV_FRAC);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_enabled(slice, true);
  }
}

static uint16_t triangleLevelFromPhase(uint64_t nowUs, uint64_t anchorUs, uint64_t periodUs) {
  if (periodUs < 2) {
    return 0;
  }

  if ((int64_t)(nowUs - anchorUs) < 0) {
    return 0;
  }

  const uint64_t elapsedUs = nowUs - anchorUs;
  const uint64_t phaseUs = elapsedUs % periodUs;
  const uint64_t halfUs = periodUs / 2u;
  if (halfUs == 0) {
    return 0;
  }

  uint64_t numer = 0;
  if (phaseUs < halfUs) {
    numer = phaseUs;
  } else {
    numer = periodUs - phaseUs;
  }

  uint64_t level = (numer * PWM_WRAP + (halfUs / 2u)) / halfUs;
  if (level > PWM_WRAP) {
    level = PWM_WRAP;
  }
  return (uint16_t)level;
}

static void scheduleNextAlarmLocked() {
  uint64_t nextDueUs = UINT64_MAX;
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    const OutputState &out = g_outputs[i];
    if (out.mode != OUTPUT_MODE_GATE) {
      continue;
    }
    uint64_t dueUs = out.gateHigh ? out.nextFallUs : out.nextRiseUs;
    if (out.ratioPending && out.pendingApplyUs < dueUs) {
      dueUs = out.pendingApplyUs;
    }
    if (dueUs < nextDueUs) {
      nextDueUs = dueUs;
    }
  }

  if (nextDueUs == UINT64_MAX) {
    g_gateAlarmId = -1;
    return;
  }

  g_gateAlarmId = add_alarm_at(absoluteFromUs(nextDueUs), gateAlarmCallback, nullptr, true);
  if (g_gateAlarmId < 0) {
    g_gateSchedulerMisses++;
  }
}

static void resetTransportLocked(uint32_t bpm, uint64_t nowUs) {
  if (g_gateAlarmId >= 0) {
    cancel_alarm(g_gateAlarmId);
    g_gateAlarmId = -1;
  }

  g_transport.bpm = clampBpm(bpm);
  g_transport.beatPeriodUs = 60000000ull / g_transport.bpm;
  g_transport.anchorUs = nowUs + START_DELAY_US;
  g_transport.resetCount++;

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_outputs[i];
    out.periodUs = periodFromRatio(g_transport.beatPeriodUs, out.ratio);
    out.lfoAnchorUs = g_transport.anchorUs;
    out.ratioPending = false;
    out.pendingApplyUs = g_transport.anchorUs;
    out.pendingRatio = out.ratio;
    out.nextRiseUs = g_transport.anchorUs;
    out.nextFallUs = g_transport.anchorUs + GATE_PULSE_US;
    out.gateHigh = false;
    out.gateRises = 0;
    out.gateFalls = 0;
    out.lfoUpdates = 0;
    out.lastPwmLevel = 0;

    if (out.mode == OUTPUT_MODE_GATE) {
      gpio_put(out.pin, 0);
    } else {
      pwm_set_gpio_level(out.pin, 0);
    }
  }

  scheduleNextAlarmLocked();
}

static void applyBpmAndReset(uint32_t bpm) {
  const uint32_t irqState = save_and_disable_interrupts();
  resetTransportLocked(bpm, time_us_64());
  restore_interrupts(irqState);
}

static bool queueRatioChange(uint8_t outIndex, uint8_t num, uint8_t den) {
  if (outIndex >= NUM_OUTPUTS || num == 0 || den == 0) {
    return false;
  }
  g_pendingI2cRatioOutput = outIndex;
  g_pendingI2cRatioNum = num;
  g_pendingI2cRatioDen = den;
  g_pendingI2cRatio = true;
  return true;
}

static void i2cReceiveHandler(int bytesCount) {
  if (bytesCount <= 0) {
    return;
  }

  uint8_t raw[6] = {0, 0, 0, 0, 0, 0};
  uint8_t n = 0;
  while (n < sizeof(raw) && n < (uint8_t)bytesCount && g_i2cBus.available()) {
    raw[n++] = (uint8_t)g_i2cBus.read();
  }
  while (g_i2cBus.available()) {
    (void)g_i2cBus.read();
  }

  g_i2cRxCount++;

  // Teletype generic I2C layout: first byte is command id.
  // cmd 0 = set BPM, with either:
  // - [0, bpm8] via IISB1
  // - [0, bpm_hi, bpm_lo] via IIS1
  if (raw[0] == 0x00) {
    uint16_t bpm = 0;
    bool valid = false;
    if (n == 2) {
      bpm = raw[1];
      valid = true;
    } else if (n >= 3) {
      bpm = (uint16_t)(((uint16_t)raw[1] << 8) | raw[2]);
      valid = true;
    }

    if (!valid || (uint32_t)bpm < MIN_BPM || (uint32_t)bpm > MAX_BPM) {
      g_i2cErrorCount++;
      return;
    }

    g_pendingI2cBpmValue = bpm;
    g_pendingI2cBpm = true;
    return;
  }

  // cmd 1 = set one output ratio: [1, out, num, den]
  // out accepts 1..8 (user-facing) or 0..7 (zero-based).
  if (raw[0] == 0x01 && n >= 4) {
    uint8_t outRaw = raw[1];
    uint8_t outIndex = 0;
    if (outRaw >= 1 && outRaw <= NUM_OUTPUTS) {
      outIndex = (uint8_t)(outRaw - 1);
    } else if (outRaw < NUM_OUTPUTS) {
      outIndex = outRaw;
    } else {
      g_i2cErrorCount++;
      return;
    }

    const uint8_t num = raw[2];
    const uint8_t den = raw[3];
    if (num == 0 || den == 0) {
      g_i2cErrorCount++;
      return;
    }

    if (!queueRatioChange(outIndex, num, den)) {
      g_i2cErrorCount++;
      return;
    }
    return;
  }

  g_i2cErrorCount++;
}

static void i2cRequestHandler() {
  uint16_t bpm = 0;
  const uint32_t irqState = save_and_disable_interrupts();
  bpm = (uint16_t)g_transport.bpm;
  restore_interrupts(irqState);

  uint8_t response[2] = {(uint8_t)(bpm >> 8), (uint8_t)(bpm & 0xFF)};
  g_i2cBus.write(response, 2);
}

static void processPendingI2cBpm() {
  bool hasPending = false;
  uint16_t bpm = 0;
  uint32_t currentBpm = 0;

  const uint32_t irqState = save_and_disable_interrupts();
  if (g_pendingI2cBpm) {
    bpm = g_pendingI2cBpmValue;
    g_pendingI2cBpm = false;
    hasPending = true;
  }
  currentBpm = g_transport.bpm;
  restore_interrupts(irqState);

  if (!hasPending) {
    return;
  }

  // Only reset transport when BPM actually changes.
  if ((uint32_t)bpm == currentBpm) {
    return;
  }

  applyBpmAndReset(bpm);
  g_i2cAppliedCount++;
}

static void processPendingI2cRatio() {
  bool hasPending = false;
  uint8_t outIndex = 0;
  uint8_t num = 1;
  uint8_t den = 1;

  const uint32_t irqState = save_and_disable_interrupts();
  if (g_pendingI2cRatio) {
    hasPending = true;
    outIndex = g_pendingI2cRatioOutput;
    num = g_pendingI2cRatioNum;
    den = g_pendingI2cRatioDen;
    g_pendingI2cRatio = false;
  }
  restore_interrupts(irqState);

  if (!hasPending) {
    return;
  }

  if (outIndex >= NUM_OUTPUTS || num == 0 || den == 0) {
    g_i2cErrorCount++;
    return;
  }

  const uint64_t nowUs = time_us_64();
  const uint32_t applyIrqState = save_and_disable_interrupts();
  OutputState &out = g_outputs[outIndex];
  const Ratio nextRatio = {(uint16_t)num, (uint16_t)den};
  const uint64_t applyAtUs = nextBeatBoundaryAfterLocked(nowUs);

  out.pendingRatio = nextRatio;
  out.pendingApplyUs = applyAtUs;
  out.ratioPending = true;

  if (out.mode == OUTPUT_MODE_GATE) {
    scheduleNextAlarmLocked();
  }
  restore_interrupts(applyIrqState);

  g_i2cRatioAppliedCount++;
}

static bool tryInitI2cBpmReceiver() {
  if (g_i2cEnabled) {
    return true;
  }

  stageMarker(8);

  g_i2cBus.setSDA(I2C_SDA_PIN);
  stageMarker(9);
  g_i2cBus.setSCL(I2C_SCL_PIN);
  stageMarker(10);
  g_i2cBus.begin(I2C_ADDRESS);
  stageMarker(11);
  g_i2cBus.onReceive(i2cReceiveHandler);
  g_i2cBus.onRequest(i2cRequestHandler);
  stageMarker(12);
  g_i2cEnabled = true;
  return true;
}

static void printStatus() {
  uint32_t bpm = 0;
  uint64_t beatUs = 0;
  uint64_t anchorUs = 0;
  uint32_t resetCount = 0;
  uint32_t gateRuns = 0;
  uint32_t gateMisses = 0;
  uint32_t pwmRuns = 0;
  uint32_t i2cRx = 0;
  uint32_t i2cApplied = 0;
  uint32_t i2cRatioApplied = 0;
  uint32_t i2cErr = 0;

  uint32_t gateRises[NUM_OUTPUTS];
  uint32_t gateFalls[NUM_OUTPUTS];
  uint16_t pwmLevels[NUM_OUTPUTS];
  uint32_t lfoUpdates[NUM_OUTPUTS];
  bool gateHigh[NUM_OUTPUTS];

  const uint32_t irqState = save_and_disable_interrupts();
  bpm = g_transport.bpm;
  beatUs = g_transport.beatPeriodUs;
  anchorUs = g_transport.anchorUs;
  resetCount = g_transport.resetCount;
  gateRuns = g_gateSchedulerRuns;
  gateMisses = g_gateSchedulerMisses;
  pwmRuns = g_pwmRuns;
  i2cRx = g_i2cRxCount;
  i2cApplied = g_i2cAppliedCount;
  i2cRatioApplied = g_i2cRatioAppliedCount;
  i2cErr = g_i2cErrorCount;
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    gateRises[i] = g_outputs[i].gateRises;
    gateFalls[i] = g_outputs[i].gateFalls;
    pwmLevels[i] = g_outputs[i].lastPwmLevel;
    lfoUpdates[i] = g_outputs[i].lfoUpdates;
    gateHigh[i] = g_outputs[i].gateHigh;
  }
  restore_interrupts(irqState);

  Serial.print("status bpm=");
  Serial.print(bpm);
  Serial.print(" beat_us=");
  Serial.print((unsigned long)beatUs);
  Serial.print(" anchor_us=");
  Serial.print((unsigned long)anchorUs);
  Serial.print(" resets=");
  Serial.print(resetCount);
  Serial.print(" gate_runs=");
  Serial.print(gateRuns);
  Serial.print(" gate_misses=");
  Serial.print(gateMisses);
  Serial.print(" pwm_runs=");
  Serial.print(pwmRuns);
  Serial.print(" i2c_rx=");
  Serial.print(i2cRx);
  Serial.print(" i2c_applied=");
  Serial.print(i2cApplied);
  Serial.print(" i2c_ratio=");
  Serial.print(i2cRatioApplied);
  Serial.print(" i2c_err=");
  Serial.println(i2cErr);

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    const OutputState &out = g_outputs[i];
    Serial.print("  out");
    Serial.print(i + 1);
    Serial.print(" mode=");
    Serial.print(modeLabel(out.mode));
    Serial.print(" ratio=");
    printRatioLabel(out.ratio);
    Serial.print(" period_us=");
    Serial.print((unsigned long)out.periodUs);
    if (out.mode == OUTPUT_MODE_GATE) {
      Serial.print(" rises=");
      Serial.print(gateRises[i]);
      Serial.print(" falls=");
      Serial.print(gateFalls[i]);
      Serial.print(" state=");
      Serial.println(gateHigh[i] ? "HIGH" : "LOW");
    } else {
      Serial.print(" level=");
      Serial.print(pwmLevels[i]);
      Serial.print("/");
      Serial.print(PWM_WRAP);
      Serial.print(" lfo_updates=");
      Serial.println(lfoUpdates[i]);
    }
  }
}

static void printBriefDiag() {
  uint32_t bpm = 0;
  uint32_t gateMisses = 0;
  uint32_t gateRises[NUM_OUTPUTS];
  uint16_t pwmLevel2 = 0;
  uint16_t pwmLevel4 = 0;
  uint16_t pwmLevel6 = 0;
  uint16_t pwmLevel8 = 0;
  uint32_t i2cApplied = 0;
  uint32_t i2cRatioApplied = 0;
  uint32_t i2cErr = 0;

  const uint32_t irqState = save_and_disable_interrupts();
  bpm = g_transport.bpm;
  gateMisses = g_gateSchedulerMisses;
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    gateRises[i] = g_outputs[i].gateRises;
  }
  pwmLevel2 = g_outputs[1].lastPwmLevel;
  pwmLevel4 = g_outputs[3].lastPwmLevel;
  pwmLevel6 = g_outputs[5].lastPwmLevel;
  pwmLevel8 = g_outputs[7].lastPwmLevel;
  i2cApplied = g_i2cAppliedCount;
  i2cRatioApplied = g_i2cRatioAppliedCount;
  i2cErr = g_i2cErrorCount;
  restore_interrupts(irqState);

  Serial.print("diag bpm=");
  Serial.print(bpm);
  Serial.print(" gate_misses=");
  Serial.print(gateMisses);
  Serial.print(" o1=");
  Serial.print(gateRises[0]);
  Serial.print(" o3=");
  Serial.print(gateRises[2]);
  Serial.print(" o5=");
  Serial.print(gateRises[4]);
  Serial.print(" o7=");
  Serial.print(gateRises[6]);
  Serial.print(" tri2=");
  Serial.print(pwmLevel2);
  Serial.print(" tri4=");
  Serial.print(pwmLevel4);
  Serial.print(" tri6=");
  Serial.print(pwmLevel6);
  Serial.print(" tri8=");
  Serial.print(pwmLevel8);
  Serial.print(" i2c_applied=");
  Serial.print(i2cApplied);
  Serial.print(" i2c_ratio=");
  Serial.print(i2cRatioApplied);
  Serial.print(" i2c_err=");
  Serial.println(i2cErr);
}

static void handleCommand(const char *cmd) {
  unsigned long parsedBpm = 0;
  if (sscanf(cmd, "bpm %lu", &parsedBpm) == 1) {
    const uint32_t nextBpm = clampBpm((uint32_t)parsedBpm);
    applyBpmAndReset(nextBpm);
    Serial.print("ok bpm=");
    Serial.println(nextBpm);
    return;
  }

  unsigned long outRaw = 0;
  unsigned long numRaw = 0;
  unsigned long denRaw = 0;
  if (sscanf(cmd, "ratio %lu %lu %lu", &outRaw, &numRaw, &denRaw) == 3) {
    if (numRaw == 0 || denRaw == 0 || numRaw > 255 || denRaw > 255) {
      Serial.println("err ratio expects: ratio <out> <num> <den>, num/den in 1..255");
      return;
    }

    uint8_t outIndex = 0xFF;
    if (outRaw >= 1 && outRaw <= NUM_OUTPUTS) {
      outIndex = (uint8_t)(outRaw - 1);
    } else if (outRaw < NUM_OUTPUTS) {
      outIndex = (uint8_t)outRaw;
    } else {
      Serial.println("err out expects 1..8 (or 0..7)");
      return;
    }

    if (!queueRatioChange(outIndex, (uint8_t)numRaw, (uint8_t)denRaw)) {
      Serial.println("err failed to queue ratio update");
      return;
    }

    Serial.print("ok ratio queued(next beat) out");
    Serial.print(outIndex + 1);
    Serial.print("=");
    Serial.print(numRaw);
    Serial.print("/");
    Serial.println(denRaw);
    return;
  }

  if (strcmp(cmd, "status") == 0) {
    printStatus();
    return;
  }

  if (strcmp(cmd, "help") == 0) {
    Serial.println("commands: bpm <20-320>, ratio <out> <num> <den>, status, help");
    return;
  }

  if (cmd[0] != '\0') {
    Serial.print("err unknown command: ");
    Serial.println(cmd);
  }
}

static void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_cmdLen > 0) {
        g_cmdBuffer[g_cmdLen] = '\0';
        handleCommand(g_cmdBuffer);
        g_cmdLen = 0;
      }
      continue;
    }

    if (g_cmdLen >= sizeof(g_cmdBuffer) - 1) {
      g_cmdLen = 0;
      Serial.println("err command too long");
      continue;
    }

    g_cmdBuffer[g_cmdLen++] = (char)tolower((unsigned char)c);
  }
}

static int64_t gateAlarmCallback(alarm_id_t id, void *user_data) {
  (void)id;
  (void)user_data;

  g_gateSchedulerRuns++;
  const uint64_t nowUs = time_us_64();

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_outputs[i];
    if (out.mode != OUTPUT_MODE_GATE) {
      continue;
    }

    if (out.ratioPending && (int64_t)(nowUs - out.pendingApplyUs) >= 0) {
      out.ratio = out.pendingRatio;
      out.periodUs = periodFromRatio(g_transport.beatPeriodUs, out.ratio);
      out.nextRiseUs = alignToGlobalPhaseGridAtOrAfterLocked(out.pendingApplyUs, out.periodUs);
      out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
      out.gateHigh = false;
      gpio_put(out.pin, 0);
      out.ratioPending = false;
    }

    if (out.gateHigh && (int64_t)(nowUs - out.nextFallUs) >= 0) {
      gpio_put(out.pin, 0);
      out.gateHigh = false;
      out.gateFalls++;

      do {
        out.nextRiseUs += out.periodUs;
        if ((int64_t)(nowUs - out.nextRiseUs) >= 0) {
          g_gateSchedulerMisses++;
        }
      } while ((int64_t)(nowUs - out.nextRiseUs) >= 0);

      out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
    }

    if (!out.gateHigh && (int64_t)(nowUs - out.nextRiseUs) >= 0) {
      gpio_put(out.pin, 1);
      out.gateHigh = true;
      out.gateRises++;
      out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
    }
  }

  scheduleNextAlarmLocked();
  return 0;
}

// Fixed-rate PWM update ISR for all LFO outputs.
static bool pwmSampleCallback(repeating_timer *rt) {
  (void)rt;
  g_pwmRuns++;

  const uint64_t nowUs = time_us_64();

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_outputs[i];
    if (out.mode != OUTPUT_MODE_LFO_TRI) {
      continue;
    }

    if (out.ratioPending && (int64_t)(nowUs - out.pendingApplyUs) >= 0) {
      out.ratio = out.pendingRatio;
      out.periodUs = periodFromRatio(g_transport.beatPeriodUs, out.ratio);
      out.lfoAnchorUs = g_transport.anchorUs;
      out.ratioPending = false;
    }

    const uint16_t level = triangleLevelFromPhase(nowUs, out.lfoAnchorUs, out.periodUs);
    pwm_set_gpio_level(out.pin, level);
    out.lastPwmLevel = level;
    out.lfoUpdates++;
  }

  return true;
}

void setup() {
  Serial.begin(115200);

  debugLedInit();
  debugLedSet(false);

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_outputs[i];
    out.pin = OUTPUT_PINS[i];
    out.ratio = DEFAULT_RATIOS[i];
    out.mode = DEFAULT_MODES[i];

    out.periodUs = 0;
    out.lfoAnchorUs = 0;
    out.ratioPending = false;
    out.pendingApplyUs = 0;
    out.pendingRatio = {1, 1};
    out.nextRiseUs = 0;
    out.nextFallUs = 0;
    out.gateHigh = false;
    out.gateRises = 0;
    out.gateFalls = 0;
    out.pwmSlice = 0;
    out.pwmChannel = 0;
    out.lastPwmLevel = 0;
    out.lfoUpdates = 0;
  }

  // Startup: 3 LED blinks.
  for (uint8_t i = 0; i < 3; ++i) {
    debugLedSet(true);
    delay(120);
    debugLedSet(false);
    delay(120);
  }

  // Startup: output chase as GPIO before switching PWM pins to PWM function.
  for (uint8_t pass = 0; pass < 2; ++pass) {
    for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
      gpio_init(g_outputs[i].pin);
      gpio_set_dir(g_outputs[i].pin, GPIO_OUT);
      gpio_put(g_outputs[i].pin, 1);
      delay(50);
      gpio_put(g_outputs[i].pin, 0);
    }
  }

  configureOutputPinsForModes();
  stageMarker(1);

  g_pwmTimerRunning =
      add_repeating_timer_us(-PWM_SAMPLE_PERIOD_US, pwmSampleCallback, nullptr, &g_pwmTimer);
  stageMarker(2);

  applyBpmAndReset(DEFAULT_BPM);
  stageMarker(3);

  if (ENABLE_I2C_BPM) {
    // Prepare bus pins as pulled-up inputs first; actual Wire init is deferred
    // to loop until after startup settles.
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    stageMarker(4);
  } else {
    stageMarker(6);
  }

  Serial.println("certainty clock phase1+3 foundation");
  Serial.println("defaults: out1/3/5/7 gate, out2/4/6/8 triangle lfo");
  Serial.println("ratios: /2,/2,1x,1x,2x,2x,4x,4x");
  Serial.print("i2c addr=0x");
  Serial.println(I2C_ADDRESS, HEX);
  Serial.println("i2c bpm write: [bpm8] or [bpm_hi bpm_lo] or [0x00 bpm_hi bpm_lo]");
  Serial.println("i2c ratio write: [0x01 out num den] (out=1..8 or 0..7, applies next beat)");
  if (ENABLE_I2C_BPM) {
    Serial.print("i2c init=deferred, delay_ms=");
    Serial.println(I2C_INIT_DELAY_MS);
  } else {
    Serial.println("i2c init=disabled (safe boot)");
  }
  Serial.print("pwm carrier hz~");
  Serial.println(
      (uint32_t)(125000000u / ((PWM_WRAP + 1u) * (PWM_CLKDIV_INT + (PWM_CLKDIV_FRAC / 16.0f)))));
  Serial.print("pwm sample hz=");
  Serial.println(PWM_SAMPLE_RATE_HZ);
  Serial.print("pwm timer=");
  Serial.println(g_pwmTimerRunning ? "ok" : "failed");
  Serial.println("commands: bpm <20-320>, ratio <out> <num> <den>, status, help");
  printStatus();
  stageMarker(7);
}

void loop() {
  static uint32_t lastLedToggleMs = 0;
  static uint32_t lastStatusMs = 0;
  static uint32_t lastI2cRetryMs = 0;
  static bool ledOn = false;

  const uint32_t now = millis();
  if (ENABLE_I2C_BPM && !g_i2cEnabled && now >= I2C_INIT_DELAY_MS &&
      (now - lastI2cRetryMs >= 1000)) {
    lastI2cRetryMs = now;
    const bool sdaHigh = (digitalRead(I2C_SDA_PIN) == HIGH);
    const bool sclHigh = (digitalRead(I2C_SCL_PIN) == HIGH);
    if (!sdaHigh || !sclHigh) {
      if (!sdaHigh && !sclHigh) {
        stageMarker(6);
      } else if (!sdaHigh) {
        stageMarker(13);
      } else {
        stageMarker(14);
      }
    }
    if (tryInitI2cBpmReceiver()) {
      stageMarker(5);
      Serial.println("i2c init=ok");
    } else {
      stageMarker(6);
    }
  }

  if (ENABLE_I2C_BPM) {
    processPendingI2cBpm();
    processPendingI2cRatio();
  }
  processSerialCommands();

  if (now - lastLedToggleMs >= HEARTBEAT_MS) {
    lastLedToggleMs = now;
    ledOn = !ledOn;
    debugLedSet(ledOn);
  }

  if (now - lastStatusMs >= STATUS_MS) {
    lastStatusMs = now;
    printBriefDiag();
  }

  delay(2);
}
