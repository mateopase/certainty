#include <Arduino.h>
#include <Wire.h>
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/regs/intctrl.h"
#include "hardware/sync.h"
#include "pico/time.h"

static const uint8_t NUM_OUTPUTS = 8;
static const uint8_t OUTPUT_PINS[NUM_OUTPUTS] = {27, 28, 29, 0, 3, 4, 2, 1};

static const uint32_t DEFAULT_BPM = 90;
static const uint32_t MIN_BPM = 20;
static const uint32_t MAX_BPM = 320;
static const uint32_t GATE_PULSE_US = 10000;
static const uint32_t START_DELAY_US = 0;
static const uint32_t HEARTBEAT_MS = 500;
static const bool ENABLE_I2C_ACTIVITY_LED = true;
static const uint32_t I2C_ACTIVITY_PULSE_MS = 20;

static const uint16_t PWM_WRAP = 255;
static const uint8_t PWM_CLKDIV_INT = 12;
static const uint8_t PWM_CLKDIV_FRAC = 3;
static const uint32_t PWM_SAMPLE_RATE_HZ = 4000;
static const int64_t PWM_SAMPLE_PERIOD_US = 1000000 / PWM_SAMPLE_RATE_HZ;

static const uint8_t I2C_ADDRESS = 0x55;
static const uint8_t I2C_SDA_PIN = 6;
static const uint8_t I2C_SCL_PIN = 7;
static const bool ENABLE_I2C_BPM = true;
static const uint32_t I2C_RETRY_MS = 250;
static const uint8_t I2C_IRQ_PRIORITY = 0x40;
static const uint8_t TIMER_IRQ_PRIORITY = 0x80;

// XIAO RP2040 maps GPIO6/7 to Wire (Wire0), not Wire1.
static TwoWire &g_i2cBus = Wire;

enum OutputShape : uint8_t {
  OUTPUT_SHAPE_TRIG = 0,
  OUTPUT_SHAPE_ASR = 1,
};

enum OutputRun : uint8_t {
  OUTPUT_RUN_ONE_SHOT = 0,
  OUTPUT_RUN_LOOP = 1,
};

struct Ratio {
  uint16_t num;
  uint16_t den;
};

static const Ratio DEFAULT_RATIOS[NUM_OUTPUTS] = {
    {1, 2}, {1, 2}, {1, 1}, {1, 1}, {2, 1}, {2, 1}, {4, 1}, {4, 1}};

static const OutputShape DEFAULT_SHAPES[NUM_OUTPUTS] = {
    OUTPUT_SHAPE_TRIG, OUTPUT_SHAPE_ASR, OUTPUT_SHAPE_TRIG, OUTPUT_SHAPE_ASR,
    OUTPUT_SHAPE_TRIG, OUTPUT_SHAPE_ASR, OUTPUT_SHAPE_TRIG, OUTPUT_SHAPE_ASR};

static const OutputRun DEFAULT_RUNS[NUM_OUTPUTS] = {
    OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP,
    OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP, OUTPUT_RUN_LOOP};

struct OutputState {
  uint8_t pin;
  Ratio ratio;
  OutputShape shape;
  OutputRun run;

  uint64_t periodUs;
  uint64_t lfoAnchorUs;
  uint8_t asrSus;
  uint8_t asrSkew;
  uint8_t asrA;
  uint8_t asrS;
  uint8_t asrR;
  bool ratioPending;
  uint64_t pendingApplyUs;
  Ratio pendingRatio;
  bool behaviorPending;
  uint64_t behaviorPendingApplyUs;
  OutputShape pendingShape;
  OutputRun pendingRun;
  uint16_t pendingTriggerCount;
  bool asrOneShotActive;
  uint64_t asrOneShotStartUs;
  uint64_t asrOneShotDurationUs;
  uint64_t asrAttackUs;
  uint64_t asrSustainUs;
  uint64_t asrReleaseUs;
  uint64_t asrOneShotAttackUs;
  uint64_t asrOneShotSustainUs;
  uint64_t asrOneShotReleaseUs;
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

static const uint8_t I2C_RX_MAX_BYTES = 6;
static const uint8_t I2C_RX_QUEUE_LEN = 128;

enum I2cEventType : uint8_t {
  I2C_EVENT_SET_BPM = 0,
  I2C_EVENT_SET_RATIO = 1,
  I2C_EVENT_SET_MODE = 2,
  I2C_EVENT_SET_ASR = 3,
  I2C_EVENT_TRIGGER = 4,
};

struct I2cEvent {
  I2cEventType type;
  uint8_t out;
  uint8_t a;
  uint8_t b;
  uint8_t mask;
  uint16_t count;
};

struct I2cRxFrame {
  uint8_t len;
  uint8_t data[I2C_RX_MAX_BYTES];
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
static I2cRxFrame g_i2cRxQueue[I2C_RX_QUEUE_LEN];
static volatile uint8_t g_i2cRxHead = 0;
static volatile uint8_t g_i2cRxTail = 0;
static volatile uint32_t g_i2cRxCount = 0;
static volatile uint32_t g_i2cEventQueueDropCount = 0;
static volatile uint32_t g_i2cEventQueueDropTriggerCount = 0;
static volatile uint32_t g_i2cEventQueueDropConfigCount = 0;
static volatile uint16_t g_i2cLedPulsePending = 0;
static volatile uint32_t g_i2cAppliedCount = 0;
static volatile uint32_t g_i2cRatioAppliedCount = 0;
static volatile uint32_t g_i2cModeAppliedCount = 0;
static volatile uint32_t g_i2cAsrAppliedCount = 0;
static volatile uint32_t g_i2cTriggerAppliedCount = 0;
static volatile uint32_t g_i2cTriggerSaturatedCount = 0;
static volatile uint32_t g_i2cErrorCount = 0;
static bool g_i2cEnabled = false;
static const uint32_t I2C_ERROR_LATCH_MS = 180;

static int64_t gateAlarmCallback(alarm_id_t id, void *user_data);
static bool pwmSampleCallback(repeating_timer *rt);
static void i2cReceiveHandler(int bytesCount);
static void i2cRequestHandler();
static bool decodeI2cEvent(const uint8_t *raw, uint8_t len, I2cEvent *event);
static bool enqueueI2cRxFromIsr(const I2cRxFrame &frame);
static bool dequeueI2cRx(I2cRxFrame *frame);
static void processI2cEvents();
static void applyI2cEvent(const I2cEvent &event);
static void applyTriggerEvent(uint8_t mask, uint16_t count);
static void processDueConfigChanges();
static void ensureGateSchedulerRunning();
static void configureIrqPriorities();
static void processI2cActivityLed(uint32_t nowMs);
static bool tryInitI2cBpmReceiver();
static void deriveAsrWeights(uint8_t sus, uint8_t skew, uint8_t *a, uint8_t *s, uint8_t *r);
static void computeAsrSegments(uint64_t durationUs,
                               uint8_t a,
                               uint8_t s,
                               uint8_t r,
                               uint64_t *attackUs,
                               uint64_t *sustainUs,
                               uint64_t *releaseUs);
static void refreshAsrTimingLocked(OutputState &out);

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

static void deriveAsrWeights(uint8_t sus, uint8_t skew, uint8_t *a, uint8_t *s, uint8_t *r) {
  // 0..10 controls from TT:
  // sus=0 -> no sustain (triangle-ish), sus=10 -> full sustain (square-ish)
  // skew=0 -> instant attack / long release, skew=10 -> long attack / instant release
  if (sus > 10) {
    sus = 10;
  }
  if (skew > 10) {
    skew = 10;
  }

  const uint8_t rem = (uint8_t)(10 - sus);
  const uint8_t attack = (uint8_t)((rem * skew + 5u) / 10u);
  const uint8_t release = (uint8_t)(rem - attack);
  *a = attack;
  *s = sus;
  *r = release;
}

static void computeAsrSegments(uint64_t durationUs,
                               uint8_t a,
                               uint8_t s,
                               uint8_t r,
                               uint64_t *attackUs,
                               uint64_t *sustainUs,
                               uint64_t *releaseUs) {
  if (durationUs < 2) {
    *attackUs = 0;
    *sustainUs = 0;
    *releaseUs = 0;
    return;
  }

  const uint32_t sum = (uint32_t)a + (uint32_t)s + (uint32_t)r;
  if (sum == 0) {
    *attackUs = 0;
    *sustainUs = 0;
    *releaseUs = 0;
    return;
  }

  uint64_t attack = (durationUs * (uint64_t)a) / sum;
  uint64_t sustain = (durationUs * (uint64_t)s) / sum;

  // Keep non-zero segments non-zero when possible.
  if (a > 0 && attack == 0 && durationUs >= 3) {
    attack = 1;
  }
  if (s > 0 && sustain == 0 && durationUs >= 3) {
    sustain = 1;
  }

  if (attack + sustain >= durationUs) {
    if (sustain > 0) {
      sustain = durationUs - 1;
    } else if (attack > 0) {
      attack = durationUs - 1;
    }
  }

  *attackUs = attack;
  *sustainUs = sustain;
  *releaseUs = durationUs - attack - sustain;
}

static void refreshAsrTimingLocked(OutputState &out) {
  computeAsrSegments(out.periodUs, out.asrA, out.asrS, out.asrR, &out.asrAttackUs, &out.asrSustainUs, &out.asrReleaseUs);
}

static inline void configurePinAsGateOutput(uint8_t pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_put(pin, 0);
}

static inline void configurePinAsPwmOutput(OutputState &out) {
  gpio_set_function(out.pin, GPIO_FUNC_PWM);
  out.pwmSlice = (uint8_t)pwm_gpio_to_slice_num(out.pin);
  out.pwmChannel = (uint8_t)pwm_gpio_to_channel(out.pin);
  pwm_set_clkdiv_int_frac(out.pwmSlice, PWM_CLKDIV_INT, PWM_CLKDIV_FRAC);
  pwm_set_wrap(out.pwmSlice, PWM_WRAP);
  pwm_set_enabled(out.pwmSlice, true);
  pwm_set_gpio_level(out.pin, 0);
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

static inline bool usesPwm(OutputShape shape) {
  return shape == OUTPUT_SHAPE_ASR;
}

static const char *shapeLabel(OutputShape shape) {
  if (shape == OUTPUT_SHAPE_TRIG) {
    return "trig";
  }
  return "asr";
}

static const char *runLabel(OutputRun run) {
  if (run == OUTPUT_RUN_LOOP) {
    return "loop";
  }
  return "one";
}

static void configureOutputPinsForModes() {
  g_pwmSliceMask = 0;

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_outputs[i];
    if (!usesPwm(out.shape)) {
      configurePinAsGateOutput(out.pin);
      continue;
    }

    configurePinAsPwmOutput(out);
    g_pwmSliceMask |= (1u << out.pwmSlice);
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

static uint16_t asrLevelFromElapsed(uint64_t elapsedUs,
                                    uint64_t durationUs,
                                    uint64_t attackUs,
                                    uint64_t sustainUs,
                                    uint64_t releaseUs) {
  if (durationUs < 2) {
    return 0;
  }

  uint64_t t = elapsedUs;
  if (attackUs > 0) {
    if (t < attackUs) {
      uint64_t level = (t * PWM_WRAP + (attackUs / 2u)) / attackUs;
      if (level > PWM_WRAP) {
        level = PWM_WRAP;
      }
      return (uint16_t)level;
    }
    t -= attackUs;
  }

  if (sustainUs > 0) {
    if (t < sustainUs) {
      return PWM_WRAP;
    }
    t -= sustainUs;
  }

  if (releaseUs > 0 && t < releaseUs) {
    const uint64_t remaining = releaseUs - t;
    uint64_t level = (remaining * PWM_WRAP + (releaseUs / 2u)) / releaseUs;
    if (level > PWM_WRAP) {
      level = PWM_WRAP;
    }
    return (uint16_t)level;
  }

  return 0;
}

static uint16_t asrLevelFromTime(uint64_t nowUs,
                                 uint64_t startUs,
                                 uint64_t durationUs,
                                 uint64_t attackUs,
                                 uint64_t sustainUs,
                                 uint64_t releaseUs,
                                 bool looping) {
  if (durationUs < 2 || (int64_t)(nowUs - startUs) < 0) {
    return 0;
  }
  uint64_t elapsedUs = nowUs - startUs;
  if (looping) {
    elapsedUs %= durationUs;
  } else if (elapsedUs >= durationUs) {
    return 0;
  }
  return asrLevelFromElapsed(elapsedUs, durationUs, attackUs, sustainUs, releaseUs);
}

static void scheduleNextAlarmLocked() {
  const uint64_t nowUs = time_us_64();
  uint64_t nextDueUs = UINT64_MAX;
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    const OutputState &out = g_outputs[i];
    uint64_t dueUs = UINT64_MAX;
    if (out.shape == OUTPUT_SHAPE_TRIG) {
      if (out.gateHigh && out.nextFallUs < dueUs) {
        dueUs = out.nextFallUs;
      } else if (out.run == OUTPUT_RUN_LOOP && out.nextRiseUs < dueUs) {
        dueUs = out.nextRiseUs;
      } else if (out.run == OUTPUT_RUN_ONE_SHOT && out.pendingTriggerCount > 0 && nowUs < dueUs) {
        dueUs = nowUs;
      }
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

static void rescheduleGateAlarmLocked() {
  if (g_gateAlarmId >= 0) {
    cancel_alarm(g_gateAlarmId);
    g_gateAlarmId = -1;
  }
  scheduleNextAlarmLocked();
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
    refreshAsrTimingLocked(out);
    out.lfoAnchorUs = g_transport.anchorUs;
    out.ratioPending = false;
    out.pendingApplyUs = g_transport.anchorUs;
    out.pendingRatio = out.ratio;
    out.behaviorPending = false;
    out.behaviorPendingApplyUs = g_transport.anchorUs;
    out.pendingShape = out.shape;
    out.pendingRun = out.run;
    out.pendingTriggerCount = 0;
    out.asrOneShotActive = false;
    out.asrOneShotStartUs = g_transport.anchorUs;
    out.asrOneShotDurationUs = out.periodUs;
    out.asrOneShotAttackUs = out.asrAttackUs;
    out.asrOneShotSustainUs = out.asrSustainUs;
    out.asrOneShotReleaseUs = out.asrReleaseUs;
    out.nextRiseUs = g_transport.anchorUs;
    out.nextFallUs = g_transport.anchorUs + GATE_PULSE_US;
    out.gateHigh = false;
    out.gateRises = 0;
    out.gateFalls = 0;
    out.lfoUpdates = 0;
    out.lastPwmLevel = 0;

    if (!usesPwm(out.shape)) {
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

static inline bool decodeOutIndex(uint8_t outRaw, uint8_t *outIndex) {
  if (outRaw >= 1 && outRaw <= NUM_OUTPUTS) {
    *outIndex = (uint8_t)(outRaw - 1);
    return true;
  }
  if (outRaw < NUM_OUTPUTS) {
    *outIndex = outRaw;
    return true;
  }
  return false;
}

static bool decodeI2cEvent(const uint8_t *raw, uint8_t len, I2cEvent *event) {
  if (len == 0 || event == nullptr) {
    return false;
  }

  // cmd 0 = set BPM, with either:
  // - [0, bpm8] via IISB1
  // - [0, bpm_hi, bpm_lo] via IIS1
  if (raw[0] == 0x00) {
    uint16_t bpm = 0;
    if (len == 2) {
      bpm = raw[1];
    } else if (len >= 3) {
      bpm = (uint16_t)(((uint16_t)raw[1] << 8) | raw[2]);
    } else {
      return false;
    }

    if ((uint32_t)bpm < MIN_BPM || (uint32_t)bpm > MAX_BPM) {
      return false;
    }

    event->type = I2C_EVENT_SET_BPM;
    event->out = 0;
    event->a = 0;
    event->b = 0;
    event->mask = 0;
    event->count = bpm;
    return true;
  }

  // cmd 1 = set one output ratio: [1, out, num, den]
  if (raw[0] == 0x01 && len >= 4) {
    uint8_t outIndex = 0;
    if (!decodeOutIndex(raw[1], &outIndex)) {
      return false;
    }
    const uint8_t num = raw[2];
    const uint8_t den = raw[3];
    if (num == 0 || den == 0) {
      return false;
    }

    event->type = I2C_EVENT_SET_RATIO;
    event->out = outIndex;
    event->a = num;
    event->b = den;
    event->mask = 0;
    event->count = 0;
    return true;
  }

  // cmd 2 = set one output behavior: [2, out, shape, run]
  if (raw[0] == 0x02 && len >= 4) {
    uint8_t outIndex = 0;
    if (!decodeOutIndex(raw[1], &outIndex)) {
      return false;
    }
    const uint8_t shapeRaw = raw[2];
    const uint8_t runRaw = raw[3];
    if (shapeRaw > (uint8_t)OUTPUT_SHAPE_ASR || runRaw > (uint8_t)OUTPUT_RUN_LOOP) {
      return false;
    }

    event->type = I2C_EVENT_SET_MODE;
    event->out = outIndex;
    event->a = shapeRaw;
    event->b = runRaw;
    event->mask = 0;
    event->count = 0;
    return true;
  }

  // cmd 3 = set one output ASR controls: [3, out, sus, skew]
  if (raw[0] == 0x03 && len >= 4) {
    uint8_t outIndex = 0;
    if (!decodeOutIndex(raw[1], &outIndex)) {
      return false;
    }
    const uint8_t sus = raw[2];
    const uint8_t skew = raw[3];
    if (sus > 10 || skew > 10) {
      return false;
    }

    event->type = I2C_EVENT_SET_ASR;
    event->out = outIndex;
    event->a = sus;
    event->b = skew;
    event->mask = 0;
    event->count = 0;
    return true;
  }

  // cmd 4 = trigger one-shot channel(s): [4, out] or [4, 0xFF, mask]
  if (raw[0] == 0x04 && len >= 2) {
    uint8_t mask = 0;
    if (raw[1] == 0xFF) {
      if (len < 3) {
        return false;
      }
      mask = raw[2];
    } else {
      uint8_t outIndex = 0;
      if (!decodeOutIndex(raw[1], &outIndex)) {
        return false;
      }
      mask = (uint8_t)(1u << outIndex);
    }

    event->type = I2C_EVENT_TRIGGER;
    event->out = 0;
    event->a = 0;
    event->b = 0;
    event->mask = mask;
    event->count = 1;
    return true;
  }

  return false;
}

static bool enqueueI2cRxFromIsr(const I2cRxFrame &frame) {
  const uint8_t head = g_i2cRxHead;
  const uint8_t tail = g_i2cRxTail;
  const uint8_t nextHead = (uint8_t)((head + 1u) % I2C_RX_QUEUE_LEN);
  if (nextHead == tail) {
    return false;
  }

  g_i2cRxQueue[head] = frame;
  g_i2cRxHead = nextHead;
  return true;
}

static bool dequeueI2cRx(I2cRxFrame *frame) {
  const uint32_t irqState = save_and_disable_interrupts();
  const uint8_t tail = g_i2cRxTail;
  if (tail == g_i2cRxHead) {
    restore_interrupts(irqState);
    return false;
  }

  *frame = g_i2cRxQueue[tail];
  g_i2cRxTail = (uint8_t)((tail + 1u) % I2C_RX_QUEUE_LEN);
  restore_interrupts(irqState);
  return true;
}

static void i2cReceiveHandler(int bytesCount) {
  if (bytesCount <= 0) {
    return;
  }

  uint8_t raw[I2C_RX_MAX_BYTES] = {0};
  uint8_t len = (uint8_t)bytesCount;
  if (len > I2C_RX_MAX_BYTES) {
    len = I2C_RX_MAX_BYTES;
  }

  // Consume exactly the reported frame length so decode length is stable.
  for (uint8_t i = 0; i < len; ++i) {
    const int value = g_i2cBus.read();
    raw[i] = (uint8_t)((value < 0) ? 0 : value);
  }
  for (int i = (int)len; i < bytesCount; ++i) {
    (void)g_i2cBus.read();
  }
  while (g_i2cBus.available()) {
    (void)g_i2cBus.read();
  }

  g_i2cRxCount++;
  if (ENABLE_I2C_ACTIVITY_LED && g_i2cLedPulsePending < 0xFFFFu) {
    g_i2cLedPulsePending++;
  }
  if (len == 0) {
    return;
  }

  I2cRxFrame frame = {};
  frame.len = len;
  for (uint8_t i = 0; i < len; ++i) {
    frame.data[i] = raw[i];
  }
  if (!enqueueI2cRxFromIsr(frame)) {
    g_i2cEventQueueDropCount++;
    if (raw[0] == 0x04) {
      g_i2cEventQueueDropTriggerCount++;
    } else {
      g_i2cEventQueueDropConfigCount++;
    }
  }
}

static void i2cRequestHandler() {
  // Request/response is intentionally disabled for now.
}

static void applyTriggerEvent(uint8_t mask, uint16_t count) {
  if (mask == 0 || count == 0) {
    return;
  }

  uint32_t appliedCount = 0;
  bool gateScheduleChanged = false;

  const uint32_t irqState = save_and_disable_interrupts();
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    if ((mask & (1u << i)) == 0) {
      continue;
    }

    OutputState &out = g_outputs[i];

    if (out.run == OUTPUT_RUN_ONE_SHOT) {
      const uint32_t sum = (uint32_t)out.pendingTriggerCount + (uint32_t)count;
      if (sum > 0xFFFFu) {
        out.pendingTriggerCount = 0xFFFFu;
        g_i2cTriggerSaturatedCount++;
      } else {
        out.pendingTriggerCount = (uint16_t)sum;
      }
      appliedCount += count;
      if (out.shape == OUTPUT_SHAPE_TRIG) {
        gateScheduleChanged = true;
      }
      continue;
    }

    // If a behavior change to one-shot is pending, accumulate triggers now and
    // consume after the mode transitions.
    if (out.behaviorPending && out.pendingRun == OUTPUT_RUN_ONE_SHOT) {
      const uint32_t sum = (uint32_t)out.pendingTriggerCount + (uint32_t)count;
      if (sum > 0xFFFFu) {
        out.pendingTriggerCount = 0xFFFFu;
        g_i2cTriggerSaturatedCount++;
      } else {
        out.pendingTriggerCount = (uint16_t)sum;
      }
      appliedCount += count;
      if (out.pendingShape == OUTPUT_SHAPE_TRIG) {
        gateScheduleChanged = true;
      }
    }
  }
  if (gateScheduleChanged) {
    rescheduleGateAlarmLocked();
  }
  restore_interrupts(irqState);

  g_i2cTriggerAppliedCount += appliedCount;
}

static void applyI2cEvent(const I2cEvent &event) {
  switch (event.type) {
    case I2C_EVENT_SET_BPM: {
      const uint32_t bpm = event.count;
      if (bpm != g_transport.bpm) {
        applyBpmAndReset(bpm);
        g_i2cAppliedCount++;
      }
      break;
    }

    case I2C_EVENT_SET_RATIO: {
      const uint8_t outIndex = event.out;
      const uint8_t num = event.a;
      const uint8_t den = event.b;
      if (outIndex >= NUM_OUTPUTS || num == 0 || den == 0) {
        g_i2cErrorCount++;
        break;
      }

      const uint64_t nowUs = time_us_64();
      const uint32_t irqState = save_and_disable_interrupts();
      OutputState &out = g_outputs[outIndex];
      out.pendingRatio = {(uint16_t)num, (uint16_t)den};
      out.pendingApplyUs = nextBeatBoundaryAfterLocked(nowUs);
      out.ratioPending = true;
      restore_interrupts(irqState);
      g_i2cRatioAppliedCount++;
      break;
    }

    case I2C_EVENT_SET_MODE: {
      const uint8_t outIndex = event.out;
      const OutputShape shape = (OutputShape)event.a;
      const OutputRun run = (OutputRun)event.b;
      if (outIndex >= NUM_OUTPUTS || (shape != OUTPUT_SHAPE_TRIG && shape != OUTPUT_SHAPE_ASR) ||
          (run != OUTPUT_RUN_ONE_SHOT && run != OUTPUT_RUN_LOOP)) {
        g_i2cErrorCount++;
        break;
      }

      const uint64_t nowUs = time_us_64();
      const uint32_t irqState = save_and_disable_interrupts();
      OutputState &out = g_outputs[outIndex];
      out.pendingShape = shape;
      out.pendingRun = run;
      out.behaviorPendingApplyUs = nextBeatBoundaryAfterLocked(nowUs);
      out.behaviorPending = true;
      restore_interrupts(irqState);
      g_i2cModeAppliedCount++;
      break;
    }

    case I2C_EVENT_SET_ASR: {
      const uint8_t outIndex = event.out;
      const uint8_t sus = event.a;
      const uint8_t skew = event.b;
      if (outIndex >= NUM_OUTPUTS || sus > 10 || skew > 10) {
        g_i2cErrorCount++;
        break;
      }

      uint8_t a = 0;
      uint8_t s = 0;
      uint8_t r = 0;
      deriveAsrWeights(sus, skew, &a, &s, &r);

      const uint32_t irqState = save_and_disable_interrupts();
      OutputState &out = g_outputs[outIndex];
      out.asrSus = sus;
      out.asrSkew = skew;
      out.asrA = a;
      out.asrS = s;
      out.asrR = r;
      refreshAsrTimingLocked(out);
      restore_interrupts(irqState);
      g_i2cAsrAppliedCount++;
      break;
    }

    case I2C_EVENT_TRIGGER:
      applyTriggerEvent(event.mask, event.count);
      break;
  }
}

static void processI2cEvents() {
  I2cRxFrame frame = {};
  I2cEvent event = {};
  while (dequeueI2cRx(&frame)) {
    if (!decodeI2cEvent(frame.data, frame.len, &event)) {
      g_i2cErrorCount++;
      continue;
    }
    applyI2cEvent(event);
  }
}

static void processDueConfigChanges() {
  const uint64_t nowUs = time_us_64();
  bool gateScheduleChanged = false;

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    bool channelAffectsGateSchedule = false;
    const uint32_t irqState = save_and_disable_interrupts();
    OutputState &out = g_outputs[i];
    const bool ratioDue = out.ratioPending && (int64_t)(nowUs - out.pendingApplyUs) >= 0;
    const bool behaviorDue = out.behaviorPending && (int64_t)(nowUs - out.behaviorPendingApplyUs) >= 0;

    if (!ratioDue && !behaviorDue) {
      restore_interrupts(irqState);
      continue;
    }

    if (ratioDue) {
      out.ratio = out.pendingRatio;
      out.periodUs = periodFromRatio(g_transport.beatPeriodUs, out.ratio);
      refreshAsrTimingLocked(out);
      out.ratioPending = false;

      if (out.shape == OUTPUT_SHAPE_TRIG && out.run == OUTPUT_RUN_LOOP) {
        out.nextRiseUs = alignToGlobalPhaseGridAtOrAfterLocked(out.pendingApplyUs, out.periodUs);
        out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
        out.gateHigh = false;
        gpio_put(out.pin, 0);
        channelAffectsGateSchedule = true;
      } else if (out.shape == OUTPUT_SHAPE_ASR && out.run == OUTPUT_RUN_LOOP) {
        out.lfoAnchorUs = g_transport.anchorUs;
      }
    }

    if (behaviorDue) {
      const OutputShape oldShape = out.shape;
      const OutputShape newShape = out.pendingShape;
      const OutputRun newRun = out.pendingRun;
      const uint64_t applyUs = out.behaviorPendingApplyUs;

      if (oldShape != newShape) {
        if (oldShape == OUTPUT_SHAPE_TRIG) {
          if (out.gateHigh) {
            gpio_put(out.pin, 0);
            out.gateHigh = false;
            out.gateFalls++;
          }
        } else {
          pwm_set_gpio_level(out.pin, 0);
          out.lastPwmLevel = 0;
        }

        if (newShape == OUTPUT_SHAPE_TRIG) {
          configurePinAsGateOutput(out.pin);
        } else {
          configurePinAsPwmOutput(out);
          out.lastPwmLevel = 0;
        }
      }

      out.shape = newShape;
      out.run = newRun;
      out.periodUs = periodFromRatio(g_transport.beatPeriodUs, out.ratio);
      refreshAsrTimingLocked(out);

      if (newShape == OUTPUT_SHAPE_TRIG) {
        out.asrOneShotActive = false;
        out.gateHigh = false;
        gpio_put(out.pin, 0);

        if (newRun == OUTPUT_RUN_LOOP) {
          out.nextRiseUs = alignToGlobalPhaseGridAtOrAfterLocked(applyUs, out.periodUs);
          out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
          out.pendingTriggerCount = 0;
        }

        channelAffectsGateSchedule = true;
      } else {
        if (newRun == OUTPUT_RUN_LOOP) {
          out.lfoAnchorUs = g_transport.anchorUs;
          out.asrOneShotActive = false;
          out.pendingTriggerCount = 0;
        } else {
          out.asrOneShotActive = false;
          pwm_set_gpio_level(out.pin, 0);
          out.lastPwmLevel = 0;

          if (out.pendingTriggerCount > 0) {
            out.asrOneShotActive = true;
            out.asrOneShotStartUs = applyUs;
            out.asrOneShotDurationUs = out.periodUs;
            out.asrOneShotAttackUs = out.asrAttackUs;
            out.asrOneShotSustainUs = out.asrSustainUs;
            out.asrOneShotReleaseUs = out.asrReleaseUs;
            out.pendingTriggerCount--;
          }
        }

        if (oldShape == OUTPUT_SHAPE_TRIG) {
          channelAffectsGateSchedule = true;
        }
      }

      out.behaviorPending = false;
    }

    restore_interrupts(irqState);
    if (!channelAffectsGateSchedule) {
      continue;
    }
    gateScheduleChanged = true;
  }

  if (gateScheduleChanged) {
    const uint32_t irqState = save_and_disable_interrupts();
    rescheduleGateAlarmLocked();
    restore_interrupts(irqState);
  }
}

static void ensureGateSchedulerRunning() {
  const uint32_t irqState = save_and_disable_interrupts();
  if (g_gateAlarmId >= 0) {
    restore_interrupts(irqState);
    return;
  }

  bool hasTrigOutput = false;
  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    if (g_outputs[i].shape == OUTPUT_SHAPE_TRIG) {
      hasTrigOutput = true;
      break;
    }
  }

  if (hasTrigOutput) {
    scheduleNextAlarmLocked();
  }
  restore_interrupts(irqState);
}

static void processI2cActivityLed(uint32_t nowMs) {
  if (!ENABLE_I2C_ACTIVITY_LED) {
    return;
  }

  static bool pulseActive = false;
  static uint32_t pulseEndMs = 0;

  if (pulseActive) {
    if ((int32_t)(nowMs - pulseEndMs) >= 0) {
      pulseActive = false;
      debugLedSet(false);
    }
    return;
  }

  if (g_i2cLedPulsePending == 0) {
    return;
  }

  bool startPulse = false;
  const uint32_t irqState = save_and_disable_interrupts();
  if (g_i2cLedPulsePending > 0) {
    g_i2cLedPulsePending--;
    startPulse = true;
  }
  restore_interrupts(irqState);

  if (startPulse) {
    pulseActive = true;
    pulseEndMs = nowMs + I2C_ACTIVITY_PULSE_MS;
    debugLedSet(true);
  }
}

static bool tryInitI2cBpmReceiver() {
  if (g_i2cEnabled) {
    return true;
  }

  g_i2cBus.setSDA(I2C_SDA_PIN);
  g_i2cBus.setSCL(I2C_SCL_PIN);
  g_i2cBus.begin(I2C_ADDRESS);
  g_i2cBus.onReceive(i2cReceiveHandler);
  g_i2cBus.onRequest(i2cRequestHandler);
  configureIrqPriorities();
  g_i2cEnabled = true;
  return true;
}

static void configureIrqPriorities() {
  // Lower numeric value is higher IRQ priority on RP2040.
  // Set both I2C IRQ lines high; Wire on XIAO RP2040 uses I2C0.
  irq_set_priority(I2C0_IRQ, I2C_IRQ_PRIORITY);
  irq_set_priority(I2C1_IRQ, I2C_IRQ_PRIORITY);

  // Pico-time default alarm pool uses hardware alarm 3 by default (TIMER_IRQ_3).
  // Keep timer lower than I2C to reduce truncated I2C frames under load.
  irq_set_priority(TIMER_IRQ_3, TIMER_IRQ_PRIORITY);
}

static int64_t gateAlarmCallback(alarm_id_t id, void *user_data) {
  (void)id;
  (void)user_data;

  g_gateSchedulerRuns++;
  const uint64_t nowUs = time_us_64();

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_outputs[i];
    if (out.shape != OUTPUT_SHAPE_TRIG) {
      continue;
    }

    // Config commits are handled in loop(); once the transition point is due,
    // stop producing trig edges until the commit completes.
    if (out.behaviorPending && out.pendingShape == OUTPUT_SHAPE_ASR &&
        (int64_t)(nowUs - out.behaviorPendingApplyUs) >= 0) {
      if (out.gateHigh) {
        gpio_put(out.pin, 0);
        out.gateHigh = false;
        out.gateFalls++;
      }
      continue;
    }

    if (!out.gateHigh && out.run == OUTPUT_RUN_ONE_SHOT && out.pendingTriggerCount > 0) {
      gpio_put(out.pin, 1);
      out.gateHigh = true;
      out.gateRises++;
      out.nextFallUs = nowUs + GATE_PULSE_US;
      out.pendingTriggerCount--;
    }

    if (out.gateHigh && (int64_t)(nowUs - out.nextFallUs) >= 0) {
      gpio_put(out.pin, 0);
      out.gateHigh = false;
      out.gateFalls++;

      if (out.run == OUTPUT_RUN_LOOP) {
        do {
          out.nextRiseUs += out.periodUs;
          if ((int64_t)(nowUs - out.nextRiseUs) >= 0) {
            g_gateSchedulerMisses++;
          }
        } while ((int64_t)(nowUs - out.nextRiseUs) >= 0);

        out.nextFallUs = out.nextRiseUs + GATE_PULSE_US;
      }
    }

    if (out.run == OUTPUT_RUN_LOOP && !out.gateHigh && (int64_t)(nowUs - out.nextRiseUs) >= 0) {
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
    if (out.shape != OUTPUT_SHAPE_ASR) {
      continue;
    }

    // Config commits are handled in loop(); once the transition point is due,
    // keep ASR output at zero until the commit completes.
    if (out.behaviorPending && out.pendingShape == OUTPUT_SHAPE_TRIG &&
        (int64_t)(nowUs - out.behaviorPendingApplyUs) >= 0) {
      pwm_set_gpio_level(out.pin, 0);
      out.lastPwmLevel = 0;
      continue;
    }

    if (out.run == OUTPUT_RUN_ONE_SHOT && out.pendingTriggerCount > 0) {
      out.asrOneShotActive = true;
      out.asrOneShotStartUs = nowUs;
      out.asrOneShotDurationUs = out.periodUs;
      out.asrOneShotAttackUs = out.asrAttackUs;
      out.asrOneShotSustainUs = out.asrSustainUs;
      out.asrOneShotReleaseUs = out.asrReleaseUs;
      out.pendingTriggerCount--;
    }

    uint16_t level = 0;
    if (out.run == OUTPUT_RUN_LOOP) {
      level = asrLevelFromTime(nowUs,
                               out.lfoAnchorUs,
                               out.periodUs,
                               out.asrAttackUs,
                               out.asrSustainUs,
                               out.asrReleaseUs,
                               true);
    } else if (out.asrOneShotActive) {
      level = asrLevelFromTime(nowUs,
                               out.asrOneShotStartUs,
                               out.asrOneShotDurationUs,
                               out.asrOneShotAttackUs,
                               out.asrOneShotSustainUs,
                               out.asrOneShotReleaseUs,
                               false);
      if ((int64_t)(nowUs - (out.asrOneShotStartUs + out.asrOneShotDurationUs)) >= 0) {
        out.asrOneShotActive = false;
      }
    }

    pwm_set_gpio_level(out.pin, level);
    out.lastPwmLevel = level;
    out.lfoUpdates++;
  }

  return true;
}

void setup() {
  debugLedInit();
  debugLedSet(false);

  for (uint8_t i = 0; i < NUM_OUTPUTS; ++i) {
    OutputState &out = g_outputs[i];
    out.pin = OUTPUT_PINS[i];
    out.ratio = DEFAULT_RATIOS[i];
    out.shape = DEFAULT_SHAPES[i];
    out.run = DEFAULT_RUNS[i];

    out.periodUs = 0;
    out.lfoAnchorUs = 0;
    out.asrSus = 0;
    out.asrSkew = 5;
    out.asrA = 1;
    out.asrS = 0;
    out.asrR = 1;
    out.ratioPending = false;
    out.pendingApplyUs = 0;
    out.pendingRatio = {1, 1};
    out.behaviorPending = false;
    out.behaviorPendingApplyUs = 0;
    out.pendingShape = out.shape;
    out.pendingRun = out.run;
    out.pendingTriggerCount = 0;
    out.asrOneShotActive = false;
    out.asrOneShotStartUs = 0;
    out.asrOneShotDurationUs = 0;
    out.asrAttackUs = 0;
    out.asrSustainUs = 0;
    out.asrReleaseUs = 0;
    out.asrOneShotAttackUs = 0;
    out.asrOneShotSustainUs = 0;
    out.asrOneShotReleaseUs = 0;
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

  configureOutputPinsForModes();

  g_pwmTimerRunning =
      add_repeating_timer_us(-PWM_SAMPLE_PERIOD_US, pwmSampleCallback, nullptr, &g_pwmTimer);

  applyBpmAndReset(DEFAULT_BPM);

  if (ENABLE_I2C_BPM) {
    // Prepare bus pins as pulled-up inputs; Wire init is retried from loop if
    // the bus is not idle yet.
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    if (digitalRead(I2C_SDA_PIN) == HIGH && digitalRead(I2C_SCL_PIN) == HIGH) {
      (void)tryInitI2cBpmReceiver();
    }
  }
}

void loop() {
  static uint32_t lastLedToggleMs = 0;
  static uint32_t lastI2cRetryMs = 0;
  static uint32_t errorLedUntilMs = 0;
  static bool ledOn = false;
  static uint32_t lastErrorCount = 0;
  static uint32_t lastDropCount = 0;
  static uint32_t lastDropTrigCount = 0;
  static uint32_t lastSatCount = 0;

  const uint32_t now = millis();
  if (ENABLE_I2C_BPM && !g_i2cEnabled && (now - lastI2cRetryMs >= I2C_RETRY_MS)) {
    lastI2cRetryMs = now;
    const bool sdaHigh = (digitalRead(I2C_SDA_PIN) == HIGH);
    const bool sclHigh = (digitalRead(I2C_SCL_PIN) == HIGH);
    if (sdaHigh && sclHigh) {
      (void)tryInitI2cBpmReceiver();
    }
  }

  processDueConfigChanges();
  ensureGateSchedulerRunning();

  if (ENABLE_I2C_BPM) {
    processI2cEvents();
    processDueConfigChanges();
    ensureGateSchedulerRunning();

    uint32_t err = 0;
    uint32_t dropAll = 0;
    uint32_t dropTrig = 0;
    uint32_t sat = 0;
    const uint32_t irqState = save_and_disable_interrupts();
    err = g_i2cErrorCount;
    dropAll = g_i2cEventQueueDropCount;
    dropTrig = g_i2cEventQueueDropTriggerCount;
    sat = g_i2cTriggerSaturatedCount;
    restore_interrupts(irqState);

    if (err != lastErrorCount || dropAll != lastDropCount || dropTrig != lastDropTrigCount ||
        sat != lastSatCount) {
      errorLedUntilMs = now + I2C_ERROR_LATCH_MS;
      lastErrorCount = err;
      lastDropCount = dropAll;
      lastDropTrigCount = dropTrig;
      lastSatCount = sat;
    }
  }

  if ((int32_t)(now - errorLedUntilMs) < 0) {
    debugLedSet(true);
  } else if (ENABLE_I2C_ACTIVITY_LED) {
    processI2cActivityLed(now);
  } else if (now - lastLedToggleMs >= HEARTBEAT_MS) {
    lastLedToggleMs = now;
    ledOn = !ledOn;
    debugLedSet(ledOn);
  }
}
