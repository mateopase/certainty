#include <Arduino.h>
#include <Wire.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

// Hardware mapping for Uncertainty.
static const uint8_t NUM_GATES = 8;
static const uint8_t GATE_PINS[NUM_GATES] = {27, 28, 29, 0, 3, 4, 2, 1};

// Follower I2C config.
static const uint8_t I2C_ADDRESS = 0x55;
static const uint8_t I2C_SDA_PIN = 6;
static const uint8_t I2C_SCL_PIN = 7;

// Phase-1 commands.
static const uint8_t CMD_SET_BPM = 0x01;
static const uint8_t CMD_GET_BPM = 0x81;

// Clock defaults and safety limits.
static const uint16_t DEFAULT_BPM = 90;
static const uint16_t MIN_BPM = 1;
static const uint16_t MAX_BPM = 300;

static const uint64_t MICROS_PER_MINUTE = 60000000ULL;
static const uint32_t BASE_PULSE_WIDTH_US = 1000;
static const uint32_t MIN_PULSE_WIDTH_US = 50;

struct Channel {
  uint16_t mul;
  uint16_t div;
  uint64_t phase;
  bool gateHigh;
  uint64_t gateOffUs;
};

Channel channels[NUM_GATES] = {
  {1, 1, 0, false, 0},
  {2, 1, 0, false, 0},
  {4, 1, 0, false, 0},
  {8, 1, 0, false, 0},
  {16, 1, 0, false, 0},
  {32, 1, 0, false, 0},
  {64, 1, 0, false, 0},
  {128, 1, 0, false, 0},
};

volatile uint16_t currentBpm = DEFAULT_BPM;
volatile uint8_t lastQueryCommand = 0;

uint64_t lastLoopUs = 0;

static inline uint16_t clampBpm(uint16_t bpm) {
  if (bpm < MIN_BPM) {
    return MIN_BPM;
  }
  if (bpm > MAX_BPM) {
    return MAX_BPM;
  }
  return bpm;
}

static inline void setBpm(uint16_t bpm) {
  noInterrupts();
  currentBpm = clampBpm(bpm);
  interrupts();
}

static inline uint16_t getBpm() {
  noInterrupts();
  uint16_t bpm = currentBpm;
  interrupts();
  return bpm;
}

static inline void triggerChannel(uint8_t index, uint64_t nowUs, uint32_t widthUs) {
  gpio_put(GATE_PINS[index], 1);
  channels[index].gateHigh = true;
  channels[index].gateOffUs = nowUs + widthUs;
}

void i2cReceive(int bytesCount) {
  if (bytesCount <= 0) {
    return;
  }

  const int command = Wire1.read();
  if (command < 0) {
    return;
  }

  bytesCount -= 1;

  if (command == CMD_SET_BPM && bytesCount >= 2) {
    int hi = Wire1.read();
    int lo = Wire1.read();
    bytesCount -= 2;

    if (hi >= 0 && lo >= 0) {
      const uint16_t requestedBpm = (static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo);
      setBpm(requestedBpm);
    }
  } else if (command == CMD_GET_BPM) {
    lastQueryCommand = CMD_GET_BPM;
  }

  while (bytesCount-- > 0) {
    Wire1.read();
  }
}

void i2cRequest() {
  uint16_t response = 0;
  if (lastQueryCommand == CMD_GET_BPM) {
    response = getBpm();
  }
  lastQueryCommand = 0;

  uint8_t payload[2] = {
    static_cast<uint8_t>((response >> 8) & 0xFF),
    static_cast<uint8_t>(response & 0xFF),
  };
  Wire1.write(payload, sizeof(payload));
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_GATES; ++i) {
    gpio_init(GATE_PINS[i]);
    gpio_set_dir(GATE_PINS[i], GPIO_OUT);
    gpio_put(GATE_PINS[i], 0);
  }

  Wire1.setSDA(I2C_SDA_PIN);
  Wire1.setSCL(I2C_SCL_PIN);
  Wire1.begin(I2C_ADDRESS);
  Wire1.onReceive(i2cReceive);
  Wire1.onRequest(i2cRequest);

  lastLoopUs = time_us_64();
}

void loop() {
  const uint64_t nowUs = time_us_64();
  const uint64_t deltaUs = nowUs - lastLoopUs;
  lastLoopUs = nowUs;

  // End any active trigger pulses.
  for (uint8_t i = 0; i < NUM_GATES; ++i) {
    if (channels[i].gateHigh && nowUs >= channels[i].gateOffUs) {
      gpio_put(GATE_PINS[i], 0);
      channels[i].gateHigh = false;
    }
  }

  const uint16_t bpm = getBpm();

  for (uint8_t i = 0; i < NUM_GATES; ++i) {
    Channel &ch = channels[i];

    const uint64_t rate = static_cast<uint64_t>(bpm) * static_cast<uint64_t>(ch.mul);
    const uint64_t threshold = MICROS_PER_MINUTE * static_cast<uint64_t>(ch.div);

    ch.phase += deltaUs * rate;

    while (ch.phase >= threshold) {
      ch.phase -= threshold;

      uint64_t periodUs = threshold / rate;
      if (periodUs == 0) {
        periodUs = 1;
      }

      uint64_t maxWidth = periodUs / 2;
      uint64_t pulseWidthUs = BASE_PULSE_WIDTH_US;
      if (maxWidth > 0 && pulseWidthUs > maxWidth) {
        pulseWidthUs = maxWidth;
      }
      if (pulseWidthUs < MIN_PULSE_WIDTH_US) {
        pulseWidthUs = MIN_PULSE_WIDTH_US;
      }

      triggerChannel(i, nowUs, static_cast<uint32_t>(pulseWidthUs));
    }
  }
}
