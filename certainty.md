# certainty clock i2c

Low-jitter primary clock firmware for Uncertainty using two interrupts:

- Gate scheduler alarm interrupt for precise clock edges.
- Fixed-rate PWM sample interrupt for clock-synced triangle LFO duty updates.

## Defaults

- BPM: `90`
- I2C address: `0x55`
- CV input ignored.

Default output layout:

- OUT1: pulse clock at `/2`
- OUT2: triangle PWM LFO at `/2`
- OUT3: pulse clock at `1x`
- OUT4: triangle PWM LFO at `1x`
- OUT5: pulse clock at `2x`
- OUT6: triangle PWM LFO at `2x`
- OUT7: pulse clock at `4x`
- OUT8: triangle PWM LFO at `4x`

When BPM changes, transport anchor and phase are reset across all gate and LFO outputs.

## Timing model

- Shared transport anchor (`time_us_64`) + BPM.
- Gate ISR schedules edge events by absolute time.
- PWM ISR runs at fixed `4000 Hz`, computing LFO phase from transport.

## Serial commands

- `bpm <20-320>`
- `ratio <1-8> <num> <den>` (each ratio term `1..64`)
- `reset`
- `status`
- `help`

## I2C protocol

16-bit values are big-endian.

Address/pins:

- address: `0x55`
- SDA: pin `6`
- SCL: pin `7`

Command byte format:

- `command = type * 8 + output`
- `type = 0` -> set BPM, payload: `[bpm_u16]` (output ignored)
- `type = 1` -> set ratio for one output, payload: `[num_u16, den_u16]`
- `type = 2` -> transport reset, no payload (output ignored)

Read request (`onRequest`) returns current BPM as 2 bytes big-endian.

Examples (raw bytes):

- set BPM 120:
  - command `0x00`
  - payload `0x00 0x78`
- set OUT4 ratio to `3/2`:
  - output index is zero-based, so OUT4 => `3`
  - command = `type1*8 + 3 = 0x0B`
  - payload `0x00 0x03 0x00 0x02`
- reset transport:
  - command `0x10` (type2, output 0)
  - no payload

Minimal leader example (Arduino):

```cpp
Wire.beginTransmission(0x55);
Wire.write((uint8_t)0x00); // type0 bpm
Wire.write((uint8_t)0x00);
Wire.write((uint8_t)120);
Wire.endTransmission();
```

## Notes

Mode changes per output via I2C are intentionally not exposed yet.
