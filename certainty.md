# certainty clock i2c

Low-jitter clock/LFO/envelope firmware for Uncertainty.

- Gate alarm ISR handles trigger edges.
- PWM ISR (4 kHz) handles ASR output levels.
- Shared transport keeps channels phase-related.

## Defaults

- BPM: `90`
- I2C address: `0x55`
- SDA/SCL: GPIO `6/7`
- CV input ignored.

Default channel setup:

- OUT1: trig loop at `/2`
- OUT2: ASR loop at `/2`
- OUT3: trig loop at `1x`
- OUT4: ASR loop at `1x`
- OUT5: trig loop at `2x`
- OUT6: ASR loop at `2x`
- OUT7: trig loop at `4x`
- OUT8: ASR loop at `4x`

Default ASR weights on all channels: `A/S/R = 1/0/1` (triangle).

## Teletype Cheat Sheet

Set I2C target once:

```text
IIA 85
```

Set BPM:

```text
IISB1 0 120
IIS1 0 300
```

Set ratio `[cmd=1, out, num, den]`:

```text
IISB3 1 1 1 2
IISB3 1 7 4 1
```

Set behavior `[cmd=2, out, shape, run]`:

- `shape`: `0=trig`, `1=asr`
- `run`: `0=one_shot`, `1=loop`

```text
IISB3 2 3 0 1
IISB3 2 4 1 0
```

Set ASR controls `[cmd=3, out, sus, skew]`:

```text
IISB3 3 4 0 5
IISB3 3 4 8 5
IISB3 3 4 0 0
IISB3 3 4 0 10
```

- `sus`: `0..10` (0 = no sustain, 10 = max sustain)
- `skew`: `0..10` (0 = instant attack/long release, 10 = long attack/instant release)

Trigger one-shot channels:

```text
IISB1 4 4
IISB2 4 255 170
```

Mask trigger format is `[4, 255, mask]`, where bit0=OUT1 ... bit7=OUT8.

## I2C Protocol

First byte is command ID.

- `0x00`: set BPM
  - payload `[bpm8]` or `[bpm_hi, bpm_lo]`
  - valid range `20..320`
- `0x01`: set ratio
  - payload `[out, num, den]`
  - `out`: `1..8` (also accepts `0..7`)
  - `num/den`: `1..255`
- `0x02`: set behavior
  - payload `[out, shape, run]`
  - `shape`: `0=trig`, `1=asr`
  - `run`: `0=one_shot`, `1=loop`
- `0x03`: set ASR controls
  - payload `[out, sus, skew]`
  - each `0..10`
- `0x04`: trigger
  - payload `[out]` or `[255, mask]`
  - affects one-shot channels only

Read request is currently disabled (no response payload).

## Timing Semantics

- BPM changes reset transport anchor and phase for all channels.
- Ratio and behavior changes apply on the next beat boundary.
- Loop channels lock to global phase on next-beat apply.
- One-shot trigger commands are queued and consumed by the output engine (trig via gate ISR, ASR on PWM sample ticks).
- Loop channels ignore trigger commands.

## Serial Commands

- `bpm <20-320>`
- `ratio <out> <num> <den>`
- `behavior <out> <shape0|1> <run0|1>`
- `mode <out> <shape0|1> <run0|1>` (alias for `behavior`)
- `asr <out> <sus> <skew>` (`sus/skew` each `0..10`)
- `trig <out>`
- `status`
- `help`
