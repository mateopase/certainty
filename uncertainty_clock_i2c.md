# uncertainty_clock_i2c (phase 1)

Stable internal clock firmware for Uncertainty.

## Behavior

- CV input is ignored.
- Output gates emit fixed trigger pulses.
- Default BPM is `90`.
- Default output rates are:
  - OUT1: `1x`
  - OUT2: `2x`
  - OUT3: `4x`
  - OUT4: `8x`
  - OUT5: `16x`
  - OUT6: `32x`
  - OUT7: `64x`
  - OUT8: `128x`

## Hardware mapping

- Gates: `27, 28, 29, 0, 3, 4, 2, 1`
- I2C SDA: `6`
- I2C SCL: `7`
- I2C address: `0x55`

## I2C commands (phase 1)

All values are binary. 16-bit values are big-endian.

- `0x01` (`SET_BPM`): set BPM with one 16-bit value
- `0x81` (`GET_BPM`): request BPM query, then read 16-bit BPM in `onRequest`

BPM is clamped to `1..300`.

## Teletype generic I2C examples

Set address:

```text
IIA 0x55
```

Set BPM to 120:

```text
IIS1 1 120
```

Query BPM (word):

```text
IIQ 0x81
```

## Notes

This firmware intentionally implements only phase 1.
Phase 2 (per-output multiplier/divider control) and phase 3 (clock-synced LFO modes)
are not implemented in this version.
