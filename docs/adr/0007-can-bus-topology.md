# ADR-0007: CAN Bus Topology & Termination

**Status:** Proposed
**Date:** 2026-04-19

## Context

ADR-0003 specifies the CAN protocol and bit rate but leaves the
physical bus layout unspecified. The production system has three
nodes on a single CAN 2.0B bus at 500 kbit/s:

- **Waveshare ESP32-S3-RS485-CAN** — the diff-drive controller.
- **Flipsky Mini FSESC6.7 Pro** (left wheel).
- **Flipsky Mini FSESC6.7 Pro** (right wheel, identical part).

ISO 11898-2 requires a **linear** topology with exactly **two 120 Ω
terminators**, one at each physical end of the trunk. The total
termination seen across CANH/CANL must be ~60 Ω.

Termination presence on each node:

| Node                             | Termination    | Controllable?                  |
|----------------------------------|----------------|--------------------------------|
| Waveshare ESP32-S3-RS485-CAN     | 120 Ω          | Jumper (install / remove)      |
| FSESC6.7 Pro                     | 120 Ω          | Not user-accessible (SMD)      |

Since the FSESC termination is not user-serviceable without board
rework, the topology must be chosen so that **both VESCs sit at the
ends** of the trunk and the controller sits in the middle with its
termination disabled.

## Decision

### Physical layout

```
[FSESC #1 (end)] ── JST CAN ── [Waveshare ESP32-S3 (mid)] ── JST CAN ── [FSESC #2 (end)]
   120 Ω ON                        120 Ω OFF                              120 Ω ON
```

### Termination rules

| Node                             | Termination  |
|----------------------------------|--------------|
| Left FSESC6.7 Pro (bus end)      | Enabled (default, unmodified) |
| Right FSESC6.7 Pro (bus end)     | Enabled (default, unmodified) |
| Waveshare ESP32-S3 (bus mid)     | **Jumper removed** |

Resulting bus impedance: 120 Ω ∥ 120 Ω = **60 Ω** across CANH/CANL.

### Verification

Before applying power, measure resistance between CANH and CANL at
any JST connector on the trunk:

| Reading | Meaning                                       |
|---------|-----------------------------------------------|
| ~60 Ω   | Correct — two terminators                     |
| ~40 Ω   | Three terminators (Waveshare jumper still in) |
| ~120 Ω  | One terminator (one VESC missing / unplugged) |
| Open    | No termination / broken cable                 |

### Bench / development taps

When a PC-side CAN adapter (USB-CAN bridge) is needed for debug or
VESC Tool access, it replaces the Waveshare board temporarily or
taps in as a short stub. It must not add a third terminator: either
disable its internal 120 Ω or physically substitute it for one of
the end nodes during the session. Production runs do **not** include
a PC-side adapter on the bus.

## Alternatives Considered

### A. Desolder termination on one FSESC (place VESC in the middle)
Would allow a `[Waveshare] ── [VESC] ── [VESC]` order by removing the
SMD termination resistor (or cutting a solder bridge) on one Flipsky
board so it can sit mid-bus. Electrically valid and spec-compliant.
Rejected on serviceability grounds:

- The modified VESC becomes a non-interchangeable "mid-bus only"
  unit. Field swaps that put it at an end — or drop an unmodified
  spare into the middle — silently break the bus.
- Adds a manufacturing / rework step per VESC.

The Waveshare ESP32-S3-RS485-CAN exposes a user-accessible
termination jumper designed exactly for this case. Using it keeps
both VESCs factory-stock and fully interchangeable.

## Consequences

- The 120 Ω jumper on the Waveshare ESP32-S3-RS485-CAN board must be
  **removed** for production. ADR-0003's mention of "120 Ω jumper on
  Waveshare board … plus termination at last VESC" is superseded by
  this ADR.
- Both FSESCs are interchangeable. Either can occupy either end of
  the trunk; no unit-specific hardware state.
- Bring-up checklist gains a multimeter step: confirm ~60 Ω across
  CANH/CANL with power off before first power-on.
- Adding a third CAN node later (e.g. a second sensor MCU) requires
  it to sit in the middle of the trunk with no termination, or one
  of the end VESCs must be moved inward and its termination dealt
  with — not a drop-in extension.
- PC-side CAN adapters used for debug must have their termination
  disabled or substitute for an end node; they must not be added as
  a third terminator.
