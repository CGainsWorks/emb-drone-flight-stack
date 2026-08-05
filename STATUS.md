# Project status

Last updated: 5 August 2026

This file records what actually exists in this repository, so that nothing in the
README or on a CV overstates it. Update it as items move between sections.

## Complete

**Flight controller PCB, `FCSTMV1.05`**

- 6-layer board: `F_Cu`, `In1_Cu`, `In2_Cu`, `In3_Cu`, `In4_Cu`, `B_Cu`
- Schematic across 7 sheets, top level plus `Part2` to `Part7`
- Controlled-impedance stackup specified, see
  `hardware/flight-controller/docs/Required_impedance_control-10.18.xlsx`
- Full Gerber set with PTH and NPTH drill files, revision 10.19
- Assembly position files for top and bottom
- BOM and interactive BOM

**Supporting subsystem design, schematic level**

- ESC, two parts, `hardware/esc/docs/`
- Control link RX and TX, `hardware/rf/docs/`
- Video link RX and TX, `hardware/rf/docs/`

## Not started

**Firmware.** No code exists yet. Nothing in `firmware/` beyond the directory
structure. This is the largest gap in the project and the next thing to build.
Required at minimum:

- IMU driver over SPI
- Attitude estimator
- PID attitude loop
- PWM or DShot ESC output
- UART telemetry
- Arming and failsafe state machine

**Board bring-up.** The boards have not been assembled or powered. No bring-up
has been performed, so nothing about the design is yet validated in hardware.

**Test results.** `test-results/` is empty. No step responses, no scope captures,
no thermal measurements, no efficiency data.

**Flight.** The aircraft has never flown. There are no flight logs.

**Power distribution board.** Referenced in the README as part of the intended
scope. Not designed.

## Honest one-line summary

Use this wording, or something equally accurate, wherever the project is described:

> A 6-layer impedance-controlled quadrotor flight controller taken from schematic
> through to fabrication-ready Gerbers and assembly files, with ESC and RF link
> subsystems designed at schematic level. Firmware and hardware bring-up are in
> progress.

Do not describe the aircraft as built, tested or flown until the corresponding
sections above have moved to Complete.
