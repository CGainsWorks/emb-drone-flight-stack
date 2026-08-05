# emb-drone-flight-stack

> **Status: hardware design complete. Firmware written and tested on host, but
> never run on hardware.**
> The 6-layer flight controller is designed through to fabrication-ready Gerbers
> and assembly files. The firmware compiles for the STM32F405 and passes 221
> host-side unit tests, but the boards have not been assembled, powered, brought
> up or flown, so not one line of it has executed on silicon. There are no
> measurements of any kind in this repository. See [STATUS.md](STATUS.md) for the
> precise breakdown of what does and does not exist.

Design files imported from [`OpenSourceDrone`](https://github.com/CGainsWorks/OpenSourceDrone),
which remains the original working repository.

A complete quadrotor built from nothing: a custom flight-controller PCB integrating the power stage with an STM32, a separate multi-layer power distribution board, and C++ firmware written from scratch with a PID attitude loop, IMU drivers, PWM and DShot ESC output and UART telemetry. This is the monorepo covering all three drone hardware and firmware entries on the CV.

### Firmware

| | |
|---|---|
| [`firmware/README.md`](firmware/README.md) | Architecture, how to build and run the tests, and what is deliberately not implemented |
| [`firmware/docs/PINOUT.md`](firmware/docs/PINOUT.md) | The pinout as extracted from the PCB netlist, including why the IMU cannot use hardware SPI1 on this board |
| [`firmware/docs/BRING_UP.md`](firmware/docs/BRING_UP.md) | Bring-up procedure with expected values. Not yet performed |

```bash
cd firmware && ./run_tests.sh        # host unit tests, no hardware needed
cd firmware && ./build_firmware.sh   # STM32F405 image, needs arm-none-eabi-gcc
```

---

## What this covers

CV entries this repository is the evidence for:

- Open-Source STM32 Quadrotor
- Drone Flight-Controller PCB, power and control
- Drone Power Distribution Board

---

## Requirements

### Hardware

- Custom flight-controller PCB: STM32 or ARM Cortex-M, IMU on SPI, barometer, GPS on UART, PWM/DShot ESC outputs, telemetry
- Power distribution board: high-current ESC rails, battery voltage and current sensing, 5V/3.3V SMPS and LDO stages, XT60 input with reverse-polarity and inrush protection
- Frame, motors, ESCs, LiPo battery, receiver, telemetry radio
- Bench test rig with propeller guard
- Test gear: oscilloscope, electronic load, thermal measurement

### Software

- KiCad and/or Altium projects
- C++ firmware from scratch (no Betaflight fork)
- PID attitude loop
- IMU driver over I2C and SPI with sensor fusion
- PWM and DShot ESC output
- UART telemetry
- Ground-station or serial-plotting tool
- Host-side unit tests for the control maths

### Report and documentation deliverables

- Schematic and layout PDFs for both boards
- Controlled-impedance stackup definition
- EMC-aware layout rationale
- BOM with alternates for both boards
- Copper pour and current-density analysis for the PDB
- Thermal analysis with measured rise
- Efficiency curves
- Protection circuit validation
- PID tuning methodology with step-response plots
- Loop timing analysis
- Bring-up and bench-test procedure
- Safety notes for propeller testing
- Flight video

---

## Inputs

- Design requirements and mechanical constraints
- Sensor datasheets
- MCU pin-mux plan
- Stackup and impedance targets
- Battery voltage and current envelope, ESC current requirements
- IMU accelerometer and gyroscope readings
- Receiver channel inputs
- PID gains and calibration offsets

## Outputs

- Schematic and PCB source files, Gerbers, drill files, pick-and-place, BOM, 3D models
- Regulated rails with measured efficiency and thermal data
- Voltage and current telemetry signals
- Firmware binary
- Motor PWM or DShot commands
- Estimated attitude in roll, pitch and yaw
- UART telemetry stream
- Logged flight data
- Fault and arming state
- Bring-up test reports

---

## Intended structure

```
hardware/
  flight-controller/
    kicad/                fc.kicad_pro, .kicad_sch, .kicad_pcb
    libraries/
    outputs/              gerbers, drill, bom.csv, pick_and_place.csv,
                          schematic.pdf, 3d/
    docs/                 stackup.md, design_notes.md, bring_up_checklist.md
  pdb/                    same layout
firmware/
  src/
    main.cpp
    control/              pid.cpp, attitude_estimator.cpp
    drivers/              imu_spi.cpp, esc_pwm.cpp, uart_telemetry.cpp
    config/
  include/
  test/                   host-side unit tests for control maths
  docs/timing_budget.md
tools/                    telemetry_plotter.py, pid_tuner.py
test-results/             scope-captures, step_response.csv, thermal/
docs/                     architecture.md, safety.md
```

Plus the standard baseline: `README.md`, `LICENSE`, `.gitignore`, `.gitattributes`
(Git LFS rules for weights, bags and media), `.github/workflows/ci.yml`, `docs/`,
`scripts/setup.sh`, `scripts/run_demo.sh` and `tests/`.

---

## Build checklist

Anything requiring a powered board is unchecked, because no board has been
powered.

- [ ] PDB schematic and layout
- [ ] PDB fabrication and bench validation: efficiency, thermal, protection
- [x] Flight controller schematic with MCU pin-mux plan
- [x] Flight controller layout with a defined stackup
- [ ] Fabrication and bring-up checklist executed — procedure written
      ([firmware/docs/BRING_UP.md](firmware/docs/BRING_UP.md)), not performed
- [ ] IMU driver over SPI with verified data — driver written and tested against
      a simulated device; no data from real silicon
- [x] Attitude estimator with host-side unit tests
- [ ] PID attitude loop with step-response tuning on a test rig — loop written
      and host-tested; gains are untuned defaults that have never driven a motor
- [ ] PWM and DShot ESC output — PWM implemented; DShot frame encoding
      implemented and tested, but its timer DMA is not wired up
- [ ] UART telemetry and ground-station plotting — telemetry implemented; no
      ground-station tool yet
- [ ] Loop timing budget measured — the firmware measures and reports its own
      loop timing, but nothing has run to measure
- [ ] Tethered hover
- [ ] Free flight and logged flight data

---

## Definition of done

This repository is finished when:

- [ ] The whole thing runs from one command on a clean machine.
- [ ] The README opens with a diagram or a GIF, not a wall of text.
- [ ] Every number quoted in the README is reproducible from a script in the repo.
- [ ] Limitations are stated honestly in their own section.
- [ ] A licence is declared.
- [ ] Large binaries live in Releases or LFS, not in the Git history.
- [ ] The repository URL is added to the matching CV bullet point.

---

## Starter prompt

Paste this into your coding agent of choice to begin.

<details>
<summary>Expand</summary>

```
You are helping me build a portfolio project from scratch. Read this brief and then
propose a concrete implementation plan before writing any code.

PROJECT: emb-drone-flight-stack
A complete quadrotor built from nothing: a custom flight-controller PCB integrating the power stage with an STM32, a separate multi-layer power distribution board, and C++ firmware written from scratch with a PID attitude loop, IMU drivers, PWM and DShot ESC output and UART telemetry. This is the monorepo covering all three drone hardware and firmware entries on the CV.

TARGET STACK
- KiCad and/or Altium projects
- C++ firmware from scratch (no Betaflight fork)
- PID attitude loop
- IMU driver over I2C and SPI with sensor fusion
- PWM and DShot ESC output
- UART telemetry
- Ground-station or serial-plotting tool
- Host-side unit tests for the control maths

HARDWARE CONTEXT
- Custom flight-controller PCB: STM32 or ARM Cortex-M, IMU on SPI, barometer, GPS on UART, PWM/DShot ESC outputs, telemetry
- Power distribution board: high-current ESC rails, battery voltage and current sensing, 5V/3.3V SMPS and LDO stages, XT60 input with reverse-polarity and inrush protection
- Frame, motors, ESCs, LiPo battery, receiver, telemetry radio
- Bench test rig with propeller guard
- Test gear: oscilloscope, electronic load, thermal measurement

THE SYSTEM MUST TAKE THESE INPUTS
- Design requirements and mechanical constraints
- Sensor datasheets
- MCU pin-mux plan
- Stackup and impedance targets
- Battery voltage and current envelope, ESC current requirements
- IMU accelerometer and gyroscope readings
- Receiver channel inputs
- PID gains and calibration offsets

AND PRODUCE THESE OUTPUTS
- Schematic and PCB source files, Gerbers, drill files, pick-and-place, BOM, 3D models
- Regulated rails with measured efficiency and thermal data
- Voltage and current telemetry signals
- Firmware binary
- Motor PWM or DShot commands
- Estimated attitude in roll, pitch and yaw
- UART telemetry stream
- Logged flight data
- Fault and arming state
- Bring-up test reports

INTENDED REPOSITORY LAYOUT
hardware/
  flight-controller/
    kicad/                fc.kicad_pro, .kicad_sch, .kicad_pcb
    libraries/
    outputs/              gerbers, drill, bom.csv, pick_and_place.csv,
                          schematic.pdf, 3d/
    docs/                 stackup.md, design_notes.md, bring_up_checklist.md
  pdb/                    same layout
firmware/
  src/
    main.cpp
    control/              pid.cpp, attitude_estimator.cpp
    drivers/              imu_spi.cpp, esc_pwm.cpp, uart_telemetry.cpp
    config/
  include/
  test/                   host-side unit tests for control maths
  docs/timing_budget.md
tools/                    telemetry_plotter.py, pid_tuner.py
test-results/             scope-captures, step_response.csv, thermal/
docs/                     architecture.md, safety.md

CONSTRAINTS
- Everything must run from a single command: `docker compose up` or `make demo`.
- No placeholder or stub code presented as working. If something is not implemented, say so.
- Every quantitative claim in the README must be reproducible from a script in this repo.
- Use only public or synthetic data. Record the licence of every dataset used.


START BY
1. Confirming or challenging the proposed repository layout.
2. Listing the specific libraries and versions you intend to pin.
3. Identifying the single riskiest technical unknown and how to de-risk it first.
Then work through the milestones in the README build checklist, one at a time.

```

</details>

---

## Notes

- Do not backdate commits to match CV dates. Publishing older work today is normal and expected.
- Record dataset licences before training on anything. Several common research datasets are
  non-commercial only.
- Flip this repository to public once there is something in it worth showing.
