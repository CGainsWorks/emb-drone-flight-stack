*** Comments ***
Runs the real fc_firmware.elf on Renode's STM32F4 platform (pinned in CI).
The IMU is the simulated ICM-42688-P in Icm42688BitBang.cs, driven by the
firmware's own bit-banged SPI master over emulated GPIO. Everything observed
here is emulated: no timing reported by the firmware under Renode says
anything about real silicon.

*** Settings ***
Library             OperatingSystem
Library             crsf_capture.py

*** Variables ***
${ELF}              ${CURDIR}/../../build-arm/fc_firmware.elf
${CAPTURE_DIR}      ${CURDIR}/../../build-arm/renode
${UART}             sysbus.uart5
${IMU}              sysbus.gpioPortA.imu
# Renode's default is 100 MIPS. The firmware spends most of its time polling
# TIM2, so a lower figure shortens the run without changing what is tested.
${MIPS}             25

*** Keywords ***
Create Flight Controller
    [Arguments]    ${capture}
    Execute Command    include @${CURDIR}/Icm42688BitBang.cs
    Execute Command    mach create "fcstmv105"
    Execute Command    machine LoadPlatformDescription @${CURDIR}/fcstmv105.repl
    Execute Command    cpu PerformanceInMips ${MIPS}
    Execute Command    sysbus LoadELF @${ELF}
    Create Directory    ${CAPTURE_DIR}
    Execute Command    ${UART} CreateFileBackend @${CAPTURE_DIR}/${capture} true
    Create Terminal Tester    ${UART}    defaultPauseEmulation=True

Imu Counter Should Be
    [Arguments]    ${property}    ${expected}
    ${value}=    Execute Command    ${IMU} ${property}
    Should Be Equal As Integers    ${value}    ${expected}

Run For Seconds
    [Arguments]    ${seconds}
    Execute Command    emulation RunFor "${seconds}"

*** Test Cases ***
Firmware Boots And Brings Up The IMU Over Bit-Banged SPI
    Create Flight Controller    boot.bin
    Wait For Line On Uart    === FCSTMV1.05 boot ===    timeout=5
    Wait For Line On Uart    IMU: ok    timeout=5
    Wait For Line On Uart    SPI burst    timeout=5
    Wait For Line On Uart    Loop budget:\\s+1000 us    timeout=5    treatAsRegex=true
    Wait For Line On Uart    === running ===    timeout=5
    # One soft reset from Icm42688::init and every transaction framed on
    # whole bytes with SCK parked high before CS fell.
    Imu Counter Should Be    SoftResets    1
    Imu Counter Should Be    ProtocolErrors    0

Control Loop Runs On The Emulated MCU
    Create Flight Controller    loop.bin
    Wait For Line On Uart    === running ===    timeout=5
    ${before}=    Execute Command    ${IMU} BurstReads
    # The loop prints its timing summary every 5000 iterations while disarmed.
    Wait For Line On Uart    loop us\\s+min=    timeout=10    treatAsRegex=true
    Wait For Line On Uart    overruns=    timeout=2
    ${after}=    Execute Command    ${IMU} BurstReads
    ${reads}=    Evaluate    int(${after}) - int(${before})
    Should Be True    ${reads} >= 5000    one IMU burst read per control cycle, got ${reads}

Estimator Follows A Simulated Tilt
    Create Flight Controller    tilt.bin
    Wait For Line On Uart    === running ===    timeout=5
    Run For Seconds    0.5
    ${level}=    Latest Roll Degrees    ${CAPTURE_DIR}/tilt.bin
    Should Be True    abs(${level}) < 0.5    level airframe should report ~0 deg roll, got ${level}
    # 10 deg right side down: specific force (0, -sin, -cos) g in the body frame.
    Execute Command    ${IMU} SetAccelG 0 -0.17364818 -0.98480775
    Run For Seconds    4
    ${roll}=    Latest Roll Degrees    ${CAPTURE_DIR}/tilt.bin
    Should Be True    abs(${roll} - 10) < 1    roll should converge on 10 deg, got ${roll}

Firmware Refuses To Run Without A Recognised IMU
    Create Flight Controller    no_imu.bin
    Execute Command    ${IMU} SetWhoAmI 0x00
    Wait For Line On Uart    === FCSTMV1.05 boot ===    timeout=5
    Wait For Line On Uart    IMU: NO RESPONSE    timeout=5
    Should Not Be On Uart    === running ===    timeout=1
