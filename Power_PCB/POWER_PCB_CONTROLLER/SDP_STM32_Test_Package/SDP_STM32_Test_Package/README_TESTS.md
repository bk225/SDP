# SDP STM32 motor + servo: simulation and hardware test package

## What this package is based on

- The most recent `main.c` you pasted (Oct. 28), with five servos and a **14-byte** UART command packet.
- Your existing modular `firmware_tests.c/.h` motor simulation framework and `firmware_types.h`.
- Your actual `tim.c` (TIM1/TIM2 20 kHz; TIM3/TIM5 50 Hz at a 100 MHz timer clock).
- This project expects the **existing CubeMX-generated** files `tim.c`, `tim.h`, `gpio.c`, `gpio.h`, `usart.c`, `usart.h`, `main.h`, startup code, and STM32 HAL libraries. They are NOT replaced by this package.

## Package layout

```
Core/Inc/firmware_tests.h    -- self-test/hardware-test mode switches and results
Core/Inc/firmware_types.h    -- existing Motor and SafetyState types
Core/Inc/hardware_tests.h    -- hardware test selection and strict opt-in
Core/Src/firmware_tests.c    -- simulations: motor, protocol, servo, CCR mapping
Core/Src/hardware_tests.c    -- scope, UART and one-at-a-time actuator tests
Core/Src/main.c              -- your most recent main.c, test-mode integration
uart_probe.py                -- optional host PC UART packet test
README_TESTS.md              -- this guide
main_changes.patch          -- exact changes versus a normalized version of your paste
```

**Important:** Back up your current `Core/Src/main.c` OUTSIDE the project first. Do not leave another `.c` file defining `main()` inside `Core/Src`. Use the included `main.c` only if it matches your latest working project. It is reconstructed from the most recent Markdown paste, not from a binary `.c` attachment, so compare before replacing. Alternatively, apply the integration changes shown in `main_changes.patch` to your own current source. Don't replace any CubeMX-generated `tim.c`, `gpio.c`, or `usart.c`.

## Test mode settings

Edit `Core/Inc/firmware_tests.h`:

| Goal | FIRMWARE_SELF_TEST | FIRMWARE_HARDWARE_TEST |
|---|---:|---:|
| Run simulation | 1 | 0 |
| Hardware tests | 0 | 1 |
| Normal firmware | 0 | 0 |

The compiler rejects both test modes being 1 simultaneously.

For hardware tests, edit `Core/Inc/hardware_tests.h`:

| FIRMWARE_HW_TEST_KIND | What it does | Safe connections |
|---:|---|---|
| 1 | 50 Hz servo PWM waveforms, 0 to 180 degrees equivalent | Scope only; all servos disconnected |
| 2 | 20 kHz motor PWM waveforms, reverse and forward | Scope only; motor drivers/actuators disconnected; motor enables LOW |
| 3 | Physical USART3 14-byte frame/ACK test | USB-UART 3.3 V TTL adapter; no actuators |
| 4 | One servo, 85/90/95 degree movement | Only selected servo wired; correct external power supply |
| 5 | One motor, forward-only 20%, **one** 200 ms pulse | Controlled bench ONLY, explicit compiler opt-in; see below |

Defaults are simulation enabled (`FIRMWARE_SELF_TEST=1`), hardware tests disabled (`FIRMWARE_HARDWARE_TEST=0`), and motor actuation DISALLOWED (`FIRMWARE_HW_ALLOW_MOTOR_ACTUATION=0`).

## Part A -- Run the simulated tests in STM32CubeIDE

1. **Disconnect all motors, bridge driver control cables, and servos.** Supply only the MCU from the normal safe programming power source. The tests do **not** call `HAL_TIM_PWM_Start()`. They do write timer compare registers for mapping verification and call `disableMotors()` (motor enable GPIO LOW).
2. Back up your current project. Copy the header files in `Core/Inc` and the `.c` sources in `Core/Src`. Ensure each filename exists **once** in the CubeIDE project.
3. Refresh the project in STM32CubeIDE, right-click the project and choose **Clean Project** or **Project > Clean**, then **Build Project**. Confirm the test `.c` files are not excluded from the build.
4. In `Core/Inc/firmware_tests.h`, set `FIRMWARE_SELF_TEST=1`, `FIRMWARE_HARDWARE_TEST=0`.
5. Launch **Debug** and resume execution. On boot the app enters the self-test branch, runs all test cases, and parks in a `while(1)` loop; the normal application loop and hardware PWM starts do not run.
6. Click **Suspend/Pause**. In **Window > Show View > Expressions** (or the Expressions pane), add these expressions:
   - `testComplete` (expected `true`)
   - `testChecks` (expected **254** with the supplied package)
   - `testFailures` (expected **0**)
   - `testFailureLog[0]` through `[23]` if there are failures.
7. If there is a failure, inspect its `stage`, `field`, `expected`, `actual`, and `line` members. `line` is the line number in `firmware_tests.c`. The suite records the first 24 mismatches, but counts every failure.
8. After recording results, set **both test switches to 0** and clean/rebuild before loading operational firmware.

What is simulated: `getFirmwareTick()` returns `mockTick`; test motor callbacks record PWM, enable and call order rather than driving real bridges. No PWM outputs are started. The CCR mapping checks use HAL compare-register operations; these touch timer registers on the MCU but **do not drive pins**, because PWM channels are not started.

### Simulation coverage

- Motor command -> PWM/CCR, positive/negative/neutral mapping, PWM-before-enable order.
- All motor state transitions, speed changes, 2000 ms and 1000 ms reversals, disabled and neutral commands, safety states, communication timeout, tick rollover.
- 14-byte UART header, checksum over bytes 2..12, five directional servo command fields, invalid command rejection, UART byte assembly, busy packet behavior.
- Servo commanded +1/-1 at exactly 100 ms, pan/tilt commands, five independent channels, stopping/restarting, wrong-axis commands, 0/180 degree limits, loss/recovery of communications and tick rollover.
- 1000/1500/2000 microsecond angle conversion and motor/servo hardware CCR channel mapping.

The simulation checks **firmware logic**, not voltage, load current, physical movement, motor driver response, or timer output timing under real hardware conditions.

## Part B -- Real hardware verification (in recommended order)

### B1. Servo waveform test (kind 1, no servos connected)

1. Set `FIRMWARE_SELF_TEST=0`, `FIRMWARE_HARDWARE_TEST=1` in `firmware_tests.h`.
2. Set `FIRMWARE_HW_TEST_KIND=1` in `hardware_tests.h`.
3. Ensure **all servos and all motor drivers are physically disconnected** from control signals and external actuator power. Supply STM32 and connect oscilloscope ground to STM32 GND.
4. Flash and run. Probe PA6, PA7, PB0, PB1, PA2 one by one.
5. Verify period is **20 ms** (50 Hz), signal level approximately **0 to 3.3 V**, and active-high pulses match test stage. Each stage lasts **2 seconds** and repeats. In the debugger, watch `hwTestStage` and `hwTestCycles`.

| Stage | TIM3 CH1 PA6 | TIM3 CH2 PA7 | TIM3 CH3 PB0 | TIM3 CH4 PB1 | TIM5 CH3 PA2 |
|---:|---:|---:|---:|---:|---:|
| 0 | 1500 us | 1500 us | 1500 us | 1500 us | 1500 us |
| 1 | **1000 us** | 1500 us | 1500 us | 1500 us | 1500 us |
| 2 | 1500 us | **2000 us** | 1500 us | 1500 us | 1500 us |
| 3 | 1500 us | 1500 us | **1250 us** | 1500 us | 1500 us |
| 4 | 1500 us | 1500 us | 1500 us | **1750 us** | 1500 us |
| 5 | 1500 us | 1500 us | 1500 us | 1500 us | **2000 us** |

Verify widths with the scope's `Pulse Width` and `Frequency` measurement tools; do **not** infer them from CCR alone. If not 50 Hz, inspect actual APB1/TIM3/TIM5 clock frequency and CubeMX PSC/ARR. Your supplied `tim.c` has PSC=99 and ARR=19999 for both.

### B2. Motor PWM waveform test (kind 2, no motor bridges connected)

1. Keep both test switches as in B1, change `FIRMWARE_HW_TEST_KIND=2`.
2. Disconnect **motor driver input cables and actuator power**. Keep all motor enables disconnected from bridges or verify they remain LOW with DMM/scope.
3. Flash and run. Period should be **50 us** (20 kHz), 0 to 3.3 V logic, nominal 50% duty when a test stage is active. Each stage is **2 seconds**.

| Stage | Expected active motor PWM pin | Inactive companion |
|---:|---|---|
| 0 | None | All LOW |
| 1 | PA8 TIM1 CH1 (ACC forward) | PA9 LOW |
| 2 | PA10 TIM1 CH3 (BRA forward) | PA11 LOW |
| 3 | PA0 TIM2 CH1 (STE forward) | PA1 LOW |
| 4 | PA9 TIM1 CH2 (ACC reverse) | PA8 LOW |
| 5 | PA11 TIM1 CH4 (BRA reverse) | PA10 LOW |
| 6 | PA1 TIM2 CH2 (STE reverse) | PA0 LOW |

All motor enable outputs are commanded LOW for all these stages. Confirm this with a scope or DMM. Note: this test checks the PWM output pins, not physical motor rotation or the full reverse-delay algorithm; the latter is exercised by the simulation suite.

### B3. Physical UART test (kind 3)

1. Change `FIRMWARE_HW_TEST_KIND=3`. Do **not** connect motors/servos; no motor/servo PWM is started in this mode.
2. Wire a **3.3 V TTL USB-UART adapter**: adapter TX -> STM32 **PB11/USART3_RX**; adapter RX -> STM32 **PB10/USART3_TX**; adapter GND -> STM32 GND. **Do not connect the adapter's VCC pin** unless you specifically intend it to power the MCU. Do not use RS-232 voltage-level signaling.
3. Confirm the physical PB10/PB11 wiring in your board schematic/header; a USB programming/debug port does not necessarily route to USART3.
4. Flash and run (115200 baud, 8 data bits, no parity, one stop bit).
5. On the attached PC, install PySerial with `python -m pip install pyserial` and run `python uart_probe.py --port COM4` (replace COM4 with your port, e.g. `/dev/ttyUSB0` on Linux).
6. The script sends one valid frame, one bad-checksum frame, and one invalid-servo-command frame. Expected replies:
   - Valid packet: `A5 5A 01`
   - Invalid packet: `A5 5A 00`
7. Watch `hwTestUartValid` (expected 1), `hwTestUartInvalid` (expected 2), `hwTestLastFrame[0..13]`, and `hwTestError` (expected 0). If frames are sent too quickly, the single-buffer receiver may drop a frame; script spaces them by 250 ms.

This test uses the **real UART RX interrupt and real 14-byte packet parser**. It checks electrical serial communication, reception, checksum, and command validity. It **does not call `applyPacket()` or energize actuators**; no motor enables are commanded HIGH.

### B4. One physical servo motion (kind 4)

1. First finish B1. Check the actual servo's datasheet for acceptable control pulse widths and power voltage. The waveform scope test uses 1000-2000 us, but the **physical movement test only uses 85-95 degrees**, around the center pulse.
2. Fix the servo to the bench, **remove the horn or unload the linkage**, attach only the **one selected servo's signal wire**. Power the servo from a **suitably rated external supply**, usually 5-6 V if the datasheet permits, NOT from the STM32 GPIO/3.3-V regulator. Connect servo ground to STM32 ground.
3. Set `FIRMWARE_HW_TEST_KIND=4` and `FIRMWARE_HW_SERVO_INDEX=0` (servo 1). The code starts all five servo PWM outputs, so **leave the other four servo signal wires disconnected**.
4. Flash and run. Watch `hwTestStage`; it cycles between 90, 95, and 85 degrees nominal, with 2-second holds. Confirm motion is small and unobstructed. Check supply voltage and current; do not force mechanical end stops.
5. Repeat with `FIRMWARE_HW_SERVO_INDEX` 1..4, connecting one servo at a time.

### B5. One physical motor pulse (kind 5; LAST and explicit opt-in)

**WARNING: This is an actuating test.** The `updateMotor()` call in this dedicated mode is passed `authorized=true` for one short pulse, **independently of the normal SAFETY_DISARMED state**. It is NOT an operational safety test. Do NOT run this on the assembled vehicle without suitable restraint.

1. First finish simulation and motor scope tests. Confirm motor-driver wiring, enable polarity, voltage/current limits, common ground, current-limited power supply and thermal protection. Remove wheels / disconnect mechanical loads, and provide an independent power cutoff or emergency stop.
2. Select only one driver/motor, and disconnect the others. Change `FIRMWARE_HW_TEST_KIND=5`, `FIRMWARE_HW_ALLOW_MOTOR_ACTUATION=1`, and `FIRMWARE_HW_MOTOR_INDEX=0` (0 accelerator, 1 braking, 2 steering).
3. Build and flash. The mode waits **2 seconds** with motor enables LOW, commands **20% forward PWM** for **200 milliseconds**, then returns to neutral, disables the selected driver, and **stays off permanently**. Reset/reflash for a new test. It never commands reverse.
4. Record voltage, input current, actual movement, and driver enable timing on scope. Confirm current is within component limits and the motor stops when requested. If motor does not stop safely, disconnect power; do not continue.
5. Restore `FIRMWARE_HW_ALLOW_MOTOR_ACTUATION=0` before other work. With `FIRMWARE_HW_TEST_KIND=5` and this opt-in at 0, compilation intentionally fails.

This test is deliberately limited. It cannot verify mechanical travel limits, brake force, or the correctness of a physical E-stop. Those require the full test procedure and proper instrumentation.

## Expected timer configuration (from supplied tim.c)

| Timer | PSC | ARR | Expected input clock | Period | PWM frequency |
|---|---:|---:|---:|---:|---:|
| TIM1 / TIM2 | 0 | 4999 | 100 MHz | 50 us | 20 kHz |
| TIM3 / TIM5 | 99 | 19999 | 100 MHz | 20 ms | 50 Hz |

These frequencies are conditional on actual peripheral timer clocks matching 100 MHz. This is consistent with the HSI/PLL and undivided APB clock setup in your current `main.c`, but measure them with the scope.

## Understanding how the code is separated

- `firmware_tests.c` uses injected `mockTick` and mocked motor callbacks. It runs **one time** while `FIRMWARE_SELF_TEST=1`, then `main()` parks.
- `hardware_tests.c` uses the real `HAL_GetTick`, real HAL timers, real GPIO and real USART3. `startHardwareTests()` runs once at startup; `serviceHardwareTests()` runs continuously in the main loop.
- `FIRMWARE_HW_TEST_KIND=1/2/4/5` cycles through scope or motion test stages, except physical motor mode 5, which stops permanently after one pulse.
- `FIRMWARE_HW_TEST_KIND=3` handles received packets and returns a short ACK via USART3 TX. PWM remains off.
- In normal mode (both switches 0), the original motor/servo main-loop logic runs.

## Verification status of this package

- Passed **254 simulation checks, 0 failures, 0 GPIO HIGH writes** in a compiled host-side harness using mocked HAL interfaces (this number is the packaged test suite's expected count).
- Compiled `main.c`, `firmware_tests.c`, and `hardware_tests.c` against HAL interface stubs with `gcc -std=c11 -Wall -Wextra -Werror` for simulation, normal, and each of the five hardware modes.
- **Not yet verified on your actual STM32 board, in STM32CubeIDE with your real generated HAL headers, or with an oscilloscope.** Any build errors caused by mismatches in CubeMX files must be resolved against your actual project.
- The supplied main.c fixes the one known RX index upper bound from `<15` to `<14` and adds hardware test dispatch; your other firmware logic remains the same as in the pasted source.
