# IMU-FuzzyPID-CAN

Clone of IMU-FuzzyPID that also sends telemetry over CAN to an ESP32 (`~/ESP32Firmware/can-rx`):
an MCP2515 on SPI1 (PA5 SCK, PA6 MISO, PA7 MOSI, PB6 CS), 500 kbps, standard ID `0x101`, 8 bytes
every 50 ms. The frame layout is in `CAN_SendTelemetry()` (main.c), and the bench wiring is at the top
of `can-rx/src/main.cpp`. At boot, an MCP2515 loopback self-test runs before the control loop starts.
Set the crystal (`MCP2515_XTAL_8MHZ` / `16MHZ`) in `Core/Inc/mcp2515.h`.

Bench test of a boat heading autopilot on an **STM32F446RE (Nucleo-F446RE)**.
An **MPU6050** gyro measures how the "boat" turns. A **fuzzy-scheduled cascade PID** turns that into
a steering command, which drives a DC motor through an **L298N (ZX-040)** with 20 kHz PWM. The board
streams a live dashboard over the ST-LINK USB serial port, and a small PC program
(`pc/ShipMonitor.exe`) shows the same data as a shaded 3D ASCII ship.

It is a bench rig: there is no hull, and no sensor on the steering motor yet. You turn and tilt the
board by hand and watch how the controller would steer.

## Hardware and wiring

| Signal | STM32 pin | Nucleo header | Connects to |
|---|---|---|---|
| I2C1 SCL (400 kHz) | PB8 | D15 | MPU6050 SCL |
| I2C1 SDA | PB9 | D14 | MPU6050 SDA |
| PWM 20 kHz (TIM1_CH1) | PA8 | D7 | L298N ENA |
| Direction IN1 | PB0 | A3 | L298N IN1 |
| Direction IN2 | PB1 | Morpho CN10 | L298N IN2 |
| USART2 TX/RX (230400 baud) | PA2 / PA3 | (on-board) | ST-LINK virtual COM port |

- MPU6050 at address `0x68` (AD0 low), powered from 3.3 V. Clones that report `WHO_AM_I = 0x70` work too.
- Mount the MPU6050 **face up with its X arrow pointing at the bow**; roll and pitch assume this.
- The L298N motor supply is separate; tie its GND to the Nucleo GND. 3.3 V logic drives ENA/IN1/IN2 fine.

## How it works

```
 MPU6050 ──14-byte burst──► yaw rate r ──∫──► heading ψ ──► error e = ψd − ψ
 (200 Hz, I2C1)              │                                  │
                             │            ┌─────────────────────┘
                             │            ▼
                             │   Fuzzy gain scheduler (e, ė = −r) ─► ΔKp ΔKi ΔKd
                             │            │
                             └──────────► Outer heading PID (20 Hz) ─► steering command δcmd (±30°)
                                          │
                                          ▼
                                 Inner steering PID (200 Hz) ─► signed motor command u (−1…+1)
                                          │
                                          ▼
                                 L298N: IN1/IN2 = direction, ENA = |u| × 20 kHz PWM
```

- **Timing.** TIM3 interrupts at 200 Hz (72 MHz / 7200 / 50). Each tick reads the IMU, integrates
  heading and runs the inner loop; every 10th tick also runs the outer loop (20 Hz). The interrupt
  takes about 430 µs of its 5000 µs budget, measured live with the DWT cycle counter.
- **IMU read.** One 14-byte I2C burst from `ACCEL_XOUT_H` gives the three accelerometer axes and gyro Z.
  The MPU6050's internal 42 Hz low-pass filter is on, so 200 Hz sampling doesn't alias gyro noise.
- **Heading.** Yaw rate (bias removed, 0.5 °/s deadband so it doesn't creep at rest) is integrated and
  wrapped to ±180°. Heading 0 is wherever the board pointed at boot, and the setpoint is 0.
- **Fuzzy gain scheduler.** Error and error rate are scaled into [−3, +3] (gains 3/30 and 3/15) and
  fuzzified over 7 triangular sets (NL…PL). Only 4 rules fire at a time; `min` AND-inference and a
  weighted average over the singletons {0, 0.25, 0.5, 0.75, 1} give ΔK in [0, 1]. Each gain is then
  `K = Kmin + ΔK × (Kmax − Kmin)`:

  | Gain | Kmin | Kmax | Fixed (`fuzzy_enabled = 0`) |
  |---|---|---|---|
  | Kp [°/°] | 0.5 | 2.0 | 1.0 |
  | Ki [°/(°·s)] | 0.0 | 0.2 | 0.05 |
  | Kd [°/(°/s)] | 0.05 | 0.3 | 0.15 |

  The rule tables (`RULE_KP`, `RULE_KI`, `RULE_KD` in `main.c`) push hard while a large error grows,
  integrate only near the setpoint, and damp most when close but still turning fast.
- **Outer PID (heading → steering).** `δcmd = clamp(Kp·e + I − Kd·r, ±30°)`: derivative on the
  measurement (no setpoint kick) and conditional-integration anti-windup.
- **Inner PID (steering → motor).** Error `δcmd − δ`, low-pass-filtered derivative on δ, a 0.5° deadband
  (motor off, integrator frozen), conditional-integration anti-windup, a +0.3 backlash kick for 4 ticks
  on each direction reversal, and a minimum-duty lift (`U_MIN = 0.15`) so small commands overcome
  motor stiction.
- **Motor.** `u > 0`: IN1 high, IN2 low. `u < 0`: the reverse. `u = 0`: both low (coast).
  Duty = |u| × ARR on TIM1 (72 MHz / 3600 = 20 kHz).

### Bench mode (`BENCH_TEST_NO_ENCODER 1`)

There is no steering-angle sensor yet, so the measured steering angle δ is fixed at 0 and the motor
command simply follows δcmd. On the bench nothing turns the heading back either, so both integral
terms are forced to 0: with no feedback they would only wind the rudder onto its stop. The rudder
therefore shows `Kp × error − Kd × yaw rate`: it counter-steers as you turn the board and returns to
centre at N.

Setting it to `0` restores both integral terms and stops the build with an `#error` until you add code
that reads the real steering angle into `delta`.

### Start-up and safety

1. Waits for the MPU6050 to power up, wakes it, and prints `WHO_AM_I`. If it doesn't answer, it scans
   the I2C bus and prints what it finds, then stops.
2. Runs a self-test of the fuzzy maths (grid points, blending, clamping, heading wrap).
3. **Calibrates for about 1 s: keep the board still and level.** It averages the gyro offset and the
   accelerometer tilt; that tilt becomes "level" (0 roll / 0 pitch). Heading resets to 0. Press RESET
   to calibrate again.
4. Starts the PWM and the 200 Hz control interrupt, then refreshes the dashboard every 100 ms.

- If 20 IMU reads in a row fail (100 ms), the motor stops and the dashboard shows `IMU FAILSAFE`.
- Any HAL error pulls IN1 and IN2 low, so the L298N coasts whatever ENA does.
- The NVIC uses priority group 4 with TIM3 below SysTick, so HAL timeouts keep running inside the
  control interrupt (the blocking I2C read there can't hang).

## Sign conventions

| Quantity | Positive means |
|---|---|
| Heading ψ | clockwise seen from above (turning to starboard) |
| Roll | starboard side down |
| Pitch | bow down |
| Steering δcmd | rudder trailing edge to starboard, so the bow turns to starboard |
| Motor u | FWD (IN1 high) |

## Serial dashboard

Open the ST-LINK virtual COM port at **230400 baud, 8N1** (for example PuTTY:
`putty -serial COM7 -sercfg 230400,8,n,1,N`; your COM number may differ). The board clears the screen
once and then redraws this 82 × 21 frame in place about 10 times a second:

```
+================================================================================+
|          STM32F446RE FUZZY CASCADE PID - TOP VIEW & BENCH TELEMETRY            |
| Mode: [FUZZY ACTIVE] | Loop: 200Hz/20Hz | ISR Load:  428 us / 5000 us          |
+-----------------------------------------+--------------------------------------+
| TOP VIEW: HEADING  (N = 0 deg, setpoint)| 1. IMU & ATTITUDE (MPU6050)          |
|                                         |  Yaw Rate (r)   :   +0.00 deg/s      |
|                    N                    |  Heading (psi)  :   +0.00 deg        |
|                    ^                    |  Roll / Pitch   :   +0.1 /  +0.0 deg |
|                  // \\                  |  Setpoint (psid):    0.00 deg        |
|                 +     +                 |  Error (e)      :   +0.00 deg        |
| W               |     |               E |  Err: [          |          ]        |
|                 |     |                 +--------------------------------------+
|                 |     |                 | 2. FUZZY GAIN SCHEDULER (20 Hz)      |
|                 +--|--+                 |  dKp: 0.25 [##--------] Kp: 0.88     |
|                    |                    |  dKi: 1.00 [##########] Ki: 0.00     |
|                    S                    |  dKd: 0.25 [##--------] Kd: 0.11     |
+-----------------------------------------+--------------------------------------+
| 3. CASCADE ACTUATOR & L298N MOTOR DRIVER (20 kHz PWM)                          |
|  Steering Cmd (delta_cmd) :  +0.00 deg  | L298N PWM (PA8):   0.0 % [ STOP]     |
|  Signed Command (u)       :  +0.00      | Drive REV [          |          ] FWD|
+================================================================================+
```

- **Mode:** `FUZZY ACTIVE`, `FIXED GAINS` (when `fuzzy_enabled = 0`) or `IMU FAILSAFE`.
- **Top view:** the ship seen from above with N = boot heading = setpoint; the bow marker (`^ > v <`)
  points where the bow points and the line behind the stern is the rudder.
- `printf("%f")` works without the "Use float with printf" project setting: `main.c` links the
  float formatter itself.

## PC ship monitor (`pc/ShipMonitor.cs`)

A Windows console program that reads the dashboard over the serial port and draws a shaded 3D ship
(hull, deck, wheelhouse, funnel, mast and rudder) on an animated sea with a compass ring. The ship
turns with heading, rolls and pitches with the accelerometer, and swings its rudder with δcmd; a green
line shows where it is heading. The status bar shows heading, yaw rate, roll, pitch and rudder angle.

Build with the C# compiler that ships with Windows (.NET Framework 4, nothing to install), from
PowerShell or cmd:

```
cd pc
C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe /nologo /optimize ShipMonitor.cs
```

Run:

```
ShipMonitor.exe                      # COM7 at 230400 baud
ShipMonitor.exe COM5 230400          # another port
ShipMonitor.exe --frame 30 5 -3 15   # print one frame (heading roll pitch rudder) and exit, no board needed
```

- Only one program can open the COM port: close PuTTY before starting the monitor, and press Ctrl+C
  in the monitor before opening PuTTY again.
- A bigger window (or a smaller font) gives a more detailed ship; it adapts when resized.
- It reconnects by itself if the board is reset or unplugged.
- It parses the dashboard text, so if you rename the `Heading (psi)`, `Yaw Rate (r)`, `Roll / Pitch`
  or `Steering Cmd (delta_cmd)` labels in `main.c`, update the patterns at the top of `ReadLoop`.

## Building and flashing

**STM32CubeIDE (1.15.1).** Import the folder as an existing project, build the Debug configuration,
and create a debug configuration for *this* project before pressing Debug (CubeIDE can otherwise reuse
another project's configuration and flash the wrong firmware).

`Debug/` is not in git, so after cloning, build once in CubeIDE to generate the makefiles. From then on
you can build and flash from the command line with the tools bundled in CubeIDE's `plugins` folder
(`...gnu-tools-for-stm32.../tools/bin`, `...make.../tools/bin` and `...cubeprogrammer.../tools/bin`):

```
cd Debug
make -j8 all
STM32_Programmer_CLI -c port=SWD -w IMU-FuzzyPID.elf -v -rst
```

**CubeMX.** All peripherals (72 MHz system clock from the 8 MHz HSE, I2C1, USART2, TIM1, TIM3,
PB0/PB1, NVIC) are configured in `IMU-FuzzyPID.ioc`. All hand-written code sits inside
`/* USER CODE BEGIN … */ /* USER CODE END … */` blocks in `Core/Src/main.c`, so regenerating from the
`.ioc` keeps it.

## Tuning knobs (`Core/Src/main.c`)

| Define / variable | Default | What it does |
|---|---|---|
| `fuzzy_enabled` | 1 | 0 = fixed `BASE_GAINS`; flip it live from CubeIDE's Live Expressions |
| `K_MIN`, `K_MAX`, `BASE_GAINS` | see table above | outer-loop gain ranges |
| `RULE_KP`, `RULE_KI`, `RULE_KD` | — | 7 × 7 fuzzy rule tables (entry n → ΔK = 0.25 n) |
| `GE`, `GEC` | 3/30, 3/15 | scaling of error [°] and error rate [°/s] into the fuzzy range |
| `DELTA_MAX` | 30° | steering command limit |
| `BENCH_TEST_NO_ENCODER` | 1 | bench mode, see above |
| `IN_KP`, `IN_KI`, `IN_KD` | 1/30, 0.10 (0 on bench), 0.002 | inner steering PID |
| `IN_DEADBAND` | 0.5° | inner-loop deadband |
| `BOOST`, `BOOST_TICKS` | 0.3, 4 | backlash kick on direction reversal |
| `U_MIN` | 0.15 | minimum motor duty when moving |
| `GYRO_DEADBAND_DPS` | 0.5 °/s | yaw rates below this read as 0 |
| `GYRO_CAL_SAMPLES` | 200 | calibration samples (× 5 ms ≈ 1 s) |
| `IMU_MAX_FAILS` | 20 | consecutive bad reads before the motor stops |
| `TELEM_PERIOD_MS` | 100 | dashboard refresh period |

## Project layout

```
IMU-FuzzyPID.ioc          CubeMX configuration (pins, clocks, peripherals, NVIC)
Core/Src/main.c           all application code, inside USER CODE blocks
Core/Src, Core/Inc        CubeMX-generated init, MSP and interrupt code
Drivers/                  STM32 HAL and CMSIS (copied by CubeMX)
STM32F446RETX_FLASH.ld    linker script
pc/ShipMonitor.cs         PC 3D ship monitor (single C# file)
```

## Known limits

- **Bench only.** No hull and no steering sensor: δ is fixed at 0 and both integral terms are off.
- **Heading drifts slowly.** It comes from integrating the gyro only, with no magnetometer; press RESET
  to re-zero it.
- **Level calibration.** Subtracting the boot roll/pitch angles is exact only for a small mounting tilt;
  if the board sits more than about 10° off level, rotate the gravity vector instead.
- **Stuck I2C bus.** A bus stuck busy costs HAL's fixed 25 ms per read; it shows as `IMU FAILSAFE` with
  an ISR load near 25,000 µs. Add SCL-toggle bus recovery if that ever happens.
- **Gains are starting points.** The fuzzy rule tables and gain ranges need tuning on a real boat.
