# Test Framework

This document organises the standalone hardware tests under `tools/`.
The current hardware route is:

- Motors and DRV8833 drivers are controlled directly by Raspberry Pi GPIO.
- HC-SR04 ultrasonic sensing is connected directly to Raspberry Pi GPIO.
- TCRT5000 downward sensors used by the main robot are connected directly to Raspberry Pi GPIO.
- MCP23017 is used for limit-switch inputs.

Markdown note:

- Lines like ```` ```bash ```` and ```` ``` ```` are Markdown code-block markers.
- They are not terminal commands and should not be typed into the Raspberry Pi.
- When viewing this file on GitHub or in Markdown preview, those markers render as a command box.
- Only copy the command lines inside the box, such as `cmake --preset linux-debug`.

## Overall Test Framework

The complete test framework has two main stages:

| Stage | Goal | What to do |
|---|---|---|
| Stage 1 | Prepare the Raspberry Pi environment | Install C++ build tools, CMake, libgpiod, and I2C tools; enable I2C; verify GPIO and I2C access. |
| Stage 2 | Run hardware module test cases | Use the motor, sensor, ultrasonic, and full-robot test commands listed below. |

Stage 1 only needs to be done when setting up a new Raspberry Pi, reinstalling the OS, or fixing missing dependencies.
Stage 2 is the normal repeated bring-up process whenever wiring, hardware, or code changes.

## 1. Stage 1 - Prepare Raspberry Pi Environment

These tests are intended to run on Raspberry Pi Linux, not Windows.
The project `CMakeLists.txt` explicitly requires a Unix-like target and uses Linux GPIO/I2C device files.

### 1.1 Required System Packages

Install the build and hardware-access dependencies on the Raspberry Pi:

```bash
sudo apt update
sudo apt install -y build-essential cmake git libgpiod-dev gpiod i2c-tools
```

Package purpose:

| Package | Why it is needed |
|---|---|
| `build-essential` | Provides `g++`, `gcc`, `make`, standard build tools, and C++ runtime headers. |
| `cmake` | Configures and generates the build system from `CMakeLists.txt` and `CMakePresets.json`. |
| `git` | Pulls / updates the project from GitHub. |
| `libgpiod-dev` | Provides `gpiod.h` and `libgpiod`, required by GPIO motor, sensor, and ultrasonic tests. |
| `gpiod` | Provides GPIO inspection tools such as `gpiodetect` and `gpioinfo`. |
| `i2c-tools` | Provides `i2cdetect`, used to verify MCP23017 on the I2C bus. |

The project currently uses C++20:

```text
target_compile_features(... cxx_std_20)
```

So `g++` must support C++20.

### 1.2 Verify Compiler And Build Tools

Check that the toolchain is present:

```bash
g++ --version
cmake --version
make --version
```

Expected:

- `g++` prints a valid compiler version.
- `cmake` exists and is version `3.21` or newer.
- `make` exists because the current Raspberry Pi preset uses `Unix Makefiles`.

### 1.3 Enable I2C For MCP23017

Enable I2C on the Raspberry Pi:

```bash
sudo raspi-config
```

Then choose:

```text
Interface Options -> I2C -> Enable
```

Or use the non-interactive command if available:

```bash
sudo raspi-config nonint do_i2c 0
```

Reboot after enabling I2C:

```bash
sudo reboot
```

After reboot, verify the I2C device exists:

```bash
ls /dev/i2c-1
```

Scan the bus:

```bash
sudo i2cdetect -y 1
```

Expected for MCP23017 with `A0/A1/A2 -> GND`:

```text
0x20
```

If `0x20` does not appear, check:

- MCP23017 `VDD -> 3.3V`
- MCP23017 `VSS -> GND`
- MCP23017 `SDA -> Raspberry Pi GPIO2 / physical pin 3`
- MCP23017 `SCL -> Raspberry Pi GPIO3 / physical pin 5`
- MCP23017 `RESET -> 3.3V`
- MCP23017 `A0/A1/A2 -> GND`
- common ground

### 1.4 Verify GPIO Access

The code uses:

```text
/dev/gpiochip0
```

Check that the GPIO chip is visible:

```bash
gpiodetect
gpioinfo gpiochip0
```

In our normal Raspberry Pi bring-up, the standalone test executables are run directly as the current user:

```bash
./out/build/linux-debug-make/<TargetName>
```

No extra `sudo gpio ...` command is required for the test programs themselves when `/dev/gpiochip0` is accessible to the current user.
Only use the following permission fix if a GPIO or I2C test fails with `permission denied`:

```bash
sudo usermod -aG gpio,i2c $USER
sudo reboot
```

After reboot, verify group membership:

```bash
groups
```

For I2C bus scanning, using `sudo i2cdetect -y 1` is still acceptable during hardware diagnostics.

### 1.5 Project Preset Expected On Raspberry Pi

The current `CMakePresets.json` uses this Linux preset:

```text
linux-debug
```

It generates Makefiles under:

```text
out/build/linux-debug-make/
```

If the executable is not found under `out/build/linux-debug/`, use:

```bash
./out/build/linux-debug-make/<TargetName>
```

### 1.6 Clock / Timestamp Note

During previous Raspberry Pi testing, `build.ninja still dirty` / future timestamp warnings were caused by incorrect system time after reboot or file transfer.
For the current Makefile preset, stale timestamps can still produce confusing rebuild warnings.

Check time:

```bash
date
timedatectl status
```

Enable NTP:

```bash
sudo timedatectl set-ntp true
```

If copied files still have future timestamps, refresh project file timestamps from the project root:

```bash
find . -type f -exec touch {} +
```

Use this only to fix timestamp skew, not as part of normal testing.

## 2. Build And Run Pattern

Run all commands from the project root on the Raspberry Pi:

```bash
cd ~/Desktop/stair-climbing-robot-main
cmake --preset linux-debug
cmake --build --preset linux-debug --target <TargetName>
./out/build/linux-debug-make/<TargetName> [arguments]
```

The configured Linux debug preset uses:

```text
out/build/linux-debug-make/
```

So the executable path should include `linux-debug-make`, not just `linux-debug`.

## 3. Test Target Summary

| Layer | Target | Source file | Purpose |
|---|---|---|---|
| Generic DRV8833 | `DRV8833_Direct_Test` | `tools/drv8833_direct_test.cpp` | Test one DRV8833 channel pair by manually passing two GPIO numbers. |
| Drive wheels | `DRV8833_Direct_Wheel_Test` | `tools/drv8833_direct_wheel_test.cpp` | Test front wheels, middle wheels, or both wheel modules. |
| Lift modules | `DRV8833_Direct_Lift_Test` | `tools/drv8833_direct_lift_test.cpp` | Timed forward/reverse test for front and rear lift actuators. |
| Lift modules | `DRV8833_Direct_Lift_Keyboard_Test` | `tools/drv8833_direct_lift_keyboard_test.cpp` | Manual keyboard control for front/rear lift actuators. |
| Rear slide | `DRV8833_Direct_Rear_Slide_Test` | `tools/drv8833_direct_rear_slide_test.cpp` | Timed forward/reverse test for the rear slider motor. |
| Downward IR, main route | `IR_Direct_All_Test` | `tools/ir_direct_gpio_all_test.cpp` | Read front, middle, and rear TCRT5000 sensors from Raspberry Pi GPIO. |
| Downward IR, legacy MCP route | `Front_IR_MCP23017_Test` | `tools/front_ir_mcp23017_test.cpp` | Read one TCRT5000 input through MCP23017. |
| Downward IR, legacy MCP route | `IR_MCP23017_All_Test` | `tools/ir_mcp23017_all_test.cpp` | Read three TCRT5000 inputs through MCP23017. |
| Limit switches | `MCP23017_Limit_Test` | `tools/mcp23017_limit_switch_test.cpp` | Read configured active-low limit switches through MCP23017. |
| Rear slide limits | `Rear_Slide_Limit_Orientation_Test` | `tools/rear_slide_limit_orientation_test.cpp` | Check whether rear slide front/rear limit switches are swapped. |
| Ultrasonic | `Front_Ultrasonic_Test` | `tools/front_ultrasonic_test.cpp` | Read HC-SR04 distance from the front ultrasonic sensor. |
| Full robot | `Climbing_Robot` | `src/main.cpp` | Run the complete stair-climbing state machine. |

## 4. Motor Test Commands

### 4.1 Generic Single DRV8833 Test

Build:

```bash
cmake --build --preset linux-debug --target DRV8833_Direct_Test
```

Run:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Test <gpio_in1_in3> <gpio_in2_in4> [forward_ms] [reverse_ms] [cycles]
```

Example:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Test 17 27 1500 1500 3
```

Expected result:

- Motor runs forward.
- Motor coasts briefly.
- Motor runs reverse.
- Motor brakes briefly.
- Outputs return to coast at the end.

### 4.2 Front / Middle Wheel Test

Build:

```bash
cmake --build --preset linux-debug --target DRV8833_Direct_Wheel_Test
```

Run:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Wheel_Test [front|middle|both] [forward_ms] [reverse_ms] [cycles]
```

Examples:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Wheel_Test front 1800 1800 3
./out/build/linux-debug-make/DRV8833_Direct_Wheel_Test middle 1800 1800 3
./out/build/linux-debug-make/DRV8833_Direct_Wheel_Test both 1800 1800 3
```

Current GPIO mapping from `config.h`:

| Module | GPIO mapping |
|---|---|
| Front left wheel | BCM17 / BCM18 |
| Front right wheel | BCM27 / BCM22 |
| Middle left wheel | BCM23 / BCM24 |
| Middle right wheel | BCM25 / BCM8 |

Expected result:

- `front` only moves the front wheel module.
- `middle` only moves the middle wheel module.
- `both` moves front and middle modules together.
- Forward and reverse should be opposite directions.

### 4.3 Timed Lift Test

Build:

```bash
cmake --build --preset linux-debug --target DRV8833_Direct_Lift_Test
```

Run:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Lift_Test [lift1|lift2|both] [forward_ms] [reverse_ms] [cycles]
```

Examples:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Lift_Test lift1 1200 1200 1
./out/build/linux-debug-make/DRV8833_Direct_Lift_Test lift2 1200 1200 1
./out/build/linux-debug-make/DRV8833_Direct_Lift_Test both 1200 1200 1
```

Current GPIO mapping from `config.h`:

| Module | GPIO mapping |
|---|---|
| Lift-1 / front lift | BCM10 / BCM9 |
| Lift-2 / rear lift | BCM11 / BCM7 |

Expected result:

- Selected lift actuator moves one direction, stops, then moves the opposite direction.
- If direction is reversed mechanically, update the logical direction aliases in `config.h`, not the test command.

### 4.4 Keyboard Lift Test

Build:

```bash
cmake --build --preset linux-debug --target DRV8833_Direct_Lift_Keyboard_Test
```

Run:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Lift_Keyboard_Test [hold_ms]
```

Example:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Lift_Keyboard_Test 250
```

Controls:

| Key | Action |
|---|---|
| `w` | Lift-1 / front lift up |
| `s` | Lift-1 / front lift down |
| `a` | Lift-2 / rear lift up |
| `d` | Lift-2 / rear lift down |
| `space` | Stop both lifts |
| `q` | Stop and quit |

Safety behaviour:

- Each key press only holds motion for `hold_ms`.
- Hold or repeatedly press the key to continue moving.
- Outputs return to coast when the program exits normally.

### 4.5 Rear Slide Motor Test

Build:

```bash
cmake --build --preset linux-debug --target DRV8833_Direct_Rear_Slide_Test
```

Run:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Rear_Slide_Test [forward_ms] [reverse_ms] [cycles]
```

Example:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Rear_Slide_Test 1000 1000 2
```

Current GPIO mapping from `config.h`:

| Module | GPIO mapping |
|---|---|
| Rear slide DRV8833 | BCM12 / BCM13 |

Expected result:

- Rear slide moves forward, stops, moves reverse, stops.
- Current main-robot logical aliases are `SLIDE_2_TO_FRONT_GPIO = BCM13` and `SLIDE_2_TO_REAR_GPIO = BCM12`.

## 5. Sensor And Limit Test Commands

### 5.1 Direct GPIO TCRT5000 All-Sensor Test

This is the current main-robot downward sensor route.

Build:

```bash
cmake --build --preset linux-debug --target IR_Direct_All_Test
```

Run:

```bash
./out/build/linux-debug-make/IR_Direct_All_Test [samples] [poll_ms]
```

Example:

```bash
./out/build/linux-debug-make/IR_Direct_All_Test 200 100
```

Current GPIO mapping from `config.h`:

| Sensor | GPIO |
|---|---|
| Front downward TCRT5000 DO | BCM4, physical pin 7 |
| Middle downward TCRT5000 DO | BCM5, physical pin 29 |
| Rear downward TCRT5000 DO | BCM21, physical pin 40 |

Output meaning:

- `valid=1` means the software has a valid reading.
- `surface=1` means the software currently believes a supporting surface is detected.
- `drop=1` means the software currently believes there is no supporting surface.
- If the physical behaviour is inverted, adjust the sensor module potentiometer or the `*_ACTIVE_ON_SURFACE` flag in `config.h`.

### 5.2 MCP23017 Limit Switch Test

Build:

```bash
cmake --build --preset linux-debug --target MCP23017_Limit_Test
```

Run:

```bash
./out/build/linux-debug-make/MCP23017_Limit_Test [samples] [interval_ms]
```

Example:

```bash
./out/build/linux-debug-make/MCP23017_Limit_Test 100 200
```

Expected wiring:

- Limit switch `NO` to MCP23017 input pin.
- Limit switch `COM` to `GND`.
- MCP23017 input pull-ups enabled in software.

Signal meaning:

- `raw=1`, `triggered=0`: switch open / not pressed.
- `raw=0`, `triggered=1`: switch pressed / connected to ground.
- `CHANGED`: this input changed since the previous sample.

Current limit mapping from `config.h`:

| Limit switch | MCP23017 pin |
|---|---|
| Lift-1 upper | GPA3 |
| Lift-1 lower | GPA4 |
| Rear slide front / upper | GPA7 |
| Rear slide rear / lower | GPB0 |
| Lift-2 upper | GPB1 |
| Lift-2 lower | GPB2 |

### 5.3 Rear Slide Limit Orientation Test

Build:

```bash
cmake --build --preset linux-debug --target Rear_Slide_Limit_Orientation_Test
```

Run:

```bash
./out/build/linux-debug-make/Rear_Slide_Limit_Orientation_Test [observe_seconds] [interval_ms]
```

Example:

```bash
./out/build/linux-debug-make/Rear_Slide_Limit_Orientation_Test 5 100
```

Expected main-robot mapping:

| Physical switch | Software signal |
|---|---|
| Physical rear limit | `slide2_lower` / GPB0 |
| Physical front limit | `slide2_upper` / GPA7 |

Use this when:

- Rear slider drives in the expected direction but the state machine waits forever.
- You suspect the front and rear limit inputs are swapped.

### 5.4 Front IR Through MCP23017 Test

This is a legacy / diagnostic route.  The main robot currently uses direct GPIO for TCRT5000 sensors.

Build:

```bash
cmake --build --preset linux-debug --target Front_IR_MCP23017_Test
```

Run:

```bash
./out/build/linux-debug-make/Front_IR_MCP23017_Test [mcp_pin] [duration_s] [active_on_surface(0|1)]
```

Example:

```bash
./out/build/linux-debug-make/Front_IR_MCP23017_Test 0 30 0
```

Default:

- MCP23017 pin `0` / GPA0.
- Duration `30s`.
- `active_on_surface` follows `config.h`.

### 5.5 All IR Through MCP23017 Test

This is also a legacy / diagnostic route.

Build:

```bash
cmake --build --preset linux-debug --target IR_MCP23017_All_Test
```

Run:

```bash
./out/build/linux-debug-make/IR_MCP23017_All_Test [duration_s] [poll_ms]
```

Example:

```bash
./out/build/linux-debug-make/IR_MCP23017_All_Test 30 250
```

Default MCP23017 mapping:

| Sensor | MCP23017 pin |
|---|---|
| Front downward | GPA0 |
| Middle support | GPA1 |
| Rear support | GPA2 |

## 6. Ultrasonic Test Command

Build:

```bash
cmake --build --preset linux-debug --target Front_Ultrasonic_Test
```

Run:

```bash
./out/build/linux-debug-make/Front_Ultrasonic_Test [duration_s] [timeout_us] [sample_interval_ms] [trig_gpio] [echo_gpio]
```

Example using current `config.h` defaults:

```bash
./out/build/linux-debug-make/Front_Ultrasonic_Test 20 30000 50
```

Current GPIO mapping:

| HC-SR04 signal | GPIO |
|---|---|
| TRIG | BCM16, physical pin 36 |
| ECHO | BCM26, physical pin 37 |

Expected result:

- `valid=1 distance_m=<value>` when echo is received.
- `valid=0 distance_m=0` when no echo is received before timeout.
- For full robot approach logic, the important threshold is `READY_TO_CLIMB_DISTANCE_M` in `config.h`.

## 7. Full Robot Integration Test

Build:

```bash
cmake --build --preset linux-debug --target Climbing_Robot
```

Run:

```bash
./out/build/linux-debug-make/Climbing_Robot
```

Only run this after these standalone checks pass:

- Front and middle wheels can drive forward and reverse.
- Lift-1 and Lift-2 move in the expected logical directions.
- Rear slide moves in the expected logical directions.
- Limit switches report the correct `triggered` state.
- Direct GPIO TCRT5000 sensors produce the expected `surface` / `drop` state changes.
- Ultrasonic readings are valid and stable near the approach threshold.

Expected high-level state flow:

```text
Startup reset
ApproachingStep
RearSliderBack
FrontLift
MiddleDriveToFrontLanding
MiddleClimb
FrontDriveToMiddleLanding
RearSliderForward
RearLift
FinalDriveToRearLanding
repeat / Completed
```

## 8. Quick Command Reference

```bash
cmake --preset linux-debug

cmake --build --preset linux-debug --target DRV8833_Direct_Test
cmake --build --preset linux-debug --target DRV8833_Direct_Wheel_Test
cmake --build --preset linux-debug --target DRV8833_Direct_Lift_Test
cmake --build --preset linux-debug --target DRV8833_Direct_Lift_Keyboard_Test
cmake --build --preset linux-debug --target DRV8833_Direct_Rear_Slide_Test
cmake --build --preset linux-debug --target IR_Direct_All_Test
cmake --build --preset linux-debug --target MCP23017_Limit_Test
cmake --build --preset linux-debug --target Rear_Slide_Limit_Orientation_Test
cmake --build --preset linux-debug --target Front_Ultrasonic_Test
cmake --build --preset linux-debug --target Climbing_Robot
```

Most useful run commands:

```bash
./out/build/linux-debug-make/DRV8833_Direct_Wheel_Test both 1800 1800 3
./out/build/linux-debug-make/DRV8833_Direct_Lift_Keyboard_Test 250
./out/build/linux-debug-make/DRV8833_Direct_Rear_Slide_Test 1000 1000 2
./out/build/linux-debug-make/MCP23017_Limit_Test 100 200
./out/build/linux-debug-make/Rear_Slide_Limit_Orientation_Test 5 100
./out/build/linux-debug-make/IR_Direct_All_Test 200 100
./out/build/linux-debug-make/Front_Ultrasonic_Test 20 30000 50
./out/build/linux-debug-make/Climbing_Robot
```
