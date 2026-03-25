

##B Layer: Perception and Judgement (Sensor Layer)
1. Layer Goal

The B Layer converts raw sensor signals into high-level information for the state machine and controller. Its core responsibilities are:

  Detect stair front faces
  Determine whether front wheels, middle wheels, and rear support wheels have landed
  Estimate robot pose using the IMU
  Assess whether the robot is in a safe posture
  Provide structured stair-climbing status for upper-level logic

This layer uses both blocking I/O and callback-based updates to provide timely and reliable perception.

2. Files in This Layer
include/imu_sensor.h / src/imu_sensor.cpp
include/downward_sensor.h / src/downward_sensor.cpp
include/front_distance_sensor.h / src/front_distance_sensor.cpp
include/limit_switch.h / src/limit_switch.cpp
3. Sensor Modules and Responsibilities
  3.1 IMU Sensor (ImuSensor)
    Purpose: Measures robot attitude (pitch, roll, yaw) and determines tilt safety.
    Data Source: I2C (MPU6050).
    Update Method: Background polling thread.
    Interface:
      latestPose() – Get the latest pose
      setCallback() – Register a callback when new data arrives
    Details: Uses complementary filter to fuse accelerometer and gyroscope data; pitch and roll are used for safety evaluation; yaw is available but may drift over time.
  3.2 Downward Sensor (DownwardSensor)
    Purpose: Detects whether the front wheels are supported, edges, drops, or step surfaces.
    Data Source: GPIO edge events.
    Update Method: Background thread listening for GPIO edges.
    Interface:
      latest() – Get the latest reading
      waitForEdge(timeout) – Block until an edge is detected
      setCallback() – Register a callback for updates
    Details: Reports:
      on_step_surface – whether front wheels are on a step
      drop_detected – if a drop is detected
      edge_detected – if an edge is detected
  3.3 Front Distance Sensor (FrontDistanceSensor)
    Purpose: Measures distance from front of robot to obstacle or stair face.
    Data Source: Ultrasonic sensor via GPIO trigger/echo lines.
    Update Method: Blocking measurement.
    Interface:
      readBlocking(timeout) – Measure distance with a timeout
      latest() – Get the latest reading
      setCallback() – Register callback after each measurement
    Details: Converts pulse width between trigger and echo to meters; used to detect stair faces or obstacles before front climbing.
  3.4 Limit Switch (LimitSwitch)
    Purpose: Protect mechanical structures by detecting upper/lower limits.
    Data Source: GPIO edge events.
    Update Method: Background thread listening for GPIO edges.
    Interface:
      isTriggered() – Check if the switch is triggered
      isUpperLimit() / isLowerLimit() – Check specific role trigger
      waitForTrigger(timeout) – Block until triggered
      setCallback() – Register callback on trigger
    Details: Supports active-low switches; maintains timestamp and validity of state.
4. Design Highlights
Thread-safe caching: All modules maintain the latest sensor readings with mutex protection.
Callback-based notification: Upstream modules receive timely updates without busy-waiting.
Hardware abstraction: Upper layers do not need to know GPIO or I2C details.
Safety integration: IMU and limit switches are used to prevent unsafe actions.
Edge/step detection: Downward and front distance sensors provide cues for stair climbing and obstacle avoidance.
5. Usage
Include header files
  #include "imu_sensor.h"
  #include "downward_sensor.h"
  #include "front_distance_sensor.h"
  #include "limit_switch.h"
Create sensor objects
  Robot::ImuSensor imu;
  Robot::DownwardSensor downward;
  Robot::FrontDistanceSensor front_distance;
  Robot::LimitSwitch upper_limit(5, Robot::LimitRole::Upper);
Start sensors
  imu.start();
  downward.start();
  front_distance.start();
  upper_limit.start();
Read latest data
  auto pose = imu.latestPose();
  auto step_info = downward.latest();
  auto distance = front_distance.latest();
  bool upper_triggered = upper_limit.isUpperLimit();
Register callbacks
  imu.setCallback([](const Robot::PoseData& pose){ /* handle new pose */ });
  downward.setCallback([](const Robot::DownwardReading& reading){ /* handle edge/drop */ });
6. Summary

The D layer provides a structured perception and judgment framework:

1.Collect sensor data (IMU, downward, front distance, limit switches)
2.Update cached readings and notify via callbacks
3.Fuse pose and sensor data to evaluate safety and step readiness
4.Provide high-level information to the upper-level stair-climbing control logic

This design ensures safe and efficient stair climbing without exposing low-level hardware details to the main control algorithms.

# C Layer: Mechanical Abstraction

## 1. Layer Goal

The mechanical abstraction layer encapsulates robot mechanisms as reusable software modules so that the upper control logic does not directly manipulate low-level motor drivers or GPIO details. This layer focuses on **mechanical actions**, including:

- front section approach and climbing
- front balance slider homing and position bookkeeping
- middle-section lifting
- middle-section wheel following
- rear support coordination during middle transfer and rear transfer

The layer hides hardware details behind common interfaces such as `IDriveSection` and `ILinearAxis`.

## 2. Files in This Layer

- `include/front_segment.h`
- `src/front_segment.cpp`
- `include/front_balance_slider.h`
- `src/front_balance_slider.cpp`
- `include/middle_lift_module.h`
- `src/middle_lift_module.cpp`
- `include/middle_drive_module.h`
- `src/middle_drive_module.cpp`
- `include/rear_support_module.h`
- `src/rear_support_module.cpp`

## 3. Mechanical Meaning of Each Module

### 3.1 `FrontSegment`

`FrontSegment` abstracts the front motorized section. It combines:

- front drive wheel motion
- front ultrasonic sensing result access
- optional first lifting axis motion
- front wheel surface confirmation callback

It supports three main actions:

- `approachStep()`
  Move the front drive wheels toward the stair until the measured front distance enters the valid stair-detection band.
- `liftFrontToStep()`
  Drive the front wheels slowly while commanding the first lifting module upward, until the front downward sensor confirms that the front wheels have landed on the upper step surface.
- `placeFrontOnStep()`
  Lower the body gradually and keep the front wheels stable on the step surface.

This module represents the **front section climbing action** rather than a pure wheel-driving helper.

### 3.2 `FrontBalanceSlider`

`FrontBalanceSlider` represents the **front sliding module**. In the current implementation it is used for:

- homing the front slider
- tracking slider extension
- checking whether the stored slider position is valid

Important design note:

The current version does **not** use this module to decide whether the front section has reached the stair edge. The front section state is judged by the **front ultrasonic sensor** and the **front downward sensor** only. Therefore, this module currently belongs to the mechanical layer as a front sliding mechanism, not as a perception module.

### 3.3 `MiddleLiftModule`

`MiddleLiftModule` abstracts the **first lifting module**. It controls the vertical motion of the main body relative to the middle drive wheels.

Its responsibilities are:

- raising the body during front climbing and middle transfer
- lowering the body after a transfer
- moving to a target height
- holding the current height

The module optionally accepts a callback `middle_support_confirmed`, which is used to confirm whether the middle drive wheels have already landed on the new support surface.

### 3.4 `MiddleDriveModule`

`MiddleDriveModule` abstracts the **middle motorized wheel pair**. It is responsible for:

- advancing the middle drive wheels during middle transfer
- driving forward at a predefined speed
- braking and holding position after landing

Its key function is `advanceToStep()`, which keeps the middle wheels moving forward slowly until the middle support confirmation callback returns true.

### 3.5 `RearSupportModule`

`RearSupportModule` abstracts the **rear support subsystem**, which includes:

- rear sliding module
- second lifting module
- rear support landing confirmation

It serves two different stages:

- `assistMiddleTransfer()`
  During middle transfer, the rear slide is held while the second lifting module raises the rear support structure to help adjust the overall body attitude.
- `transferSupportToStep()`
  During rear transfer, the rear slide moves forward and the second lifting module lowers the rear support wheel until the rear support sensor confirms successful landing on the new step surface.

`stabilizeSupport()` keeps the rear support subsystem fixed after landing or during emergency stop.

## 4. Relationship Between Modules

The C layer modules cooperate as follows:

1. `FrontSegment` handles front-section stair approach and front-wheel climbing.
2. `MiddleLiftModule` and `RearSupportModule::assistMiddleTransfer()` jointly participate in body attitude adjustment during middle transfer.
3. `MiddleDriveModule` advances the middle drive wheels until the middle support sensor confirms landing.
4. `RearSupportModule` performs the final rear support transfer by sliding forward and lowering the rear support wheel.

This matches the real robot structure:

- front and middle wheels are motorized
- rear wheels are passive support wheels
- middle transfer is achieved by **cooperation of the first and second lifting modules**
- rear transfer is achieved by **rear slide forward + second lift lowering + rear support confirmation**

## 5. Interface Design

This layer depends on common interfaces defined in `hardware_interfaces.h`:

- `IDriveSection`
  Used by `FrontSegment` and `MiddleDriveModule` to command wheel motion.
- `ILinearAxis`
  Used by `FrontBalanceSlider`, `MiddleLiftModule`, and `RearSupportModule` to control lifting and sliding actions.
- callback functions
  Used to confirm front, middle, and rear landing conditions without embedding sensor details into every mechanical module.

This design decouples **mechanical behaviour** from **hardware implementation**, which improves readability, reusability, and testing flexibility.

## 6. Current Scope and Simplifications

- The front balance slider is currently a mechanical helper only.
- Automatic center-of-gravity balancing is not enabled in the current version.
- Mechanical completion is mainly judged by support confirmation callbacks and limit states, rather than by continuous position feedback.

These decisions were intentionally made to keep the current version aligned with the present hardware validation stage.

# D Layer: Perception and Judgement

## 1. Layer Goal

The perception and judgement layer converts raw sensor signals into high-level decisions used by the state machine and controller. Its core responsibilities are:

- detect the stair front face
- determine whether the front wheels, middle wheels, and rear support wheels have landed
- estimate robot attitude using the IMU
- determine whether the robot is in a safe posture
- produce a structured stair-climbing assessment for the upper logic

This layer uses both **blocking I/O** and **callback-based updates**, which matches the course requirement.

## 2. Files in This Layer

- `include/front_distance_sensor.h`
- `src/front_distance_sensor.cpp`
- `include/downward_sensor.h`
- `src/downward_sensor.cpp`
- `include/imu_sensor.h`
- `src/imu_sensor.cpp`
- `include/pose_monitor.h`
- `src/pose_monitor.cpp`
- `include/step_detector.h`
- `src/step_detector.cpp`
- `include/safety_manager.h`
- `src/safety_manager.cpp`

## 3. Sensor Roles

### 3.1 Front Ultrasonic Sensor

`FrontDistanceSensor` measures the distance between the front of the robot and the stair face.

It uses a **blocking measurement model**:

- send trigger pulse
- wait for echo rising edge and falling edge
- compute pulse width
- convert pulse width into distance

The ultrasonic sensor is used to determine:

- whether the stair face is detected
- whether the robot is close enough to begin front climbing
- whether the front section has enough clearance after climbing

Important note:

The system does **not** use `distance == 0` as a valid climbing condition. Instead, the code checks whether the distance falls inside configured thresholds:

- `STEP_FACE_MIN_DISTANCE_M`
- `STEP_FACE_MAX_DISTANCE_M`
- `READY_TO_CLIMB_DISTANCE_M`
- `STEP_COMPLETION_CLEARANCE_M`

### 3.2 Front Downward Sensor

The front downward sensor is mounted near the front drive wheels and is used to detect:

- whether the front wheel area becomes unsupported near the stair edge
- whether the front wheels have landed on the upper step surface

It works with GPIO edge events and callback-based updates. In software it produces:

- `drop_detected`
- `on_step_surface`
- `edge_detected`

### 3.3 Middle Support Sensor

The middle support sensor confirms whether the middle drive wheels have landed on the new support surface during middle transfer.

This sensor is important because middle transfer is not judged by time alone. The controller needs a clear signal that the middle drive wheels are actually supported on the new step.

### 3.4 Rear Support Sensor

The rear support sensor confirms whether the rear passive support wheel has landed on the new step surface during rear transfer.

This closes the rear transfer stage using landing confirmation instead of open-loop timing only.

### 3.5 MPU6050 IMU

`ImuSensor` reads the MPU6050 using Linux I2C. It estimates:

- pitch
- roll
- yaw

The current implementation uses accelerometer and gyroscope fusion with a complementary filter. In practice:

- pitch and roll are the main variables used for safety
- yaw is available but may drift over time because MPU6050 has no magnetometer

The IMU should be mounted on the rigid main body near the body center, not on a moving slide or support structure.

## 4. Perception Fusion Logic

### 4.1 `PoseMonitor`

`PoseMonitor` binds to the IMU and stores the latest pose sample. It provides:

- `currentPose()`
- `isSafe()`
- `isOverTilt()`

The safety decision is based on:

- whether the pose sample is valid
- whether the pose sample is fresh
- whether pitch and roll remain inside configured safe thresholds

### 4.2 `StepDetector`

`StepDetector` is the main perception-fusion module. It subscribes to:

- front ultrasonic sensor
- front downward sensor
- optional middle support sensor
- optional rear support sensor

It stores the latest sensor readings and builds a `StepAssessment` for the upper control logic.

The current implementation intentionally does **not** use the front balance slider as an input to front stair detection. Front stair judgement is based on sensors only.

## 5. Judgement Conditions Used in `StepDetector`

### 5.1 Edge Detection

The stair edge is treated as detected when:

- the front ultrasonic reading is valid and fresh
- the measured distance is within the stair face detection band
- the front downward reading is valid and fresh
- the front downward sensor reports `drop_detected`

This means the robot has both:

- seen the stair face in front
- observed loss of support below the front wheel area

### 5.2 Ready for Front Climb

The robot is considered ready for front climb when:

- pose is safe
- front ultrasonic reading is valid and fresh
- front downward reading is valid and fresh
- measured distance is smaller than or equal to `READY_TO_CLIMB_DISTANCE_M`
- edge detection is already true

So the front climb condition is:

front distance threshold + front downward edge cue + safe posture

### 5.3 Ready for Middle Transfer

Middle transfer becomes valid when:

- the pose is safe
- the front downward sensor confirms that the front section is already on the step surface

### 5.4 Ready for Rear Transfer

Rear transfer becomes valid when:

- the pose is safe
- the middle support sensor confirms that the middle wheels are already on the new support surface

### 5.5 Step Completed

The full step-climbing cycle is treated as completed when:

- the rear support sensor confirms that the rear support wheel has landed on the new support surface

## 6. Callback Usage

This layer makes active use of callback registration:

- `StepDetector` registers callbacks on the front distance sensor, front downward sensor, middle support sensor, and rear support sensor
- `PoseMonitor` registers a callback on the IMU
- `SafetyManager` stores emergency stop handlers

Therefore this layer satisfies the requirement of using **callback-based notification** rather than using polling only.

## 7. Safety Judgement

`SafetyManager` evaluates:

- custom safety rules
- IMU-based tilt status
- front distance validity
- emergency stop conditions

It can latch faults and call emergency stop handlers so that mechanical modules immediately stop their motion.

## 8. Design Summary

The D layer performs perception and judgement in a structured way:

1. sensors collect raw data
2. callbacks update the latest cached readings
3. `PoseMonitor` estimates safe posture
4. `StepDetector` fuses front distance, downward support status, and pose state
5. `SafetyManager` decides whether motion may continue

This design keeps sensor handling, judgement logic, and safety policy separate from low-level drivers and from the higher-level state machine.

# E. Control & Scheduling Layer – Detailed Module Summary
## Overview
The E Layer (Control & Scheduling Layer) serves as the decision-making and coordination backbone of the three‑stage stair‑climbing robot.
It is responsible for:
State‑driven control – managing the robot’s high‑level climbing state machine.
Motion sequencing – coordinating the front, middle, and rear mechanisms in the correct order.
System integration – aggregating perception (D layer), mechanical abstraction (C layer), and safety logic into a single control loop.
## 1. ClimbingFsm – High‑Level State Machine
### Design Purpose
ClimbingFsm implements the core climbing logic using a finite state machine.
It consumes external completion signals (from the motion coordinator) and safety/perception assessments (from the D layer) to decide when to transition between states.
### Key Design Choice:
The FSM does not contain any timing logic or direct actuator calls. It only evaluates boolean completion flags and perception inputs. This keeps the state logic simple, testable, and independent of hardware timing.
### Safety Integration
If safety_status.level == SafetyLevel::Fault at the beginning of updateState(), the FSM immediately transitions to Fault.
The handleError() method allows external modules (e.g., SafetyManager) to forcibly set the FSM into fault mode with a specific fault code.
## 2. MotionCoordinator – Phase Execution Engine
### Design Purpose
MotionCoordinator translates the abstract states from ClimbingFsm into concrete mechanical actions.
It owns references to the C‑layer modules (FrontSegment, MiddleLiftModule, MiddleDriveModule, RearSupportModule) and calls their phase‑specific methods.
### Key Design Choice:
The coordinator does not implement any state machine logic of its own. It simply executes the action associated with the current state and updates its internal flags. The decision to move to the next state remains with ClimbingFsm.
## 3. RobotController – Top‑Level Orchestrator
### Design Purpose
RobotController is the single entry point for the entire control system. It:
Aggregates all other layers (FSM, coordinator, step detector, pose monitor, safety manager).
Implements the main control loop (update()).
Provides thread‑safe access to the current robot state.
Handles initialization, emergency stops, and system resets.
### Initialization (init())
Resets motion phases and transitions the FSM to Idle.
Performs an initial safety check via SafetyManager.
Populates the initial RobotState snapshot.
If the initial safety check returns a fault, init() returns false, indicating the system is not ready to run.
### Thread Safety
RobotController uses a std::mutex to protect access to the internal robot_state_.
All public methods that read or write state (init(), update(), stopAll(), resetSystem(), state()) are protected by the mutex where necessary.
#  F Module (General Support Layer)
## Module Overview
The F Module serves as the **general support layer** for the entire three‑stage stair‑climbing robot project. It provides cross‑module shared data types, global configuration, utility functions, and logging functionality. This layer offers unified infrastructure for all upper‑level modules (sensors, actuators, state machines, etc.), ensuring code portability and maintainability.

---
## File Structure
include/
├── config.h    # Configuration for global hardware pins, sensor thresholds, motion parameters, etc.
├── types.h     # Definitions of common data types, enumerations, and structs
├── utils.h     # Declarations of utility functions (clamping, filtering, timeout detection)
└── logger.h    # Logging interface declarations

src/
├── utils.cpp   # Implementation of utility functions
└── logger.cpp  # Logging implementation (with timestamp and level output)

---
## File Descriptions
### `config.h`
Stores all hardware‑related constant configurations, including:
- I2C bus, PCA9685 address, PWM frequency
- Pin mapping for motor drivers, servos, and linear actuators
- Threshold and timeout parameters for sensors (ultrasonic, downward‑looking, IMU)
- Geometric dimensions, movement speed, safety limits, etc.

### `types.h`
Defines common data types and enumerations used across the project:
- MotionState: robot state machine states (idle, approaching step, front‑stage climbing, etc.)
- FaultCode: fault code enumeration
- Structs: PoseData (posture), DistanceReading (distance), DownwardReading (downward‑looking), AxisState (linear axis state), etc.
- Callback function types: PoseCallback, DistanceCallback, etc.

### `utils.h` / `utils.cpp`
Provides general utility functions:
- `clamp()`: value clamping
- `lowPassFilter()`: first‑order low‑pass filtering
- `hasTimedOut()` / `isFresh()`: data freshness judgment

### `logger.h` / `logger.cpp`
A simple logging system supporting four levels:
- `Logger::debug()`: debug information
- `Logger::info()`: general information
- `Logger::warn()`: warning
- `Logger::error()`: error

Log output format: `[HH:MM:SS] [LEVEL] message`

## Usage
1. Include header files
Include the corresponding headers in source files that require configuration, types, or utilities:
```cpp
#include "config.h"
#include "types.h"
#include "utils.h"
#include "logger.h"
```

2. Use logging
```cpp
Logger::info("Robot initializing...");
if (error_occurred) {    
Logger::error("Failed to start IMU sensor");
}
```

3. Use utility functions
