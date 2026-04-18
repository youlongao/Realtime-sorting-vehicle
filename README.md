## Team 4 member 
Yi Liu 2983702L
Jiayang Sun 3075065S
Yonghao Zhang 2986320Z
Youlong Ao 3097772A
Zengnan Tang 3091228T

## TikTok Video Links

Three-stage stair-climbing robot-1: https://vm.tiktok.com/ZNRq2LvgB/ 

Three-stage stair-climbing robot-2: https://vm.tiktok.com/ZNRq2rPE9/

Three-stage stair-climbing robot-3: https://vm.tiktok.com/ZNRq2DjgA/

Three-stage climbing robot Complete: https://vm.tiktok.com/ZNRq2NjrW/ 

## Table of Contents

1. Project Introduction
2. Function List
3. Project Management
4. Required Hardware and Wiring Diagram
5. Hardware Design
6. Installation Steps
7. Software Design
8. Code Layering
9. Realtime Design and Analysis
10. Testing Notes
11. Result Analysis
12. Conclusion

# README Plus for the Three-Stage Stair-Climbing Robot (English Version)

## Project Introduction

The design scenario of this project comes from a practical problem in daily life: stairs often limit the movement of mobile robots. In teaching buildings, residential buildings, and indoor places without accessible ramps, ordinary wheeled robots can usually move steadily only on flat ground. Once they encounter steps, it becomes difficult for them to continue moving forward. To address this problem, our group hoped to design a robot that can autonomously complete staged climbing actions in a stair environment, so as to explore the practical application of an embedded control system in complex terrain.

Based on this goal, we designed and implemented a three-stage stair-climbing robot running on the Raspberry Pi Linux platform. The system is developed with C++20 and CMake, and uses libgpiod, I2C, and the MCP23017 GPIO expander to achieve unified control and management of hardware such as motors, linear actuators, ultrasonic sensors, downward/support sensors, and limit switches. The robot is made up of three parts: the front section, middle section, and rear section. When approaching a staircase, the robot first uses forward ranging to identify the position of the step, and then, together with support detection and limit-switch feedback, completes actions such as front lifting, middle following, and rear support transfer one by one under the logic of a state machine, finally achieving continuous stair climbing.

An important feature of this project is that it has relatively strong realtime requirements. When the robot approaches a step, it needs to identify the step riser and the suspended edge in time. During lifting and sliding, it needs to receive limit feedback quickly. When abnormal conditions occur, it must also stop as soon as possible to ensure system safety. Therefore, the control program needs the ability to run continuously, respond to events, and protect against faults. To meet these needs, we adopted an object-oriented C++ structure in software design. The driver layer, sensor layer, mechanism control layer, state machine logic, and safety protection logic are divided into several classes that cooperate with each other while keeping clear responsibilities. This improves the maintainability of the system and also makes later debugging and function expansion easier.

## Function List

The current version already has a complete main control path for single-step stair climbing. After the program starts, it first completes hardware bring-up and the initial posture reset, and then continuously reads the forward ultrasonic sensor, the front downward sensor, the middle support sensor, the rear support sensor, and multiple mechanical limit signals. It judges whether the step has entered the climbable distance range, and then advances the motion in the order of approaching the step, lifting the front section, transferring the middle section, following with the rear section, and ending the cycle. During the control process, the front mechanism is responsible for front-wheel approach and step-up motion, the middle lifting module is responsible for lifting the main body, the middle drive module is responsible for middle-wheel transfer, and the rear support module is responsible for the cooperation between the slide table and the rear lifting section. In the end, the state machine and the coordinator connect these actions into one complete cycle.

In addition to the normal action path, the system also includes a relatively complete set of safety and fault-tolerance behaviors. If the forward distance fails continuously during a critical stage, the controller will trigger a fault state. If a mechanism action does not receive confirmation from sensors or limit switches within the specified time, it will also be judged as a timeout fault. When a termination signal is received, the program uses a unified stop entry to retract all actuators and avoid leaving the robot in an uncontrollable state. These designs make the project closer to a real embedded control system rather than experimental code that works only under ideal conditions.

## Project Management

According to the code structure of the project, we divided the system into six module categories, A-F, where A is the basic driver layer, B is the sensor layer, C is the mechanism abstraction layer, D is the perception and judgement layer, E is the control and scheduling layer, and F is the general support layer and program entry. Since our group has five members, the final task allocation was arranged as A, B, C+D, E, and F. On the one hand, this arrangement ensured that every module had a clear person in charge who could maintain it continuously. On the other hand, it also matched the course requirements for code structure, realtime performance, testing, and engineering management. A and B correspond to the lowest execution and input paths of the system. C+D are responsible for lifting hardware capability into understandable action abstraction and safety judgement. E is responsible for the most important realtime control and state-machine progression. F is responsible for configuration, logging, program entry, and the organization of the README and presentation materials. This way of division reflects good modular design and also makes it easier to clearly explain each member’s contribution to system structure, realtime performance, and engineering management in the final presentation.

Yi Liu is responsible for Layer A, namely the basic driver layer, corresponding to modules such as `motor_driver`, `linear_actuator`, and `mcp23017_driver` in the current repository. This part of the work mainly focuses on the low-level drivers for actuators and expanders, and is responsible for making the front wheels, middle wheels, lifting axes, rear slide table, and the MCP23017 input expander work steadily. It directly determines whether the robot can reliably send motion commands, build the limit-input path, and complete the most basic hardware bring-up. Therefore, it is closely related to the course assessment items concerning class encapsulation, low-level reliability, and hardware implementation quality.

Jiayang Sun is responsible for Layer B, namely the sensor layer, corresponding to modules such as `front_distance_sensor`, `downward_sensor`, `mcp23017_downward_sensor`, `mcp23017_limit_switch`, and `limit_switch`. The main task of this part is to transform real-world distance, contact, drop, and limit states into structured data that can be directly processed by the software, and to provide unified reading, waiting, and callback interfaces for the upper layers. In essence, it builds the system input path, so it is directly related to assessment dimensions such as realtime input handling, reliable data acquisition, and the foundation of event-source design.

Youlong Ao is responsible for Layers C+D, namely the mechanism abstraction layer and the perception and judgement layer, corresponding to modules such as `front_segment`, `middle_lift_module`, `middle_drive_module`, `rear_support_module`, `step_detector`, and `safety_manager`. The focus of this work is not to control one hardware component independently, but to further organize low-level drivers and sensors into an intermediate layer with engineering meaning. For example, logic such as front-section approach to the step, main-body lifting, middle-wheel support transfer, rear-section stabilization, step detection, and emergency-stop judgement all fall within this responsibility range. This division shows well the rationality of module partitioning, fail-safe design, logic clarity, and system maintainability.

Yonghao Zhang is responsible for Layer E, namely the control and scheduling layer, corresponding to `climbing_fsm`, `motion_coordinator`, and `robot_controller`. This layer takes charge of the highest-level control logic of the whole robot. It defines the state machine, arranges the execution order of the front, middle, and rear mechanisms, and enters the fault or stop path in a unified way when abnormal conditions occur. In other words, when the robot approaches a step, when it lifts, when it advances, and when one action cycle ends are all scheduled in this layer. Therefore, it is also the part that best reflects realtime coding, event-driven control, state-machine design, and abnormal-response ability.

Zengnan Tang is responsible for Layer F, namely the general support layer and program entry, corresponding to `config.h`, `types.h`, `hardware_interfaces.h`, `utils`, `logger`, and `main.cpp`. On the one hand, this part is responsible for configuration parameters, shared data structures, interface abstraction, the logging system, and the main program entry. On the other hand, it is also the most suitable part for organizing project documents, maintaining the README, writing build instructions, and integrating the final presentation materials. Since this layer directly affects the maintainability, reproducibility, and completeness of the documentation, it is not only the software support foundation but also an important part of the course presentation and engineering expression.

The `main` function was completed jointly.

In the early stage, all members worked together on GitHub. In the later stage, the group revised the code offline together and debugged it on one member’s computer.

In terms of the development process, the project was more suitable to move forward in the order of “components first, whole machine later.” At the beginning, it was necessary to confirm whether low-level hardware paths such as actuators, sensors, limit switches, and expanders could work independently. Then these inputs and outputs were abstracted into unified software interfaces, further combined into the front, middle, and rear mechanical action modules, and only after that did the project move to state-machine scheduling, whole-machine bring-up, and continuous stair-climbing integration. This process is consistent with the large number of functional test tools currently kept in the repository, and it also helps reduce troubleshooting difficulty. Once a problem appears during whole-machine integration, the team can quickly return to the corresponding module-level test instead of blindly searching for the cause in a huge main program.

## Required Hardware and Wiring Diagram

This section is left blank for now. The current repository has already fixed the pin mappings of Raspberry Pi GPIO, the MCP23017 input expander, the ultrasonic sensor, three downward/support sensors, and each drive channel in `include/config.h`. However, to avoid inconsistency between the document and the actual hardware version, we do not write a fixed hardware list and wiring diagram here in advance. After the final whole-machine wiring version is fully confirmed, the formal module list, GPIO mapping, MCP23017 port allocation, and schematic diagram can be added to this section as a unified reference for reproduction and assessment.

Before the formal wiring diagram is completed, the hardware BOM of this project can first be maintained using the table below. The current table mainly includes the key components that have already appeared clearly in the code or can be directly inferred from it. The model numbers and purchase links are temporarily left as they are and can be completed later after the procurement and assembly plan is fully decided.

| Device Name | Quantity | Model | Purchase Link |
| --- | --- | --- | --- |
| Raspberry Pi main control board | 1 | Raspberry Pi5 | https://www.raspberrypi.com/products/raspberry-pi-5/ |
| Drive motor | 5 | TT Motor | https://amzn.eu/d/0iTeeLt6 |
| Lifting motor | 2 | N20 | https://amzn.eu/d/023zRYXu |
| Motor driver module | 4 | DRV8833 | https://amzn.eu/d/00IRAClV |
| I/O expander | 1 | MCP23017 | https://amzn.eu/d/0cOaLlFO |
| Ultrasonic sensor | 1 | HC-SR04 | https://amzn.eu/d/02XIXrI9 |
| Infrared sensor | 3 | TCRT5000 | https://amzn.eu/d/0f92Bmtv |
| Mechanical limit switch | 6 | D2F-01F | https://amzn.eu/d/04ajv4dc |
| Wooden board | 2 | 300 x 200 x 1.5mm | https://amzn.eu/d/0fxccDur |
| Battery box | 2 | 6V | https://amzn.eu/d/05qSEsOO |
| Dupont wires | 2 | 15cm 30cm | https://amzn.eu/d/08mhdWQe |
| Breadboard | 1 | 1*400 | https://amzn.eu/d/03bcV44T |
| wheels | 6 | 65mm | https://amzn.eu/d/0foCCfac |
| Fully Threaded Rod & Studs | 2 | M5 x 250mm M5-0.8 | https://amzn.eu/d/0598glIo |
| Gear Plastic Connecting Rod | 1 | 0.5 Module | https://amzn.eu/d/06gFpbkb |
| U Type Pulley Roller Wheel | 8 | 5x8.5x19mm | https://amzn.eu/d/0iJI21WH |

## Hardware Design

In hardware, our three-stage stair-climbing robot is not simply a normal wheeled chassis with one lifting mechanism added onto it. Instead, it is designed around the question of “how to transfer the weight of the whole machine step by step onto a higher stair.” The front section is responsible for first identifying the staircase and starting the initial climbing action. The middle section is responsible for main-body lifting and middle-wheel transfer. The rear section is responsible for completing support transition and overall stability in the later stage. This arrangement allows the robot to gradually change its center of gravity and support relationship through several controlled stages, which improves the controllability and safety of the climbing process.

In terms of the layout of sensors and actuators, we use a forward ultrasonic sensor to judge the distance to the step riser, and use the front downward sensor and the middle and rear support sensors to confirm whether each stage has already contacted a new support surface. In addition, mechanical limit switches are used to control the travel distance of the lifting axes and the slide table. The driving part uses DRV8833 to control the wheel sets and linear actuators, while the input expansion part uses MCP23017 to connect multiple limit signals in a unified way. This not only reduces the pressure on Raspberry Pi GPIO usage, but also provides a clearer hardware foundation for a future interrupt-driven input path.

## Installation Steps

This section records the structural assembly process of the current prototype. Since the whole machine is handmade from wooden boards, the materials can first be prepared according to the table below, and then the installation can be completed in the order of the middle section, front section, and rear section. The table below is arranged according to the board sizes and quantities clearly appearing in the current steps, and boards of the same size are combined for easy checking during processing and assembly.

| Wooden Board Size (cm) | Quantity |
| --- | --- |
| 30 × 2.5 | 4 |
| 0.45 × 2.5 | 2 |
| 5 × 2.5 | 2 |
| 6 × 1 | 1 |
| 6 × 2 | 1 |
| 6 × 2.5 | 15 |
| 6 × 3 | 1 |
| 7 × 20 | 2 |
| 10 × 2.5 | 2 |
| 10 × 6 | 1 |
| 12 × 2.5 | 2 |
| 20 × 2.5 | 2 |
| 24 × 2.5 | 2 |

### 1. Building the Middle Section of the Robot

First, take two sets of `30 × 2.5` long wooden boards as the main beams of the middle section. In each set, place two boards in parallel, leaving an installation slot of about `0.5 cm` in the middle, and fix them at the bottom and top respectively with `6 × 2.5` boards. Then insert `0.45 × 2.5` vertical boards into the slots as the installation base for the rear columns, and fix them on both sides with `6 × 2.5` boards to complete one slider structure. Make the other side main beam and slider in the same way.

Glue one TT motor on the top of each main beam, connect the `M5 × 250 mm` threaded rod to the motor shaft, and fix the M5 nut at the center of the slider so that it can move linearly along the threaded rod. Then take one `10 × 6` wooden board and drill one circular hole with a radius of about `0.5 cm` at `1 cm` from each long edge. Pass the two threaded rods of the two main beams through the holes and fix the bottom board at a position about `2.5 cm` above the bottom of the main beams.

Then glue two `10 × 2.5` boards above the main beams to form the main frame of the middle section, and glue two `7 × 20` boards on top as carrying plates for the batteries and control parts. After that, fix the two `6V` battery boxes on the carrying plates and install the reserved breadboard under the battery boxes. Then fix the TT motors on both sides of the bottom board and install the wheels, glue the DRV8833 in the middle of the upper bottom board, and finally install the middle infrared sensor under the center of the bottom board. The main body of the middle section is then complete.

### 2. Building the Front Section of the Robot

Glue one `5 × 2.5` wooden board onto the left and right outer sides of one slider on the middle section as the connection base between the front section and the main beam. Then glue two `6 × 2.5` boards downward at the very front ends of these two boards, and install one TT motor with a wheel at the bottom of each board. Then glue one `6 × 3` board between the two front wheels to form the bottom support of the front section.

Glue one `6 × 2` board above the two `6 × 2.5` vertical boards, and fix the DRV8833 used by the front section at this position. Install one TCRT5000 infrared sensor under the `6 × 3` board between the front wheels, then add one `6 × 1` board at the front end and install one HC-SR04 ultrasonic sensor in the middle for front step distance detection. At this point, the drive and sensing parts of the front section are basically complete.

### 3. Building the Rear Section of the Robot

Take two `12 × 2.5` wooden boards and glue them in parallel to the left and right sides of the other slider on the main beam. On one side, glue an extra `12 cm` rack. Glue one `6 × 2.5` board at the rear end, then extend two `24 × 2.5` wooden strips backward and install two wheels at the bottom of these strips to form the basic support structure of the rear section.

Then take two `20 × 2.5` boards as the vertical plates of the rear pulley mechanism. Glue one `6 × 2.5` board at the top of each as the wide-side support for the pulleys, then leave about `2.5 cm` downward and add another `6 × 2.5` board on each side for pulley fixing. Next, drill two holes in the top `6 × 2.5` board at positions `0.5 cm` from the bottom edge and `0.5 cm` from both sides, and drill two corresponding holes in the lower `6 × 2.5` board at positions `0.5 cm` from the top edge and `0.5 cm` from both sides. Pass four shafts through the corresponding holes and install eight `6.5 × 3` pulleys on the shafts, with four pulleys above and four below matched one by one, forming a pulley mechanism that can clamp the beam and guide sliding.

Install one TT motor at the exact center of the `6 × 2.5` board on the rack side, making sure that the motor gear meshes with the rack. Finally, install one TCRT5000 infrared sensor in the middle under the two `20 × 2.5` vertical boards to detect the support state of the rear section. The rear section structure is then completed.

## Software Design

At the software design level, this project uses a layered embedded control structure. First, the lowest layer encapsulates drive outputs and sensor inputs into independent classes. Then these classes are combined into modules for mechanical actions. After that, perception fusion, the state machine, and the motion coordinator are used to build the whole-machine control logic step by step. The advantage of this design is that each layer only deals with its own task: the driver layer is only responsible for execution, the sensor layer is only responsible for providing structured readings, the mechanical modules are only responsible for expressing action meaning, the judgement layer is only responsible for providing control conditions, and the overall controller gathers these results into one complete operating path. This is more helpful for later code modification and maintenance.

Threads, callbacks, state caching, timeout protection, and event binding are used together at different levels. On the one hand, this makes the code boundaries clearer and helps debugging and maintenance. On the other hand, it also makes the whole-machine behavior easier to explain, because each action progression, each abnormal stop, and each kind of input event can be traced back to the corresponding layer module and judgement condition, rather than being hidden in mixed process control.

## Software Implementation Logic

After the robot starts, the system first enters the initialization and reset stage. At this stage, the program checks in sequence whether each hardware module is working normally, including the front and middle drive wheels, the forward ultrasonic sensor, the three downward infrared sensors, the MCP23017 expansion board, the limit switches, and the front lifting module, rear lifting module, and rear slide module. After the hardware check is completed, the system performs the initial posture reset of the robot body: the front lifting module returns to the lower limit, and the rear lifting module returns to the upper limit. This posture is regarded as the safe initial state before the robot starts climbing stairs, and it also provides a unified starting point for the following actions.

After initialization is completed, the robot begins the process of climbing one step. It first enters the approach stage. The front drive wheels and middle drive wheels move the whole robot forward together, and the forward ultrasonic sensor continuously measures the distance between the robot and the step riser. When the measured distance has not yet reached the set threshold, the robot keeps moving forward. When the distance enters the target range, it means the robot has reached a suitable position. At this time, the front and middle drive wheels stop, preparing for the following climbing actions.

Before the formal lifting begins, the system first performs the backward movement of the rear slide table. The rear slide table moves backward until the rear limit switch is triggered. The purpose of this action is to shift the center of gravity of the whole machine backward, thereby improving stability during front-section lifting. After the rear slide table completes its backward movement, the system enters the front-section lifting stage.

In the front-section lifting stage, the front wheels and middle wheels remain still, and the front lifting module moves upward to lift the front part of the robot body. During the lifting process, the forward ultrasonic sensor is used to help judge whether the front section has already passed over the step riser and whether enough space has been obtained for the next forward movement. When the system confirms that the front section has reached a suitable height, the lifting action ends and the system enters the front-section landing stage.

In the front-section landing stage, the front drive wheels and middle drive wheels rotate forward together at a low speed, sending the front section onto the step surface. When the front downward infrared sensor detects the support surface, the system does not stop immediately. Instead, it lets the drive wheels continue to move forward for a short period so that the front section can land on the step more steadily. After that, the drive wheels stop, and the front lifting module continues to hold the lifting action for a short extra time to ensure that the front section has completely passed the step edge and formed stable support. At this point, the front section has successfully climbed onto the step.

After the front section has established support, the system enters the middle-section lifting and landing stage. The purpose of this stage is to raise the middle body to the step height and let the middle drive wheels land smoothly on the step surface. First, the front and middle drive wheels remain stopped, the front lifting module starts to move downward, and the rear lifting module also joins the downward action to complete the height adjustment of the middle section. When the front lifting module triggers the lower limit, the system considers that the middle section has reached the target height. Then the front drive wheels slowly drive the robot forward so that the middle section gradually approaches and lands on the step surface. When the middle downward infrared sensor detects that the middle section has contacted the new support surface while the front section is still keeping stable support, the system judges that middle landing has been completed. To further improve stability, the front and middle drive wheels continue moving forward for about `0.1 s` and then stop. At this point, both the front and middle sections of the robot are already on the step.

After the middle section has completed landing, the system needs to prepare for the climbing of the rear section, so it enters the forward-movement stage of the rear slide table. The whole robot remains still, and the rear slide table moves forward from the rear-limit direction until the front limit switch is triggered. The function of this process is to shift the center of gravity of the whole machine forward again and provide a more suitable force condition for rear-section lifting. After the rear slide table finishes moving forward, the system enters the rear-section lifting stage.

In the rear-section lifting stage, the rear lifting module moves upward to lift the rear part of the robot body. Since the front and middle sections have already formed stable support on the step surface at this time, lifting the rear section can separate the rear wheels or rear support part from the original lower ground. The rear lifting module continues moving until the upper limit switch of the rear section is triggered, and the system uses this as the sign that rear-section lifting is complete. It then enters the rear-section landing stage. At this stage, the front and middle drive wheels rotate forward together, driving the whole robot to continue moving so that the rear section gradually moves onto the step surface. When the rear downward infrared sensor detects that the rear section has contacted the new support surface, it means that all three sections of the whole robot have completed the climb of one step. At this time, the front and middle drive wheels stop, and the one-step stair-climbing process is completed.

After one-step stair climbing is completed, the system enters the cycle reset stage. At this time, the rear slide table moves back to the rear limit again, restoring the robot to the center-of-gravity preparation state required before the next climb. After the reset is completed, the system enters the step-approach stage again and continues to search for the next step. If the robot is set to continuous stair-climbing mode, it will keep repeating the above process until manual stop or fault detection by the system.

To ensure the reliability of the whole process, the system also includes state confirmation and safety protection mechanisms. In this project, key actions are not completed simply by fixed timing. Instead, they mainly depend on sensors and limit switches for confirmation. For example, step approach depends on forward ultrasonic ranging, front, middle, and rear landing depend on downward infrared sensors to judge the support state, and the end positions of the slide table and lifting modules depend on mechanical limit switches for confirmation. If one action does not receive valid feedback within the specified time, the system triggers timeout protection, immediately stops all motors, and enters a fault state, thus avoiding motor stall, mechanism overrun, or the robot continuing dangerous actions.

## Code Layering

This part is developed according to the real implementation in the current repository and tries to keep a structure close to the reference README. The whole project can be understood as a six-layer structure from bottom to top: Layer A is responsible for the lowest-level hardware drivers, Layer B is responsible for sensors and input events, Layer C organizes drivers into mechanical action units, Layer D fuses multiple perception results into judgement conclusions, Layer E is responsible for the state machine, motion coordination, and whole-machine scheduling, and Layer F provides all modules with unified data types, configuration, interface abstraction, logging, and utility functions. The meaning of this division is not only to make the directory look neat, but also to make it clear why a function belongs here, so that later maintenance, assessment explanation, and oral defense become more convincing.

### Layer A: Basic Driver Layer (Driver Layer)

#### 1. Goal of This Layer

Layer A corresponds to the control part closest to the hardware. It deals directly with Raspberry Pi GPIO, the I2C bus, the DRV8833 driver, and the MCP23017 expander, and translates the action commands given by the upper layer into specific control that can be applied to pins and registers. In the current version, this layer no longer uses the old organization based on PCA9685 and servo drivers. Instead, it is implemented around direct GPIO motor control, linear actuator control, and MCP23017 input expansion. Therefore, Layer A is better understood as a “low-level hardware driver abstraction layer.” Its duty is to hide details such as device initialization, direction control, speed normalization, limit linkage, and I2C interrupt configuration, so that upper-layer code can work only with unified interfaces.

#### 2. Files in This Layer

```text
include/motor_driver.h / src/motor_driver.cpp
include/linear_actuator.h / src/linear_actuator.cpp
include/mcp23017_driver.h / src/mcp23017_driver.cpp
```

#### 3. Driver Modules and Responsibilities

##### 3.1 `MotorDriver`

`MotorDriver` is responsible for the lowest-level execution control of dual-wheel differential drive. In the current repository, it is used for the front-wheel section and the middle-wheel section. Each object holds a fixed group of GPIO definitions, completes line-request initialization through `start()`, and then controls left and right wheel actions through `setNormalizedSpeed()`, `forward()`, `backward()`, `stop()`, and `brake()`. The key design here is not simply to “make the wheels spin,” but to normalize the input speed into the interval `[-1.0, 1.0]`, and then let `applyMotorCommand()` decide the output direction and state of each path such as IN1/IN2 or IN3/IN4. In this way, upper mechanical modules do not need to know which BCM pin is connected to which driver path. They only need to express “forward,” “backward,” or “run the left and right wheels at a certain ratio.”

From the perspective of responsibility boundaries, `MotorDriver` does not take part in state-machine judgement, nor does it decide when the robot should stop. It only cares about how to send reliable motor commands at the current moment. For this reason, braking, stopping, initialization, and command application are all concentrated in this layer instead of being scattered across different action modules. This helps guarantee consistent actuator behavior.

##### 3.2 `LinearActuator`

`LinearActuator` encapsulates two kinds of linear actuators: the front lifting axis, the rear slide table, and the rear lifting axis. Its core public interfaces include `start()`, `extend()`, `retract()`, `moveToPosition()`, `moveNormalized()`, `holdPosition()`, `stop()`, and `getAxisState()`. Compared with ordinary open-loop motor control, this class already has basic axis-state management ability. It maintains estimated position, motion state, upper and lower limit states, and timestamps, and cooperates with the `ILimitSwitch` interface during movement to prevent the mechanism from continuing to push toward its extreme direction.

An important feature of this layer is that it not only keeps the direct control style of “extend forward / retract backward,” but also provides upper layers with more suitable meanings such as “move to target height” and “hold current position.” In other words, although Layer A is still the driver layer, it has already begun to offer a more stable and safer basic execution unit for upper-layer actions.

##### 3.3 `Mcp23017Driver`

`Mcp23017Driver` is a very important low-level driver in the current repository. It is responsible for opening the I2C device, reading and writing registers, configuring input ports, reading the status of the whole port, and creating the interrupt thread and dispatching callbacks. The most important public interfaces are `start()`, `configureInput()`, `readPin()`, `readAllPins()`, `startInterrupts()`, `registerPinChangeCallback()`, and `unregisterPinChangeCallback()`. In the current main control path, six mechanical limit switches are all connected through it. Therefore, it is not only an auxiliary tool for one sensor, but also the low-level infrastructure of the whole event-driven input path.

Compared with a simple implementation that polls GPIO levels, `Mcp23017Driver` uses `poll()` to wait for the INTA and INTB interrupt signals. After an event arrives, it reads GPIOA/GPIOB to clear the interrupt latch and then dispatches the changed result to registered objects. This means that the upper MCP-based sensor and limit-switch modules can share one common low-level event source, which is more in line with the event-driven mode expected in realtime embedded code.

#### 4. Design Features of This Layer

The design focus of Layer A is hardware isolation, interface unification, and action safety. Both `MotorDriver` and `LinearActuator` use class encapsulation to package pins and device states inside objects, so upper-layer logic does not directly operate bare GPIO. `Mcp23017Driver` centralizes I2C registers and interrupt handling in one place, avoiding repeated maintenance of expander access logic by multiple modules. At the same time, the linkage between linear actuators and limit switches also reflects this layer’s concern for safety: some protections are better completed at the lowest layer instead of waiting for the upper state machine to discover the abnormal condition and then trying to fix it.

#### 5. Summary of This Layer

If the whole system is compared to a control chain, Layer A is the lowest execution base. It does not decide what the robot should do next, but it determines whether what the upper layer wants to do can be applied to real hardware in a stable and safe way. Therefore, it is the foundation for the later perception, judgement, and scheduling layers.

### Layer B: Sensor Layer (Sensor Layer)

#### 1. Goal of This Layer

Layer B is responsible for turning raw hardware input into structured readings that the upper layers can use. In the current implementation, it no longer focuses on the IMU or posture fusion mentioned in the old README. Instead, it builds an input system around the existing real hardware path for forward ranging, downward step/drop detection, support confirmation, and mechanical limit detection. It needs not only to acquire signals, but also to add timestamps, validity flags, and unified access interfaces to these signals, so that later perception fusion and control decisions can be completed without touching low-level details.

The path actually used in the current main program is as follows: the forward ultrasonic data are provided by `FrontDistanceSensor`; the three downward/support sensors in the front, middle, and rear use the Raspberry Pi direct-connection version `DownwardSensor` by default; and the six mechanical limit switches use `Mcp23017LimitSwitch` through the MCP23017 interrupt path. `Mcp23017DownwardSensor` and the direct GPIO version `LimitSwitch` are still kept in the repository, mainly for compatibility, replacement, and single-function debugging.

#### 2. Files in This Layer

```text
include/front_distance_sensor.h / src/front_distance_sensor.cpp
include/downward_sensor.h / src/downward_sensor.cpp
include/mcp23017_downward_sensor.h / src/mcp23017_downward_sensor.cpp
include/mcp23017_limit_switch.h / src/mcp23017_limit_switch.cpp
include/limit_switch.h / src/limit_switch.cpp
```

#### 3. Sensor Modules and Responsibilities

##### 3.1 `FrontDistanceSensor`

`FrontDistanceSensor` is responsible for forward ultrasonic ranging. It creates a working thread through `start()`, completes one full trigger-wait-distance conversion process through `readBlocking()`, and then provides the latest reading and update notification through `latest()` and `setCallback()`. Its main role is to tell the system whether a step riser has appeared ahead and whether the robot has reached the distance range that allows the front section to start climbing.

Although this module internally uses a blocking measurement process, it does not place the measurement logic directly into the main control loop. Instead, it runs periodically in an independent thread and then sends the result upward through a callback. Therefore, from the perspective of the whole system, it still follows the design idea of event-driven and asynchronous perception.

##### 3.2 `DownwardSensor`

`DownwardSensor` is the Raspberry Pi GPIO direct-connection version of the downward/support sensor wrapper. It senses input changes through an edge-event waiting mechanism and converts the raw electrical level into `DownwardReading`, which contains `on_step_surface`, `drop_detected`, `edge_detected`, timestamps, and validity states. In the current main program, front landing detection, middle support confirmation, and rear support confirmation all rely on objects of this class by default, but they are instantiated on different physical channels.

Its value is not simply in reading a digital signal, but in lifting “sensor high/low level” into semantic states that can directly take part in control judgement, such as “whether the front wheels are on the step,” “whether the middle wheels have completed support transfer,” and “whether the rear support wheels have landed on the new step.”

##### 3.3 `Mcp23017DownwardSensor`

`Mcp23017DownwardSensor` has a function similar to `DownwardSensor`, but uses a different input path. It depends on `Mcp23017Driver` to register pin-change callbacks, updates its own cache when an expander interrupt event arrives, and also provides `latest()`, `waitForEdge()`, and `setCallback()` interfaces. The current main program does not enable this version by default, but it is still meaningful in the repository because it preserves another possible input topology and is convenient for verifying whether the MCP path is available together with related test tools.

##### 3.4 `Mcp23017LimitSwitch`

`Mcp23017LimitSwitch` is the limit-input implementation used by the whole machine by default. It is responsible for reading the six mechanical limit switches and providing interfaces such as `latestState()`, `waitForTrigger()`, `setCallback()`, `isTriggered()`, `isUpperLimit()`, and `isLowerLimit()`. During startup, it first synchronously reads the current pin state once to ensure that the initial value is valid, and then hands all later changes to the interrupt callback path of MCP23017. This allows limit signals to take part more quickly in startup reset, action termination, and fault judgement, without depending on fixed-period polling.

##### 3.5 `LimitSwitch`

`LimitSwitch` is the direct GPIO version of the limit-switch wrapper. It keeps an upper-layer interface similar to `Mcp23017LimitSwitch`, but uses the Raspberry Pi GPIO edge-event thread underneath. The current main controller does not use this path by default, but it still has value as a compatible implementation because it ensures that the upper layer only depends on the `ILimitSwitch` abstraction rather than a specific hardware connection method.

#### 4. Design Features of This Layer

The most important design choice in Layer B is to separate “how data are acquired” from “how upper layers access them.” No matter whether the input comes from the ultrasonic sensor, Raspberry Pi GPIO, or the MCP23017 interrupt path, what the upper layer finally gets is a structure with timestamps and validity bits, together with unified `latest()`, waiting functions, and callback interfaces. The advantage of this is clear: the perception-fusion module only needs to care about the meaning of the data and does not need to know whether the data come from a blocking measurement thread, a GPIO edge event, or an I2C-expander interrupt.

#### 5. Summary of This Layer

If Layer A answers the question of “how to control hardware,” then Layer B answers the question of “how to read hardware states back reliably.” What it provides upward is organized and semantic sensor input rather than raw level flow, so it is the basis on which the later judgement layer can be built.

### Layer C: Mechanical Abstraction Layer (Mechanical Abstraction Layer)

#### 1. Goal of This Layer

The duty of Layer C is to reorganize low-level actuators into function modules for mechanical actions. At this layer, the code no longer focuses on a GPIO combination or the rotation direction of a certain motor, but on processes that directly match the real actions of the robot, such as “front section approaching the step,” “main body lifting,” “middle wheel transfer,” and “rear support landing.” The meaning of this design is to separate complicated hardware actions from the state machine, so that the state machine only decides the stage, while the detailed action is executed by mechanism modules with clear meanings.

#### 2. Files in This Layer

```text
include/front_segment.h / src/front_segment.cpp
include/middle_lift_module.h / src/middle_lift_module.cpp
include/middle_drive_module.h / src/middle_drive_module.cpp
include/rear_support_module.h / src/rear_support_module.cpp
```

#### 3. Mechanism Modules and Responsibilities

##### 3.1 `FrontSegment`

`FrontSegment` represents the front mechanism. It is a combination of front-wheel drive, forward ranging, the front lifting axis, and the front landing-confirmation logic. Its public interfaces include `approachStep()`, `approachAssistSpeed()`, `liftFrontToStep()`, `liftFrontUntilClearance()`, `continueFrontLift()`, `placeFrontOnStep()`, `isSurfaceConfirmed()`, and `stopFrontSegment()`. In other words, it is not just “a helper for front-wheel driving,” but a complete package of the whole front-stage action, from approaching the step to placing the front wheels onto the higher step.

From the perspective of layering, the meaning of `FrontSegment` is that it organizes multiple low-level objects into one unit that can express a mechanical stage. The upper state machine does not need to know how forward distance is measured, in which direction the lifting axis should move, or when the front landing-confirmation function returns true. It only needs to know “execute front-section approach,” “execute front-section lifting,” and “execute front-section stable landing.”

##### 3.2 `MiddleLiftModule`

`MiddleLiftModule` abstracts the first lifting mechanism, namely the vertical movement ability of the main body relative to the middle drive wheels. It provides interfaces such as `raiseBody()`, `lowerBody()`, `lowerUntilLowerLimit()`, `moveToHeight()`, and `holdPosition()`. In the control flow, this module plays the role of “main-body posture adjustment.” After the front wheels have climbed, it is necessary to use it to continue lifting or lowering the body, so as to create conditions for middle-wheel transfer and rear support.

It can also receive a callback indicating that “middle support has been confirmed,” allowing the module to adjust control behavior according to support conditions during motion. This shows that although Layer C mainly focuses on mechanical actions, it has already begun to cooperate with Layers B and D through abstract callbacks rather than existing in isolation.

##### 3.3 `MiddleDriveModule`

`MiddleDriveModule` corresponds to the middle drive-wheel module. Its main interfaces are `advanceToStep()`, `driveForward()`, `isSupportConfirmed()`, and `holdPosition()`. Its function is very clear: after the front section has established stable support, it lets the middle wheels move forward until the middle support sensor confirms that new grounding or support transfer has been completed.

There is an important structural feature here: middle-wheel transfer does not move forward blindly in open loop according to time, but depends on support-confirmation callbacks to judge whether the action is complete. Therefore, although `MiddleDriveModule` belongs to the mechanical abstraction layer by name, it naturally has the closed-loop property of “execute action until the condition is satisfied.”

##### 3.4 `RearSupportModule`

`RearSupportModule` integrates the rear slide table, the rear lifting axis, and the rear support-confirmation logic into one rear support module. Its public interfaces include `transferSupportToStep()`, `moveSlideBackwardUntilLimit()`, `moveSlideForwardUntilLimit()`, `liftRearUntilUpperLimit()`, `lowerRearUntilLowerLimit()`, `isSupportConfirmed()`, and `stabilizeSupport()`. It covers both the posture coordination of the rear part during middle-section transfer and the final rear-support transfer action.

Like the front and middle modules, its value is not in driving a single actuator independently, but in bringing multiple actuators and confirmation conditions together into a higher-level mechanical action description. For the scheduling layer, rear support only needs to be treated as one action unit that can “slide forward, lower, confirm stable landing, and finally keep stable,” rather than requiring knowledge of which linear axis is called inside.

#### 4. Relationships Among the Modules

From the perspective of actual cooperation, the four modules in Layer C exactly cover the main action chain for the robot to climb one step. `FrontSegment` is responsible for making the front wheels approach and pass the step edge, `MiddleLiftModule` adjusts the body height at key stages, `MiddleDriveModule` lets the middle wheels complete support transfer, and `RearSupportModule` is responsible for rear following and final stabilization. This division is basically consistent with the structure of the real mechanism, and it also allows the scheduling logic in Layer E to be developed directly according to the real mechanical order, rather than having to assemble low-level commands inside the state machine.

#### 5. Summary of This Layer

Layer C is the key step that lifts a “set of drivers” into “mechanical action units.” Without this layer, the controller would have to directly operate many low-level objects. With this layer, the state machine and coordinator can organize the code around the real action semantics of the robot.

### Layer D: Perception and Judgement Layer (Perception and Judgement Layer)

#### 1. Goal of This Layer

The duty of Layer D is to further fuse the multiple readings coming from Layer B into judgement results that can be directly used by the controller. In the current project, the key issue is not general environment understanding, but making several high-value judgements around the stair-climbing action itself: whether a recognizable step riser has appeared ahead, whether the front section has already crossed the edge and landed on the new step surface, whether the middle wheels and rear support have completed support transfer, and whether there is any state that should immediately trigger a fault or emergency stop. For this reason, Layer D currently centers on two core modules, `StepDetector` and `SafetyManager`, rather than following the reference README with a more complete combination including IMU and posture monitoring.

#### 2. Files in This Layer

```text
include/step_detector.h / src/step_detector.cpp
include/safety_manager.h / src/safety_manager.cpp
```

#### 3. Core Modules and Responsibilities

##### 3.1 `StepDetector`

`StepDetector` is the most important perception-fusion module in the current repository. It holds interface objects for forward ranging, the front downward sensor, middle support, and rear support, and actively registers callbacks for these objects during construction. Whenever any one of these data updates, it refreshes its internal cache and, when necessary, notifies the upper controller through `setUpdateCallback()`. Externally, it provides `detectStepEdge()`, `detectStepSurface()`, `isReadyForClimb()`, `isStepCompleted()`, `latestAssessment()`, and the internal `buildAssessmentLocked()` logic, finally generating a unified `StepAssessment`.

This design is very suitable to be presented as a highlight in the README because it clearly shows that there is still a fusion-logic layer between “raw data acquisition” and “control judgement.” The state machine does not directly read four or five sensors by itself. Instead, it first gets a structured judgement result from `StepDetector`, and then decides whether to advance the state.

##### 3.2 `SafetyManager`

`SafetyManager` is responsible for centralized fault and emergency-stop handling. Its core public interfaces include `addRule()`, `addEmergencyStopHandler()`, `checkAllSafetyConditions()`, `emergencyStop()`, `clearFault()`, and `currentStatus()`. In the current whole-machine assembly process, the controller registers rules such as continuous forward-distance failure and action timeout, and also registers the stop behavior of multiple mechanical modules as unified emergency-stop handlers. In this way, when any one rule judges that a fault has occurred, the system can immediately enter a consistent stop path without requiring each module to invent its own error-handling method.

#### 4. Judgement Logic of `StepDetector`

The judgements made by `StepDetector` are not arbitrary. They are built around the key points of the stair-climbing action. For the step edge ahead to be judged as usable, the forward ultrasonic reading must be valid and within the set riser-distance range, while at the same time the front downward sensor must report that the front part has shown a drop or edge feature. Only when the two conditions of “seeing a step ahead” and “the front wheel area is approaching an edge” are both satisfied does the system believe that the robot is not facing a normal obstacle, but has truly entered a step-edge state that can prepare for climbing.

Then, the judgement for whether the front section is ready to start lifting further requires that the distance has entered an even closer threshold range and that edge detection has already been established. Whether the current cycle is completed depends on the support-confirmation signals of the middle and rear sections progressing in order: only after front landing is established does middle transfer become meaningful; only after middle landing is established is rear transfer allowed; and only when rear support is finally confirmed is the whole one-step climb regarded as completed. This kind of sequential judgement makes every step of the controller’s progression supported by real sensor evidence rather than moving forward only according to estimated time.

#### 5. Design Features of This Layer

The value of Layer D lies in separating “perception updates” from “control-usable judgements.” Layer B is responsible for collecting data, while Layer D turns these data into control conditions, fault states, and structured assessment results. In this way, the state machine in Layer E does not have to combine many sensor thresholds by itself, and the safety strategy does not need to be scattered across different action modules.

#### 6. Summary of This Layer

From the perspective of the whole architecture, Layer D is the bridge connecting perception input and control scheduling. It lets the system not only “receive sensor data,” but truly form the judgement result of “whether the next action can be executed now,” which is also one important reason why the whole project can show realtime control characteristics.

### Layer E: Control and Scheduling Layer (Control and Scheduling Layer)

#### 1. Goal of This Layer

Layer E is the core of decision-making and scheduling in the whole system. The problems it needs to solve include: which climbing stage the robot is currently in, which mechanism module should be called after entering this stage, whether the action has already been completed, whether a timeout or fault has occurred, and when the whole robot should end operation. Similar to the way it is written in the reference README, this layer is best understood as a combination of “state machine + execution coordinator + overall controller,” except that the current implementation is more closely connected to the existing real hardware path.

#### 2. Files in This Layer

```text
include/climbing_fsm.h / src/climbing_fsm.cpp
include/motion_coordinator.h / src/motion_coordinator.cpp
include/robot_controller.h / src/robot_controller.cpp
```

#### 3. Control Modules and Responsibilities

##### 3.1 `ClimbingFsm`

`ClimbingFsm` implements the high-level finite state machine. It provides `updateState()`, `transitionTo()`, `handleError()`, and `getCurrentState()` externally, and is responsible for switching among states such as `Idle`, `ApproachingStep`, `FrontLift`, `MiddleDriveToFrontLanding`, `RearLift`, `Completed`, and `Fault`. The greatest advantage of this class is that it does not directly drive actuators, nor does it maintain action timers by itself. Instead, it only decides “what the next state is” based on safety state and stage-completion flags. This keeps the state definition itself clear and also makes it easier to explain the basis of state transition in the document.

##### 3.2 `MotionCoordinator`

`MotionCoordinator` is responsible for translating the abstract state given by the state machine into concrete mechanical actions. It holds references to the mechanism modules in Layer C, calls the corresponding mechanism methods in different states through `executeState()`, and manages phase entry, phase completion, and timeout information using `enterState()`, `resetPhases()`, `isPhaseComplete()`, `hasStateTimeout()`, and `stateTimeoutMessage()`. It does not decide whether the next state should be entered, but it decides how the action corresponding to the current state should really be executed.

This point is worth highlighting in the architecture description: the state machine is responsible for “decision,” while the coordinator is responsible for “execution,” and the two are not mixed together. The direct benefit is that state-transition logic and action details do not become tangled.

##### 3.3 `RobotController`

`RobotController` is the only control entry of the whole machine. It gathers the state machine, coordinator, `StepDetector`, and `SafetyManager`, and uses `init()`, `bindEventSources()`, `update()`, `waitUntilFinished()`, `requestStop()`, `stopAll()`, and `resetSystem()` to connect the whole system into one complete operating chain. During initialization, it completes the initial safety check and event binding. During operation, it drives the control loop according to perception updates. When completion or fault occurs, it converges to a unified stop path.

From the perspective of concurrency safety, `RobotController` also protects the internal `RobotState` through a mutex and condition variable. This allows the whole-machine state to be updated safely and also to be waited for and read when needed. For a project that emphasizes realtime event-driven behavior, this is more convincing than a simple `while` loop control.

#### 4. Summary of This Layer

If Layer D is responsible for judging the current world state, then Layer E is responsible for truly driving the robot forward according to these judgements. It is both the organizer of the whole-machine behavior and the closing layer for fault handling and operation ending, so it plays a central role in the whole project structure.

### Layer F: General Support Layer (General Support Layer)

#### 1. Goal of This Layer

Layer F does not directly drive motors and does not directly take part in state transition, but it provides unified infrastructure for the whole project. Without this layer, upper modules would have to maintain their own configuration constants, data structures, interface definitions, log formats, and utility functions, which often makes the code hard to maintain, scatters parameters everywhere, and causes inconsistent interface styles. The Layer F in the current repository exists precisely to solve these problems.

#### 2. Files in This Layer

```text
include/config.h
include/types.h
include/hardware_interfaces.h
include/logger.h / src/logger.cpp
include/utils.h / src/utils.cpp
```

#### 3. Description of Public Files

##### 3.1 `config.h`

`config.h` is the static configuration center of the whole project. It centrally defines GPIO pins, the I2C bus number, MCP23017 pin mapping, ultrasonic thresholds, the data-freshness window, geometric dimensions, motion speeds, action timeouts, and safety parameters. For a project directly facing physical hardware like this one, scattering these constants across multiple source files would make debugging very difficult, so centralized configuration is very necessary.

##### 3.2 `types.h`

`types.h` defines shared data structures and enumerations across modules, including `MotionState`, `FaultCode`, `DistanceReading`, `DownwardReading`, `AxisState`, `LimitSwitchState`, `StepAssessment`, `SafetyStatus`, and `RobotState`. These types are not only tools for convenient coding, but also the basis of semantic consistency across the whole system. Because these structures are uniformly defined here, each layer can exchange states steadily without confusion caused by different modules defining their own local structures.

##### 3.3 `hardware_interfaces.h`

`hardware_interfaces.h` defines abstract interfaces such as `IDriveSection`, `ILinearAxis`, `IFrontDistanceSensor`, `IDownwardSensor`, and `ILimitSwitch`. This is one part of the current project that is worth emphasizing, because it separates “what abilities the mechanical modules depend on” from “what concrete hardware implementation is used.” It is also because of this interface layer that the mechanical modules in Layer C can be reused on different sensors or different input paths without being directly tied to one low-level implementation class.

##### 3.4 `logger.h` / `logger.cpp`

The logging module provides a unified `debug()`, `info()`, `warn()`, `error()`, and low-level `log()` interface for the whole project. For a robot project like this, logs are not only used to print debug information. They are also an important means for runtime tracing, startup failure locating, action-stage confirmation, and fault analysis. Therefore, building logging as a unified interface rather than using `std::cout` randomly in different source files is a more mature approach and also easier to present.

##### 3.5 `utils.h` / `utils.cpp`

The `utils` module provides a group of lightweight but frequently used helper functions, such as numerical clamping, low-pass filtering, timeout judgement, and data-freshness checking. If these functions are not collected in one place, they are often reimplemented repeatedly in different files, which eventually introduces style and behavior differences. Putting them together in Layer F helps the whole project stay consistent in details.

#### 4. Summary of This Layer

Although Layer F does not directly take part in the stair-climbing action, it determines whether the whole project has clear data boundaries, a unified interface style, and maintainable parameter organization. From the point of view of course assessment, this layer corresponds exactly to the requirements for code structure, encapsulation, and maintainability, and it is also an important reason why the coding style of the current project can remain consistent.

## Realtime Design and Analysis

The reason this project can be considered a realtime embedded application is not simply because it “runs fast,” but because the code is organized around the response time to physical events. When the robot approaches a step, the forward ultrasonic sensor needs to update the distance continuously within a short period. When the front wheel leaves the support surface, the downward sensor needs to report the edge or drop state as quickly as possible. When the lifting mechanism reaches an extreme position, the limit event must immediately take part in control decision-making. The implementation in the repository does not put all of these processes into one long loop with sequential `sleep` and polling. Instead, it chooses a combination of threads, callbacks, condition variables, `poll()`, `signalfd`, and interrupt paths according to the characteristics of different devices.

More specifically, the forward ultrasonic module `FrontDistanceSensor` samples periodically in an independent working thread and sends each new reading upward through a callback. `DownwardSensor` and the direct version of `LimitSwitch` use the edge-event waiting mechanism of libgpiod to block and wait for GPIO changes. When an event arrives, the thread wakes up, refreshes the cache, and notifies the upper layer. `Mcp23017Driver` separately maintains an interrupt thread and uses `poll()` to wait for the `INTA/INTB` trigger of the expander, and then dispatches port changes to registered callback objects. Therefore, the MCP23017 input path itself is also event-driven rather than based on timed scanning. The direct benefit of this method is that the threads stay in the waiting state most of the time and wake up only when real data or events arrive. This reduces meaningless CPU consumption and also lowers the risk of missing key state changes because the polling interval is too large.

At a higher level, `StepDetector` collects callbacks from multiple sensors into its internal cache and triggers its own update callback after data refresh. During initialization, `RobotController` binds this update callback to `update()`, so the controller can advance the state together with perception events rather than relying on a large fixed-period polling main loop. At the same time, `main.cpp` uses `signalfd` together with `poll()` in both startup reset and runtime stages to process termination signals, allowing the system to interrupt waiting in time after receiving an external stop request and enter the unified stop path. The project also makes wide use of `steady_clock` timestamps, data-freshness checking, and explicit timeout windows, for example in sensor-failure judgement, landing-confirmation timeout, lifting timeout, and startup-reset timeout. All of these give the control logic clear and explainable time boundaries instead of waiting forever for a condition to happen by chance.

If the terminology is distinguished more strictly, this system belongs to a soft realtime control implementation running in Linux user space rather than a hard realtime system that requires a dedicated RTOS. However, for the application scenario of this project, it already satisfies the realtime characteristics emphasized by the course: inputs come from real physical signals, event handling depends on thread wake-up, callbacks, and interrupt-driven paths, action execution is constrained by explicit timing requirements and safety protection, and the program changes its internal state and execution path in time when the external state changes. Therefore, it is reasonable and well supported to define it as a realtime embedded application for real robot actions.

## Testing Notes

The current repository adopts a component-level functional testing approach rather than placing all verification in the whole-machine main program at one time. A more reasonable order is to first confirm actuator direction and basic actions, then confirm forward ranging, downward/support sensors, mechanical limit switches, and interrupt paths, and finally carry out whole-machine integration testing. The reason is simple: the three-stage stair-climbing robot involves multiple levels, including drive output, sensor input, limit protection, and state scheduling. If the main program alone is run at the beginning, once a problem appears, it is often difficult to quickly judge whether the cause is reversed drive direction, an incorrect threshold, no sensor response, or the failure of the state machine to receive the correct event.

### 1. Component-Level Testing

Component-level testing is mainly divided into actuator testing and input-path testing. For actuators, `tools/drv8833_direct_test.cpp`, `tools/drv8833_direct_wheel_test.cpp`, `tools/drv8833_direct_lift_test.cpp`, `tools/drv8833_direct_lift_keyboard_test.cpp`, and `tools/drv8833_direct_rear_slide_test.cpp` can be used in sequence to confirm whether the direction and response of the single drive path, the front and middle wheel sets, the two lifting mechanisms, and the rear slide table are consistent with the current configuration. The goal of this step is not to complete complex actions, but to first eliminate the most basic problems such as whether the component can move correctly, whether the direction is right, and whether it stays stable after stopping.

For the input path, it is necessary to confirm whether the signals required for step approach, support transfer, and safety protection are reliable. Forward ranging can be checked through `tools/front_ultrasonic_test.cpp` to see whether the distance data are continuous, stable, and correct in direction. The three downward/support sensors can be verified through `tools/ir_direct_gpio_all_test.cpp` to see the response changes under support-surface and suspended conditions. Mechanical limit switches and rear-slide forward/backward direction can be checked through `tools/mcp23017_limit_switch_test.cpp` and `tools/rear_slide_limit_orientation_test.cpp` respectively. If additional investigation of the expander input path is needed, `tools/front_ir_mcp23017_test.cpp` and `tools/ir_mcp23017_all_test.cpp` can also be run as diagnostic tests.

### 2. Whole-Machine Integration Testing

Only after the above actuator, sensor, and limit tests have all passed separately is it suitable to enter whole-machine integration testing. The whole-machine test corresponds to the main program `src/main.cpp`, that is, the built `Climbing_Robot`. At this stage, the focus of verification is no longer whether a single component responds, but whether startup bring-up, forward ranging initialization, MCP23017 interrupt-thread creation, startup reset, state-machine progression, motion coordination, and fault-stop paths can be connected into one complete main chain. In other words, the earlier tests answer the question “can each component work independently,” while whole-machine integration answers the question “when these components are put together, can the system complete one real stair-climbing control cycle in the intended order.”

### 3. Detailed Test Documentation

This section is mainly used to explain the testing ideas and testing layers of the project. If it is necessary to further check the build commands, running methods, GPIO or MCP23017 mapping, expected phenomena, and recommended testing order for each test target, the independent testing document `TEST_FRAMEWORK.md` can be consulted.

## Result Analysis

From the overall results, the biggest feature of this project is that it has completed a full realtime embedded system. In the GitHub repository, the relationships among low-level drivers, sensor reading, mechanism abstraction, state-machine scheduling, safety protection, and test tools are clear, and the programs are organized in layers according to their own logic and functions. The project not only allows the robot to perform realtime stair-climbing tasks, but also uses sensor feedback for realtime control of actions such as front lifting, middle following, and rear support transfer. At the same time, it gives the system good readability, maintainability, and expandability. This shows that we have completed a realtime embedded system with clear engineering boundaries, a debugging path, and room for further improvement.

From the perspective of function implementation, another advantage of the project is that both the “action path” and the “safety path” already have an initial form. Forward distance detection, downward/support confirmation, limit protection, action timeout, and unified stop logic have all been included in one common control system. This means that when the robot faces a non-planar environment such as stairs, it does not simply depend on fixed delays to execute actions blindly, but advances the state as much as possible according to current sensor feedback. Although this design does not necessarily mean that the system has already reached industrial-level stability, it at least shows that the project already has the core features of a realtime system, namely continuous program operation, timely response to input changes, and fast entry into a controlled stop state under abnormal conditions. The multiple groups of component-level test programs kept in the GitHub repository also show that the project was developed according to the engineering idea of “verify components first, then integrate the whole machine,” which helped project adjustment and troubleshooting.

However, the project still has some limitations at present. First, in terms of software and experimental records, although the project can already complete the expected functions, it still lacks more complete experimental data to support the stability, accuracy, and operating efficiency of the system. The GitHub repository already shows modular drivers, event-driven input paths, and functional testing paths clearly, but it is still necessary to add more quantitative data on action duration, error range, sensor-trigger stability, the basis for timeout thresholds, and the success rate of continuous multi-step climbing.

Second, the hardware of this project also places some limits on the final effect. The current robot is built with wooden boards and hand-made processing, and does not use 3D printing or higher-precision manufacturing methods. The advantage of this approach is that the materials are easy to obtain, the cost is low, the production threshold is low, and modifications can be made more quickly, which is suitable for rapid trial and repeated adjustment during the course-project stage. However, the problems it brings are also very direct. A hand-made wooden-board structure is naturally less stable than 3D-printed or CNC structural parts in terms of hole-position consistency, connector parallelism, dimensional precision, overall rigidity, and repeated assembly precision. Therefore, during lifting, sliding, center-of-gravity transfer, and support processes, small deviations are likely to appear. For example, if the mounting angles of the wheels and sensors are not fully consistent, this can easily cause changes in slide resistance, uneven force on the left and right sides, and insufficient structural rigidity. These errors may not be obvious in flat-ground testing, but in a stair-climbing scene, which is more sensitive to posture and contact conditions, they become amplified into unsmooth actions, shifted limit-trigger timing, repeated threshold retuning, and even a lower success rate for continuous actions. At the same time, because the wooden structure lacks enough rigidity, the robot in the final implementation sometimes needs to be supported by hand to provide complete support and help it finish the function. In the future, it can be improved through 3D printing or by changing to higher-strength materials.

Therefore, the most realistic conclusion at the current stage can be summarized as follows: the software architecture and control idea are already relatively mature and can support a complete three-stage stair-climbing experiment, while the upper limit of the final whole-machine performance is still largely restricted by machining precision and the depth of experimental data accumulation. If the project continues to be improved later, one clear direction is to keep the existing layered control framework and testing system, while further improving the manufacturing precision of structural parts, enhancing body rigidity and assembly consistency, and adding continuous experimental data and performance statistics. In this way, it can be better applied in future practical scenarios.

## Conclusion

Overall, this project focuses on the real problem of the passing ability of mobile robots in stair environments and completes the combined hardware and software design of a three-stage stair-climbing robot. Through the Raspberry Pi 5 Linux platform, the C++20 software stack, event-driven sensor input, modular mechanical abstraction, and state-machine scheduling, the system can complete a full closed loop from perception and judgement to execution, and can show well the application value of realtime embedded control in tasks involving complex terrain.

This project combines realtime system design, object-oriented software structure, functional testing, and engineering-style documentation. If a more complete wiring diagram, installation steps, quantitative test results, and long-term stability verification are added in the future, this project result can be further improved in reproducibility, presentation quality, and depth of analysis.
