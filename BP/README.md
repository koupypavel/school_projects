# Vehicle for Small and Remote Space Mapping

**Bachelor's thesis** · Brno University of Technology, Faculty of Information Technology, Department of Computer Systems · 2018
**Author:** Pavel Koupý

> English adaptation of the original Czech thesis ([`dokumentace.pdf`](dokumentace.pdf)). The text is translated and condensed. Formal parts (declaration, acknowledgements) are omitted.

<p align="center">
  <img src="docs/images/final-build.jpg" alt="Final build of the vehicle" width="420">
</p>

## Abstract

A robotic vehicle for remote mapping of small indoor spaces. Scope: mechanical design, selection and wiring of electronics, and an operator interface for remote control, live camera streaming and autonomous room exploration.

**Keywords:** autonomous vehicle, robotics, Arduino, Raspberry Pi, ROS, localization and mapping (SLAM), remote control

## Contents

1. [Introduction](#1-introduction)
2. [Construction and electronics](#2-construction-and-electronics)
3. [Software](#3-software)
   - [3.1 ROS and the operating system](#31-ros-and-the-operating-system)
   - [3.2 Drive control and user interface](#32-drive-control-and-user-interface)
   - [3.3 Autonomous control](#33-autonomous-control)
   - [3.4 Mapping and localization (ORB-SLAM2)](#34-mapping-and-localization-orb-slam2)
4. [Conclusion](#4-conclusion)
5. [Repository layout](#repository-layout)
6. [Appendix A – Costs](#appendix-a--costs)
7. [References](#references)

---

## 1. Introduction

Objective: a **low-cost, general-purpose platform**, built end to end (3D-printed wheels and chassis through user interface and software), carrying a computer that runs:

- localization and mapping of the vehicle,
- drive control,
- a camera stream for remote operation,
- additional tools and sensors.

Operating modes:

- **Autonomous exploration** of indoor spaces, started by the operator.
- **Remote driving** by the operator via camera and sensor feedback.

Sensors and drive:

| Item | Description |
|---|---|
| Primary sensor | Single (monocular) camera |
| Auxiliary sensors | Ultrasonic distance sensor, touch (wire) bumper |
| Drive | Four **Mecanum wheels** (omnidirectional motion in confined spaces) |

Work packages:

1. Vehicle construction.
2. Electronics selection and circuit design.
3. Functionality implemented as **ROS nodes**.

Experiments were conducted in an indoor room with artificial walls and cardboard-box obstacles of various sizes (for the ultrasonic and touch sensors). Evaluation was qualitative (pass/fail of expected component behavior) and covered:

- autonomous mode,
- simultaneous localization and mapping (SLAM),
- manual control.

A demo video of the experiments was submitted with the thesis.

## 2. Construction and electronics

### 2.1 Existing vehicles and platforms

Commercial robots fall into three groups:

| Group | Examples | Notes |
|---|---|---|
| Specialized service platforms | [Fetch Robotics](https://fetchrobotics.com/) warehouse robots | Expensive, task-specific. |
| Tele-operation platforms | [Sanbot](http://www.sanbot.com/), [PR2](http://www.willowgarage.com/pages/pr2/overview) | Automated tasks plus remote control / "virtual presence"; typically SLAM-based path planning in unknown spaces. |
| Kits for kids and hobbyists | [DRC mark 1](https://www.robotshop.com/letsmakerobots/daddys-robot-car-drc-mark-1) (three wheels, differential front drive), [J-bot office](https://www.jameco.com/jameco/workshop/JamecoBuilds/jbotrobot.html) (four driven wheels), [KUKA youBot](http://www.youbot-store.com/) (mobile manipulator on **Mecanum wheels**) | Quality strongly price-dependent. |

Design target: combine elements of all three groups into a low-cost, extensible platform.

**Mecanum wheels** allow translation in any direction without changing body heading, and rotation in place; they are common in warehouses and other confined areas. A freely available printable kit from [Thingiverse](https://www.thingiverse.com/thing:1358552) is used; no custom wheel design was required.

### 2.2 Construction

- Plastic parts: **PLA** (melting point approx. 215 °C), **0.20 mm** layer height; **0.35 mm** for parts without surface-finish requirements (reduced print time).
- Models: Thingiverse, **CC BY-SA 3.0**, used unmodified.
- A simplified **prototype** was built first for electronics testing; some of its printed parts are reused in the final build.

<p align="center">
  <img src="docs/images/prototype.jpg" alt="Prototype" width="480"><br>
  <em>Figure 3: Prototype</em>
</p>

| Parameter | Value |
|---|---|
| Drive configuration | **Four-motor differential drive** |
| Motors | [Pololu micro metal gearmotors](https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors.pdf), **1:100** metal gearbox, rated **9 V** |
| Wheel | Load-bearing frame, **nine rollers** on metal pins, each roller wrapped in heat-shrink tubing (traction) |
| Wheel/motor mounting | Two screws + plastic clamp per unit |
| Frame | Chassis plates, computer case and battery holder joined with **standoffs** of various lengths (extensible for further sensors/actuators) |
| Maximum slope | **35°** |

<p align="center">
  <img src="docs/images/mecanum-wheels.jpg" alt="Mecanum wheel detail" width="620"><br>
  <em>Figure 4: Wheel detail</em>
</p>

Mecanum steering differs from a standard four-motor differential drive. Figure 5: per-wheel rotation (red arrows) and resulting vehicle motion (black arrow).

<p align="center">
  <img src="docs/images/mecanum-drive.png" alt="Mecanum drive directions" width="620"><br>
  <em>Figure 5: Drive and steering</em>
</p>

Known issue: electronics and batteries are mounted high, resulting in a **high center of gravity**.

### 2.3 Electronics

Selection criteria: ease of use, price, library support, local availability.

| Role | Component | Rationale |
|---|---|---|
| Main computer | **Raspberry Pi 3** | Runs ROS and SLAM on board without an external computer. Built-in Wi-Fi used for the operator link. Serves as a test of SLAM feasibility on a single-board ARM computer with limited RAM. |
| Motor controller | **Arduino UNO** (ATmega328P) + L293D motor shield | Isolates motor current spikes (notably at start-up) from the Pi. Connected to the Pi over **USB serial**, which also powers it from the Pi during debugging. |
| Camera | Raspberry Pi Camera Module (5 MP, 2592×1944) | Connected via ribbon cable. Operated at 640×480 to reduce CPU load and bandwidth. |
| Distance sensor | **HC-SR04** ultrasonic | Low-cost distance measurement. |
| Touch sensor | Wire bumper | Collision detection. |
| Power | 6× AAA NiMH (7.2 V) + **LM2596** step-down regulator | Converts battery voltage to 5 V for the Pi. |

Rejected alternative: FPGA with a microprocessor (higher development effort than an off-the-shelf ARM board).

<p align="center">
  <img src="docs/images/wiring-diagram.png" alt="Wiring diagram" width="620"><br>
  <em>Figure 7: Wiring diagram</em>
</p>

#### Motor driver

Arduino **shield** components:

- 2× **L293D** (four half H-bridges each, i.e. two full H-bridges); each drives one motor pair via **PWM**.
- 1× **74HC595N** shift register: serial-to-parallel conversion of direction signals M1A/B … M4A/B.
- PWM duty cycle on PWM2A/PWM2B sets motor speed (average voltage at the motor terminals).
- Diodes D1–D8: protection against motor voltage spikes.
- Capacitor C1: smoothing of the 5 V reference.
- VCC1 = V+ (5 V), since the board is USB-powered.

<p align="center">
  <img src="docs/images/motor-driver-schematic.png" alt="DC motor driver schematic" width="620"><br>
  <em>Figure 8: DC motor wiring (front motors)</em>
</p>

#### Power

| Parameter | Value |
|---|---|
| Source | 6× AAA NiMH, **7.2 V** |
| Regulator | **LM2596** step-down |
| Regulator input | up to 46 V |
| Regulator output | **5 V** (adjustable in 0.1 V steps), up to 3 A |
| Pi recommended supply | 2.5 A (3 A available → headroom) |
| Protection | short circuit, overtemperature, reverse polarity |

#### Sensors

- **Touch bumper:** 3.3 V applied to the bumper wire; standoffs wired to a Pi GPIO input. Contact between wire and standoff → logic 1.
- **Ultrasonic sensor HC-SR04:** 5 V logic vs. 3.3 V Pi GPIO.
  - **Trigger** (sensor input): driven directly from a 3.3 V pin, physical pin **16**.
  - **Echo** (5 V TTL output): via **BSS138 MOSFET logic-level shifter** (LV = 3.3 V, HV = 5 V), physical pin **18**.
  - Software access: [WiringPi](http://wiringpi.com/).

<p align="center">
  <img src="docs/images/level-shifter.png" alt="BSS138 logic level shifter" width="360"><br>
  <em>Figure 10: Logic-level shifter</em>
</p>

## 3. Software

The software runs on **ROS** ([Robot Operating System](http://www.ros.org/)), which provides process separation and integration of existing tools. The **autonomous control node** supervises the other nodes and selects actions using a **subsumption architecture** [5].

Architecture options considered:

1. **Client–server**, as in the ROSberryPi SLAM Robot project [2]: SLAM on a separate PC.
2. **All processing on the Raspberry Pi**, controlled remotely over **SSH** from a graphical interface. **Selected**, with Pi computing capacity as an open question.

Rationale for SSH instead of [Rosbridge](http://wiki.ros.org/rosbridge_suite) (WebSocket access to ROS without a local ROS install):

- Controlling the Linux system through ROS requires superuser rights, with a risk of system breakage or SD-card corruption.
- The available operator computer runs Windows, which has no fully working ROS implementation.

### 3.1 ROS and the operating system

ROS concepts used:

- **Nodes:** separate processes (ultrasonic sensor, touch sensor, autonomous control, SLAM, …).
- **Topics:** nodes **publish** to or **subscribe** to topics; messages are built-in types (strings, numbers, predefined structures) or custom types.
- **Launch files:** start and configure nodes.

Nodes are written in **C++** (`roscpp`). Arduino communication uses **ROSSerial**.

| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `ult_sensor` | `Float32` | ultrasonic node → auto mode, GUI status |
| `whiskers_sensor` | `Bool` | touch-bumper node → auto mode, GUI status |
| `motors_ctrl` | `UInt16` | auto mode / GUI → Arduino |
| `motors_speed` | `String` | GUI → Arduino |
| `automode_ctrl` | `String` | GUI → auto mode |
| `automode_status` | `Int16` | auto mode → GUI status |
| `orb_slam` | `Int16` | ORB-SLAM2 (modified mono node) → GUI status |
| `/camera/image_raw` | `Image` | camera (decompressed) → ORB-SLAM2 |

**Platform:** [ROS Kinetic](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi) on **Ubuntu MATE 16.04.4 LTS (32-bit)**. The Ethernet interface has a static IP for initial configuration of the wireless link; the wireless link is used for remote control.

> **Build note:** the Pi has **1 GB of RAM**. Compiling ORB-SLAM2 and its dependencies requires a **2 GB swap file** on the SD card and `-j2` instead of `-j4`.

### 3.2 Drive control and user interface

The **Arduino** runs as a **ROS node** via ROSSerial. An internal state machine receives commands on `motors_ctrl` and executes them in a fixed-period loop. PWM generation and motor control use the [Adafruit Motor Shield library](https://github.com/adafruit/Adafruit-Motor-Shield-library):

| Call | Function |
|---|---|
| `run(direction)` | Sets motor direction or stops the motor. Each call sends a data word to the shield shift register, which drives the L293D direction inputs. |
| `setSpeed(value)` | Sets PWM duty cycle (speed). |

#### 3.2.1 User interface (`PiInterface`)

Windows **MFC** application; connects to the vehicle via [libssh](https://www.libssh.org/). Two screens selected by a Tab Control; each screen is a separate class. Configuration is loaded by the main dialog and passed to the screens by reference.

**Motion control (`MotionControlTab`):** motor speed setting, autonomous-mode commands, sensor readout, camera view. Driving input: latching on-screen buttons, or a **joystick** read via **Raw Input** HID events.

<p align="center">
  <img src="docs/images/ui-motion-control.png" alt="Remote-control user interface" width="720"><br>
  <em>Figure 11: Remote-control interface</em>
</p>

Background threads (each with its own SSH channel and shell):

| Thread | Direction | Function |
|---|---|---|
| `WatchStatusRun()` | ROS → UI | Starts a ROS node that prints `ult_sensor`, `whiskers_sensor`, `automode_status` and `orb_slam` to stdout; the UI parses this output from the SSH channel. |
| `ControlUpdate()` | UI → ROS | Writes commands to the shell; a ROS node reads them from stdin and forwards them to the corresponding topics. |

Synchronization: threads are started from the UI and controlled through atomic flags. The driving direction is an atomic global variable; no mutex is used (single writer, single reader).

**Video:**

- Client: [libVLC](https://www.videolan.org/vlc/libvlc.html) via a C++ [wrapper](https://www.codeproject.com/Articles/38952/VLCWrapper-A-Little-C-wrapper-Around-libvlc).
- Server: **UV4L** streams **MJPEG** (a CCTV-oriented format); web server on port **8080** for resolution, rotation and format settings. UV4L OpenCV face detection is available but reduces stream throughput noticeably.
- On SLAM start the stream is terminated: ORB-SLAM2 captures the camera through a different tool (device conflict) and requires **raw** uncompressed frames, whose bandwidth exceeds the Pi Wi-Fi capacity alongside control traffic.

**System control (`SystemControlTab`):** starts ROS nodes and manages the OS.

- **Console:** displays command output, accepts custom commands.
- Button commands are defined in an **INI file**, parsed at startup with [inih](https://github.com/jtilly/inih); changes require an application restart.
- Functions: shutdown/reboot, system parameters, network settings, **remote-desktop server** start.
- Remote desktop is the only way to view SLAM output; the 3D map is not streamed into the application.

<p align="center">
  <img src="docs/images/ui-system-control.png" alt="System control screen" width="720"><br>
  <em>Figure 12: OS control and vehicle system startup</em>
</p>

#### 3.2.2 System test

- **Setup:** all ROS nodes started from the UI.
- **Procedure:** CPU temperature, frequency, load and memory monitored at idle and under full load.
- **Result:**

| Metric | Idle | Everything running |
|---|---|---|
| CPU temperature | ~35 °C | ~58 °C |
| CPU core frequency | 0.6 GHz | 1.2 GHz |
| CPU load | ~5 % | ~85 % |
| Memory used (no swap) | 200 MB | 750 MB |

- **Observed issues:** performance margin is small; a **fan and passive heatsinks** are required.

#### 3.2.3 Manual control test

- **Setup:** sensor nodes and Arduino serial node only.
- **Procedure:** vehicle driven through the test area using camera and UI only, without direct line of sight.
- **Result:** passed. The interface provides full terminal functionality plus preset commands and driving controls.
- **Observed issues:** none recorded.

### 3.3 Autonomous control

#### 3.3.1 Subsumption architecture

Classic robot control is a pipeline of functional stages (sensing → mapping → planning → action); each stage consumes only the previous stage's output, so errors accumulate along the chain.

The **subsumption architecture** [5] decomposes behavior into **layers**:

- Each layer pursues one goal (e.g. avoid obstacle, wander) and reacts directly to the environment.
- Layers are **reactive agents**: no symbolic world model, no central planner.
- Every layer can issue motion commands.
- Higher layers (exploration, wandering) normally suppress lower ones; lower, primitive layers (collision handling) take over on urgent events.
- Result: complex emergent behavior capable of long-running operation without an operator.

```mermaid
flowchart LR
    P([Perception]) --> E[Explore]
    P --> W[Wander]
    P --> A[Avoid obstacles]
    E --> S1((S))
    W --> S1
    S1 --> S2((S))
    A --> S2
    S2 --> ACT([Action])
```
<p align="center"><em>Figure 13: Example reactive-agent architecture (S = suppression node)</em></p>

Each layer is typically a **finite state machine** driven by sensor input. A reactive agent is formally the six-tuple **{P, A, I, see, next, action}**:

- `see : E → P`: maps part of the environment state *E* to a percept *P*,
- `next : P × I → I`: percept and current internal state yield a new internal state,
- `action : P × I → A`: percept and state select an action,
- `env : A × E → E`: the action changes the environment.

A **purely reactive** agent has no internal state: {P, A, see, action}. The implementation follows this model loosely, adapted to ROS.

#### 3.3.2 Sensors

Autonomous mode uses the **wire bumper** and the **ultrasonic sensor**, each in its own ROS node, both on GPIO, read via WiringPi (physical pin numbering).

Distance computation: speed of sound in dry air at 25 °C ≈ **346.1 m/s** (rounded to 346 m/s); *t* = time between the Trigger pulse and the falling edge of Echo, measured with microsecond resolution.

```
distance = t × 346 m/s / 2
```

<p align="center">
  <img src="docs/images/ultrasonic-timing.png" alt="HC-SR04 timing diagram" width="620"><br>
  <em>Figure 14: Ultrasonic sensor timing. A 10 µs TTL Trigger pulse sends a burst of 8 ultrasonic pulses ("8 ult. zv. vln"), and Echo stays high for time t.</em>
</p>

#### 3.3.3 Implementation

Autonomous mode is a separate ROS node ([`auto_mode.cpp`](src/pi_rover/src/auto_mode.cpp)) with **three state machines**:

- Each machine is a function called from the main ROS loop.
- Inputs: global variables. Outputs: gated by global **inhibit** flags.
- Dependencies: sensor nodes and motor node must be running.
- Control: commands on `automode_ctrl` switch between autonomous and manual driving.

**Level 1: collision handling** (wire bumper)

<p align="center">
  <img src="docs/images/fsm-level1-collision.png" alt="Level 1 state machine" width="560"><br>
  <em>Figure 15: Level 1 state transitions</em>
</p>

| State | Action |
|---|---|
| `HCS_INIT` | → `HCS_COL_CHECK` |
| `HCS_COL_CHECK` | Checks `last_touch`; on collision → `HCS_BACK`. |
| `HCS_BACK` | Reverses a few steps → `HCS_COL_SOLVE`. |
| `HCS_COL_SOLVE` | **Rotates 270°**, distance reading at every third of the turn. |
| `HCS_COL_CHOOSER` | Selects the **largest distance** and computes the steps required to face it. If the last reading is the largest, heading is already correct → `HCS_COL_CHECK`. |
| `HCS_COL_REVIVE` | Rotates to the selected heading. Labelled `HCS_REVIVE` in the diagram. |

**Level 2: wall following** (ultrasonic)

<p align="center">
  <img src="docs/images/fsm-level2-wall.png" alt="Level 2 state machine" width="420"><br>
  <em>Figure 16: Level 2 state transitions</em>
</p>

| State | Action |
|---|---|
| `WWS_INIT` | → `WWS_CHECK_DISTANCE` |
| `WWS_CHECK_DISTANCE` | Distance **< 10 cm** → `WWS_OBJECT_ALIGNPP`. |
| `WWS_OBJECT_ALIGNPP` | Rotates until distance **≥ 15 cm** → `WWS_CHECK_DISTANCE`. |

Level 2 is the **most frequently active** layer and prevents most collisions; levels 1 and 3 mainly handle edge cases.

**Level 3: random wandering**

<p align="center">
  <img src="docs/images/fsm-level3-wander.png" alt="Level 3 state machine" width="480"><br>
  <em>Figure 17: Level 3 state transitions</em>
</p>

| State | Action |
|---|---|
| `RWS_INIT` | → `RWS_CHECK_STEPS` |
| `RWS_CHECK_STEPS` | Counts forward steps; above **200 steps** → `RWS_ROTATE`; otherwise continues straight. |
| `RWS_ROTATE` | Rotates and measures distance at **four random headings**. |
| `RWS_CHOOSER` | Selects the longest distance. |
| `RWS_REVIVE` | Steers to the selected heading if a correction is required. |

**Layer interaction:**

1. Default command in the main loop: "step forward".
2. The direction variable is **filtered through the three machines in sequence**; the result is sent to the Arduino control state machine.
3. While level 1 handles a collision, higher levels are paused until it reports the collision resolved.
4. Priority between levels 2 and 3 depends on step history: after prolonged straight driving, a 360° distance scan starts.
5. Manual mode disables all autonomous layers; sensor data is routed to the UI only.

#### 3.3.4 Testing

- **Setup:** vehicle at a random position; SLAM off.
- **Procedure:** autonomous exploration of the room; criterion: no trapping in one location or dead end.
- **Result:** passed with exceptions. The vehicle traversed the space and navigated between obstacles.
- **Observed issues:**
  - Occasional loss of orientation and tip-over.
  - **Steps per full rotation** vary with floor surface (wheel imperfections); an averaged value is used, as precision is not critical for this function.

#### 3.3.5 Limitations

- Low obstacles (e.g. **cables**) are not detected by any sensor.
- **No state machine for mapping and localization**, specifically for SLAM initialization and **recovery after tracking loss**.
  - Recovery is difficult in manual mode as well: the vehicle must be maneuvered back to a location where re-localization is possible.
  - With a good manually built map, localization also works in autonomous mode; rotation in place or reversing a few steps is usually sufficient.

### 3.4 Mapping and localization (ORB-SLAM2)

#### 3.4.1 Navigation

Navigation (autonomous or manual) requires a pose estimate and therefore a map; **SLAM** solves both jointly. Typical setups use **LIDAR, stereo or RGB-D** cameras, which are costly and computationally expensive to fuse. This system uses **a single monocular camera**, allowing the operator to view the mapped space and the vehicle's position in it.

#### 3.4.2 SLAM

The vehicle explores unknown space, incrementally builds a map, and estimates its own pose in it [4].

<p align="center">
  <img src="docs/images/slam-problem.png" alt="The SLAM problem" width="480"><br>
  <em>Figure 18: The SLAM problem in general</em>
</p>

- **x<sub>k</sub>**: vehicle position and orientation,
- **m<sub>i</sub>**: position of (static) landmark *i*,
- **z<sub>k,i</sub>**: observation of landmark *m<sub>i</sub>* from pose *x<sub>k</sub>*,
- **u<sub>k</sub>**: control vector applied at *x<sub>k−1</sub>* to reach *x<sub>k</sub>*.

Properties:

- Estimation is probabilistic (conditional probability densities); a history of motions and observations reduces pose uncertainty relative to landmarks.
- Common representation: state-space model with Gaussian noise → **Extended Kalman Filter (EKF)**. Alternative: **FastSLAM**.
- **Monocular** SLAM additionally requires solving **initialization** and **landmark depth**, which a single image does not provide directly [1], [3].

#### 3.4.3 Implementation

[ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) [6] build dependencies: [Pangolin](https://github.com/stevenlovegrove/Pangolin) (map visualization), [OpenCV](https://opencv.org/), [Eigen3](http://eigen.tuxfamily.org/), [DBoW2](https://github.com/dorian3d/DBoW2).

Modifications:

- **Pangolin over X11:** framebuffer setting changed in source; otherwise Pangolin crashes on a remote display ([Pangolin#194](https://github.com/stevenlovegrove/Pangolin/issues/194)).
- **Memory:** swap and `-j2` (see [3.1](#31-ros-and-the-operating-system)).
- **ROS mono example:** main loop rewritten to **publish the tracking state** of the `ORB_SLAM2::System` instance on `orb_slam` for other nodes and the UI ([`ros_mono.cc`](src/ORB_SLAM2/src/ros_mono.cc)).

Inputs:

1. **Camera calibration file:** **distortion coefficients** and **camera matrix** (pixels → real-world units) for distance estimation and undistortion. Values obtained with the ROS `camera_calibration` tool (**chessboard** with known square count and size) and copied into the ORB-SLAM2 config.
2. **Bag-of-Words vocabulary:** the generic vocabulary shipped with ORB-SLAM2, validated indoors and outdoors by its authors and community.

Camera pipeline:

- Resolution **640×480** (trade-off between CPU load and video streaming).
- `raspicam_node` publishes **compressed** images; ORB-SLAM2 accepts **raw** frames only. ROS `image_transport` decompresses and republishes on `image_raw`.

<p align="center">
  <img src="docs/images/camera-calibration.png" alt="Camera calibration" width="720"><br>
  <em>Figure 19: Camera calibration</em>
</p>

#### 3.4.4 Testing

**Setup (all tests):** vehicle at a random position; ORB-SLAM2, sensor nodes (including camera) and Arduino serial node started from the UI.

**Static test (rotation in place)**

- **Procedure:** ORB-SLAM2 initialized; vehicle rotated on the spot.
- **Result:** coarse room map produced.
- **Observed issues:** pure rotation provides no parallax for landmark depth estimation → high map error.

<p align="center">
  <img src="docs/images/slam-static-test.jpg" alt="ORB-SLAM2 static test" width="720"><br>
  <em>Figure 20: ORB-SLAM2 static test (rotation around the vehicle's y axis)</em>
</p>

**Small scene**

- **Procedure:** mapping of a small scene.
- **Result:** initialization without issues; reconstruction **very accurate**.
- **Observed issues:** occasional tracking loss during fast turns.

<p align="center">
  <img src="docs/images/slam-small-scene.jpg" alt="ORB-SLAM2 small scene" width="720"><br>
  <em>Figure 21: ORB-SLAM2 example</em>
</p>

**Whole room**

- **Procedure:** mapping of the full room.
- **Result:** mapping succeeded in smaller sub-areas only.
- **Observed issues:**
  - Initialization was harder due to lighting and the low number of objects in the room.
  - Original camera mount pointed largely at the floor; ORB-SLAM2 tracked features of a carpet with a dense blue pattern, distorting the map beyond use. Mitigation: camera moved to the **highest point** of the frame and **tilted slightly upward** (partial fix).
  - Poor results in rooms with **few objects** or **repetitive, similar-looking objects**: new points matched to already-mapped areas, corrupting the map; tracking frequently lost after some time.

<p align="center">
  <img src="docs/images/slam-manual-drive.jpg" alt="Manual control with ORB-SLAM2" width="720"><br>
  <em>Figure 22: Manual control with ORB-SLAM2</em>
</p>

## 4. Conclusion

A vehicle for mapping small, hard-to-reach spaces with manual and autonomous control was designed, built and tested per component (construction and wiring, manual control, autonomous mode, mapping and localization).

**Hardware**

- **Raspberry Pi 3:** performance and thermal behavior sufficient, contrary to initial concerns; fan and heatsinks required (see [3.2.2](#322-system-test)).
- **Arduino UNO:** fully met requirements.
- **Motor shield:** functional; uses bulky through-hole components that increase height and raise the center of gravity. A newer, more compact version exists.
- **Voltage regulator:** adequate; requires a heatsink for long-term operation.

**Software**

- Sensor nodes: basic arithmetic and GPIO access.
- Autonomous-mode node: supports manual and autonomous control.
- Missing: a **state machine managing the SLAM node** (mapping initialization, recovery after tracking loss). SLAM state is currently displayed in the UI only.

**User interface**

- Covers all functions required for remote control.
- Known issue: the video wrapper occasionally **froze the entire application** under weak Wi-Fi signal.
- The **SSH-based** interface provides functionality close to Rosbridge.

**Mapping and localization:** results are limited by the sensor; **better sensors** are expected to improve performance, including in poor lighting.

## Repository layout

```
BP/
├── dokumentace.pdf            # original thesis (Czech)
├── docs/images/               # figures used in this README
└── src/
    ├── Arduino/
    │   └── MotorControl.ino   # ROSSerial node driving the L293D motor shield
    ├── pi_rover/              # ROS package running on the Raspberry Pi
    │   └── src/
    │       ├── auto_mode.cpp        # subsumption-style autonomous control
    │       ├── ult_sensor.cpp       # HC-SR04 ultrasonic sensor node
    │       ├── whiskers_sensor.cpp  # wire bumper node
    │       ├── gui_status.cpp       # ROS → stdout bridge for the UI
    │       └── gui_control.cpp      # stdin → ROS bridge for the UI
    ├── ORB_SLAM2/             # modified ORB-SLAM2 ROS examples (publishes tracking state)
    └── PiInterface/           # Windows MFC remote-control application (libssh + libVLC)
```

## Appendix A – Costs

Cost reduction: reuse of existing components and low-cost electronics variants.

| Component | Price (CZK) |
|---|---:|
| Raspberry Pi 3 | 869 |
| Arduino UNO rev. 3 | 600 |
| Motor shield | 160 |
| Step-down regulator | 100 |
| Camera | 270 |
| Ultrasonic sensor | 45 |
| Logic-level shifter | 30 |
| Wiring and connectors | 50 |
| 3D printing (material) | 100 |
| Motors | 240 |
| **Total** | **2,464 CZK** (≈ €95 at 2018 rates) |

## References

1. DAVISON, Andrew J. *SLAM with a Single Camera.* <https://www.doc.ic.ac.uk/~ajd/Publications/davison_cml2002.pdf>
2. GAUTHAM P., JACOB G. *Real-time ROSberryPi SLAM Robot.* <https://courses.cit.cornell.edu/ece6930/ECE6930_Spring16_Final_MEng_Reports/SLAM/Real-time%20ROSberryPi%20SLAM%20Robot.pdf>
3. DAVISON, Andrew J. et al. *MonoSLAM: Real-Time Single Camera SLAM.* <https://www.doc.ic.ac.uk/~ajd/Publications/davison_etal_pami2007.pdf>
4. NEDUCHAL, Petr. *Návrh a testování metod vizuální simultánní lokalizace a mapování* (Design and testing of visual SLAM methods). <https://otik.uk.zcu.cz/bitstream/11025/7432/1/dp_prace.pdf>
5. BROOKS, Rodney A. *A Robust Layered Control System for a Mobile Robot.* <http://www.dtic.mil/dtic/tr/fulltext/u2/a160833.pdf>
6. MUR-ARTAL, Raúl; MONTIEL, J. M. M.; TARDÓS, Juan D. *ORB-SLAM: a Versatile and Accurate Monocular SLAM System.* <http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf>
