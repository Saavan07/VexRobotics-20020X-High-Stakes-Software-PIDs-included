# VEX High Stakes

C++ competition control software for Velocity Robotics 20020X's VEX V5 robot during the 2024–2025 High Stakes season.

This source snapshot contains robot configuration, drivetrain and arm PID controllers, driver controls, an active autonomous routine, additional commented routine scaffolds, and an error graph on the V5 Brain for tuning.

> Lead Programmer: **Saavan Kiran** • Team: **Velocity Robotics 20020X** • Environment: **VEXcode Pro V5 (C/C++)**

---

## Features

- **Drivetrain PID (lateral + turn)**
  - Parallel odometry wheel (2.75") for precise linear position
  - Inertial sensor–based heading hold / turn PID
  - Per-move speed caps (`maxDriveSpeed`, `maxTurnSpeed`)
  - Settling detection and time-based completion logic
- **“Lady Brown” Arm PID**
  - Rotation sensor feedback
  - 3-state controller (Rest ↔ Loading ↔ Scoring) via controller toggles
- **Autonomous Routines (slots)**
  - Red Rush Winpoint (Slot 1) : **active**
  - Blue Rush Winpoint (Slot 2) : scaffolded (commented)
  - Red/Blue Non-Rush (Slots 3–4) : scaffolded
  - Skills (Slot 5) : scaffolded
  - Solo Winpoints (Slots 6–7) : scaffolded
- **Driver Control**
  - Tank drive (Axis3/Axis2) with coast/hold behavior
  - Intake, clamp, doinker, intake-lift toggles
- **On-Brain PID Graph**
  - Real-time plot of turn error history for tuning (green trace)

---

## Hardware Map (Ports)

**Drivetrain** (6 11W V5 motors)
- Left: `PORT2` (front, reversed), `PORT1` (upper), `PORT10` (back, reversed)
- Right: `PORT3` (front), `PORT4` (upper, reversed), `PORT9` (back)

**Mechanisms** (2 5.5W motors & 1 11 W motor)
- Lady Brown arm: `PORT13`
- Upper Intake: `PORT19`
- Lower Intake: `PORT11`

**Sensors**
- Inertial: `PORT5`
- Lady Brown rotation: `PORT8`
- Parallel odometry rotation: `PORT6` (reversed)

**Pneumatics (3-wire)**
- Mobile-goal clamp: `A`
- Doinker: `B`
- Intake lift: `C`

---

## Project Structure

/ (source snapshot for a VEXcode project)
- robot-config.h # Port assignments & device setup
- main.cpp # Competition template, PID loops, auton & driver code

---

## Controller Mappings (Driver)

| Control | Action |
|---|---|
| **Axis3 / Axis2** | Tank drive (left/right) |
| **R1** | Intake both (100%) |
| **R2** | Outtake both (75%) |
| **B** | Lower intake only (upper off) |
| **L1** | Toggle clamp open/close |
| **L2** | Toggle doinker down/up |
| **A** | Toggle intake lift up/down |
| **Lady Brown states** | From **R**est: `Y`→**L**oading, `Right`→**S**coring; From **L**: `Y`→**R**, `Right`→**S**; From **S**: `Y`→**L**, `Right`→**R** |
| **Skills quick start** | Hold `Up + X` |

*Debounce is handled with internal cooldown counters.*

---

## PID Control Overview

### Drivetrain (Lateral + Turn)
- **Constants (tuned for my robot):**
  - `kP=0.78, kI=0.35, kD=0.575`
  - `turnkP=0.134, turnkI=0.011, turnkD=0.11`
- **Sensors:**
  - Lateral: `Parallel_Odom.position(deg)` → converted to inches
  - Turn: `Inertial.rotation()` in degrees
- **Helpers:** integral used only near target; zero-cross resets; per-move speed caps; live **turnError** graph (480×240)

### Lady Brown Arm
- **Constants (tuned for my robot):** `LBkP=0.25, LBkI=0.0, LBkD=0.01`
- **States & targets (deg):** Rest ≈ `0`, Loading ≈ `120/5`, Scoring ≈ `655/5`
- Hold via `LBBrake` if desired.

---

## Tuning Guide

- Calibrate inertial in `pre_auton()`; wait for “done!” on the controller.
- Start with **P** only (raise until fast response with mild overshoot).
- Add **D** to dampen overshoot.
- Add **I** sparingly to remove small steady-state error.
- Use the **on-Brain graph** to monitor `turnError`: aim for quick decay, minimal oscillation.
- For long dashes/spins, lower `maxDriveSpeed` / `maxTurnSpeed` for stability.

---

## Utilities & Toggles

- `drivePIDenable()` / `drivePIDdisable()` : quick on/off for drive PID
- `turnIenable()` / `turnIdisable()` : toggle turn-integral for long turns
- Reset helpers: `resetDriveSensors`, `resetTurnSensors`, `resetLBsensor`

---

## Autonomous Routines

#### Slot 1 – Red Rush Winpoint
- Active default autonomous routine in `autonomous()`.
- Sequence:
  - Intake down + spin to grab rings.
  - Doinker deploys to secure mobile goal.
  - Clamp and reposition using PID-based moves.
  - Score multiple rings before moving toward ladder.
- Uses both **drive PID** and **Lady Brown PID** simultaneously.

#### Slot 2 – Blue Rush Winpoint (scaffolded)
- Mirror routine of Slot 1 adapted for Blue Alliance.
- Code is scaffolded but **commented out** in `autonomous()`.
- Strategy:
  - Initial dash to mobile goal with intake spinning.
  - Doinker + clamp combo to secure mobile goal.
  - Repositions to drop goal near corner.
  - Collects additional rings with intakes.
- Requires enabling relevant code blocks for your team’s setup.

#### Slot 3 – Red Non-Rush Winpoint (scaffolded)
- Starts with Lady Brown scoring on alliance stake.
- Then transitions into securing the nearest mobile goal.
- Collects rings sequentially with coordinated intake + drive PID moves.
- Scoring path designed for consistency over speed.
- Commented block in `autonomous()`; enable as needed.

#### Slot 4 – Blue Non-Rush Winpoint (scaffolded)
- Same concept as Slot 3 but mirrored for Blue side.
- Begins with scoring on alliance stake, then clamps mobile goal.
- Collects rings and navigates toward ladder.
- Good backup autonomous for non-rush strategies.

#### Slot 5 – Skills Routine (scaffolded)
- Scaffolded long routine designed for Skills Challenge.
- Builds sequences of:
  - Lady Brown scores stake rings.
  - Multiple mobile goals scored sequentially.
  - Large number of field rings collected and scored.
- Includes PID resets for odometry correction mid-run.
- Not enabled by default; requires uncommenting and tuning.

#### Slot 6 – Red Solo Winpoint (scaffolded)
- Autonomous designed for **solo WP bonus** from Red side.
- Starts with Lady Brown stake score, then grabs first mobile goal.
- Sequences through ring collection and secondary goal pickup.
- Ends with placement at ladder or corner.

#### Slot 7 – Blue Solo Winpoint (scaffolded)
- Same idea as Slot 6 but for Blue side.
- Stake score with Lady Brown → mobile goal clamp → ring collection.
- Paths optimized for single-robot win point bonus.
- Code is scaffolded and commented.

---

## Demo Video
#### Watch the video to see our best autonomous skills run at home and our state-winning skills runs at the Illinois State Championships 2025:
[![Autonomous Route Demo](https://img.youtube.com/vi/EQBw6I2zAq8/0.jpg)](https://www.youtube.com/watch?v=EQBw6I2zAq8)

---

### Autonomous Notes
- **Only Slot 1 is fully enabled** in `autonomous()` by default.
- Slots 2–7 are left scaffolded/commented for easy switching.
- Timing (`task::sleep`) values may require tuning per robot condition (weight, battery, wheels).
- Use `resetDriveSensors` and `resetTurnSensors` carefully to re-zero sensors mid-run.
- Simple functions are utilized to control specific tasks (ie. on/off intake, clamp up/down, etc.).
- PID constants tuned for **6-motor drivetrain with 2.75” odom wheel**; re-tune if drivetrain changes.

---

### Future Improvements
- Replace time-based delays with **settle detection** for cleaner autonomous runs.
- Modularize auton routines into separate functions for readability.
- Expand PID to use perpendicular odometry for arc turns.

---

## Tech Stack

- **Language:** C/C++
- **Environment:** VEXcode Pro V5
- **Hardware:** VEX V5 Brain, 6-motor drivetrain, dual intakes, Lady Brown arm, inertial, dual rotation sensors, pneumatics

---

## Notes & Safety

- Confirm **port mapping** and **motor inversion** after hardware changes.
- Pneumatics default to safe states on boot (clamp up, doinker up, intake down).
- Re-tune PID with significant changes (wheel size, weight, battery condition).

---

## Attribution

Developed by **Saavan Kiran** for the **2024–2025 High Stakes** season.
Feel free to fork for educational use; please credit the original repo.


## Verification and setup scope

The repository contains `main.cpp` and `robot-config.h`, rather than a complete exported VEXcode project with its SDK and build configuration. Create or open a VEX V5 C++ project in your development environment, incorporate these source files, and reconcile its generated configuration with the hardware map above.

The October 2026 review inspected source and documentation. Compilation against the VEX SDK and behavior on the physical robot were not rerun. Controller constants and routines are specific to the original robot and require validation before operating different hardware. Standard VEX competition scaffolding and vendor APIs retain their original attribution.
