# TurtleBot IITD — Kiwi Drive Firmware Guide

A plain-language walkthrough of the robot's firmware: what the hardware is, which
wire goes where, and how each piece of software does its job.

The robot is a **kiwi drive** — a round chassis with three omni-wheels spaced 120°
apart. Unlike a car, it can slide sideways and spin at the same time. A **Teensy
4.1** is the brain, a **FlySky** radio is the remote control, and two **L298N**
H-bridge boards push current into the three geared motors.

---

## Contents

- [A. How to read this guide](#a-how-to-read-this-guide)
- [B. Hardware wiring — block diagram](#b-hardware-wiring--block-diagram)
- [C. Pin mapping table](#c-pin-mapping-table)
- [D. How the firmware is organised](#d-how-the-firmware-is-organised)
- [1. RC Control — reading the remote](#1-rc-control--reading-the-remote)
- [2. Kinematics — turning sticks into wheel speeds](#2-kinematics--turning-sticks-into-wheel-speeds)
- [3. Motor Control — driving the H-bridges](#3-motor-control--driving-the-h-bridges)
- [4. Encoders — measuring how far each wheel turned](#4-encoders--measuring-how-far-each-wheel-turned)
- [5. Diagnostics — the LED and serial output](#5-diagnostics--the-led-and-serial-output)
- [6. Startup Sequence — the safe boot](#6-startup-sequence--the-safe-boot)
- [7. main.ino — putting it together](#7-mainino--putting-it-together)
- [Quick reference: the arm ritual](#quick-reference-the-arm-ritual)

---

## A. How to read this guide

Sections **B–D** describe the robot as a whole — the wiring, the pins, and the
shape of the code. Sections **1–7** each take one source-code module and explain
it on its own: what problem it solves, how it works, and the API it offers to the
rest of the program.

Every number in this guide (pins, channels, tuning values) lives in one file,
`robot_config.h`. Treat that file as the single source of truth — this document
just explains what those numbers mean.

---

## B. Hardware wiring — block diagram

Trace the signal from your thumb to the wheel. You move a stick, the radio sends it
over the air, the receiver hands it to the Teensy as serial data, the Teensy does
some maths, and the motor drivers turn that into current. Separately, each motor's
encoder streams position pulses back to the Teensy.

```
   ┌──────────────┐   radio    ┌──────────────┐
   │  Hand-held   │  2.4 GHz   │   Receiver   │
   │ transmitter  │ ─────────▶ │   FS-iA6B    │
   │  FlySky FS-i6│            └──────┬───────┘
   └──────────────┘                   │ iBUS (one wire, all channels)
                                       │ into Teensy pin 0 (RX1)
                                       ▼
   ┌────────────────────────────────────────────────────────┐
   │                      TEENSY 4.1                          │
   │                                                          │
   │   reads radio ─▶ computes wheel mix ─▶ sends PWM+dir     │
   │   ▲                                          │           │
   │   │ encoder pulses (pins 12,14,15,16,17,18)  │ speed+dir │
   └───┼──────────────────────────────────────────┼──────────┘
       │                    │ pin 13               │
       │                    ▼                      ▼
       │             ┌────────────┐      ┌──────────────────┐
       │             │ Status LED │      │  L298N driver #1 │──▶ Motor A  ⟳enc
       │             │ (on-board) │      │  (Motors A & B)  │──▶ Motor B  ⟳enc
       │             └────────────┘      └──────────────────┘
       │                                 ┌──────────────────┐
       └── quadrature counts ────────────│  L298N driver #2 │──▶ Motor C  ⟳enc
                                         │    (Motor C)     │
                                         └──────────────────┘

   Power:  12 V ─▶ L298N motor supply      3.3 V ─▶ encoders (NOT 5 V!)
           5 V  ─▶ L298N logic             GND   ─▶ shared by everything
```

**Reading it:** the top half is the *command* going out to the motors; the return
arrow on the left is *feedback* coming back from the encoders. The status LED and
the USB cable (for the serial monitor, not shown) are how the robot talks back to
you without a network.

> **Voltage warning:** the Teensy 4.1 is **not** 5 V tolerant. Encoder VCC must be
> 3.3 V. Only the L298N boards see 5 V (logic) and 12 V (motors).

---

## C. Pin mapping table

Everything the firmware touches on the Teensy, in pin order. "Configuration" is how
the pin is set up in code; "Used for" is what it connects to.

| Pin | Configuration | Used for / sensor | Comment |
|----:|---------------|-------------------|---------|
| 0  | Serial RX (`Serial1`) | iBUS signal from the receiver | The entire radio link arrives on this one pin |
| 1  | Serial TX (idle)      | `Serial1` transmit | Unused — iBUS only needs to be read |
| 2  | PWM output | `EN_B` — Motor B speed | Enable pin of L298N #1, channel B |
| 3  | Digital output | `IN4` — Motor B direction | L298N #1 |
| 4  | Digital output | `IN3` — Motor B direction | L298N #1 |
| 5  | PWM output | `EN_C` — Motor C speed | Enable pin of L298N #2 |
| 6  | Digital output | `IN2` — Motor A direction | L298N #1 |
| 8  | Digital output | `IN1` — Motor A direction | L298N #1 |
| 9  | PWM output | `EN_A` — Motor A speed | Enable pin of L298N #1, channel A |
| 10 | Digital output | `IN3_C` — Motor C direction | L298N #2 |
| 11 | Digital output | `IN4_C` — Motor C direction | L298N #2 |
| 12 | Input + interrupt | `ENC_B1` — Motor B encoder, channel 1 | Interrupt fires here to count |
| 13 | Digital output | On-board status LED | Built into the Teensy |
| 14 | Input | `ENC_B2` — Motor B encoder, channel 2 | Tells the counter which way |
| 15 | Input + interrupt | `ENC_A1` — Motor A encoder, channel 1 | Counting interrupt |
| 16 | Input | `ENC_A2` — Motor A encoder, channel 2 | Direction bit |
| 17 | Input + interrupt | `ENC_C1` — Motor C encoder, channel 1 | Counting interrupt |
| 18 | Input | `ENC_C2` — Motor C encoder, channel 2 | Direction bit |

**Notice the pattern:** each motor uses **five** pins — one PWM (speed), two digital
(direction), and two encoder (one to count, one to sense direction). Pin 7 is free,
and pin 1 is technically reserved by the serial port but does nothing.

---

## D. How the firmware is organised

The firmware used to be a single long sketch. It is now split so that each file has
one clear responsibility and a small set of public functions. `main.ino` is kept
thin on purpose — it holds only `setup()` and `loop()` and calls into the modules.

The heart of it is a **left-to-right pipeline** that runs on every loop:

```
   RCControl            kinematics           motorControl
  ┌──────────┐  Vx,Vy,Ω ┌──────────┐ wheel   ┌──────────┐  PWM +   ┌────────┐
  │ read the │─────────▶│ mix into │ powers  │ push to  │ direction│ MOTORS │
  │  remote  │          │ 3 wheels │────────▶│ H-bridge │─────────▶│ A,B,C  │
  └──────────┘          └──────────┘         └──────────┘          └────────┘

  supporting cast:  startupSequence (boots safely) │ encoder (reads distance)
                    diagnostics (LED + serial)     │ config + IBusReader (shared)
```

| File | One-line job |
|------|--------------|
| `RCControl.*` | Read the radio, clean up the sticks, and decide when it's safe to arm. |
| `kinematics.*` | Pure maths: convert a movement request into three wheel speeds. |
| `motorControl.*` | Convert one wheel speed into the pins an L298N understands. |
| `encoder.*` | Count encoder pulses and report distance travelled. |
| `diagnostics.*` | Blink the status LED and print debug text. |
| `startupSequence.*` | The blocking boot flow that runs before driving is allowed. |
| `main.ino` | Wire everything together in `setup()` / `loop()`. |
| `robot_config.h` | All pins, channels and tuning constants (shared, unchanged). |
| `IBusReader.h` | Low-level parser for the FlySky iBUS byte stream (shared). |

---

## 1. RC Control — reading the remote

**File:** `RCControl.cpp` / `RCControl.h`

### What it does

This is the only part of the firmware that talks to the radio. It hides all the
messy details — the serial protocol, the raw numbers, the safety rules — and hands
the rest of the program three clean values and a simple "are we allowed to drive?"
answer.

### The signal it receives

The FlySky receiver speaks **iBUS**: instead of one wire per channel, it packs all
the channels into a 32-byte packet and sends the whole packet down a single wire
about every 7 milliseconds. `IBusReader` reassembles those bytes and checks each
packet's checksum; `RCControl` sits on top and interprets the channels.

Raw stick values arrive as numbers around **1000 (min) to 2000 (max)**, with the
resting centre near 1500. Three of them matter:

| Stick | Becomes | Symbol |
|-------|---------|--------|
| Right stick, left↔right | Sideways slide | **Vx** |
| Right stick, up↕down | Forward / back | **Vy** |
| Left stick, left↔right | Spin in place | **Ω** (omega) |

A fourth channel is a two-position switch used as the **arm / kill switch**.

### Cleaning up a stick

Raw radio numbers are never perfect, so `RCControl` reshapes each one into a tidy
value from **−1.0 to +1.0**:

1. **Calibrate the centre.** At boot it averages each stick for one second to learn
   exactly where "released" sits. While the robot is disarmed it keeps nudging that
   centre, so slow drift never becomes a problem.
2. **Apply a deadzone.** Movements within ±30 counts of centre are treated as zero,
   so a stick that rests a hair off-centre doesn't creep the robot.
3. **Scale to ±1.0.** Anything past the deadzone is stretched proportionally so the
   stick's full throw maps to full command.

### Deciding when to arm

Turning the motors on is deliberately awkward, so it can't happen by accident. Two
conditions must both be true at the same instant:

- every stick is centred, **and**
- the arm switch has just been flipped from **OFF to ON** (a fresh edge, not
  already-on).

If you flip the switch while a stick is off-centre, arming is refused and a warning
is printed. And if the radio link ever drops (**failsafe**), the robot disarms and
stops no matter what.

```
                       radio link lost  ┌─────────────┐
             ┌─────────────────────────▶│  FAILSAFE   │
             │                          │  (stopped)  │
             │                          └──────┬──────┘
             │                    link restored │
             │                                  ▼
        ┌──────────┐  all sticks    ┌──────────┐  switch    ┌──────────┐
        │ DISARMED │  centred       │  READY    │  OFF→ON    │  ARMED   │
        │          │───────────────▶│ (ready to │───────────▶│ motors   │
        │          │◀───────────────│  arm)     │            │  live    │
        └──────────┘  a stick moved └──────────┘            └────┬─────┘
             ▲                                                    │
             └────────────────── switch turned OFF ───────────────┘
```

### The API it offers

| Function | Meaning |
|----------|---------|
| `rcInit()` | Start the iBUS serial link. |
| `rcCalibrateCenters()` | Learn the resting stick positions (1-second average). |
| `rcUpdate()` | Call every loop: read radio, run the arm logic, track drift. |
| `rcVx()` `rcVy()` `rcOmega()` | The three cleaned-up control values, −1…+1. |
| `rcEnabled()` | `true` only when armed, switch on, and link healthy. |
| `rcFailsafe()` `rcArmed()` `rcSticksCentered()` | Individual state checks. |

---

## 2. Kinematics — turning sticks into wheel speeds

**File:** `kinematics.cpp` / `kinematics.h`

### What it does

Given a movement request — some sideways (Vx), some forward (Vy), some spin (Ω) —
this module works out how fast and which way each of the three wheels must turn. It
is **pure maths**: no pins, no hardware, so it is easy to reason about and test.

### The wheel layout

Looking down from above, wheel **C** is at the front, with **A** and **B** behind
it, all 120° apart:

```
                FRONT
               C (90°)
              /       \
             /         \
        A (210°)     B (330°)

     Vy ▲                    Vx points right →
        │                    Ω is a counter-clockwise spin ↺
        └──▶ Vx
```

### The mixing formula

Each wheel only "feels" the part of the motion aimed along its own direction, plus
its share of the spin. That gives three short equations:

```
   Motor C  =  −Vx                    +  Ω
   Motor A  =   0.5·Vx  −  0.866·Vy   +  Ω
   Motor B  =   0.5·Vx  +  0.866·Vy   +  Ω
```

(`0.866` is √3⁄2, which falls out of the 120° geometry.) Push the right stick right
and Vx dominates; push it forward and Vy dominates; push the left stick sideways and
Ω spins the whole robot.

### Keeping it in range

When you combine a big slide with a big spin, the maths can ask a wheel for more
than 100%. The module handles this by finding the largest of the three values and,
if it's above 1.0, dividing **all three** by it. The robot's *direction* stays exactly
right; it just can't exceed full speed. The function always returns values between
−1.0 and +1.0.

### The API it offers

| Function | Meaning |
|----------|---------|
| `kiwiMix(Vx, Vy, Omega)` | Returns a `WheelPowers` struct: `.front_C`, `.backLeft_A`, `.backRight_B`, each −1…+1. |

> **Wiring vs. maths:** `kiwiMix()` deliberately ignores how the motors are physically
> wired. `main.ino` multiplies each result by that motor's direction constant
> (`DIR_A_CW`, `DIR_B_CW`, `DIR_C_CW` from config) before sending it out, so a positive
> number always spins the wheel the way the maths expects.

---

## 3. Motor Control — driving the H-bridges

**File:** `motorControl.cpp` / `motorControl.h`

### What it does

This is the bottom layer. It takes a single signed number (−1.0 to +1.0) for one
motor and turns it into the three signals an L298N channel needs: two direction pins
and one PWM speed pin. It knows nothing about kinematics or the radio.

### How a number becomes motion

The **sign** of the number chooses direction; its **size** becomes the PWM duty
cycle (scaled up to 255, full power):

| Command | Direction pin 1 | Direction pin 2 | Speed (PWM) | Effect |
|---------|-----------------|-----------------|-------------|--------|
| positive | HIGH | LOW  | size × 255 | spin forward |
| negative | LOW  | HIGH | size × 255 | spin backward |
| zero     | LOW  | LOW  | 0 | stop |

So `+0.5` is "forward at half power", `−1.0` is "backward, full power", and `0` is
"stop".

### The API it offers

| Function | Meaning |
|----------|---------|
| `motorsInit()` | Set all motor pins as outputs and leave everything stopped. |
| `driveMotor(en, in1, in2, power)` | Drive one L298N channel with a signed power. |
| `stopAllMotors()` | Stop all three wheels at once. |

`stopAllMotors()` is the safe default: it runs at boot, whenever the robot isn't
enabled, and whenever the radio link is lost.

---

## 4. Encoders — measuring how far each wheel turned

**File:** `encoder.cpp` / `encoder.h`

### What it does

Every motor has a **quadrature encoder** — a sensor with two output channels that
click as the shaft turns. By counting the clicks the firmware can tell how far a
wheel has rolled; by comparing the two channels it can tell which way. This module
keeps a running count for each wheel and can convert it to millimetres.

> This module is **new**. The encoder pins and counts-per-revolution were already
> defined in `robot_config.h`, but nothing was reading them until now — it was built
> from those existing constants.

### How quadrature counting works

The two channels (call them C1 and C2) are identical square waves, but one lags the
other by a quarter step. An interrupt fires on every rising edge of C1; at that
instant the firmware reads C2. Whether C2 is high or low tells it the spin direction:

```
              C1 rising edge (interrupt fires)
                     │
          ┌────┐     ▼    ┌────┐    ┌────┐
   C1 ────┘    └──────────┘    └────┘    └───
                     │
   C2 ──────┐   ┌────┴──────┐   ┌────┐   ┌───
            └───┘           └───┘    └───┘
                     │
        read C2 now →  HIGH = count +1,   LOW = count −1
```

Each count represents about **0.055 mm** of travel at the wheel rim, because there
are ~2115 counts per full turn of a 37.3 mm wheel.

### The API it offers

| Function | Meaning |
|----------|---------|
| `encoderInit()` | Set up the pins and attach the counting interrupts. |
| `encoderCount('A')` | Signed running count for a wheel (`'A'`, `'B'`, `'C'`). |
| `encoderDistanceMM('A')` | That count converted to millimetres. |
| `encoderReset('A')` / `encoderResetAll()` | Zero the count(s). |

> **If a wheel counts backwards:** the +/− direction depends on how C1 and C2 are
> wired. If driving a wheel forward makes its count go *down*, either swap that
> motor's two encoder wires or flip the sign in its interrupt handler.

### Reference numbers

| Quantity | Value |
|----------|-------|
| Counts per wheel revolution | ~2115 (±10) |
| Wheel diameter (rolling) | 37.3 mm |
| Distance per count | ~0.055 mm |
| Counts for a full 360° robot spin | A: 7566 · B: 7813 · C: 7634 |
| Robot radius (centre to wheel) | 84 mm |

---

## 5. Diagnostics — the LED and serial output

**File:** `diagnostics.cpp` / `diagnostics.h`

### What it does

This module is how the robot reports its state — first through the single on-board
LED (readable across the room, no laptop needed), and second through detailed text
on the USB serial monitor (for bench debugging).

### The LED language

The firmware picks a pattern based on state, checked in priority order so a lost
link always shows through:

| LED behaviour | What it means | Your move |
|---------------|---------------|-----------|
| Slow blink (½ second) | Still hunting for the radio link | Check receiver power / binding / wiring |
| Fast blink (~7 Hz) | Link is good, but not ready — a stick is off-centre | Centre all sticks |
| Solid ON | Centred and ready to arm | Flip the arm switch OFF then ON |
| Solid OFF | **Armed — motors are live** | Be careful |
| Frantic blink (~16 Hz) | Failsafe — the link dropped | Motors are stopped; fix the link |

### The serial dump

About ten times a second (when not in failsafe) it prints one block showing the
cleaned stick values, the raw channel numbers, the live calibration centres, and —
for each wheel — the commanded power and the actual PWM being sent. If the robot
moves oddly, this line usually shows why.

### The API it offers

| Function | Meaning |
|----------|---------|
| `diagInit()` | Configure the LED pin. |
| `statusLedUpdate(failsafe, enabled, centred)` | Pick and show the right pattern. |
| `diagSetWheelDebug(...)` | Hand the latest wheel powers/PWM in for the next print. |
| `debugPrint(enabled, failsafe)` | Print the full state block (call throttled). |

---

## 6. Startup Sequence — the safe boot

**File:** `startupSequence.cpp` / `startupSequence.h`

### What it does

This runs once, before the main loop is allowed to drive anything. Its whole reason
to exist is safety: earlier versions of the robot could twitch on power-up or arm on
garbage data. The startup sequence guarantees the radio link is genuinely alive
before the motors can ever move.

### The steps

```
   1. wait briefly for the USB serial monitor (up to 3 seconds)
   2. wait for a real iBUS packet
        └─ LED slow-blinks, and a reminder is printed every 3 seconds,
           until the receiver is actually sending valid frames
   3. calibrate the stick centres (1-second average)
   4. hand control to the main loop
```

Step 2 is a deliberate hold: nothing past it happens until the link is proven good,
so the robot can never calibrate off missing or corrupt data.

### The API it offers

| Function | Meaning |
|----------|---------|
| `startupSequence()` | Run the whole blocking boot flow above. |

---

## 7. main.ino — putting it together

**File:** `main.ino`

`main.ino` is intentionally short. It doesn't contain any real logic of its own — it
just initialises each module and then calls them in the right order.

### `setup()` — run once

```
   start the serial port
   rcInit()          → radio link up
   motorsInit()      → motor pins ready, motors stopped
   encoderInit()     → encoder counting started
   diagInit()        → LED ready
   startupSequence() → wait for link, then calibrate
```

### `loop()` — run forever

```
   rcUpdate()                        read radio + update arm state
        │
        ▼
   work out: failsafe? enabled?
        │
        ▼
   statusLedUpdate(...)              show the current state on the LED
        │
        ▼
   is the robot enabled? ──── no ──▶ stopAllMotors()
        │ yes
        ▼
   driveKiwi(Vx, Vy, Ω):
        kiwiMix(...)                 → three wheel powers
        × direction constants        → correct for wiring
        driveMotor(...) × 3           → send to the H-bridges
        │
        ▼
   every 100 ms: debugPrint(...)     print the state block
        │
        └──────────────▶ (repeat)
```

The only branch that ever moves the robot is the **enabled** one. Disarmed,
un-centred sticks, switch off, or a lost link all funnel into `stopAllMotors()`.

---

## Quick reference: the arm ritual

To make the robot go, in order:

1. Power on. The LED **slow-blinks** while it looks for the receiver.
2. Once linked, **centre every stick**. The LED goes **solid ON** = ready.
3. Flip the arm switch **OFF, then ON**. The LED goes **solid OFF** = armed, motors live.
4. Drive. Right stick slides and moves forward/back; left stick spins.
5. To stop instantly, flip the arm switch OFF (or the link dropping does it for you).

If you flip the arm switch while a stick is off-centre, nothing arms — re-centre and
flip OFF→ON again.

---

*This guide describes the modular firmware in `main/`. All pins, channels and
constants come from `robot_config.h`.*
