# 🤖 TurtleBot IITD — Omnidirectional Drive Release Notes

> **Platform:** Teensy 4.1 · **Motors:** 3× N20 12V 200RPM with Quadrature Encoder · **Driver:** 2× L298N · **Wheels:** 3× Omni (120° apart)

---

## 📐 Robot Layout

```
              FRONT
               ▲
               │
           ┌───C───┐        C = Motor C  (90°)
          /         \       A = Motor A  (210°)
         /           \      B = Motor B  (330°)
        A             B
         \           /
          \_________/

        Top-down view
        All wheels are omni-wheels
        Spaced exactly 120° apart
```

---

## 📁 Project Structure

```
TurtleBotIITDProject/
│
├── robot_config.h              ← Central config (pins, CPR, geometry)
│
├── omni_drive_pid/
│   └── omni_drive_pid.ino      ← ✅ MAIN SKETCH — Omni drive + PID
│
├── rotation_360/
│   └── rotation_360.ino        ← 360° rotation calibration sketch
│
├── calibration/
│   └── motor_calibration.ino   ← CPR measurement sketch
│
└── 3MOTORWORKING/
    └── 3MOTORWORKING.ino       ← Basic motor + encoder test sketch
```

---

## ✅ What's Working

| Feature | Status |
|--------|--------|
| Forward / Backward | ✅ Working |
| Strafe Left / Right | ✅ Working (minor jitter, see Known Issues) |
| Rotate CW / CCW | ✅ Working |
| PID velocity control per motor | ✅ Working |
| Encoder feedback (all 3 motors) | ✅ Working |
| Speed adjust via serial | ✅ Working |
| Debug output via serial | ✅ Working |

---

## 🎮 Serial Commands

Open **Serial Monitor at 115200 baud** with **Newline** line ending.

| Key | Action |
|-----|--------|
| `F` | ▲ Forward |
| `B` | ▼ Backward |
| `L` | ◄ Strafe Left |
| `R` | ► Strafe Right |
| `<` | ↺ Rotate Counter-Clockwise |
| `>` | ↻ Rotate Clockwise |
| `S` | ⏹ Stop |
| `+` | ⏩ Speed Up (+200 counts/sec) |
| `-` | ⏪ Speed Down (-200 counts/sec) |
| `D` | 🔍 Debug — prints velocity, target, PWM for all 3 motors |

---

## ⚙️ Inverse Kinematics

The robot uses a **3-omni-wheel holonomic drive** model.

Given:
- `vx` — lateral velocity (right = +1, left = -1)
- `vy` — longitudinal velocity (forward = +1, backward = -1)
- `ω`  — rotation (CCW = +1, CW = -1)

Each wheel's speed is computed as:

```
wC  =  -vx                +  ω        (Motor C, front, 90°)
wA  =  +0.5·vx  - 0.866·vy  +  ω     (Motor A, back-left, 210°)
wB  =  +0.5·vx  + 0.866·vy  +  ω     (Motor B, back-right, 330°)
```

Outputs are normalized so no wheel exceeds ±1.0, then scaled by `targetSpeed`.

---

## 🔄 PID Velocity Control

Each motor runs an **independent PID loop at 50 Hz (every 20 ms)**.

```
Error      = target_velocity − measured_velocity
Integral  += error × dt          (clamped ±800)
Derivative = (error − last_error) / dt

PWM = Kp × error + Ki × integral + Kd × derivative
```

> ⚠️ **Direct output** (not incremental) — PWM is set directly from PID output each cycle, preventing runaway saturation.

### Tuned Gains

| Parameter | Value |
|-----------|-------|
| `targetSpeed` | 1200 counts/sec |
| `Kp` | 0.12 |
| `Ki` | 0.01 |
| `Kd` | 0.005 |
| Loop rate | 50 Hz (20 ms) |
| Integral clamp | ±800 |
| Dead-band | PWM < 25 → forced to ±25 |

---

## 📏 Calibrated Robot Constants

### Encoder (CPR)

| Motor | Counts Per Revolution |
|-------|-----------------------|
| A | 2115 |
| B | 2115 |
| C | 2115 |
| Tolerance | ±10 counts |

### Wheel Geometry

| Measurement | Value |
|-------------|-------|
| Wheel type | Omni (rollers at 45°) |
| Hub diameter (screw-to-screw) | 35.0 mm |
| Rolling diameter (roller-to-roller) | **37.3 mm** ← used in code |
| Circumference | ≈ 117.1 mm |
| mm per encoder count | ≈ 0.0554 mm |

### Chassis Geometry

| Measurement | Value |
|-------------|-------|
| Chassis diameter | 162.0 mm |
| Chassis radius | 81.0 mm |
| Wheel protrusion beyond chassis | 3.0 mm |
| Robot radius (center → wheel contact) | **84.0 mm** |

### 360° Rotation Counts (measured empirically on flat floor)

| Motor | Counts for one full rotation |
|-------|------------------------------|
| A | 7566 |
| B | 7813 |
| C | 7634 |

### Rotation Direction

All three motors spin **backward** for a **clockwise** rotation of the robot.

```
DIR_A_CW = -1   (backward = CW)
DIR_B_CW = -1
DIR_C_CW = -1
```

---

## 🔌 Pin Assignments

### Motor Driver 1 (L298N) — Motors A & B

| Pin | Signal | Function |
|-----|--------|----------|
| 9 | EN_A | Motor A PWM speed |
| 8 | IN1 | Motor A direction |
| 6 | IN2 | Motor A direction |
| 2 | EN_B | Motor B PWM speed |
| 4 | IN3 | Motor B direction |
| 3 | IN4 | Motor B direction |

### Motor Driver 2 (L298N) — Motor C

| Pin | Signal | Function |
|-----|--------|----------|
| 5 | EN_C | Motor C PWM speed |
| 10 | IN3_C | Motor C direction |
| 11 | IN4_C | Motor C direction |

### Encoders

> ⚠️ All encoder VCC must be connected to **3.3V** — Teensy 4.1 is NOT 5V tolerant on signal pins.

| Pin | Signal | Function |
|-----|--------|----------|
| 15 | ENC_A1 | Motor A encoder C1 (interrupt) |
| 16 | ENC_A2 | Motor A encoder C2 (direction) |
| 12 | ENC_B1 | Motor B encoder C1 (interrupt) |
| 14 | ENC_B2 | Motor B encoder C2 (direction) |
| 17 | ENC_C1 | Motor C encoder C1 (interrupt) |
| 18 | ENC_C2 | Motor C encoder C2 (direction) |

### Power Rails

| Rail | Connected To |
|------|-------------|
| 3.3V | Encoder VCC (all 3 motors) |
| 5V | L298N logic supply (VSS) |
| 12V | L298N motor supply (VS) — external battery |
| GND | Common ground — Teensy + both L298N + encoders |

---

## 🐛 Encoder Wiring Notes

During hardware bring-up, two wiring issues were discovered and corrected in software:

1. **Motor C encoder phase is inverted** — the ISR counts `-1` where others count `+1`:
   ```cpp
   void isrC() { encC += (digitalRead(ENC_C1) == digitalRead(ENC_C2)) ? -1 : 1; }
   ```

2. **Motors A and B encoder ISRs are normal:**
   ```cpp
   void isrA() { encA += (digitalRead(ENC_A1) == digitalRead(ENC_A2)) ? 1 : -1; }
   void isrB() { encB += (digitalRead(ENC_B1) == digitalRead(ENC_B2)) ? 1 : -1; }
   ```

---

## ⚠️ Known Issues

| Issue | Severity | Notes |
|-------|----------|-------|
| Strafe jitter | Low | Minor velocity oscillation during L/R strafe — PID gains need fine-tuning (try increasing Kd slightly) |
| No odometry | — | Encoder counts not yet used for position/distance tracking |
| Open-loop stop | — | Robot coasts to a stop on `S`; no active braking |

---

## 🗺️ Roadmap (Next Steps)

- [ ] Reduce strafe jitter — tune `Kd` upward (try 0.01–0.02)
- [ ] Add odometry — integrate encoder counts into X/Y position estimate
- [ ] Distance-based movement — `GO 500mm FORWARD` type commands
- [ ] Autonomous navigation / path following

---

*Built at IIT Delhi · Teensy 4.1 · Arduino framework*
