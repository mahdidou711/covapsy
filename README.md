# CoVAPSy — Autonomous RC Car

Autonomous RC car navigation using the **Follow The Gap (FTG)** algorithm and a 360° LiDAR.
Platform: **Raspberry Pi 4** — Competition: CoVAPSy 2026.

---

## Hardware

| Component | Details |
|---|---|
| SBC | Raspberry Pi 4 |
| LiDAR | RPLidar A2M12 — 256000 baud — `/dev/ttyUSB0` |
| ESC | Hobby ESC — PWM 50 Hz — GPIO 12 (channel 0) |
| Servo | Steering servo — PWM 50 Hz — GPIO 13 (channel 1) |
| PWM driver | `rpi_hardware_pwm` (hardware PWM) |

### ESC notes
- **Neutral**: 7.78% duty — **Forward**: above neutral (7.88+)
- **Reverse**: requires a double-tap sequence (brake → neutral → brake → stable reverse)
- Reverse is physically confirmed working on this ESC

### LiDAR frame convention
- `scan[0]` = front, `scan[90]` = left, `scan[270]` = right
- `0` = no measurement (too close to measure, treat as ~100 mm)

---

## Software Architecture

```
main.py             # Main control loop — state machine + stuck detection
├── ftg.py          # Follow The Gap algorithm
├── actuators.py    # PWM control: servo, ESC, reverse sequence
├── lidar_thread.py # LiDAR acquisition thread (RPLidar)
├── lidar_consumer.py # Non-blocking scan reader
├── steering.py     # Angle → duty cycle conversion
└── config.py       # All tunable parameters (only file to edit on race day)
```

### Control loop (50 Hz)

1. Read latest LiDAR scan
2. Compute front minimum distance (±15° sector)
3. If `escape_ticks > 0`: forced forward phase post-reverse
4. Else: run FTG → compute steering angle + proportional speed
5. Stuck detection: if front < 400 mm for 25 consecutive ticks → trigger reverse + escape

### Stuck / Reverse / Escape sequence

```
front_min < STUCK_DIST_MM for STUCK_TICKS ticks
  → choose open side (left_d vs right_d, treating 0 as 100 mm)
  → set steering toward open side
  → reculer() : brake → neutral → brake → stable reverse → T_REVERSE_S → neutral
  → escape phase: drive forward with same steering for 25 ticks (0.5 s)
  → cooldown 50 ticks (1 s) before stuck detection resumes
```

---

## Installation (Raspberry Pi)

```bash
pip install rplidar-roboticia rpi_hardware_pwm
```

Enable hardware PWM in `/boot/config.txt`:
```
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

---

## Deployment

From your dev machine:
```bash
scp config.py main.py actuators.py voituree3a6@192.168.123.97:~/covapsy/
```

Run on the Pi:
```bash
python3 main.py
```

Stop: `Ctrl+C` — cleanly stops PWM and LiDAR motor.

---

## Tuning (config.py)

> **Only `config.py` should be modified on race day.**

### Speed

| Parameter | Default | Effect |
|---|---|---|
| `VITESSE_MS` | 0.4 m/s | Cruise speed — start at 0.3, increase if stable |
| `VITESSE_MIN` | 0.28 m/s | Floor speed — do not go below (motor loses torque) |

### FTG

| Parameter | Default | Effect |
|---|---|---|
| `D_MIN_MM` | 1500 mm | Safety bubble radius — increase = more conservative |
| `K_FTG` | 1.0 | Steering gain — increase = more aggressive turns |
| `FTG_SECTOR_DEG` | 150° | Forward analysis sector — reduce if reacting to side walls |

### Collision / Speed control

| Parameter | Default | Effect |
|---|---|---|
| `COLLISION_DIST_MM` | 700 mm | Slowdown threshold — increase to brake earlier in corners |
| `COLLISION_SECTOR_DEG` | 15° | Front detection sector width |

### Stuck detection & Reverse

| Parameter | Default | Effect |
|---|---|---|
| `STUCK_DIST_MM` | 400 mm | Front distance to consider the car stuck |
| `STUCK_TICKS` | 25 (0.5 s) | Consecutive ticks before triggering reverse |
| `T_REVERSE_S` | 1.0 s | Reverse duration |
| `REVERSE_ENGAGE_S` | 0.15 s | Delay between steps of the double-tap sequence |

---

## Known Issues / Remaining Work

| Issue | Cause | Planned fix |
|---|---|---|
| Corners missed | Not slowing down early enough | Increase `COLLISION_DIST_MM`, lower `VITESSE_MS` |
| Reverse slow to engage | Double-tap sequence takes ~0.45 s | Reduce `REVERSE_ENGAGE_S` to 0.10 s |
| After reverse, returns to same stuck point | FTG resumes same angle, escape too short | Longer escape phase, add rear ultrasonic sensor |
| Rear ultrasonic sensor | No rear distance feedback during reverse | Add HC-SR04 on GPIO → stop reverse when obstacle detected behind |

---

## Calibration Tools

| File | Purpose |
|---|---|
| `cal_servo.py` | Find `SERVO_DUTY_MIN/CENTER/MAX` |
| `cal_esc_avant.py` | Find `ESC_DUTY_FWD_START` |
| `cal_esc_reverse.py` | Tune reverse duty values |
| `cal_lidar.py` | Verify LiDAR angle offsets |
