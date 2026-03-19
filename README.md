# CoVAPSy 2026

![Python](https://img.shields.io/badge/language-Python-blue)
![License](https://img.shields.io/github/license/mahdidou711/covapsy)
![Last Commit](https://img.shields.io/github/last-commit/mahdidou711/covapsy)
![Repo Size](https://img.shields.io/github/repo-size/mahdidou711/covapsy)

**Autonomous 1/10-scale RC car — ENS Paris-Saclay CoVAPSy 2026 competition**

---

## Hardware

| Component | Model | Role |
|---|---|---|
| Single-board computer | Raspberry Pi 4 | Main compute + communications |
| Shield | STM32 (CMSIS, no HAL) | Low-level PWM motor/servo control |
| Lidar | RPLidar A2M12 | 360° distance sensing at 256000 baud |
| ESC | Hobby-grade brushless ESC | Motor speed control (forward + reverse double-tap) |
| Servo | Standard RC servo | Steering control (hardware PWM GPIO 12/13) |

---

## Software Architecture

| File | Role |
|---|---|
| `main.py` | Entry point — main control loop, state machine (stuck detection, escape mode) |
| `ftg.py` | Follow-the-Gap algorithm — gap detection, best gap selection, angle output |
| `config.py` | **Single source of truth** — all tuning parameters (speed, angles, lidar sectors) |
| `actuators.py` | PWM control for motor (ESC) and steering servo, reverse double-tap sequence |
| `lidar_thread.py` | RPLidar A2M12 acquisition thread |
| `lidar_consumer.py` | Lidar data processing and scan buffer management |
| `steering.py` | Angle-to-duty-cycle conversion for servo |
| `cal_servo.py` | Servo calibration script |
| `cal_esc_avant.py` | Forward ESC calibration script |
| `cal_esc_reverse.py` | Reverse ESC calibration script |
| `cal_lidar.py` | Lidar calibration and angle verification script |

---

## Algorithms

### Follow-the-Gap (FTG) — current
The car uses a gap-based reactive navigation algorithm:
1. Extract a forward lidar sector (`FTG_SECTOR_DEG`)
2. Detect free gaps (distances above threshold)
3. Select the best gap by average depth
4. Steer toward the gap center, with proportional speed reduction based on frontal distance

**Stuck detection**: if `front_min < STUCK_DIST_MM` for `STUCK_TICKS` consecutive ticks → trigger reverse + escape maneuver.

**Escape mode**: after reversing, force steering toward the open side for `escape_ticks` ticks, then hand back control to FTG.

### Planned — PPO sim-to-real transfer
- Train a PPO agent (Stable-Baselines3) in Webots simulation
- Transfer policy weights to the RPi 4 for real-world inference
- Curriculum: oval track → chicanes → full race track

---

## Installation

```bash
git clone https://github.com/mahdidou711/covapsy.git
cd covapsy
pip install -r requirements.txt
```

> **Note**: must run on a Raspberry Pi 4 with the RPLidar connected on `/dev/ttyUSB0` and hardware PWM enabled on GPIO 12 and GPIO 13.

---

## Usage

### Run the car
```bash
python main.py
```

### Calibration (run once before first use)
```bash
python cal_servo.py        # find servo CENTER, MIN, MAX duty cycles
python cal_esc_avant.py    # find ESC NEUTRAL and FWD_START duty cycles
python cal_esc_reverse.py  # find ESC reverse duty cycles and double-tap timing
python cal_lidar.py        # verify lidar angle mapping (scan[0]=front, scan[90]=left)
```

---

## Configuration

All tuning parameters are centralized in `config.py`. **This is the only file to edit on race day.**

Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `VITESSE_MS` | 0.4 | Target forward speed (m/s) |
| `VITESSE_MIN` | 0.28 | Minimum speed (ESC deadband floor) |
| `COLLISION_DIST_MM` | 700 | Distance at which speed starts reducing |
| `STUCK_DIST_MM` | 400 | Front distance threshold to declare stuck |
| `STUCK_TICKS` | 25 | Consecutive ticks below threshold to trigger reverse |
| `FTG_SECTOR_DEG` | 150 | Forward lidar sector width for FTG |
| `T_REVERSE_S` | 1.0 | Reverse duration (seconds) |

---

## Project Structure

```
covapsy/
├── main.py               # Main control loop
├── ftg.py                # Follow-the-Gap algorithm
├── config.py             # All tuning parameters
├── actuators.py          # Motor + servo PWM control
├── lidar_thread.py       # Lidar acquisition
├── lidar_consumer.py     # Lidar data processing
├── steering.py           # Servo angle conversion
├── cal_servo.py          # Servo calibration
├── cal_esc_avant.py      # Forward ESC calibration
├── cal_esc_reverse.py    # Reverse ESC calibration
├── cal_lidar.py          # Lidar calibration
├── requirements.txt      # Python dependencies
└── .github/
    ├── workflows/
    │   └── lint.yml      # CI: flake8 linting
    └── ISSUE_TEMPLATE/   # Bug report / feature request templates
```

---

## Roadmap

- [ ] PID lateral controller (wall-following complement to FTG)
- [ ] PPO training in Webots simulation
- [ ] Sim-to-real policy transfer
- [ ] Rear ultrasonic sensor for safe reverse maneuvers
- [ ] Multi-car avoidance / overtaking

---

## Author

**mahdidou711** — ENS Paris-Saclay, CoVAPSy 2026

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.
