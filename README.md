# NAO Balance & Fall Detection Controller
### Webots R2025a — RoboCup Soccer Field Simulation

A Python controller for the 'NAO V5 robot' that continuously reads IMU data to detect whether the robot is standing or has fallen, and actively resists external pushes using a PID balance loop while upright.

---

## Robot & Environment

- **Robot:** NAO V5 (red) humanoid robot
- **Simulator:** Webots R2025a
- **World:** RoboCup standard soccer field
- **Language:** Python 

---

## Features

### 1. IMU-Based State Detection
The controller reads pitch and roll angles from the robot's inertial measurement unit every time step (`TIME_STEP = 32 ms`) and classifies the robot into one of four states:

| State | Condition |
|-------|-----------|
| `STANDING` | pitch and roll within safe range |
| `FALLEN_FORWARD` | pitch > 0.5 rad |
| `FALLEN_BACKWARD` | pitch < -0.5 rad |
| `FALLEN_SIDEWAYS` | \|roll\| > 0.5 rad |

Every state change is logged to the console with the exact pitch, roll, and yaw values at the moment of detection.

### 2. Auto-Calibration
The NAO's IMU has a mounting offset in the Webots model. At startup, the controller reads 20 IMU samples and uses their average as the zero reference. This ensures `roll ≈ 0.000` and `pitch ≈ 0.000` while standing, regardless of the world file's initial orientation.

### 3. PID Balance Controller
While the robot is standing, a PID controller applies small corrective offsets to the ankle and hip joints every time step to resist external pushes and maintain upright balance.

- **Derivative term** uses `gyro.getValues()` directly (angular velocity in rad/s), which is more accurate than finite-differencing IMU angles
- **Pitch correction** is applied to ankle pitch (primary) and hip pitch (secondary, 40% weight)
- **Roll correction** is applied to ankle roll joints

```
KP = 0.25    KI = 0.005    KD = 0.04
```

> No stand-up / recovery motion sequence is implemented. The controller focuses purely on state detection from IMU data and active balancing while upright.

---

## File Structure

```
my_project/
├── controllers/
│   └── Balance_controller/
│       └── Balance_controller.py   ← main controller
│
main controller logic
├── worlds/
│    └──Balance Control.wbt
│
└── README.md
```

---

## How to Run

1. Open **Webots R2025a**
2. Load your RoboCup soccer field world file
3. Assign `Balance_controller` as the controller for the NAO robot
4. Press **Real Time Run** button in Webots to start simulation

The console will show calibration output followed by live IMU readings every 2 seconds:

```
====================================================
[CAL] Calibrating IMU offset...
[CAL] Offset found → roll=+1.5707  pitch=-0.0120
[CAL] From now on, standing = roll≈0.000 pitch≈0.000
====================================================
[INFO] Balance & Detection Controller running!
[INFO] Thresholds: fwd=0.5 bwd=-0.5 side=±0.5
[INFO] Push the robot with Alt+click+drag to test
====================================================
[IMU] t=2.0s | pitch=+0.001 | roll=+0.000 | state=STANDING
```

To test fall detection, push the robot using **Alt + click + drag** in the Webots viewport.

---

## Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `TIME_STEP` | 32 ms | Simulation time step |
| `FALL_FWD_THRESHOLD` | 0.5 rad | Pitch threshold for forward fall |
| `FALL_BWD_THRESHOLD` | -0.5 rad | Pitch threshold for backward fall |
| `FALL_SIDE_THRESHOLD` | 0.5 rad | Roll threshold for sideways fall |
| `KP` | 0.25 | PID proportional gain |
| `KI` | 0.005 | PID integral gain |
| `KD` | 0.04 | PID derivative gain |
| Calibration samples | 20 | IMU samples averaged at startup |

---

## Dependencies

- [Webots R2025a](https://cyberbotics.com/) — robot simulator
- Webots built-in `controller` Python 3.11 library 

---

## Development

This project was developed incrementally over 5 days. Each feature was built on a separate Git branch and merged into `main` via a pull request:

| Branch                  |          Feature |
|--------------------------|---------------------------------------------------|
| `feature/motor-limits-and-pose` | Motor limits, safe `_set()`, standing pose |
| `feature/pid-balance` | Full PID balance controller with gyro D-term |
| `fix/console-messages` | Startup message corrections, dt spike fix |
| `test:pid-optimization ` |Evaluate alternative PID gains & fall thresholds |

---

## Personal Reflection
The most challenging part of this project was tuning the **KD (Derivative)** gain. Initially, using finite-differencing for the D-term caused significant jitter due to sensor noise. Switching to raw gyroscope data from the IMU provided a much smoother response. Additionally, balancing the weight between the ankle and hip joints required extensive trial and error to ensure the robot didn't over-correct and fall sideways.


