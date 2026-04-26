"""
Balancing and Position Detection Controller — NAO Robot on Soccer Field
Webots R2025a | Python
- Reads IMU to detect robot state (standing / fallen)
- PID balance loop while standing (resists pushes)
- Logs fall direction when fallen
"""

from controller import Robot

TIME_STEP = 32

# Fall thresholds in radians (after offset correction)
FALL_FWD_THRESHOLD  =  0.5
FALL_BWD_THRESHOLD  = -0.5
FALL_SIDE_THRESHOLD =  0.5

# PID gains
KP = 0.25
KI = 0.005
KD = 0.04

# NAO joint limits (prevents console warnings)
MOTOR_LIMITS = {
    'RElbowRoll':     ( 0.0,     1.5446),
    'LElbowRoll':     (-1.5446,  0.0),
    'RShoulderRoll':  (-1.3265,  0.3142),
    'LShoulderRoll':  (-0.3142,  1.3265),
    'LHipPitch':      (-1.7308,  0.4848),
    'RHipPitch':      (-1.7308,  0.4848),
    'LKneePitch':     (-0.0923,  2.1125),
    'RKneePitch':     (-0.0923,  2.1125),
    'LAnklePitch':    (-1.1868,  0.9227),
    'RAnklePitch':    (-1.1868,  0.9227),
    'LAnkleRoll':     (-0.7854,  0.7854),
    'RAnkleRoll':     (-0.7854,  0.7854),
    'LHipRoll':       (-0.7854,  0.7854),
    'RHipRoll':       (-0.7854,  0.7854),
    'LShoulderPitch': (-2.0857,  2.0857),
    'RShoulderPitch': (-2.0857,  2.0857),
}


class State:
    STANDING    = "STANDING"
    FALLEN_FWD  = "FALLEN_FORWARD"
    FALLEN_BWD  = "FALLEN_BACKWARD"
    FALLEN_SIDE = "FALLEN_SIDEWAYS"


class BalanceAndDetectionController:

    def __init__(self):
        self.robot = Robot()
        self.state = State.STANDING

        # PID internal state
        self.integral_pitch = 0.0
        self.integral_roll  = 0.0

        # Offsets start at zero; _calibrate() will overwrite them at runtime
        self.roll_offset  = 0.0
        self.pitch_offset = 0.0

        self._init_sensors()
        self._init_motors()
        self._go_to_standing_pose()

    # ── Sensors ──────────────────────────────────────────────────────────────

    def _init_sensors(self):
        self.imu = self.robot.getDevice('inertial unit')
        self.imu.enable(TIME_STEP)
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(TIME_STEP)
        print("[OK] Sensors enabled")

    # ── Motors ───────────────────────────────────────────────────────────────

    def _init_motors(self):
        self.motors = {}
        for name in MOTOR_LIMITS:
            dev = self.robot.getDevice(name)
            if dev:
                dev.setVelocity(0.5)
                self.motors[name] = dev
        print(f"[OK] {len(self.motors)} motors initialised")

    def _set(self, name, position):
        """Set motor position safely within joint limits."""
        if name not in self.motors:
            return
        lo, hi = MOTOR_LIMITS[name]
        position = max(lo, min(hi, position))
        self.motors[name].setPosition(position)

    # ── Standing pose ─────────────────────────────────────────────────────────

    def _go_to_standing_pose(self):
        self._set('LHipPitch',      -0.45)
        self._set('RHipPitch',      -0.45)
        self._set('LKneePitch',      0.87)
        self._set('RKneePitch',      0.87)
        self._set('LAnklePitch',    -0.40)
        self._set('RAnklePitch',    -0.40)
        self._set('LHipRoll',        0.0)
        self._set('RHipRoll',        0.0)
        self._set('LAnkleRoll',      0.0)
        self._set('RAnkleRoll',      0.0)
        self._set('LShoulderPitch',  1.4)
        self._set('RShoulderPitch',  1.4)
        self._set('LShoulderRoll',   0.3)
        self._set('RShoulderRoll',  -0.3)
        print("[OK] Standing pose set")

    # ── Auto-calibration ──────────────────────────────────────────────────────

    def _calibrate(self, samples=20):
        """
        Read IMU for a few steps at startup and use the
        average as the zero reference. This automatically
        corrects for any mounting offset in your NAO model.
        """
        print("[CAL] Calibrating IMU offset...")
        roll_sum  = 0.0
        pitch_sum = 0.0
        for _ in range(samples):
            self.robot.step(TIME_STEP)
            rpy = self.imu.getRollPitchYaw()
            roll_sum  += rpy[0]
            pitch_sum += rpy[1]
        self.roll_offset  = roll_sum  / samples
        self.pitch_offset = pitch_sum / samples
        print(f"[CAL] Offset found → roll={self.roll_offset:+.4f}  pitch={self.pitch_offset:+.4f}")
        print(f"[CAL] From now on, standing = roll≈0.000 pitch≈0.000")

    # ── IMU reading ───────────────────────────────────────────────────────────

    def _get_orientation(self):
        rpy   = self.imu.getRollPitchYaw()
        roll  = rpy[0] - self.roll_offset
        pitch = rpy[1] - self.pitch_offset
        yaw   = rpy[2]
        return roll, pitch, yaw

    # ── State classification ──────────────────────────────────────────────────

    def _classify(self, roll, pitch):
        if abs(roll) > FALL_SIDE_THRESHOLD:
            return State.FALLEN_SIDE
        if pitch > FALL_FWD_THRESHOLD:
            return State.FALLEN_FWD
        if pitch < FALL_BWD_THRESHOLD:
            return State.FALLEN_BWD
        return State.STANDING

    # ── PID balance controller ────────────────────────────────────────────────

    def _pid_balance(self, roll, pitch, dt):
        dt = max(dt, 0.001)

        # Read angular velocities directly from gyroscope for the D-term
        # gyro returns [wx, wy, wz] — wy = pitch rate, wx = roll rate
        gyro_vals = self.gyro.getValues()
        d_pitch   = gyro_vals[1]   # angular velocity around Y axis (pitch)
        d_roll    = gyro_vals[0]   # angular velocity around X axis (roll)

        # Pitch correction
        self.integral_pitch += pitch * dt
        cp = KP * pitch + KI * self.integral_pitch + KD * d_pitch
        cp = max(-0.35, min(0.35, cp))

        # Roll correction
        self.integral_roll += roll * dt
        cr = KP * roll + KI * self.integral_roll + KD * d_roll
        cr = max(-0.25, min(0.25, cr))

        # Apply to ankles (primary) and hips (secondary)
        self._set('LAnklePitch', -0.40 + (-cp))
        self._set('RAnklePitch', -0.40 + (-cp))
        self._set('LAnkleRoll',           -cr)
        self._set('RAnkleRoll',           -cr)
        self._set('LHipPitch',   -0.45 + (cp * 0.4))
        self._set('RHipPitch',   -0.45 + (cp * 0.4))

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        # Auto-calibrate first — reads IMU at startup to find real zero
        self._calibrate(samples=20)

        # Initialize prev_time AFTER calibration so the first dt is not
        # inflated by the calibration steps
        prev_time = self.robot.getTime()

        print("=" * 52)
        print("[INFO] Balance & Detection Controller running!")
        print(f"[INFO] Thresholds: fwd={FALL_FWD_THRESHOLD} "
              f"bwd={FALL_BWD_THRESHOLD} "
              f"side=±{FALL_SIDE_THRESHOLD}")
        print("[INFO] Push the robot with Alt+click+drag to test")
        print("=" * 52)

        while self.robot.step(TIME_STEP) != -1:
            t  = self.robot.getTime()
            dt = t - prev_time
            prev_time = t

            roll, pitch, yaw = self._get_orientation()
            new_state = self._classify(roll, pitch)

            # Detect and log state changes
            if new_state != self.state:
                print(f"\n{'='*52}")
                print(f"[STATE CHANGE] {self.state} --> {new_state}")
                print(f"  pitch = {pitch:+.4f} rad")
                print(f"  roll  = {roll:+.4f} rad")
                print(f"  yaw   = {yaw:+.4f} rad")  # logged for diagnostics
                if new_state != State.STANDING:
                    print(f"  !! Fall detected: {new_state} !!")
                print(f"{'='*52}\n")
                self.state = new_state

            # While standing, run PID balance to resist external pushes.
            if self.state == State.STANDING:
                self._pid_balance(roll, pitch, dt)

            # Print live IMU every 2 seconds
            if int(t * 10) % 20 == 0:
                print(f"[IMU] t={t:.1f}s | "
                      f"pitch={pitch:+.3f} | "
                      f"roll={roll:+.3f} | "
                      f"state={self.state}")


# Entry point
controller = BalanceAndDetectionController()
controller.run()


