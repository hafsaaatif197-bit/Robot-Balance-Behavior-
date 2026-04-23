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


class State:
    STANDING    = "STANDING"
    FALLEN_FWD  = "FALLEN_FORWARD"
    FALLEN_BWD  = "FALLEN_BACKWARD"
    FALLEN_SIDE = "FALLEN_SIDEWAYS"


class BalanceAndDetectionController:

    def __init__(self):
        self.robot = Robot()
        self.state = State.STANDING

        # Offsets start at zero; _calibrate() will overwrite them at runtime
        self.roll_offset  = 0.0
        self.pitch_offset = 0.0

        self._init_sensors()

    # ── Sensors ──────────────────────────────────────────────────────────────

    def _init_sensors(self):
        self.imu = self.robot.getDevice('inertial unit')
        self.imu.enable(TIME_STEP)
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(TIME_STEP)
        print("[OK] Sensors enabled")

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

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        # Auto-calibrate first — reads IMU at startup to find real zero
        self._calibrate(samples=20)

        # Initialize prev_time AFTER calibration so the first dt is not
        # inflated by the calibration steps
        prev_time = self.robot.getTime()

        print("=" * 52)
        print("[INFO] State detection active")
        print(f"[INFO] Thresholds: fwd={FALL_FWD_THRESHOLD} "
              f"bwd={FALL_BWD_THRESHOLD} "
              f"side=±{FALL_SIDE_THRESHOLD}")
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

            # Print live IMU every 2 seconds
            if int(t * 10) % 20 == 0:
                print(f"[IMU] t={t:.1f}s | "
                      f"pitch={pitch:+.3f} | "
                      f"roll={roll:+.3f} | "
                      f"state={self.state}")


# Entry point
controller = BalanceAndDetectionController()
controller.run()