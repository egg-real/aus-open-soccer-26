import time
import math
import threading
from lib.motor import Motor
from lib.config import Config
from lib.imu import IMU
from math import radians, sin

MAX_VELOCITY_CHANGE_PER_SEC = 10.0  # Max change in the (dx, dy) speed vector, per second.
# Fixed drive-loop period. The yaw correction is a proportional controller, so
# a stable, high update rate keeps it from overshooting. Pacing the loop (rather
# than free-running) also stops it from starving other threads that share the
# CPU/I2C bus (e.g. localisation), which previously made yaw correction oscillate.
DRIVE_LOOP_PERIOD = 0.005
YAW_CORRECT_THRESHOLD = 3.0
YAW_CORRECT_SPEED = 0.4
POSSESSION_YAW_CORRECT_SPEED = 0.1
YAW_CORRECT_MAX_SPEED_THRESHOLD = 60 # If the error is greater than this angle yaw correction will be at the maximum speed.
YAW_CORRECT_ACCELERATION = 0.1
# Derivative (damping) term. The correction is otherwise pure-proportional, which
# overshoots when the yaw feedback lags (e.g. IMU starved by I2C contention).
# Subtracting a term proportional to how fast the error is closing damps that
# oscillation. YAW_CORRECT_KD is effectively a derivative time in seconds.
YAW_CORRECT_KD = 0.08
# If the gap between yaw updates exceeds this, the derivative estimate is
# unreliable (stale IMU / long stall), so skip damping that cycle.
YAW_CORRECT_MAX_DT = 0.1

# Dribbler Spin Constants
DRIBBLER_SPEED = -1.0
RAD_TO_DEG = 180.0 / math.pi

def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))

def wrap_angle(theta):
    """Input an angle in degrees. Returns same angle but in [-180, 180)."""
    if theta is None:
        return None
    return (theta + 180) % 360 - 180

def capture_startup_yaw(imu:IMU, sample_count=25, sample_interval=0.02):
    """Average a short burst of IMU samples so startup yaw is not just the first reading."""
    print("Stabilizing IMU yaw reference...")
    sin_sum = 0.0
    cos_sum = 0.0
    samples = 0
    while samples < sample_count:
        yaw = imu.get_yaw()
        if yaw is not None:
            yaw_rad = math.radians(yaw)
            sin_sum += math.sin(yaw_rad)
            cos_sum += math.cos(yaw_rad)
            samples += 1
        time.sleep(sample_interval)
    return math.degrees(math.atan2(sin_sum, cos_sum))

class Drive:
    def __init__(self, imu:IMU=None, config:Config=None, motors=("ne", "se", "sw", "nw")):
        self.config = config if config is not None else Config()
        motors_config = self.config.get_value("motors", {})
        self.motors = {}
        self.current_direction = 0
        self.current_speed = 0
        self.target_direction = 0
        self.target_speed = 0
        self.target_rotation = 0
        self.target_yaw_correct_speed = YAW_CORRECT_SPEED
        self.possession = False
        self.target_lock = threading.Lock()
        self.last_update_time = time.monotonic()
        self._last_yaw_error = 0.0
        self._last_yaw_correct_time = time.monotonic()
        self._last_yaw_correct_speed = 0.0
        self.imu = imu if imu is not None else IMU()
        self.initial_yaw = capture_startup_yaw(self.imu)
        self.yaw = 0
        for motor_direction in motors:
            motor_config = motors_config.get(motor_direction)
            try:
                self.motors[motor_direction] = Motor(motor_config["address"],
                                            elec_angle_offset=motor_config["elec_angle_offset"],
                                            sin_cos_centre=motor_config["sin_cos_centre"])
            except Exception as e:
                print(f"Error initializing motor at address {motor_config['address']}: {e}")

        self.drive_thread = threading.Thread(target=self._drive_loop, daemon=True)
        self.drive_thread.start()

    def move(self, angle, speed=0.5, rotation=0, possession=False): # rotation is the desired yaw value
        with self.target_lock:
            self.target_direction = angle
            self.target_speed = speed
            self.target_rotation = wrap_angle(rotation)
            self.possession = possession
            if possession:
                self.target_yaw_correct_speed = POSSESSION_YAW_CORRECT_SPEED
            else:
                self.target_yaw_correct_speed = YAW_CORRECT_SPEED

    def _drive_loop(self):
        motors_config = self.config.get_value("motors", {})
        while True:
            loop_start = time.monotonic()
            self._update_current_velocity()

            drive_direction = self.current_direction
            drive_speed = self.current_speed
            yaw_correction = self._get_yaw_correction()

            for motor_direction, motor in self.motors.items():
                motor_config = motors_config.get(motor_direction)
                motor_angle = drive_direction - motor_config["angle_off"]
                motor_speed = drive_speed * -sin(radians(motor_angle))
                motor_speed = clamp(motor_speed - yaw_correction, -1.0, 1.0)
                motor.set_speed(motor_speed)

            elapsed = time.monotonic() - loop_start
            if elapsed < DRIVE_LOOP_PERIOD:
                time.sleep(DRIVE_LOOP_PERIOD - elapsed)

    def _get_yaw_correction(self):
        now = time.monotonic()
        dt = now - self._last_yaw_correct_time
        self._last_yaw_correct_time = now
        if not self._update_yaw():
            self._last_yaw_correct_speed = 0
            return 0

        with self.target_lock:
            target_rotation = self.target_rotation
            if self.target_yaw_correct_speed > self._last_yaw_correct_speed:
                self._last_yaw_correct_speed += dt * YAW_CORRECT_ACCELERATION
            else:
                self._last_yaw_correct_speed = self.target_yaw_correct_speed

        yaw_error = wrap_angle(target_rotation - self.yaw)

        derivative = 0.0
        if self._last_yaw_correct_time is not None:
            if 0.0 < dt <= YAW_CORRECT_MAX_DT:
                # Rate of change of the error (deg/s). wrap_angle keeps the
                # difference on the short arc across the +/-180 seam.
                derivative = wrap_angle(yaw_error - self._last_yaw_error) / dt
        self._last_yaw_error = yaw_error
        self._last_yaw_correct_time = now

        if abs(yaw_error) <= YAW_CORRECT_THRESHOLD:
            self._last_yaw_correct_speed = 0
            return 0

        # PD control, normalised so |P| = 1 at YAW_CORRECT_MAX_SPEED_THRESHOLD.
        p_term = yaw_error / YAW_CORRECT_MAX_SPEED_THRESHOLD
        d_term = YAW_CORRECT_KD * derivative / YAW_CORRECT_MAX_SPEED_THRESHOLD
        return clamp(
            (p_term + d_term) * self._last_yaw_correct_speed,
            -YAW_CORRECT_SPEED,
            YAW_CORRECT_SPEED
        )

    def _update_yaw(self):
        raw_yaw = self.imu.get_yaw()
        if raw_yaw is None:
            return False

        self.yaw = wrap_angle(self.initial_yaw - raw_yaw)
        return True

    def _update_current_velocity(self):
        with self.target_lock:
            target_direction = self.target_direction
            target_speed = self.target_speed

        dx = math.cos(math.radians(self.current_direction)) * self.current_speed
        dy = math.sin(math.radians(self.current_direction)) * self.current_speed

        target_dx = math.cos(math.radians(target_direction)) * target_speed
        target_dy = math.sin(math.radians(target_direction)) * target_speed

        now = time.monotonic()
        dt = now - self.last_update_time
        self.last_update_time = now

        delta_dx = target_dx - dx
        delta_dy = target_dy - dy
        delta_magnitude = math.hypot(delta_dx, delta_dy)

        max_step = MAX_VELOCITY_CHANGE_PER_SEC * dt
        if delta_magnitude > max_step and delta_magnitude > 0:
            scale = max_step / delta_magnitude
            delta_dx *= scale
            delta_dy *= scale

        new_dx = dx + delta_dx
        new_dy = dy + delta_dy

        self.current_direction = math.degrees(math.atan2(new_dy, new_dx))
        self.current_speed = math.hypot(new_dx, new_dy)

    def get_body_velocity(self):
        """Return measured body-frame velocity (vx, vy) in m/s from wheel QDR.

        Inverts the omni mixing used in _drive_loop:
        w_i = vx*sin(angle_off_i) - vy*cos(angle_off_i)
        """
        motors_config = self.config.get_value("motors", {})
        ata_00 = 0.0
        ata_01 = 0.0
        ata_11 = 0.0
        atb_0 = 0.0
        atb_1 = 0.0

        for motor_direction, motor in self.motors.items():
            motor.update_quick_data_readout()
            wheel_speed = motor.get_wheel_speed()
            angle_off = math.radians(motors_config[motor_direction]["angle_off"])
            sin_a = math.sin(angle_off)
            cos_a = math.cos(angle_off)

            ata_00 += sin_a * sin_a
            ata_01 -= sin_a * cos_a
            ata_11 += cos_a * cos_a
            atb_0 += sin_a * wheel_speed
            atb_1 -= cos_a * wheel_speed

        det = ata_00 * ata_11 - ata_01 * ata_01
        if abs(det) < 1e-9:
            return 0.0, 0.0

        vx = (ata_11 * atb_0 - ata_01 * atb_1) / det
        vy = (ata_00 * atb_1 - ata_01 * atb_0) / det
        return vx, vy

    def get_speed_in_direction(self, direction_deg):
        """Return signed speed (m/s) along a body-frame direction."""
        vx, vy = self.get_body_velocity()
        direction = math.radians(direction_deg)
        return vx * math.cos(direction) + vy * math.sin(direction)

    def stop(self):
        with self.target_lock:
            self.target_direction = 0
            self.target_speed = 0
            self.target_rotation = self.yaw

        self.current_direction = 0
        self.current_speed = 0

        for motor in self.motors.values():
            motor.set_speed(0)