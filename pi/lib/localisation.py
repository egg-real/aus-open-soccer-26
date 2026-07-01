import math
import threading
import time

import numpy as np

from lib.drive import Drive

# ----- Field geometry (mm) ----- #
FIELD_X = 1820.0
FIELD_Y = 2430.0

# ----- ToF layout ----- #
# Sensors are body-fixed. Order matches the tofs tuple passed in
# (0x50, 0x51, 0x52, 0x53) -> North(+y), East(+x), South(-y), West(-x).
# Body angles are measured clockwise from the robot front (0 deg = +y),
# matching the yaw convention used in drive.py / logic.py.
TOF_BODY_ANGLES_DEG = np.array([0.0, 90.0, 180.0, 270.0])
# Forward offset of each sensor from the robot centre along its own beam (mm).
# Distance read = (distance from centre to wall) - offset. Tune per build.
TOF_MOUNTING_OFFSET_MM = np.array([0.0, 0.0, 0.0, 0.0])

# ----- Sensor model ----- #
SIGMA_TOF_MM = 40.0            # Std dev of a good wall return.
SHORT_SCALE_MM = 300.0         # Decay scale for short (occlusion) returns.
# Mixture weights (hit / short-occlusion / uniform). Sum ~ 1.
# A beam can read short (an opponent occludes the wall) but never longer than
# the wall distance, so there is no max-range / beam-escape component.
W_HIT = 0.88
W_SHORT = 0.09
W_RAND = 0.03

# ----- Motion model ----- #
POS_PROCESS_SIGMA_MM = 12.0    # Per-step positional diffusion (wheel slip).
VEL_PROCESS_SIGMA_MM_S = 150.0  # Spread on the odometry velocity estimate.
MAX_DT = 0.5                   # Clamp dt to avoid huge jumps after a stall.

# ----- Slip / collision detection (IMU vs wheel odometry) ----- #
# Wheel odometry is only trustworthy while the wheels grip. During slip (wheels
# spin, robot doesn't move) or a collision/shove (robot moves, wheels don't),
# the IMU accelerometer and the odometry-derived acceleration disagree. When
# they do, we inflate the prediction noise so the filter leans on the ToF wall
# corrections instead of the (currently lying) odometry. The IMU is NOT used as
# a primary position integrator, only as a trust modulator on odometry.
SLIP_ACCEL_THRESHOLD_MS2 = 3.0   # Accel discrepancy below this is treated as normal.
SLIP_NOISE_GAIN = 1.5            # Extra noise multiplier per m/s^2 above threshold.
SLIP_NOISE_SCALE_MAX = 6.0       # Cap on the process-noise multiplier.
# Discrepancies are transient (they show up on the slip onset / impact cycle),
# so hold the inflated noise and let it decay back over a short window instead of
# resetting after one cycle. ~0.85/cycle at 100 Hz gives a ~100 ms cooldown.
SLIP_DECAY = 0.85

# ----- Particle filter ----- #
NUM_PARTICLES = 1000
RESAMPLE_THRESHOLD_RATIO = 0.5  # Resample when N_eff < ratio * N.
# Startup cloud spread, per axis. Robots begin lined up vertically (along y),
# so the North/South ToFs (which give y) are occluded by neighbouring robots and
# the y seed is unreliable; the East/West ToFs (x) see the side walls cleanly.
# Keep x tight and let y stay broad so the filter can collapse y once an N/S
# beam clears, instead of staying confident around a wrong seed.
INIT_SPREAD_X_MM = 150.0
INIT_SPREAD_Y_MM = 800.0

# ----- Augmented MCL (kidnapped-robot recovery) ----- #
ALPHA_SLOW = 0.01
ALPHA_FAST = 0.2

# ----- Output ----- #
LOCALIZED_STD_MM = 300.0        # Position std below which we call it localised.
LOOP_SLEEP_S = 0.01

_EPS_DIR = 1e-9
_TINY = 1e-300


class Localisation:
    def __init__(self, imu, drive, tofs, num_particles=NUM_PARTICLES):
        self.imu = imu
        self.drive: Drive = drive
        self.tofs = tofs
        self.num_particles = num_particles

        self._lock = threading.Lock()
        self._running = True

        # Particle state: columns [x, y, vx, vy]; positions mm, velocities mm/s.
        seed_x, seed_y = self._triangulate_position()
        self._particles = np.empty((num_particles, 4), dtype=np.float64)
        self._particles[:, 0] = np.clip(
            seed_x + np.random.normal(0.0, INIT_SPREAD_X_MM, num_particles), 0.0, FIELD_X
        )
        self._particles[:, 1] = np.clip(
            seed_y + np.random.normal(0.0, INIT_SPREAD_Y_MM, num_particles), 0.0, FIELD_Y
        )
        self._particles[:, 2] = 0.0
        self._particles[:, 3] = 0.0
        self._weights = np.full(num_particles, 1.0 / num_particles)

        # Published estimate.
        self._est_x = seed_x
        self._est_y = seed_y
        self._cov = np.array([[INIT_SPREAD_X_MM**2, 0.0], [0.0, INIT_SPREAD_Y_MM**2]])

        # Augmented-MCL running averages of measurement quality.
        self._w_slow = 0.0
        self._w_fast = 0.0

        # Slip/collision detection state.
        self._prev_body_velocity = (0.0, 0.0)
        self._slip_scale = 1.0

        self._last_time = time.monotonic()

        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    # ------ Sensor helpers ------ #
    def _read_tofs(self):
        """Return a list of raw ToF readings (mm), with None for failed reads."""
        readings = []
        for tof in self.tofs:
            try:
                readings.append(tof.read())
            except Exception:
                readings.append(None)
        return readings

    def _triangulate_position(self):
        """Single-shot position estimate from the ToFs (used to seed particles)."""
        readings = self._read_tofs()
        north, east, south, west = readings[0], readings[1], readings[2], readings[3]

        x_estimates = []
        if east is not None:
            x_estimates.append(FIELD_X - east)
        if west is not None:
            x_estimates.append(float(west))
        x = float(np.mean(x_estimates)) if x_estimates else FIELD_X / 2.0

        y_estimates = []
        if north is not None:
            y_estimates.append(FIELD_Y - north)
        if south is not None:
            y_estimates.append(float(south))
        y = float(np.mean(y_estimates)) if y_estimates else FIELD_Y / 2.0

        return float(np.clip(x, 0.0, FIELD_X)), float(np.clip(y, 0.0, FIELD_Y))

    def _get_yaw(self):
        """Field-referenced heading in degrees (0 = front/+y, clockwise +)."""
        yaw = getattr(self.drive, "yaw", None)
        if yaw is None:
            return 0.0
        return float(yaw)

    def _get_body_velocity(self):
        """Body-frame wheel velocity (m/s), or (0, 0) if the read fails."""
        try:
            vx_b, vy_b = self.drive.get_body_velocity()
            return float(vx_b), float(vy_b)
        except Exception:
            return 0.0, 0.0

    @staticmethod
    def _field_from_body(vx_b, vy_b, yaw_deg):
        """Rotate a body-frame velocity (m/s) into the field frame (mm/s)."""
        # vx_b = forward (towards front/+y at yaw 0), vy_b = rightward (+x at yaw 0).
        yaw = math.radians(yaw_deg)
        sin_y = math.sin(yaw)
        cos_y = math.cos(yaw)
        vfx = (vx_b * sin_y + vy_b * cos_y) * 1000.0
        vfy = (vx_b * cos_y - vy_b * sin_y) * 1000.0
        return vfx, vfy

    def _slip_noise_scale(self, vx_b, vy_b, dt):
        """Process-noise multiplier from the IMU-vs-odometry acceleration mismatch.

        Compares the magnitude of the wheel-odometry-derived horizontal
        acceleration against the IMU's measured horizontal acceleration. A large
        discrepancy means the wheels are slipping or the robot was hit, so the
        odometry prediction should be trusted less. Magnitudes are used (rather
        than per-axis vectors) so the check does not depend on precise alignment
        between the IMU axes and the wheel/body frame.
        """
        prev_vx, prev_vy = self._prev_body_velocity
        self._prev_body_velocity = (vx_b, vy_b)

        # Decay any previous slip inflation back toward 1.0 (the cooldown window).
        decayed = 1.0 + (self._slip_scale - 1.0) * SLIP_DECAY

        if dt <= 0.0 or self.imu is None:
            return decayed

        try:
            accel = self.imu.get_acceleration()
        except Exception:
            accel = None
        if accel is None:
            return decayed

        # Wheel-odometry acceleration magnitude (body frame, m/s^2).
        wheel_accel = math.hypot((vx_b - prev_vx) / dt, (vy_b - prev_vy) / dt)
        # Measured horizontal acceleration magnitude. The BNO08x accelerometer
        # includes gravity on z, so on a flat field ax, ay are ~horizontal.
        imu_accel = math.hypot(float(accel[0]), float(accel[1]))

        discrepancy = abs(imu_accel - wheel_accel)
        target = 1.0
        if discrepancy > SLIP_ACCEL_THRESHOLD_MS2:
            target = min(
                1.0 + SLIP_NOISE_GAIN * (discrepancy - SLIP_ACCEL_THRESHOLD_MS2),
                SLIP_NOISE_SCALE_MAX,
            )

        # Rise instantly to a fresh disturbance, then fall off via the decay.
        return max(target, decayed)

    # ------ Filter steps ------ #
    def _predict(self, yaw_deg):
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        dt = max(0.0, min(dt, MAX_DT))

        vx_b, vy_b = self._get_body_velocity()
        self._slip_scale = self._slip_noise_scale(vx_b, vy_b, dt)
        vfx, vfy = self._field_from_body(vx_b, vy_b, yaw_deg)

        # During slip/collision the odometry is untrustworthy, so widen the
        # prediction spread and let the ToF corrections dominate this cycle.
        vel_sigma = VEL_PROCESS_SIGMA_MM_S * self._slip_scale
        pos_sigma = POS_PROCESS_SIGMA_MM * self._slip_scale
        n = self.num_particles

        # Treat odometry as a noisy control input on per-particle velocity.
        self._particles[:, 2] = vfx + np.random.normal(0.0, vel_sigma, n)
        self._particles[:, 3] = vfy + np.random.normal(0.0, vel_sigma, n)

        self._particles[:, 0] += (
            self._particles[:, 2] * dt + np.random.normal(0.0, pos_sigma, n)
        )
        self._particles[:, 1] += (
            self._particles[:, 3] * dt + np.random.normal(0.0, pos_sigma, n)
        )

        np.clip(self._particles[:, 0], 0.0, FIELD_X, out=self._particles[:, 0])
        np.clip(self._particles[:, 1], 0.0, FIELD_Y, out=self._particles[:, 1])

    def _expected_distance(self, x, y, dx, dy):
        """Vectorized ray-rectangle intersection.

        Returns the distance from each particle at (x, y) to the first field
        wall hit by a ray in field direction (dx, dy). Walls: x=0, x=FIELD_X,
        y=0, y=FIELD_Y.
        """
        candidates = []
        if dx > _EPS_DIR:
            candidates.append((FIELD_X - x) / dx)
        elif dx < -_EPS_DIR:
            candidates.append((0.0 - x) / dx)

        if dy > _EPS_DIR:
            candidates.append((FIELD_Y - y) / dy)
        elif dy < -_EPS_DIR:
            candidates.append((0.0 - y) / dy)

        if not candidates:
            return np.full_like(x, np.inf)

        t = np.minimum.reduce(candidates) if len(candidates) > 1 else candidates[0]
        # Guard against tiny negative values from numerical noise.
        return np.where(t > 0.0, t, np.inf)

    def _beam_likelihood(self, z, expected):
        """Robust mixture likelihood for one beam, scaled to a [W_RAND, ~1] peak.

        A reading can fall short of the true wall distance (an opponent occludes
        the beam), but never overshoot it, so the model has no max-range term.
        """
        diff = z - expected
        p_hit = np.exp(-0.5 * (diff / SIGMA_TOF_MM) ** 2)
        p_short = np.where(z < expected, np.exp(-z / SHORT_SCALE_MM), 0.0)
        return W_HIT * p_hit + W_SHORT * p_short + W_RAND

    def _update_weights(self, readings, yaw_deg):
        """Reweight particles from the ToF readings. Returns measurement quality."""
        loglik = np.zeros(self.num_particles)
        n_used = 0

        for i, z in enumerate(readings):
            if z is None or z < 0:
                continue
            beam_angle = math.radians(yaw_deg + TOF_BODY_ANGLES_DEG[i])
            dx = math.sin(beam_angle)
            dy = math.cos(beam_angle)

            t = self._expected_distance(
                self._particles[:, 0], self._particles[:, 1], dx, dy
            )
            expected = np.clip(t - TOF_MOUNTING_OFFSET_MM[i], 0.0, None)
            loglik += np.log(self._beam_likelihood(float(z), expected) + _TINY)
            n_used += 1

        if n_used == 0:
            return None

        # Per-particle quality is the geometric mean over beams, in [W_RAND, ~1],
        # which stays numerically stable and drives Augmented-MCL injection.
        quality = float(np.mean(np.exp(loglik / n_used)))

        log_w = np.log(self._weights + _TINY) + loglik
        log_w -= log_w.max()
        w = np.exp(log_w)
        w_sum = w.sum()
        if w_sum <= 0 or not np.isfinite(w_sum):
            self._weights = np.full(self.num_particles, 1.0 / self.num_particles)
        else:
            self._weights = w / w_sum

        return quality

    def _compute_estimate(self):
        w = self._weights
        mean_x = float(np.sum(w * self._particles[:, 0]))
        mean_y = float(np.sum(w * self._particles[:, 1]))

        dx = self._particles[:, 0] - mean_x
        dy = self._particles[:, 1] - mean_y
        cov_xx = float(np.sum(w * dx * dx))
        cov_yy = float(np.sum(w * dy * dy))
        cov_xy = float(np.sum(w * dx * dy))
        cov = np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]])
        return mean_x, mean_y, cov

    def _maybe_resample(self, quality):
        # Augmented-MCL: track fast/slow averages of measurement quality.
        if self._w_slow == 0.0:
            self._w_slow = quality
            self._w_fast = quality
        else:
            self._w_slow += ALPHA_SLOW * (quality - self._w_slow)
            self._w_fast += ALPHA_FAST * (quality - self._w_fast)

        p_inject = 0.0
        if self._w_slow > 0.0:
            p_inject = max(0.0, 1.0 - self._w_fast / self._w_slow)

        n_eff = 1.0 / np.sum(self._weights**2)
        if n_eff >= RESAMPLE_THRESHOLD_RATIO * self.num_particles and p_inject <= 0.0:
            return

        n = self.num_particles
        n_inject = int(round(n * p_inject))
        n_keep = n - n_inject

        new_particles = np.empty_like(self._particles)
        if n_keep > 0:
            idx = self._systematic_resample(self._weights, n_keep)
            new_particles[:n_keep] = self._particles[idx]
        if n_inject > 0:
            new_particles[n_keep:, 0] = np.random.uniform(0.0, FIELD_X, n_inject)
            new_particles[n_keep:, 1] = np.random.uniform(0.0, FIELD_Y, n_inject)
            new_particles[n_keep:, 2] = 0.0
            new_particles[n_keep:, 3] = 0.0
            # Reset the fast average so we don't inject forever after recovery.
            self._w_fast = self._w_slow

        self._particles = new_particles
        self._weights = np.full(n, 1.0 / n)

    @staticmethod
    def _systematic_resample(weights, n):
        positions = (np.arange(n) + np.random.random()) / n
        cumulative = np.cumsum(weights)
        cumulative[-1] = 1.0
        return np.searchsorted(cumulative, positions)

    def _update_loop(self):
        while self._running:
            yaw = self._get_yaw()
            self._predict(yaw)

            readings = self._read_tofs()
            quality = self._update_weights(readings, yaw)

            mean_x, mean_y, cov = self._compute_estimate()
            with self._lock:
                self._est_x = mean_x
                self._est_y = mean_y
                self._cov = cov

            if quality is not None:
                self._maybe_resample(quality)

            time.sleep(LOOP_SLEEP_S)

    # ------ Public API ------ #
    def get_position(self):
        with self._lock:
            return self._est_x, self._est_y

    def get_pose(self):
        with self._lock:
            return self._est_x, self._est_y, self._get_yaw()

    def get_covariance(self):
        with self._lock:
            return self._cov.copy()

    def get_position_std(self):
        with self._lock:
            return math.sqrt(max(self._cov[0, 0], 0.0)), math.sqrt(max(self._cov[1, 1], 0.0))

    def is_localized(self):
        std_x, std_y = self.get_position_std()
        return std_x < LOCALIZED_STD_MM and std_y < LOCALIZED_STD_MM

    def get_slip_scale(self):
        """Current process-noise multiplier (1.0 = gripping, higher = slip/impact)."""
        return self._slip_scale

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
