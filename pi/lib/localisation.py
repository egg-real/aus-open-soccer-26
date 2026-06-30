import threading
import time

from pi.lib.drive import Drive

class Localisation:
    def __init__(self, imu, drive, tofs):
        self.imu = imu
        self.drive: Drive = drive
        self.tofs = tofs

        self.imu_last_acceleration_dx = 0
        self.imu_last_acceleration_dy = 0

        self.last_velocity_dx = 0
        self.last_velocity_dy = 0

        self.last_x = 0.5 * (1820 - tofs[1].read()) + 0.5 * tofs[3].read()
        self.last_y = 0.5 * (2430 - tofs[0].read()) + 0.5 * tofs[2].read()

        self.last_time = time.monotonic()
        self.dt = 0

        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _update_loop(self):
        motor_velocity = self.drive.get_body_velocity()
        
        for tof in self.tofs:
            distance = tof.read()
        
        imu_acceleration = self.imu.get_acceleration()

        # Update dt
        current_time = time.monotonic()
        self.dt = current_time - self.last_time
        self.last_time = current_time

        imu_veloctiy_dx = ((imu_acceleration[0] + self.imu_last_acceleration_dx) / 2) * self.dt
        imu_veloctiy_dy = ((imu_acceleration[1] + self.imu_last_acceleration_dy) / 2) * self.dt
        
        motor_x = ((motor_velocity[0] + self.last_velocity_dx) / 2) * self.dt
        motor_y = ((motor_velocity[1] + self.last_velocity_dy) / 2) * self.dt

        imu_x = ((imu_veloctiy_dx + self.last_velocity_dx) / 2) * self.dt
        imu_y = ((imu_veloctiy_dy + self.last_velocity_dy) / 2) * self.dt

        # Basic sensor fusion TODO: Improve with pf or EKF
        self.last_velocity_dx = 0.5 * imu_veloctiy_dx + 0.5 * motor_velocity[0]
        self.last_veloctiy_dy = 0.5 * imu_veloctiy_dy + 0.5 * motor_velocity[1]

        self.last_x = 0.5 * imu_x + 0.5 * motor_x
        self.last_y = 0.5 * imu_y + 0.5 * motor_y