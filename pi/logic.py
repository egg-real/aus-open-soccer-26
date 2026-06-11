import numpy as np
import math
import time
from enum import Enum
import json
from lib.drive import Drive
from camera import Cameras

# low level stuff done by copilot

# ----- Main Thing ----- #
class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0
        self.error_prev = 0
    
    def update(self, error, dt):
        proportional = error * self.kp
        self.integral = (self.integral + error * dt) * self.ki
        derivative = (error - self.error_prev) / dt * self.kd
        
        self.error_prev = error

        return proportional + self.integral + derivative

class bot_states(Enum):
        # Add more later
        NONE = -1
        NO_SEE_BALL = 0
        CHASING_BALL = 1


class attack_bot():

    def on_startup(self):
        # Load motor config
        with open("config.json") as f:
            config = json.load(f)
        
        # Constants (from config.json)
        self.BASE_BALL_CHASE_SPD = 0.3
        self.HEAD_TO_GOAL_SPD = 0.4
        self.HEAD_TO_OWN_GOAL_SPD = 0.2
        self.BALL_ORBIT_RADIUS = 300  # (mm if pixel-to-mm conversion is accurate, if not might be an arbitrary number)
        
        ## Control
        self.YAW_CORRECT_SPD = 0.1

        # Toggles
        self.can_detect_possession = False
        self.enable_ball_edge_hide = False

        # States
        self.state : bot_states = bot_states.NONE
        self.see_ball = False
        self.have_ball = False
        self.see_goal = False
        self.see_own_goal = False

        # Initialize hardware interfaces
        self.drive = Drive()  # Motor controller
        self.cameras = Cameras(["/dev/ttyAMA0"], naive=True)  # Vision system

        # Variables
        ## Time
        self.last_time = time.monotonic()
        self.dt = 0

        ## Movement
        self.move_spd = 0.3  # 0-1 normalized speed
        self.move_dir = 0
        self.rot_dir = 0
        self.yaw_correct_PID = PID(1, 0, 0)

        ## Bot data
        self.bot_dir = 0  # Compass sensor

        ## Environment data
        ### Ball
        self.ball_dir = 0
        self.ball_dist = 500
        self.last_ball_dir = 0
        self.last_ball_pos = (0, 0)
        self.last_ball_see_time = time.monotonic()

        ### Goal
        self.goal_dir = 0
        self.own_goal_dir = 180

    def on_update(self):
        # Update dt
        self.dt = time.monotonic() - self.last_time
        self.last_time = time.monotonic()
        
        # Update sensor data
        self.ball_dir = self.cameras.get_ball_dir() or self.ball_dir
        self.ball_dist = self.cameras.get_ball_dist() or self.ball_dist
        self.goal_dir = self.cameras.get_blue_goal_dir() or self.goal_dir
        
        # Update state
        self.see_ball = self.cameras.get_ball_dir() is not None
        self.see_goal = self.cameras.get_blue_goal_dir() is not None

        # Logic
        if self.have_ball:
            if self.see_goal:
                self.move_dir = self.goal_dir
            else:
                pass
        else:
            self.ball_capture()
        
        # Execute movement
        self.execute_movement()

    def ball_capture(self):
        if self.see_ball:

            # If ball is in front, move towards it
            if -15 <= self.ball_dir <= 15:
                self.move_dir = self.ball_dir * 1.5

            # Else if too close to ball, go away from it
            elif self.ball_dist < 170:
                distance_ratio = (self.BALL_ORBIT_RADIUS - self.ball_dist) / self.BALL_ORBIT_RADIUS
                orbit_angle = 90 + distance_ratio * 90
                self.move_dir = self.ball_dir + np.copysign(orbit_angle, self.ball_dir)

            # Else move in an angle that is tangent to a circle centered at the ball
            else:
                self.move_dir = self.ball_dir + np.copysign(np.asin(self.BALL_ORBIT_RADIUS / self.ball_dist), self.ball_dir)
        else:
            pass  # Move to centre or own goal?

    def execute_movement(self):
        """Convert move_dir and move_spd into Drive commands"""
        # Normalize movement direction to radians for Drive.move()
        x = np.sin(np.radians(self.move_dir)) * self.move_spd
        y = np.cos(np.radians(self.move_dir)) * self.move_spd
        
        self.drive.move(x, y, 0)  # Assuming Drive.move(x, y, rotation)

    def yaw_correct(self, target_angle):
        error = target_angle - self.bot_dir
        self.yaw_correct_PID.update(error, self.dt)

    def kick(self):
        pass

    # ------ Helper functions ------ #
    def to_absolute_dir(self, relative_dir): 
        """Input a direction relative to the bot orientation\nReturns a direction that ignores bot orientation"""
        return relative_dir + self.bot_dir

    def to_relative_dir(self, absolute_dir):
        """Input a direction that ignores bot orientation\nReturns a direction relative to the bot orientation"""
        return absolute_dir - self.bot_dir

    def angle_towards(self, bot_x, bot_y, obj_x, obj_y):
        """Returns angle in degrees"""
        theta = math.degrees(math.atan2(obj_y - bot_y, obj_x - bot_x))
        theta = (theta - 90 + 180) % 360 - 180
        return theta

    def wrap_angle(self, theta):
        """Input an angle in degrees\nReturns same angle but in [-180,180)"""
        return (theta + 180) % 360 - 180


if __name__ == "__main__":
    bot1 = attack_bot()
    bot1.on_startup()
    
    try:
        while True:
            bot1.on_update()
            time.sleep(0.01)  # limits update rate
    except KeyboardInterrupt:
        print("Shutting down...")
        bot1.drive.stop()