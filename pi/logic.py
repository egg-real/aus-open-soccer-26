import numpy as np
import math
import time
from enum import Enum
import json
from lib.dribbler import Dribbler
from lib.drive import Drive
from camera import Cameras
from lib.break_beam import BreakBeam

# ----- Main Thing ----- #
class bot_states(Enum):
    NONE = -1
    NO_SEE_BALL = 0
    CHASING_BALL = 1
    HAVE_BALL = 2

class possession_states(Enum):
    NONE = -1
    HEADING_TO_GOAL = 0
    READY_TO_SHOOT = 1

    # Hopefully we have time to code these
    BALL_HIDING = 2
    SPIN_SHOOT = 3


class AttackBot():

    def on_startup(self):
        """Initialise"""
        # Load motor config
        with open("config.json") as f:
            config = json.load(f)
        
        # Constants
        self.BASE_BALL_CHASE_SPD = 0.3
        self.HEAD_TO_GOAL_SPD = 0.4
        self.HEAD_TO_OWN_GOAL_SPD = 0.2
        self.BALL_ORBIT_RADIUS = 300  # (mm if pixel-to-mm conversion is accurate, if not might be an arbitrary number)
        self.GIVE_UP_CHASING_BALL_TIME = 0.5 # seconds

        self.READY_TO_SHOOT_ANGLE = 15  # degrees
        self.READY_TO_SHOOT_DISTANCE = 200  # mm from goal

        self.DRIBBLER_ROT_SPD = 1.0

        self.YAW_CORRECT_KP = 0.1
        self.YAW_CORRECT_KD = 0

        # Toggles
        self.ENABLE_BALL_EDGE_HIDE = False
        self.ENABLE_BALL_SPIN_SHOOT = True

        self.target_goal = "Blue" # "Blue" or "Yellow", pls type it right

        # States
        self.state : bot_states = bot_states.NONE
        self.possession_state : possession_states = possession_states.NONE
        self.see_ball = False
        self.have_ball = False
        self.see_goal = False
        self.see_own_goal = False

        # Initialize hardware interfaces
        self.drive = Drive()  # Motor controller
        self.cameras = Cameras(["/dev/ttyAMA0"])  # Vision system
        self.dribbler = Dribbler()
        self.break_beam = BreakBeam("/dev/ttyAMA0")

        # Variables
        ## Time
        self.last_time = time.monotonic()
        self.dt = 0

        ## Movement
        self.move_spd = 0  # 0-1 normalised speed
        self.move_dir = 0
        self.target_yaw = 0

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
        """Main Loop"""
        # Update dt
        self.dt = time.monotonic() - self.last_time
        self.last_time = time.monotonic()
        
        # Update camera data
        self.cameras.process()

        self.ball_dir = self.cameras.get_ball_dir()
        self.ball_dist = self.cameras.get_ball_dist()

        if self.target_goal == "Blue":
            self.goal_dir = self.cameras.get_blue_goal_dir()
            self.goal_dist = self.cameras.get_blue_goal_dist()

            self.own_goal_dir = self.cameras.get_yellow_goal_dir()
            self.own_goal_dist = self.cameras.get_yellow_goal_dist()

        elif self.target_goal == "Yellow":
            self.goal_dir = self.cameras.get_yellow_goal_dir()
            self.goal_dist = self.cameras.get_yellow_goal_dist()

            self.own_goal_dir = self.cameras.get_blue_goal_dir()
            self.own_goal_dist = self.cameras.get_blue_goal_dist()
            
        # TODO: Add self.have_ball update
        

        # Update sensor states
        self.see_ball = self.ball_dir is not None and self.ball_dist is not None
        self.see_goal = self.goal_dir is not None
        if self.break_beam.read():
            self.see_ball = True
            self.have_ball = True
        
        # Update ball tracking
        if self.see_ball:
            self.last_ball_see_time = time.monotonic()

        # State machine
        self.update_main_state()
        self.execute_behaviour()
        
        # Execute movement
        self.move()


    # ------ State Machine ------ #
    # General logic
    def update_main_state(self):
        """Top-level state transitions"""
        if self.state == bot_states.NONE:
            self.state = bot_states.NO_SEE_BALL
        
        elif self.state == bot_states.NO_SEE_BALL:
            if self.see_ball: # Found ball
                self.state = bot_states.CHASING_BALL
        
        elif self.state == bot_states.CHASING_BALL:
            if (time.monotonic() - self.last_ball_see_time) > self.GIVE_UP_CHASING_BALL_TIME: # Lost sight of ball for some time
                self.state = bot_states.NO_SEE_BALL   
            elif self.have_ball: # Captured ball
                self.state = bot_states.HAVE_BALL
        
        elif self.state == bot_states.HAVE_BALL:
            self.update_possession_state()
            if not self.have_ball: # Lost possession
                self.state = bot_states.CHASING_BALL
                self.possession_state = possession_states.NONE


    def execute_behaviour(self):
        """Execute the current state's behaviour"""
        if self.state == bot_states.NO_SEE_BALL:
            self.target_yaw = self.wrap_angle(self.target_yaw + 1)
            # Search for ball or go defend goal
            # TODO: Implement search pattern (rotate, move to center, etc.)
            self.move_dir = 0
            self.move_spd = 0
        
        elif self.state == bot_states.CHASING_BALL:
            self.target_yaw = 0
            # Chase and acpture ball
            self.move_spd = self.BASE_BALL_CHASE_SPD
            self.ball_capture()
        
        elif self.state == bot_states.HAVE_BALL:
            self.execute_have_ball_behaviour()

        elif self.state == bot_states.NONE:
            pass

    
    # Possession logic
    def update_possession_state(self):
        """Possession sub-state transitions (only when HAVE_BALL)"""
        
        if self.possession_state == possession_states.NONE:
            self.possession_state = possession_states.HEADING_TO_GOAL # Default to heading to goal
        
        elif self.possession_state == possession_states.HEADING_TO_GOAL:
            if abs(self.goal_dir) < self.READY_TO_SHOOT_ANGLE and self.ball_dist < self.READY_TO_SHOOT_DISTANCE: # Goal is aligned and close enough!!
                self.possession_state = possession_states.READY_TO_SHOOT

            elif not self.have_ball: # Lost ball while taking it to goal
                self.state = bot_states.CHASING_BALL
                self.possession_state = possession_states.NONE
        
        elif self.possession_state == possession_states.READY_TO_SHOOT:
            if not self.have_ball: # Lost possession (either due to kicking it or losing it)
                self.state = bot_states.CHASING_BALL
                self.possession_state = possession_states.NONE
    

    def execute_have_ball_behaviour(self):
        """Ball possession behaviour"""
        self.dribble() # Keep dribbler running
        
        if self.possession_state == possession_states.HEADING_TO_GOAL:
            if self.see_goal:
                self.move_dir = self.goal_dir
                self.move_spd = self.HEAD_TO_GOAL_SPD
        
        elif self.possession_state == possession_states.READY_TO_SHOOT:
            self.dribbler.set_speed(0)
            self.kick()
        

    def ball_capture(self):
        """Ball capture algorithm from 2025"""
        # A lot of magic numbers here, I cbb making constants for all of them

        # If ball is in front, move towards it
        if -15 <= self.ball_dir <= 15:
            self.move_dir = self.ball_dir * 1.5
            if self.ball_dist < 200:
                self.dribble()

        # Else if too close to ball, go away from it
        elif self.ball_dist < 170:
            distance_ratio = (self.BALL_ORBIT_RADIUS - self.ball_dist) / self.BALL_ORBIT_RADIUS
            orbit_angle = 90 + distance_ratio * 90
            self.move_dir = self.ball_dir + np.copysign(orbit_angle, self.ball_dir)

        # Else move in an angle that is tangent to a circle centered at the ball
        else:
            self.move_dir = self.ball_dir + np.copysign(np.asin(self.BALL_ORBIT_RADIUS / self.ball_dist), self.ball_dir)

        
    # ------ Primitive actions ------ #

    def move(self):
        self.drive.move(self.move_dir, self.move_spd, self.target_yaw)

    def dribble(self):
        self.dribbler.set_speed(self.DRIBBLER_ROT_SPD)

    def kick(self):
        print("KICK")
        # TODO: actually kick
        pass

    # ------ Misc functions ------ #

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

    def print_state(self):
        """Print current state for debugging"""
        print(f"State: {self.state.name} | Possession: {self.possession_state.name} | "
              f"Ball: {self.see_ball} (dir={self.ball_dir:.1f}°, dist={self.ball_dist:.0f}mm) | "
              f"Goal: {self.see_goal} (dir={self.goal_dir:.1f}°)")
    



if __name__ == "__main__":
    bot1 = AttackBot()
    bot1.on_startup()
    
    try:
        while True:
            bot1.on_update()
            bot1.print_state() # for debugging
            time.sleep(0.01)  # limit update rate

    except KeyboardInterrupt:
        print("Shutting down...")
        bot1.drive.stop()
