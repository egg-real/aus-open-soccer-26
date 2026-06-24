import numpy as np
import math
import time
from enum import Enum
import json
from lib.dribbler import Dribbler
from lib.drive import Drive
from camera import Cameras
from lib.break_beam import BreakBeam
import board

# ----- Main Thing ----- #
class BotStates(Enum):
    NONE = -1
    NO_SEE_BALL = 0
    CHASING_BALL = 1
    HAVE_BALL = 2

class PossessionStates(Enum):
    NONE = -1
    HEADING_TO_GOAL = 0
    READY_TO_SHOOT = 1

    # Hopefully we have time to code these
    BALL_HIDING = 2
    SPIN_SHOOT = 3

class GoalColour(Enum):
    BLUE = "Blue"
    YELLOW = "Yellow"


class AttackBot():

    def on_startup(self):
        """Initialise"""
        # Load motor config
        with open("/home/dsa/Robotics/config.json") as f:
            config = json.load(f)

        # ----- TARGET GOAL ----- #
        self.target_goal = GoalColour.BLUE
        self.TARGET_GOAL_Y = 110  # TODO: Tune value
        
        # Constants
        ## General
        self.BASE_BALL_CHASE_SPD = 0.6
        self.HEAD_TO_GOAL_SPD = 0.6
        self.HEAD_TO_OWN_GOAL_SPD = 0.4
        self.BALL_ORBIT_RADIUS = 14  # might be an arbitrary number
        self.GIVE_UP_CHASING_BALL_TIME = 1.5 # seconds

        self.READY_TO_SHOOT_ANGLE = 15  # degrees
        self.READY_TO_SHOOT_DISTANCE = 40

        self.DRIBBLER_ROT_SPD = -1.0
        self.POSSESSION_ROT_SPD = 0.1

        ## Ball Hiding TODO: Values & Thresholds to be tuned
        self.EDGE_BALL_HIDE_X_SPD = 0.3
        self.EDGE_BALL_HIDE_Y_SPD = 0.1
        self.EDGE_BALL_HIDE_MIN_X = 40 # When gained possession of the ball, if x_coord is > this number, start edge hiding. 
        self.CRAB_WALK_STOP_Y = 70 # When crab walking across the side, if y_coord is > this number, stop and turn towards the goal. 
        self.CRAB_WALK_X = 75 # When crab walking across the side, aim to be at this x coord 

        # Ball capture PD control
        self.CAPTURE_WIDTH = 4 # Max lateral distance to decide to move forward (close to cm)
        self.CAPTURE_KP = 0.8
        self.CAPTURE_KD = 0.1

        self.prev_ball_x_error = 0.0 # Memory for the PD controller
        
        


        # Toggles
        self.ENABLE_EDGE_BALL_HIDE = False
        self.ENABLE_BALL_SPIN_SHOOT = False

        # States
        self.state : BotStates = BotStates.NONE
        self.possession_state : PossessionStates = PossessionStates.NONE
        self.see_ball = False
        self.have_ball = False
        self.see_goal = False
        self.see_own_goal = False

        # Initialize Hardware Interfaces
        self.drive = Drive()
        self.cameras = Cameras()
        self.cameras.start_streaming()
        self.dribbler = Dribbler()
        self.break_beam = BreakBeam(board.D17)

        # Variables
        ## Time
        self.last_time = time.monotonic()
        self.dt = 0

        ## Movement
        self.move_spd = 0  # 0-1 normalised speed
        self.move_dir = 0
        self.target_yaw = 0  # [-180, 180), 0 is front, positive is clockwise

        ## Position & Orientation data
        self.x_coord = 0
        self.y_coord = 0  
        self.bot_dir = 0  # Compass sensor

        ## Ball
        self.ball_dir = 0
        self.ball_dist = 100
        self.last_ball_dir = 0
        self.last_ball_dist = 0
        self.last_ball_see_time = time.monotonic()

        self.last_ball_x_error = 0 # Memory for ball capture PD controller

        ## Goal
        self.goal_dir = 0
        self.own_goal_dir = 180

        ## Line
        self.line_dir = None
        self.line_dist = None

        ## Bot Communication
        self.other_bot_have_ball = False
        self.other_bot_see_ball = False
    

    def on_update(self):
        """Main Loop"""
        # Update dt
        current_time = time.monotonic()
        self.dt = current_time - self.last_time
        self.last_time = current_time
        
        # Update camera data
        self.cameras.process()

        if self.ball_dir is not None and self.ball_dist is not None:
            self.last_ball_dir = self.ball_dir
            self.last_ball_dist = self.ball_dist
            self.last_ball_see_time = time.monotonic()

        self.ball_dir = self.cameras.get_ball_dir()
        self.ball_dist = self.cameras.get_ball_dist()
        self.line_dir = self.cameras.get_line_dir()
        self.line_dist = self.cameras.get_line_dist()

        if self.target_goal == GoalColour.BLUE:
            self.goal_dir = self.cameras.get_blue_goal_dir()
            self.goal_dist = self.cameras.get_blue_goal_dist()
            self.own_goal_dir = self.cameras.get_yellow_goal_dir()
            self.own_goal_dist = self.cameras.get_yellow_goal_dist()

        elif self.target_goal == GoalColour.YELLOW:
            self.goal_dir = self.cameras.get_yellow_goal_dir()
            self.goal_dist = self.cameras.get_yellow_goal_dist()
            self.own_goal_dir = self.cameras.get_blue_goal_dir()
            self.own_goal_dist = self.cameras.get_blue_goal_dist()

        else:
            print("Invalid target_goal:", self.target_goal)
            self.goal_dir = None
            self.goal_dist = None
            self.own_goal_dir = None
            self.own_goal_dist = None

        self.have_ball = self.break_beam.read()
        self.see_ball = self.have_ball or (self.ball_dir is not None and self.ball_dist is not None)
        self.see_goal = self.goal_dir is not None

        # TODO: Update self.x_coord and self.y_coord if localisation works
    
        # State machine
        self.update_main_state()
        self.execute_behaviour()
        
        # Execute movement
        self.move()


    # ------ State Machine ------ #
    # General logic
    def update_main_state(self):
        """Top-level state transitions"""
        if self.state == BotStates.NONE:
            self.state = BotStates.NO_SEE_BALL
        
        elif self.state == BotStates.NO_SEE_BALL:
            if self.see_ball:  # Found ball via camera
                self.state = BotStates.CHASING_BALL
            elif self.have_ball:  # Break beam triggered (gained possession without camera seeing)
                self.state = BotStates.HAVE_BALL
        
        elif self.state == BotStates.CHASING_BALL:
            if (time.monotonic() - self.last_ball_see_time) > self.GIVE_UP_CHASING_BALL_TIME:  # Lost sight of ball for some time
                self.state = BotStates.NO_SEE_BALL   
            elif self.have_ball:  # Captured ball
                self.state = BotStates.HAVE_BALL
        
        elif self.state == BotStates.HAVE_BALL:
            if not self.have_ball:  # Lost possession
                self.state = BotStates.CHASING_BALL
                self.possession_state = PossessionStates.NONE
                return
            self.update_possession_state()


    def execute_behaviour(self):
        """Execute the current state's behaviour"""
        self.target_yaw = 0
        if self.state == BotStates.NO_SEE_BALL:
            # self.target_yaw = self.wrap_angle(self.target_yaw + 1)
            # Search for ball or go defend goal
            # TODO: Implement search pattern (rotate, move to center, etc.)
            self.move_dir = 0
            self.move_spd = 0
        
        elif self.state == BotStates.CHASING_BALL:
            # Chase and capture ball
            self.ball_capture()
        
        elif self.state == BotStates.HAVE_BALL:
            self.execute_have_ball_behaviour()

        elif self.state == BotStates.NONE:
            pass

    
    # Possession logic
    def update_possession_state(self):
        """Possession sub-state transitions while the bot has the ball."""
        if not self.have_ball:
            self.state = BotStates.CHASING_BALL
            self.possession_state = PossessionStates.NONE
            return

        if self.possession_state == PossessionStates.NONE:
            if self.ENABLE_EDGE_BALL_HIDE and abs(self.x_coord) > self.EDGE_BALL_HIDE_MIN_X:
                self.possession_state = PossessionStates.BALL_HIDING
            else:
                self.possession_state = PossessionStates.HEADING_TO_GOAL
            return

        if self.possession_state == PossessionStates.HEADING_TO_GOAL:
            if self.is_ready_to_shoot():
                self.possession_state = PossessionStates.READY_TO_SHOOT
            return

        if self.possession_state == PossessionStates.BALL_HIDING:
            if self.is_ready_to_shoot():
                self.possession_state = PossessionStates.READY_TO_SHOOT
            return

        if self.possession_state == PossessionStates.SPIN_SHOOT:
            print("No code for this yet, please set ENABLE_BALL_SPIN_SHOOT to False")
            return

        if self.possession_state == PossessionStates.READY_TO_SHOOT:
            if not self.is_ready_to_shoot():
                self.possession_state = PossessionStates.HEADING_TO_GOAL
            return


    def execute_have_ball_behaviour(self):
        """Ball possession behaviour"""
        self.dribble()  # Keep dribbler running

        if self.possession_state == PossessionStates.HEADING_TO_GOAL:
            if self.see_goal and self.goal_dir is not None:
                self.move_dir = self.goal_dir
                self.move_spd = self.HEAD_TO_GOAL_SPD
                self.target_yaw = self.goal_dir
            else:
                self.move_dir = 0
                self.move_spd = 0

        elif self.possession_state == PossessionStates.BALL_HIDING:
            if self.is_ready_to_shoot():
                self.move_dir = 0
                self.move_spd = 0
                return

            if self.y_coord > self.CRAB_WALK_STOP_Y:  # Close to goal, try to align kicker with goal
                if self.see_goal and self.goal_dir is not None:
                    self.target_yaw = self.to_absolute_dir(self.goal_dir)
                    self.move_dir = 0
                    self.move_spd = 0
                else:
                    goal_dir = self.angle_towards(self.x_coord, self.y_coord, 0, self.TARGET_GOAL_Y)
                    self.move_dir = goal_dir
                    self.move_spd = self.EDGE_BALL_HIDE_Y_SPD
                    self.target_yaw = goal_dir
                return

            line_dir = self.line_dir
            if line_dir is None:
                self.move_dir = 0
                self.move_spd = 0
                return

            self.target_yaw = line_dir

            if abs(self.x_coord) < self.CRAB_WALK_X:
                self.move_dir = line_dir
                self.move_spd = self.EDGE_BALL_HIDE_X_SPD
            else:
                self.move_dir = 0
                self.move_spd = self.EDGE_BALL_HIDE_Y_SPD

        elif self.possession_state == PossessionStates.SPIN_SHOOT:
            print("No code for this yet, please set ENABLE_BALL_SPIN_SHOOT to False")
            self.move_dir = 0
            self.move_spd = 0

        elif self.possession_state == PossessionStates.READY_TO_SHOOT:
            self.move_dir = 0
            self.move_spd = 0
            self.dribbler.set_speed(0)
            self.kick()


    def is_ready_to_shoot(self):
        return (
            self.have_ball
            and self.see_goal
            and self.goal_dir is not None
            and self.ball_dist is not None
            and abs(self.goal_dir) < self.READY_TO_SHOOT_ANGLE
            and self.ball_dist < self.READY_TO_SHOOT_DISTANCE
        )
        

    def ball_capture(self):
        """Go around ball and try to capture it with dribbler"""
        self.move_spd = self.BASE_BALL_CHASE_SPD
        # Sorry, a lot of magic numbers here, I cbb making constants for all of them
        # https://yuta.techblog.jp/archives/40889399.html

        if self.see_ball:
            ball_dir = self.ball_dir
            ball_dist = self.ball_dist
        else:
            ball_dir = self.last_ball_dir
            ball_dist = self.last_ball_dist
        ball_pos_x = ball_dist * math.sin(math.radians(ball_dir))

        # If ball is in front, move towards it
        if self.have_ball or (-15 <= ball_dir <= 15 and abs(ball_pos_x) < self.CAPTURE_WIDTH):
            # PD Calculations
            error_x = ball_pos_x
            derivative_x = (error_x - self.prev_ball_x_error) / self.dt if self.dt > 0 else 0
            self.prev_ball_x_error = error_x

            move_vel_x = (error_x * self.CAPTURE_KP) + (derivative_x * self.CAPTURE_KD) # PD
            move_vel_y = (math.sqrt(self.CAPTURE_WIDTH) - math.sqrt(abs(ball_pos_x))) / math.sqrt(self.CAPTURE_WIDTH) * self.BASE_BALL_CHASE_SPD # Moves forward fast the more centered it is

            self.move_dir = math.degrees(math.atan2(move_vel_x, move_vel_y)) # Calculate direction of movement vector
            self.move_spd = math.sqrt(move_vel_x**2 + move_vel_y**2) # Calculate magnitude of movement vector

            if ball_dist < 50 or self.have_ball:
                self.dribble()

        # Else if too close to ball, go away from it
        elif ball_dist < 20:
            distance_ratio = (self.BALL_ORBIT_RADIUS - ball_dist) / self.BALL_ORBIT_RADIUS
            orbit_angle = 90 + distance_ratio * 90
            self.move_dir = ball_dir + np.copysign(orbit_angle, ball_dir)

        # Else move in an angle that is tangent to a circle centered at the ball
        else:
            if self.BALL_ORBIT_RADIUS / ball_dist > 1:
                print("arcsin argument is > 1. Adjust BALL_ORBIT_RADIUS")
            else:
                self.move_dir = ball_dir + np.copysign(math.degrees(np.asin(self.BALL_ORBIT_RADIUS / ball_dist)), ball_dir)

        
    # ------ Primitive actions ------ #

    def move(self):
        self.drive.move(self.move_dir, self.move_spd, self.target_yaw, self.have_ball)

    def dribble(self):
        self.dribbler.set_speed(self.DRIBBLER_ROT_SPD)

    def kick(self):
        print("KICK")
        # TODO: actually kick

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
        ball_dir = f"{self.ball_dir:.1f}°" if self.ball_dir is not None else "None"
        ball_dist = f"{self.ball_dist:.0f}mm" if self.ball_dist is not None else "None"
        goal_dir = f"{self.goal_dir:.1f}°" if self.goal_dir is not None else "None"
        print(f"State: {self.state.name} | Possession: {self.possession_state.name} | "
              f"Ball: {self.see_ball} (dir={ball_dir}, dist={ball_dist}) | "
              f"Goal: {self.see_goal} (dir={goal_dir})")
    



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
