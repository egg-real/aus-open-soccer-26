import numpy as np
import math
import time
from enum import Enum
from lib.dribbler import Dribbler
from lib.drive import Drive
from camera import Cameras
from lib.break_beam import BreakBeam
import board

from kicker import Kicker
from lib.comm_module import CommModule

USE_COMM_MODULE = False

SOLENOID_PIN = board.D27
COMM_MODULE_PIN = board.D18
PULSE_S = 0.02

# ----- MODES/STATES ----- #
class RobotState(Enum):
    NONE = -1
    NO_SEE_BALL = 0
    CHASING_BALL = 1
    HAVE_BALL = 2
    LINING_UP = 3
    CLUELESS = 4

class PossessionState(Enum):
    NONE = -1
    HEADING_TO_GOAL = 0
    BALL_HIDING = 1
    SPIN_SHOOT = 2 # Not coded yet

class GoalColour(Enum):
    BLUE = "Blue"
    YELLOW = "Yellow"

class RobotMode(Enum):
    PENALTY = 0
    OFFENCE = 1
    DEFENCE = 2

# ----- Main logic ----- #
class Robot():

    def on_startup(self):
        """Initialise"""

        # ----- SETTINGS ----- #
        # Target Goal
        self.target_goal = GoalColour.YELLOW

        # Role
        self.mode = RobotMode.OFFENCE
        self.PRIORITY_MODE = RobotMode.DEFENCE

        # Constants
        ## General
        self.BASE_BALL_CHASE_SPD = 1.0
        self.CLOSE_BALL_CHASE_SPD = 0.6
        self.HEAD_TO_GOAL_SPD = 0.6
        self.HEAD_TO_OWN_GOAL_SPD = 0.4
        self.BALL_ORBIT_RADIUS = 14  # might be an arbitrary number
        self.GIVE_UP_CHASING_BALL_TIME = 0.5 # seconds

        self.READY_TO_SHOOT_ANGLE = 15  # degrees
        self.READY_TO_SHOOT_DISTANCE = 125 # Note: This is currently unused

        self.READY_TO_REBOUND_SHOOT_ANGLE = 15
        self.READY_TO_REBOUND_SHOOT_DISTANCE = 50
        self.REBOUND_SHOOT_PRECISION = 0.5 # Somewhere inbetween 0 and 2, lower the more precise

        self.DRIBBLER_ROT_SPD = -1.0
        self.POSSESSION_ROT_SPD = 0.1
        self.SPIN_SHOT_SPEED = 0.4

        ## Ball Hiding TODO: Values & Thresholds to be tuned
        self.EDGE_BALL_HIDE_X_SPD = 0.3 # Speed to move towards wall
        self.EDGE_BALL_HIDE_Y_SPD = 0.1 # Speed to move towards goal
        self.EDGE_BALL_HIDE_READY_TO_SHOOT_DISTANCE = 70 # When gained possession of the ball, if x_coord is > this number, start edge hiding.
        self.CRAB_WALK_DIST_TO_WALL = 40 # Aim to keep this distance from wall when crab walking
        self.CRAB_WALK_ANGLE = 90 # Somewhere inbetween 45 to 135, 135 means it faces more towards own goal
        self.TRIGGER_BALL_HIDE_LINE_DIST = 40
        self.LINE_AVOID_THRESHOLD = 30  # close to cm, start filtering out-of-bounds component within this distance

        ## Ball capture PD control
        self.CAPTURE_WIDTH = 50 # Max lateral distance to decide to move forward (close to cm)
        self.CAPTURE_KP = 0.01
        self.CAPTURE_KD = 0.001

        ## Goalie tuning
        self.TURN_TO_OFFENCE_BALL_DIST = 15
        self.DEFENCE_GOAL_DIST = 40

        # Toggles
        self.ENABLE_EDGE_BALL_HIDE = False
        self.ENABLE_FLICK_SHOT = False
        self.ENABLE_REBOUND_SHOT = False


        # States
        self.state : RobotState = RobotState.NONE
        self.possession_state : PossessionState = PossessionState.NONE
        self.see_ball = False
        self.have_ball = False
        self.see_goal = False
        self.see_own_goal = False
        self.avoiding_line = False

        # Initialize Hardware Interfaces
        self.drive = Drive()
        self.cameras = Cameras()
        self.cameras.start_streaming()
        self.dribbler = Dribbler()
        self.break_beam = BreakBeam(board.D17)
        self.kicker = Kicker(SOLENOID_PIN, PULSE_S)
        if USE_COMM_MODULE:
            self.comm_module = CommModule(COMM_MODULE_PIN)

        # Variables
        ## Time
        self.last_time = time.monotonic()
        self.dt = 0

        ## Movement
        self.move_spd = 0  # 0-1 normalised speed
        self.move_dir = 0
        self.target_yaw = 0  # [-180, 180), 0 is front, clockwise is increasing

        ## Position & Orientation data
        self.bot_dir = 0  # Compass sensor

        ## Ball
        self.ball_dir = 0
        self.ball_dist = 100
        self.last_ball_dir = 0
        self.last_ball_dist = 0
        self.last_ball_see_time = time.monotonic()

        self.last_ball_x_error = 0 # Memory for ball_capture PD control

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

        self.ball_dir = self.wrap_angle(self.cameras.get_ball_dir())
        self.ball_dist = self.cameras.get_ball_dist()
        self.line_dir = self.wrap_angle(self.cameras.get_line_dir())
        self.line_dist = self.cameras.get_line_dist()

        if self.target_goal == GoalColour.BLUE:
            self.goal_dir = self.wrap_angle(self.cameras.get_blue_goal_dir())
            self.goal_dist = self.cameras.get_blue_goal_dist()
            self.own_goal_dir = self.wrap_angle(self.cameras.get_yellow_goal_dir())
            self.own_goal_dist = self.cameras.get_yellow_goal_dist()

        elif self.target_goal == GoalColour.YELLOW:
            self.goal_dir = self.wrap_angle(self.cameras.get_yellow_goal_dir())
            self.goal_dist = self.cameras.get_yellow_goal_dist()
            self.own_goal_dir = self.wrap_angle(self.cameras.get_blue_goal_dir())
            self.own_goal_dist = self.cameras.get_blue_goal_dist()

        else:
            print("Invalid target_goal:", self.target_goal)
            self.goal_dir = None
            self.goal_dist = None
            self.own_goal_dir = None
            self.own_goal_dist = None

        self.have_ball = self.break_beam.read()
        self.see_ball = self.ball_dir is not None and self.ball_dist is not None
        self.see_goal = self.goal_dir is not None and self.goal_dist is not None
        self.see_own_goal = self.own_goal_dir is not None and self.own_goal_dist is not None

        if self.see_ball or self.have_ball:
            self.last_ball_see_time = time.monotonic()

        # State machine
        self.execute_behaviour()

        if USE_COMM_MODULE and self.comm_module.read():
            self.drive.stop()
        else:
            # Execute movement
            self.move()


    # ------ State Machine ------ #
    # General logic
    def execute_behaviour(self):
        """Execute behaviour based on robot role"""
        self.target_yaw = 0

        if self.mode == RobotMode.OFFENCE:
            self.update_offence_state()
            self.execute_offence()

        elif self.mode == RobotMode.DEFENCE:
            self.execute_defence()

        elif self.mode == RobotMode.PENALTY:
            pass


    def update_offence_state(self):
        """Top-level state transitions"""

        ball_dir = self.ball_dir if self.ball_dir is not None else self.last_ball_dir

        if self.state == RobotState.NONE:
            self.state = RobotState.NO_SEE_BALL

        elif self.state == RobotState.NO_SEE_BALL:
            if self.see_ball:  # Found ball via camera
                self.state = RobotState.CHASING_BALL
            elif self.have_ball:  # Break beam triggered (gained possession without camera seeing)
                self.state = RobotState.HAVE_BALL
            elif not self.see_goal and not self.see_own_goal:
                self.state = RobotState.CLUELESS
        
        elif self.state == RobotState.CLUELESS:
            if self.see_ball:  # Found ball via camera
                self.state = RobotState.CHASING_BALL
            elif self.have_ball:  # Break beam triggered (gained possession without camera seeing)
                self.state = RobotState.HAVE_BALL
            elif self.see_goal or self.see_own_goal:
                self.state == RobotState.NO_SEE_BALL
            else:
                self.try_to_find_centre()

        elif self.state == RobotState.CHASING_BALL:
            if (time.monotonic() - self.last_ball_see_time) > self.GIVE_UP_CHASING_BALL_TIME:  # Lost sight of ball for some time
                self.state = RobotState.NO_SEE_BALL
                # self.mode = RobotMode.DEFENCE
            elif self.have_ball:  # Captured ball
                self.state = RobotState.HAVE_BALL
            elif (-30 <= ball_dir <= 30):
                self.state = RobotState.LINING_UP
                
        elif self.state == RobotState.LINING_UP:
            if (time.monotonic() - self.last_ball_see_time) > self.GIVE_UP_CHASING_BALL_TIME:
                self.state = RobotState.NO_SEE_BALL
                # self.mode = RobotMode.DEFENCE
            elif self.have_ball:
                self.state = RobotState.HAVE_BALL
            elif not (-15 <= ball_dir <= 15):
                self.state = RobotState.CHASING_BALL

        elif self.state == RobotState.HAVE_BALL:
            if not self.have_ball:  # Lost possession
                self.state = RobotState.CHASING_BALL
                self.possession_state = PossessionState.NONE
                return
            self.update_possession_state()

    def execute_offence(self):
        # State transitions for offence
        if self.state == RobotState.NO_SEE_BALL:
            self.move_dir = 0
            self.move_spd = 0

        elif self.state == RobotState.CHASING_BALL:
            self.ball_capture()

        elif self.state == RobotState.HAVE_BALL:
            self.execute_have_ball_behaviour()

        elif self.state == RobotState.LINING_UP:
            self.lining_up()

        elif self.state == RobotState.NONE:
            pass

    def execute_defence(self):
        # If ball very close, turn to attack mode
        if self.see_ball and self.ball_dist is not None and self.ball_dist < self.TURN_TO_OFFENCE_BALL_DIST:
            # self.mode = RobotMode.OFFENCE
            return

        # Stay near own goal
        if self.own_goal_dir is not None and self.own_goal_dist is not None and abs(self.own_goal_dist - self.DEFENCE_GOAL_DIST) > 5:
            # If ball is behind, try to swerve around it
            if self.see_ball and self.ball_dir is not None and abs(self.own_goal_dir - self.ball_dir) < 10:
                self.ball_capture()
            else:
                self.move_dir = self.own_goal_dir
                self.move_spd = self.HEAD_TO_OWN_GOAL_SPD
        else:
            if self.see_ball:
                self.ball_capture()
            else:
                self.move_dir = 0
                self.move_spd = 0.1

        # Face forward
        self.target_yaw = 0

    # Possession logic
    def update_possession_state(self):
        """Possession sub-state transitions while the bot has the ball."""
        if not self.have_ball:
            self.state = RobotState.CHASING_BALL
            self.possession_state = PossessionState.NONE
            return

        if self.possession_state == PossessionState.NONE:
            line_dist_E = self.line_dist(self.to_relative_dir(90))
            line_dist_W = self.line_dist(self.to_relative_dir(-90))
            if (self.ENABLE_EDGE_BALL_HIDE
                and line_dist_E is not None
                and line_dist_W is not None
                and min(line_dist_E, line_dist_W) < self.TRIGGER_BALL_HIDE_LINE_DIST):
                self.possession_state = PossessionState.BALL_HIDING
            else:
                self.possession_state = PossessionState.HEADING_TO_GOAL
            return

        if self.possession_state == PossessionState.HEADING_TO_GOAL:
            return

        if self.possession_state == PossessionState.BALL_HIDING:
            return

        if self.possession_state == PossessionState.SPIN_SHOOT:
            print("No code for this yet, please set ENABLE_FLICK_SHOT to False")
            return


    def execute_have_ball_behaviour(self):
        """Ball possession behaviour"""
        self.dribble()  # Keep dribbler running

        if self.possession_state == PossessionState.HEADING_TO_GOAL:
            if self.is_ready_to_shoot():
                self.stop_dribbler()
                self.kick()
                self.possession_state = PossessionState.NONE
                return
            if self.see_goal and self.goal_dir is not None:
                self.drive.dribbler_spin(self.dribbler, math.copysign(1, self.goal_dir), self.SPIN_SHOT_SPEED, self.goal_dir, self.break_beam, self.kicker)
            elif self.see_own_goal and self.own_goal_dir is not None:
                self.move_dir = -(self.own_goal_dir + 180) % 360
                self.move_spd = self.HEAD_TO_GOAL_SPD
                self.target_yaw = -(self.own_goal_dir + 180) % 360
                print(self.move_dir)
            else:
                self.try_to_find_centre()

        elif self.possession_state == PossessionState.BALL_HIDING:
            # Can normally shoot
            if self.is_ready_to_shoot():
                self.stop_dribbler()
                self.kick()
                self.possession_state = PossessionState.NONE
                return
            
            # Can rebound shoot
            elif self.ENABLE_REBOUND_SHOT and self.is_ready_to_rebound_shoot():
                self.stop_dribbler()
                self.kick()
                self.possession_state = PossessionState.NONE
                return
            
            # Close to goal, but no rebound shoot opportunity found
            elif self.goal_dist is not None and self.goal_dist < self.EDGE_BALL_HIDE_READY_TO_SHOOT_DISTANCE:
                self.target_yaw = self.bot_dir + np.sign(self.goal_dir)
            
            # Else move toward side wall
            else:
                field_side = np.sign(self.line_dist(self.to_relative_dir(-90)) - self.line_dist(self.to_relative_dir(90))) # right: 1, left: -1
                wall_error = self.CRAB_WALK_DIST_TO_WALL - self.line_dist(self.to_absolute_dir(self.CRAB_WALK_ANGLE))

                forward_dir = self.to_relative_dir(0)
                correction_dir = self.to_relative_dir(field_side * 90)

                correction_strength = 0.01 * wall_error  # TODO: Tune value

                # Convert to vectors
                forward_x = self.EDGE_BALL_HIDE_X_SPD * math.sin(math.radians(forward_dir))
                forward_y = self.EDGE_BALL_HIDE_Y_SPD * math.cos(math.radians(forward_dir))

                correction_x = correction_strength * math.sin(math.radians(correction_dir))
                correction_y = correction_strength * math.cos(math.radians(correction_dir))

                move_x = forward_x + correction_x
                move_y = forward_y + correction_y

                self.move_dir = math.degrees(math.atan2(move_x, move_y))
                self.move_spd = min(math.hypot(move_x, move_y), 1)
                    

        elif self.possession_state == PossessionState.SPIN_SHOOT:
            print("No code for this yet, please set ENABLE_FLICK_SHOT to False")
            self.move_dir = 0
            self.move_spd = 0


    def is_ready_to_shoot(self):
        print(self.goal_dir, self.goal_dist, self.own_goal_dir, self.own_goal_dist)
        return (
            self.have_ball
            and self.see_goal
            and self.goal_dir is not None
            and self.goal_dist is not None
            and abs(self.goal_dir) < self.READY_TO_SHOOT_ANGLE
            # and self.goal_dist < self.READY_TO_SHOOT_DISTANCE
        )
    
    def is_ready_to_rebound_shoot(self):
        # https://www.desmos.com/calculator/xbejygmwek

        if (
            self.have_ball
            and self.see_goal
            and self.goal_dir is not None
            and self.goal_dist is not None
            and abs(self.wrap_angle(self.bot_dir)) < 90 # Make sure it doesn't own goal
            and self.goal_dist < self.READY_TO_REBOUND_SHOOT_DISTANCE
        ):
            # The three angles of the triangle connecting the robot, goal and the point where the ball would hit the wall
            theta_1 = abs(self.bot_dir) + abs(self.goal_dir)
            theta_2 = 2 * abs(90 - abs(self.bot_dir))
            theta_3 = 180 - theta_1 - theta_2

            value = self.goal_dist * math.sin(math.radians(theta_3)) - self.wall_dist(0) * math.sin(math.radians(theta_2))  
            if abs(value) < self.REBOUND_SHOOT_PRECISION:
                return True
        return False

    def line_dist(self, relative_angle):
        pass

    def line_dir(self, relative_angle):
        # Please make this output an angle relative to the field
        pass

    def wall_dist(self, relative_angle):
        theta = self.line_dir(relative_angle) - self.bot_dir
        return self.line_dist(relative_angle) + 12 / math.cos(math.radians(theta)) # 12cm is the distance from the centre of the line to the wall

    def ball_capture(self):

        """Go around ball and try to capture it with dribbler"""
        self.move_spd = self.BASE_BALL_CHASE_SPD
        # Sorry, a lot of magic numbers here
        # https://yuta.techblog.jp/archives/40889399.html

        if self.see_ball:
            ball_dir = self.ball_dir
            ball_dist = self.ball_dist
        else:
            ball_dir = self.last_ball_dir
            ball_dist = self.last_ball_dist

        # Else if too close to ball, go away from it
        if ball_dist < self.BALL_ORBIT_RADIUS:
            distance_ratio = (self.BALL_ORBIT_RADIUS - ball_dist) / self.BALL_ORBIT_RADIUS
            orbit_angle = 90 + distance_ratio * 90
            self.move_dir = ball_dir + np.copysign(orbit_angle, ball_dir)
            self.move_spd = self.CLOSE_BALL_CHASE_SPD

        # Else move in an angle that is tangent to a circle centered at the ball
        else:
            if self.BALL_ORBIT_RADIUS / ball_dist > 1:
                print("arcsin argument is > 1 in ball_capture().") # This should never happen due to the if statement above but just in case something goes wrong
            else:
                self.move_dir = ball_dir + np.copysign(math.degrees(np.asin(self.BALL_ORBIT_RADIUS / ball_dist)), ball_dir)
            
                # Test logic to handle moving balls by adjusting movement speed
                distance_rate = (self.last_ball_dist - ball_dist) / self.dt if self.dt > 0 else 0
                expected_closing_rate = self.move_spd * math.cos(math.radians(abs(self.move_dir - ball_dir)))

                if expected_closing_rate > 10:
                    self.move_spd *= 2 - max(0.5, min(1.5, distance_rate / expected_closing_rate)) # Adjust movement speed (boost is ball is moving away, slow down if ball is moving closer)
        
        if self.avoiding_line and self.ball_dir is not None:
            self.target_yaw = self.ball_dir

    def lining_up(self):
        # print("LINING UP")
        ball_dir = self.ball_dir if self.ball_dir is not None else self.last_ball_dir
        ball_dist = self.ball_dist if self.ball_dist is not None else self.last_ball_dist
        ball_pos_x = ball_dist * math.sin(math.radians(ball_dir))

        # If ball is in front, move towards it
        if self.see_ball:
            # PD Calculations
            error_x = ball_pos_x
            derivative_x = (error_x - self.last_ball_x_error) / self.dt if self.dt > 0 else 0
            self.last_ball_x_error = error_x

            move_vel_x = (error_x * self.CAPTURE_KP) + (derivative_x * self.CAPTURE_KD) # PD
            move_vel_y = max(0, (math.sqrt(self.CAPTURE_WIDTH) - math.sqrt(abs(ball_pos_x))) / math.sqrt(self.CAPTURE_WIDTH) * self.BASE_BALL_CHASE_SPD) # Moves forward fast the more centered it is

            self.move_dir = math.degrees(math.atan2(move_vel_x, move_vel_y)) # Calculate direction of movement vector
            self.move_spd = math.sqrt(move_vel_x**2 + move_vel_y**2) # Calculate magnitude of movement vector

            if ball_dist < 50 or self.have_ball:
                self.dribble()
        else:
            self.dribble()
            self.move_dir = 0
            self.move_spd = 0.8
    
    def try_to_find_centre(self):
        # Get distances
        dist_N = self.line_dist(self.to_absolute_dir(0))
        dist_E = self.line_dist(self.to_absolute_dir(90))
        dist_S = self.line_dist(self.to_absolute_dir(-180))
        dist_W = self.line_dist(self.to_absolute_dir(-90))

        INF = 1e6
        dist_N = INF if dist_N is None else dist_N
        dist_E = INF if dist_E is None else dist_E
        dist_S = INF if dist_S is None else dist_S
        dist_W = INF if dist_W is None else dist_W
        eps = 1e-6

        # move away from line
        wx = (1.0 / (dist_W + eps)) - (1.0 / (dist_E + eps))
        wy = (1.0 / (dist_S + eps)) - (1.0 / (dist_N + eps))

        # convert to vector
        mag = math.hypot(wx, wy)

        self.move_dir = math.degrees(math.atan2(wx, wy))
        self.move_spd = min(1, mag * 40)

    # ------ Action Functions ------ #

    def move(self):
        move_dir, move_spd = self.avoid_line(self.move_dir, self.move_spd)
        self.drive.move(move_dir, move_spd, self.target_yaw, self.have_ball)
    
    def avoid_line(self, move_dir, move_spd):
        """
        If the bot is within LINE_AVOID_THRESHOLD of a line, remove the velocity
        component perpendicular to the line and keep the parallel component
        """

        self.avoiding_line = False
        if self.line_dist(move_dir) is None or self.line_dir(move_dir) is None:
            return move_dir, move_spd
        if self.line_dist(move_dir) >= self.LINE_AVOID_THRESHOLD:
            return move_dir, move_spd
        
        self.avoiding_line = True

        # Unit vector pointing toward the line (perpendicular-to-line axis)
        line_rad = math.radians(self.to_relative_dir(self.line_dir))
        toward_line_x = math.sin(line_rad)
        toward_line_y = math.cos(line_rad)

        # Decompose the intended movement vector into x and y
        move_rad = math.radians(move_dir)
        vx = move_spd * math.sin(move_rad)
        vy = move_spd * math.cos(move_rad)

        # Project movement onto the toward-line axis
        dot = vx * toward_line_x + vy * toward_line_y

        # Only suppress the component if it's moving toward the line (dot > 0)
        if dot > 0:
            vx -= dot * toward_line_x
            vy -= dot * toward_line_y

        new_spd = math.hypot(vx, vy)
        if new_spd < 1e-6:
            return move_dir, 0.0  # Fully blocked, keep dir but zero speed
        new_dir = math.degrees(math.atan2(vx, vy))
        return new_dir, new_spd

    def dribble(self):
        self.dribbler.set_speed(self.DRIBBLER_ROT_SPD)

    def stop_dribbler(self):
        self.dribbler.set_speed(0)

    def kick(self):
        self.kicker.kick()

    # ------ Utility Functions ------ #

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
        if theta == None:
            return None
        return (theta + 180) % 360 - 180


    def print_state(self):
        """Print current state for debugging"""
        ball_dir = f"{self.ball_dir:.1f}°" if self.ball_dir is not None else "None"
        ball_dist = f"{self.ball_dist:.0f}mm" if self.ball_dist is not None else "None"
        goal_dir = f"{self.goal_dir:.1f}°" if self.goal_dir is not None else "None"
        print(f"Mode: {self.mode.name} | State: {self.state.name} | Possession: {self.possession_state.name} | "
              f"Ball: {self.see_ball} (dir={ball_dir}, dist={ball_dist}) | "
              f"Goal: {self.see_goal} (dir={goal_dir})")

if __name__ == "__main__":
    bot1 = Robot()
    bot1.on_startup()

    try:
        while True:
            bot1.on_update()
            # bot1.print_state() # for debugging
            time.sleep(0.01)  # limit update rate

    except KeyboardInterrupt:
        print("Shutting down...")
        bot1.drive.stop()
        bot1.stop_dribbler()