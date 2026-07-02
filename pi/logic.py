import numpy as np
import math
import time
from enum import Enum
from lib.dribbler import Dribbler
from lib.drive import Drive
from lib.camera import Cameras
from lib.break_beam import BreakBeam
import board

from lib.kicker import Kicker
from lib.comm_module import CommModule
from lib.communication import Communication
from lib.config import Config
from lib.imu import IMU
from lib.localisation import FIELD_X, FIELD_Y, Localisation
from lib.switch import Switch
from lib.tof import ToF

USE_COMM_MODULE = True
USE_COMMUNICATION = False

SOLENOID_PIN = board.D27
COMM_MODULE_PIN = board.D18
PULSE_S = 0.02

# Goal positions in localisation field coordinates (mm).
BLUE_GOAL_POSITION = (0.0, FIELD_Y / 2.0)
YELLOW_GOAL_POSITION = (FIELD_X, FIELD_Y / 2.0)

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
    FLICK_SHOT = 2 # Not coded yet

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

        # Role
        self.PRIORITY_MODE = RobotMode.OFFENCE
        self.mode = self.PRIORITY_MODE

        # Constants
        ## General
        self.BASE_BALL_CHASE_SPD = 0.1
        self.CLOSE_BALL_CHASE_SPD = 0.01
        self.HEAD_TO_GOAL_SPD = 0.1
        self.HEAD_TO_OWN_GOAL_SPD = 0.1
        self.BALL_ORBIT_RADIUS = 14  # might be an arbitrary number
        self.GIVE_UP_CHASING_BALL_TIME = 0.5 # seconds

        self.READY_TO_SHOOT_ANGLE = 20  # degrees
        self.READY_TO_SHOOT_DISTANCE = 125 # Note: This is currently unused

        self.READY_TO_REBOUND_SHOOT_ANGLE = 15
        self.READY_TO_REBOUND_SHOOT_DISTANCE = 50
        self.REBOUND_SHOOT_PRECISION = 0.5 # Somewhere inbetween 0 and 2, lower the more precise

        self.GOALIE_MAX_ANGLE_FROM_GOAL = 15
        self.DEFENCE_GOAL_DIST = 80

        self.DRIBBLER_ROT_SPD = 1.0
        self.POSSESSION_ROT_SPD = 0.1
        self.SPIN_SHOT_SPEED = 0.4

        ## Ball Hiding TODO: Values & Thresholds to be tuned
        self.EDGE_BALL_HIDE_X_SPD = 0.3 # Speed to move towards wall
        self.EDGE_BALL_HIDE_Y_SPD = 0.1 # Speed to move towards goal
        self.EDGE_BALL_HIDE_READY_TO_SHOOT_DISTANCE = 70 # When gained possession of the ball, if x_coord is > this number, start edge hiding.
        self.CRAB_WALK_DIST_TO_WALL = 40 # Aim to keep this distance from wall when crab walking
        self.TRIGGER_BALL_HIDE_WALL_DIST = 40
        self.WALL_AVOID_THRESHOLD = 30  # cm, start filtering into-wall movement within this distance

        ## Ball capture PD control
        self.CAPTURE_WIDTH = 50 # Max lateral distance to decide to move forward (close to cm)
        self.CAPTURE_KP = 0.01
        self.CAPTURE_KD = 0.001

        ## Goalie tuning
        self.TURN_TO_OFFENCE_BALL_DIST = 15
        self.DEFENCE_MAXIMUM_DISTANCE_FROM_WALL = 100
        self.DEFENCE_MINIMUM_DISTANCE_FROM_WALL = 50

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
        self.avoiding_wall = False

        # Initialize Hardware Interfaces
        self.imu = IMU()
        self.config = Config()
        self.drive = Drive(self.imu, self.config)
        self.cameras = Cameras()
        self.cameras.start_streaming()
        self.dribbler = Dribbler(self.config)
        self.break_beam = BreakBeam(board.D17)
        self.kicker = Kicker(SOLENOID_PIN, PULSE_S)
        if USE_COMM_MODULE:
            self.comm_module = CommModule(COMM_MODULE_PIN)
        self.pause_switch = Switch(board.D16, self.config)
        self.goal_switch = Switch(board.D12, self.config)
        self.communication = Communication()
        self.tofs = (ToF(0x50), ToF(0x51), ToF(0x52), ToF(0x53))

        self.localisation = Localisation(self.imu, self.drive, self.tofs)

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
        self.nearest_wall_dir = None
        self.nearest_wall_dist = None

        ## Ball
        self.ball_dir = 0
        self.ball_dist = 100
        self.last_ball_dir = 0
        self.last_ball_dist = 0
        self.last_ball_see_time = time.monotonic()

        self.last_ball_x_error = 0 # Memory for ball_capture PD control

        ## Goal
        self.target_goal = GoalColour.YELLOW if self.goal_switch.read() else GoalColour.BLUE
        self.goal_dir = 0
        self.goal_dist = None
        self.localized_goal_dist = None
        self.own_goal_dir = 180
        self.own_goal_dist = None
        self.bot_position = None
        self.bot_position_std = None
        self.bot_localized = False

        ## Bot Communication
        self.other_bot_have_ball = False
        self.other_bot_see_ball = False
        self.robot_id = self.communication.client_id
        self.other_bot_status = None
        self.pending_attack_request = None
        self.pending_attack_request_time = None
        self.pending_defence_request = None
        self.pending_defence_request_time = None
        self.acknowledged_attack_requests = set()
        self.acknowledged_defence_requests = set()
        self.ROLE_HANDOFF_TIMEOUT_S = 1.0

        if USE_COMMUNICATION:
            self.communication.on_message = self.handle_communication_message
            self.communication.start()


    def on_update(self):
        """Main Loop"""
        # Update dt
        current_time = time.monotonic()
        self.dt = current_time - self.last_time
        self.last_time = current_time

        # Update orientation data
        self.bot_dir = self.drive.yaw

        # Update camera data
        self.cameras.process()

        if self.ball_dir is not None and self.ball_dist is not None:
            self.last_ball_dir = self.ball_dir
            self.last_ball_dist = self.ball_dist
            self.last_ball_see_time = time.monotonic()

        self.ball_dir = self.wrap_angle(self.cameras.get_ball_dir())
        self.ball_dist = self.cameras.get_ball_dist()

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
        self.update_localisation_state()

        if self.see_ball or self.have_ball:
            self.last_ball_see_time = time.monotonic()

        if self.pause_switch.read() or (USE_COMM_MODULE and not self.comm_module.read()):
            self.drive.stop()
            self.stop_dribbler()
            return

        # State machine
        self.execute_behaviour()

        if USE_COMMUNICATION:
            self.maybe_request_attack_handoff()
            self.maybe_request_defence_handoff()
            self.communication.send(self.build_status_message())

        # Execute movement
        self.move()


    # ------ Bot Communication ------ #
    def communication_number(self, value):
        if value is None:
            return None
        return float(value)

    def update_localisation_state(self):
        self.bot_position = self.localisation.get_position()
        self.bot_position_std = self.localisation.get_position_std()
        self.bot_localized = self.localisation.is_localized()
        self.localized_goal_dist = self.distance_to_target_goal(self.bot_position)

    def communication_position(self, position):
        if position is None:
            return None
        x, y = position
        return {
            "x": self.communication_number(x),
            "y": self.communication_number(y),
        }

    def position_from_message(self, message):
        position = message.get("position")
        if not isinstance(position, dict):
            return None

        x = position.get("x")
        y = position.get("y")
        if x is None or y is None:
            return None

        try:
            return float(x), float(y)
        except (TypeError, ValueError):
            return None

    def message_goal_dist(self, message):
        position = self.position_from_message(message)
        if position is not None:
            return self.distance_to_target_goal(position)
        return message.get("goal_dist")

    def target_goal_position(self):
        if self.target_goal == GoalColour.BLUE:
            return BLUE_GOAL_POSITION
        if self.target_goal == GoalColour.YELLOW:
            return YELLOW_GOAL_POSITION
        return None

    def distance_to_target_goal(self, position):
        goal_position = self.target_goal_position()
        if position is None or goal_position is None:
            return None

        return math.hypot(
            float(position[0]) - goal_position[0],
            float(position[1]) - goal_position[1],
        )

    def build_status_message(self):
        attack_request = None
        if self.pending_attack_request is not None:
            attack_request = self.pending_attack_request

        defence_request = None
        if self.pending_defence_request is not None:
            defence_request = self.pending_defence_request

        return {
            "type": "status",
            "mode": self.mode.name,
            "state": self.state.name,
            "possession_state": self.possession_state.name,
            "see_ball": self.see_ball,
            "have_ball": self.have_ball,
            "ball_dist": self.communication_number(self.ball_dist),
            "position": self.communication_position(self.bot_position),
            "position_std": self.communication_position(self.bot_position_std),
            "localized": self.bot_localized,
            "goal_dist": self.communication_number(self.localized_goal_dist),
            "own_goal_dist": self.communication_number(self.own_goal_dist),
            "attack_priority": self.has_attack_priority(),
            "attack_request": attack_request,
            "defence_request": defence_request,
        }

    def handle_communication_message(self, payload, _topic):
        sender = payload.get("sender")
        if sender is None or sender == self.robot_id:
            return

        message = payload.get("message", payload)
        if not isinstance(message, dict):
            return

        message_type = message.get("type")
        if message_type == "status":
            self.handle_status_message(sender, message)
        elif message_type == "attack_request":
            self.handle_attack_request(sender, message)
        elif message_type == "attack_ack":
            self.handle_attack_ack(message)
        elif message_type == "defence_request":
            self.handle_defence_request(sender, message)
        elif message_type == "defence_ack":
            self.handle_defence_ack(message)

    def handle_status_message(self, sender, message):
        self.other_bot_status = {
            "sender": sender,
            "received_at": time.monotonic(),
            **message,
        }
        self.other_bot_see_ball = bool(message.get("see_ball", False))
        self.other_bot_have_ball = bool(message.get("have_ball", False))

        attack_request = message.get("attack_request")
        if isinstance(attack_request, dict):
            self.handle_attack_request(sender, attack_request)

        defence_request = message.get("defence_request")
        if isinstance(defence_request, dict):
            self.handle_defence_request(sender, defence_request)

    def handle_attack_request(self, sender, request):
        if request.get("requester") != sender:
            return
        if request.get("target_mode") != RobotMode.OFFENCE.name:
            return
        if self.mode != RobotMode.DEFENCE:
            return
        if self.pending_attack_request is not None:
            if self.PRIORITY_MODE == RobotMode.OFFENCE:
                return
            self.pending_attack_request = None
            self.pending_attack_request_time = None
            self.send_attack_ack(sender, request)
            return

        requester_goal_dist = self.message_goal_dist(request)
        if requester_goal_dist is None or self.localized_goal_dist is None:
            return
        if requester_goal_dist > self.localized_goal_dist:
            return

        self.send_attack_ack(sender, request)

    def send_attack_ack(self, sender, request):
        self.communication.send({
            "type": "attack_ack",
            "request_id": request.get("request_id"),
            "target": sender,
            "acknowledged_by": self.robot_id,
        })

    def handle_attack_ack(self, message):
        if message.get("target") != self.robot_id:
            return
        if self.pending_attack_request is None:
            return
        if message.get("request_id") != self.pending_attack_request.get("request_id"):
            return

        self.acknowledged_attack_requests.add(message["request_id"])
        self.pending_attack_request = None
        self.pending_attack_request_time = None
        self.mode = RobotMode.OFFENCE

    def handle_defence_request(self, sender, request):
        if request.get("requester") != sender:
            return
        if request.get("target_mode") != RobotMode.DEFENCE.name:
            return
        if self.mode != RobotMode.OFFENCE:
            return
        if self.pending_defence_request is not None:
            if self.PRIORITY_MODE == RobotMode.DEFENCE:
                return
            self.pending_defence_request = None
            self.pending_defence_request_time = None
            self.send_defence_ack(sender, request)
            return
        if not self.should_accept_defence_request(request):
            return

        self.send_defence_ack(sender, request)

    def send_defence_ack(self, sender, request):
        self.communication.send({
            "type": "defence_ack",
            "request_id": request.get("request_id"),
            "target": sender,
            "acknowledged_by": self.robot_id,
        })

    def handle_defence_ack(self, message):
        if message.get("target") != self.robot_id:
            return
        if self.pending_defence_request is None:
            return
        if message.get("request_id") != self.pending_defence_request.get("request_id"):
            return

        self.acknowledged_defence_requests.add(message["request_id"])
        self.pending_defence_request = None
        self.pending_defence_request_time = None
        self.mode = RobotMode.DEFENCE

    def has_attack_priority(self):
        return (
            self.have_ball
            or (
                self.see_ball
                and self.ball_dist is not None
                and self.ball_dist < self.TURN_TO_OFFENCE_BALL_DIST
            )
        )

    def status_has_attack_priority(self, status):
        if status.get("attack_priority") is not None:
            return bool(status.get("attack_priority"))
        return (
            bool(status.get("have_ball", False))
            or (
                bool(status.get("see_ball", False))
                and status.get("ball_dist") is not None
                and status.get("ball_dist") < self.TURN_TO_OFFENCE_BALL_DIST
            )
        )

    def _wins_tiebreak(self):
        """Deterministic fallback so both robots never drop back at once."""
        other_id = ""
        if self.other_bot_status is not None:
            other_id = self.other_bot_status.get("sender", "")
        return self.robot_id < other_id

    def _self_is_closer(self, self_dist, other_dist):
        """True if self should win a "closer is better" contest (lower distance)."""
        if self_dist is None and other_dist is None:
            return self._wins_tiebreak()
        if self_dist is None:
            return False
        if other_dist is None:
            return True
        if self_dist < other_dist:
            return True
        if self_dist > other_dist:
            return False
        return self._wins_tiebreak()

    def offence_contest_self_attacks(self):
        """Decide which of two offence robots should remain the attacker.

        Returns True if this robot should attack, False if it should drop to
        defence. Priority order: attack priority, then ball possession, then
        closeness to the ball; if neither has priority, closeness to the goal.
        """
        status = self.other_bot_status
        self_priority = self.has_attack_priority()
        other_priority = self.status_has_attack_priority(status)

        if self_priority != other_priority:
            return self_priority

        if self_priority and other_priority:
            self_have = self.have_ball
            other_have = bool(status.get("have_ball", False))
            if self_have != other_have:
                return self_have
            return self._self_is_closer(self.ball_dist, status.get("ball_dist"))

        return self._self_is_closer(self.localized_goal_dist, self.message_goal_dist(status))

    def should_accept_defence_request(self, request):
        if self.mode != RobotMode.OFFENCE or self.other_bot_status is None:
            return False
        return self.offence_contest_self_attacks() is True

    def maybe_request_attack_handoff(self):
        if self.mode != RobotMode.DEFENCE:
            self.pending_attack_request = None
            self.pending_attack_request_time = None
            return
        if self.localized_goal_dist is None or self.other_bot_status is None:
            return
        if self.other_bot_status.get("mode") != RobotMode.DEFENCE.name:
            self.pending_attack_request = None
            self.pending_attack_request_time = None
            return

        other_goal_dist = self.message_goal_dist(self.other_bot_status)
        if other_goal_dist is None:
            return

        now = time.monotonic()
        should_attack = self.localized_goal_dist < other_goal_dist
        if not should_attack:
            self.pending_attack_request = None
            self.pending_attack_request_time = None
            return

        if self.pending_attack_request is not None:
            if now - self.pending_attack_request_time < self.ROLE_HANDOFF_TIMEOUT_S:
                return
            self.pending_attack_request = None
            self.pending_attack_request_time = None

        request_id = f"{self.robot_id}-{int(now * 1000)}"
        self.pending_attack_request = {
            "type": "attack_request",
            "request_id": request_id,
            "requester": self.robot_id,
            "target_mode": RobotMode.OFFENCE.name,
            "position": self.communication_position(self.bot_position),
            "goal_dist": self.communication_number(self.localized_goal_dist),
            "created_at": now,
        }
        self.pending_attack_request_time = now

    def maybe_request_defence_handoff(self):
        if self.mode != RobotMode.OFFENCE:
            self.pending_defence_request = None
            self.pending_defence_request_time = None
            return
        if self.localized_goal_dist is None or self.other_bot_status is None:
            return
        if self.other_bot_status.get("mode") != RobotMode.OFFENCE.name:
            self.pending_defence_request = None
            self.pending_defence_request_time = None
            return

        now = time.monotonic()
        self_has_priority = self.has_attack_priority()
        should_defend = not self.offence_contest_self_attacks()

        if not should_defend:
            self.pending_defence_request = None
            self.pending_defence_request_time = None
            return

        if self.pending_defence_request is not None:
            if now - self.pending_defence_request_time < self.ROLE_HANDOFF_TIMEOUT_S:
                return
            self.pending_defence_request = None
            self.pending_defence_request_time = None

        request_id = f"{self.robot_id}-{int(now * 1000)}"
        self.pending_defence_request = {
            "type": "defence_request",
            "request_id": request_id,
            "requester": self.robot_id,
            "target_mode": RobotMode.DEFENCE.name,
            "position": self.communication_position(self.bot_position),
            "goal_dist": self.communication_number(self.localized_goal_dist),
            "attack_priority": self_has_priority,
            "created_at": now,
        }
        self.pending_defence_request_time = now

    def request_mode_change(self, mode):
        if mode != RobotMode.OFFENCE:
            self.mode = mode
            return

        if not USE_COMMUNICATION:
            self.mode = mode
            return

        self.maybe_request_attack_handoff()


    # ------ State Machine ------ #
    # General logic
    def execute_behaviour(self):
        """Execute behaviour based on robot role"""
        self.target_yaw = 0
        # print(f"State: {self.state.name}")

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
                self.state = RobotState.NO_SEE_BALL
            else:
                self.try_to_find_centre()

        elif self.state == RobotState.CHASING_BALL:
            if (time.monotonic() - self.last_ball_see_time) > self.GIVE_UP_CHASING_BALL_TIME:  # Lost sight of ball for some time
                self.state = RobotState.NO_SEE_BALL
                self.mode = self.PRIORITY_MODE
            elif self.have_ball:  # Captured ball
                self.state = RobotState.HAVE_BALL
            elif (-30 <= ball_dir <= 30):
                self.state = RobotState.LINING_UP
                
        elif self.state == RobotState.LINING_UP:
            if (time.monotonic() - self.last_ball_see_time) > self.GIVE_UP_CHASING_BALL_TIME:
                self.state = RobotState.NO_SEE_BALL
                self.mode = self.PRIORITY_MODE
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
            self.try_to_find_centre()

        elif self.state == RobotState.CLUELESS:
            self.try_to_find_centre()

        elif self.state == RobotState.CHASING_BALL:
            self.ball_capture()

        elif self.state == RobotState.HAVE_BALL:
            self.execute_have_ball_behaviour()

        elif self.state == RobotState.LINING_UP:
            self.lining_up()

        elif self.state == RobotState.NONE:
            pass
    

    # Defence Logic
    def execute_defence(self):
        self.target_yaw = self.to_absolute_dir(self.ball_dir)
        if self.target_yaw is None:
            self.target_yaw = 0

        # If ball very close, turn to attack mode
        if self.have_ball or (
            self.see_ball 
            and self.ball_dist is not None
            and self.ball_dist < self.TURN_TO_OFFENCE_BALL_DIST
            and (
                abs(self.wrap_angle(self.to_absolute_dir(self.ball_dir))) < 120
                or (self.own_goal_dist is not None and self.own_goal_dist > 80)
            ) # Ball not too behind or not too close to goal (reduce risk of own goaling)
            ):

            previous_mode = self.mode
            self.request_mode_change(RobotMode.OFFENCE)
            if self.mode != previous_mode:
                return

        # Stay near own goal if too far away from it
        elif self.own_goal_dir is not None and self.own_goal_dist is not None and abs(self.own_goal_dist - self.DEFENCE_GOAL_DIST) > 10:
            # If ball is behind, try to swerve around it (may trigger attack handover request anyway)
            if self.see_ball and self.ball_dir is not None and abs(self.wrap_angle(self.own_goal_dir - self.ball_dir)) < 20:
                self.ball_capture()
            else:
                self.move_dir = self.own_goal_dir
                self.move_spd = self.HEAD_TO_OWN_GOAL_SPD
        else:
            if self.see_ball:
                if self.own_goal_dir is not None:
                    self.execute_goalie()
                else:
                    self.ball_capture()
            else:
                # TODO: Stay centred
                self.move_dir = 0
                self.move_spd = 0
    
    def execute_goalie(self):
        inline_error = self.wrap_angle(self.own_goal_dir - self.ball_dir - 180)
        if abs(inline_error) > 30:
            self.move_dir = 90 + np.sign(inline_error) * 90
            self.move_spd = 1.0
        else:
            self.move_dir = 0
            self.move_spd = 0
        self.target_yaw = self.to_absolute_dir(self.ball_dir)

        # if self.ball_dir < 10 and (180 - self.goal_dir) % 360 < self.GOALIE_MAX_ANGLE_FROM_GOAL:
        #     self.move_dir = 270
        #     self.move_spd = 1.0
        # elif self.ball_dir > 10 and (180 - self.goal_dir) % 360 > -self.GOALIE_MAX_ANGLE_FROM_GOAL:
        #     self.move_dir = 90
        #     self.move_spd = 1.0
        # else:
        #     self.move_dir = 0
        #     self.move_spd = 1.0
        # self.target_yaw = self.wrap_angle(self.bot_dir + self.ball_dir)


    # Possession Logic
    def update_possession_state(self):
        """Possession sub-state transitions while the bot has the ball."""
        if not self.have_ball:
            self.state = RobotState.CHASING_BALL
            self.possession_state = PossessionState.NONE
            return

        if self.possession_state == PossessionState.NONE:
            wall_dists = self.axis_wall_dists()
            wall_dist_E = wall_dists["E"]
            wall_dist_W = wall_dists["W"]
            if (self.ENABLE_EDGE_BALL_HIDE
                and wall_dist_E is not None
                and wall_dist_W is not None
                and min(wall_dist_E, wall_dist_W) < self.TRIGGER_BALL_HIDE_WALL_DIST):
                self.possession_state = PossessionState.BALL_HIDING
            else:
                self.possession_state = PossessionState.HEADING_TO_GOAL
            return

        if self.possession_state == PossessionState.HEADING_TO_GOAL:
            return

        if self.possession_state == PossessionState.BALL_HIDING:
            return

        if self.possession_state == PossessionState.FLICK_SHOT:
            print("No code for this yet, please set ENABLE_FLICK_SHOT to False")
            return


    def execute_have_ball_behaviour(self):
        """Ball possession behaviour"""
        self.dribble()  # Keep dribbler running

        if self.possession_state == PossessionState.HEADING_TO_GOAL:
            if self.is_ready_to_shoot() or self.is_ready_to_rebound_shoot():
                self.stop_dribbler()
                self.kick()
                self.possession_state = PossessionState.NONE
                return
            if self.see_goal and self.goal_dir is not None:
                # Head toward the goal; the possession orbit in Drive aims us by
                # orbiting the ball toward target_yaw, and is_ready_to_shoot kicks.
                self.move_dir = self.goal_dir
                self.move_spd = self.HEAD_TO_GOAL_SPD
                self.target_yaw = self.to_absolute_dir(self.goal_dir)
            elif self.see_own_goal and self.own_goal_dir is not None:
                away_from_own_goal_dir = self.wrap_angle(self.own_goal_dir + 180)
                self.move_dir = away_from_own_goal_dir
                self.move_spd = self.HEAD_TO_GOAL_SPD
                self.target_yaw = self.to_absolute_dir(away_from_own_goal_dir)
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
                wall_dists = self.axis_wall_dists()
                wall_dist_E = wall_dists["E"]
                wall_dist_W = wall_dists["W"]
                if wall_dist_E is None or wall_dist_W is None:
                    self.try_to_find_centre()
                    return

                field_side = 1 if wall_dist_E < wall_dist_W else -1
                side_wall_dist = wall_dist_E if field_side == 1 else wall_dist_W
                wall_error = side_wall_dist - self.CRAB_WALK_DIST_TO_WALL

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
                    

        elif self.possession_state == PossessionState.FLICK_SHOT:
            print("No code for this yet, please set ENABLE_FLICK_SHOT to False")
            self.move_dir = 0
            self.move_spd = 0


    def is_ready_to_shoot(self):
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

            wall_dist = self.wall_dist(0)
            if wall_dist is None:
                return False

            value = self.goal_dist * math.sin(math.radians(theta_3)) - wall_dist * math.sin(math.radians(theta_2))  
            if abs(value) < self.REBOUND_SHOOT_PRECISION:
                return True
        return False

    def axis_wall_dists(self):
        """Return N/E/S/W wall distances in cm from the localized field position."""
        if self.bot_position is None:
            return {"N": None, "E": None, "S": None, "W": None}

        x, y = self.bot_position
        if x is None or y is None:
            return {"N": None, "E": None, "S": None, "W": None}

        x = min(max(float(x), 0.0), FIELD_X)
        y = min(max(float(y), 0.0), FIELD_Y)
        return {
            "N": (FIELD_Y - y) / 10.0,
            "E": (FIELD_X - x) / 10.0,
            "S": y / 10.0,
            "W": x / 10.0,
        }

    def field_wall_dist(self, field_angle):
        """Distance in cm to the field wall along a field-absolute bearing."""
        if self.bot_position is None:
            return None

        x, y = self.bot_position
        if x is None or y is None:
            return None
        x = min(max(float(x), 0.0), FIELD_X)
        y = min(max(float(y), 0.0), FIELD_Y)

        angle = math.radians(field_angle)
        dx = math.sin(angle)
        dy = math.cos(angle)
        candidates = []

        if dx > 1e-9:
            candidates.append((FIELD_X - x) / dx)
        elif dx < -1e-9:
            candidates.append((0.0 - x) / dx)

        if dy > 1e-9:
            candidates.append((FIELD_Y - y) / dy)
        elif dy < -1e-9:
            candidates.append((0.0 - y) / dy)

        positive_dists = [dist for dist in candidates if dist > 0]
        if not positive_dists:
            return None
        return min(positive_dists) / 10.0

    def wall_dist(self, relative_angle):
        return self.field_wall_dist(self.to_absolute_dir(relative_angle))

    def ball_capture(self):

        """Go around ball and try to capture it with dribbler"""
        # Sorry, a lot of magic numbers here
        # https://yuta.techblog.jp/archives/40889399.html

        if self.see_ball:
            ball_dir = self.ball_dir
            ball_dist = self.ball_dist
        else:
            ball_dir = self.last_ball_dir
            ball_dist = self.last_ball_dist
        
        if self.see_ball and self.ball_dist is not None and self.ball_dist > 50:
            self.stop_dribbler()

        # Speed scales with distance; BASE_BALL_CHASE_SPD applies at 2x orbit radius
        self.move_spd = max(
            self.CLOSE_BALL_CHASE_SPD,
            min(1.0, self.BASE_BALL_CHASE_SPD * ball_dist / (2 * self.BALL_ORBIT_RADIUS)),
        )

        # Else if too close to ball, go away from it
        if ball_dist < self.BALL_ORBIT_RADIUS:
            distance_ratio = (self.BALL_ORBIT_RADIUS - ball_dist) / self.BALL_ORBIT_RADIUS
            orbit_angle = 90 + distance_ratio * 90
            self.move_dir = ball_dir + np.copysign(orbit_angle, ball_dir)

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
        
        if self.avoiding_wall and self.ball_dir is not None:
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
            # self.target_yaw = self.wrap_angle(self.ball_dir + self.bot_dir)

            if not self.see_ball or ball_dist < 50 or self.have_ball:
                self.dribble()
        else:
            self.dribble()
            self.move_dir = 0
            self.move_spd = 0.05
    
    def try_to_find_centre(self):
        if self.bot_position is None:
            self.move_dir = 0
            self.move_spd = 0
            return

        x, y = self.bot_position
        if x is None or y is None:
            self.move_dir = 0
            self.move_spd = 0
            return

        centre_x = FIELD_X / 2.0
        centre_y = FIELD_Y / 2.0
        dx = centre_x - float(x)
        dy = centre_y - float(y)
        dist_mm = math.hypot(dx, dy)

        if dist_mm < 100:
            self.move_dir = 0
            self.move_spd = 0
            return

        absolute_dir = math.degrees(math.atan2(dx, dy))
        self.move_dir = self.to_relative_dir(absolute_dir)
        self.move_spd = min(self.HEAD_TO_GOAL_SPD, max(0.03, dist_mm / 1000.0 * self.HEAD_TO_GOAL_SPD))

    # ------ Action Functions ------ #

    def move(self):
        move_dir, move_spd = self.avoid_wall(self.move_dir, self.move_spd)
        print(move_dir, move_spd, self.target_yaw, self.have_ball)
        self.drive.move(move_dir, move_spd, self.target_yaw, self.have_ball)
    
    def closest_wall(self):
        """Return the closest actual field-wall normal and distance in cm."""
        tof_readings = (
            (0, self.tofs[0].read()),
            (90, self.tofs[1].read()),
            (180, self.tofs[2].read()),
            (-90, self.tofs[3].read()),
        )
        wall_normals = (0, 90, 180, -90)

        closest_normal = None
        closest_dist = None
        for wall_normal in wall_normals:
            best_alignment = None
            best_distance = None

            for sensor_dir, distance_mm in tof_readings:
                if distance_mm is None or distance_mm <= 0:
                    continue

                sensor_abs_dir = self.wrap_angle(self.bot_dir + sensor_dir)
                sensor_to_wall_angle = self.wrap_angle(sensor_abs_dir - wall_normal)
                alignment = math.cos(math.radians(sensor_to_wall_angle))
                if alignment <= 0:
                    continue

                # Project the angled ToF ray onto the real wall normal.
                distance_cm = (distance_mm / 10) * alignment
                if best_alignment is None or alignment > best_alignment:
                    best_alignment = alignment
                    best_distance = distance_cm

            if best_distance is None:
                continue

            if closest_dist is None or best_distance < closest_dist:
                closest_normal = wall_normal
                closest_dist = best_distance

        return closest_normal, closest_dist

    def avoid_wall(self, move_dir, move_spd):
        """
        If the bot is near a wall, remove the velocity component driving into it.
        """

        self.avoiding_wall = False
        self.nearest_wall_dir, self.nearest_wall_dist = self.closest_wall()
        if self.nearest_wall_dir is None or self.nearest_wall_dist is None:
            return move_dir, move_spd
        if self.nearest_wall_dist >= self.WALL_AVOID_THRESHOLD:
            return move_dir, move_spd

        self.avoiding_wall = True

        # Unit vector pointing toward the actual field wall normal in the robot frame.
        wall_rad = math.radians(self.to_relative_dir(self.nearest_wall_dir))
        toward_wall_x = math.sin(wall_rad)
        toward_wall_y = math.cos(wall_rad)

        # Decompose the intended movement vector into x and y
        move_rad = math.radians(move_dir)
        vx = move_spd * math.sin(move_rad)
        vy = move_spd * math.cos(move_rad)

        # Project movement onto the toward-wall axis
        dot = vx * toward_wall_x + vy * toward_wall_y

        # Only suppress the component if it's moving toward the wall (dot > 0)
        if dot > 0:
            vx -= dot * toward_wall_x
            vy -= dot * toward_wall_y

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
        if relative_dir is None:
            return None
        return relative_dir + self.bot_dir

    def to_relative_dir(self, absolute_dir):
        """Input a direction that ignores bot orientation\nReturns a direction relative to the bot orientation"""
        if absolute_dir is None:
            return None
        return absolute_dir - self.bot_dir

    def angle_towards(self, bot_x, bot_y, obj_x, obj_y):
        """Returns angle in degrees"""
        theta = math.degrees(math.atan2(obj_y - bot_y, obj_x - bot_x))
        theta = (theta - 90 + 180) % 360 - 180
        return theta

    def wrap_angle(self, theta):
        """Input an angle in degrees\nReturns same angle but in [-180,180)"""
        if theta is None:
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