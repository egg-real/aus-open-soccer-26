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
        self.HEAD_TO_GOAL_SPD = 0.05
        self.HEAD_TO_OWN_GOAL_SPD = 0.1
        self.BALL_ORBIT_RADIUS = 14  # might be an arbitrary number
        self.GIVE_UP_CHASING_BALL_TIME = 0.5 # seconds

        self.READY_TO_SHOOT_ANGLE = 20  # degrees
        self.READY_TO_SHOOT_DISTANCE = 125 # Note: This is currently unused

        self.CENTRE_X_TOLERANCE = 100  # mm, how close to field-centre x counts as "centred"
        self.CENTRE_X_READY_TO_SHOOT_YAW = 10  # degrees, yaw must be within +/- this to kick while centring x

        self.READY_TO_REBOUND_SHOOT_ANGLE = 15
        self.READY_TO_REBOUND_SHOOT_DISTANCE = 50
        self.REBOUND_SHOOT_PRECISION = 0.5 # Somewhere inbetween 0 and 2, lower the more precise

        self.GOALIE_MAX_ANGLE_FROM_GOAL = 15  # degrees, max angle (from goal) the goalie will shift laterally to track the ball
        self.DEFENCE_GOAL_DIST = 80  # cm, fixed standing distance in front of the own goal (via localisation)

        ## Wall boundary safety (hard floor near a wall, via localisation)
        self.MIN_DIST_FROM_FRONT_WALL = 50  # cm, never get closer than this to whichever wall the bot's front currently faces
        self.MIN_DIST_FROM_BACK_GOAL = 50  # cm, offence-only floor from the wall behind the bot's own ("back") goal
        self.WALL_BOUNDARY_BLOCK_SPD = 0.15  # Speed used to slide along a boundary line to block the ball
        self.WALL_BOUNDARY_CORRECTION_KP = 0.02  # P gain pushing the bot back to the boundary line if it dips inside it

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
        self.DEFENCE_MAXIMUM_DISTANCE_FROM_WALL = 150
        self.DEFENCE_MINIMUM_DISTANCE_FROM_WALL = 80

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
        self.was_paused = True
        self._recalibrate_on_resume = False
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

        paused_by_switch = not self.pause_switch.read()
        paused_by_comm = USE_COMM_MODULE and not self.comm_module.read()
        if paused_by_switch or paused_by_comm:
            self.was_paused = True
            if paused_by_switch:
                self._recalibrate_on_resume = True
            else:
                self._recalibrate_on_resume = False
            self.drive.stop()
            self.stop_dribbler()
            return

        if self.was_paused:
            self.was_paused = False
            self.cameras.start_streaming()
            if self._recalibrate_on_resume:
                self._recalibrate_on_resume = False
                self.target_goal = GoalColour.YELLOW if self.goal_switch.read() else GoalColour.BLUE
                self.drive.recalibrate_yaw()

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

    def own_goal_position(self):
        if self.target_goal == GoalColour.BLUE:
            return YELLOW_GOAL_POSITION
        if self.target_goal == GoalColour.YELLOW:
            return BLUE_GOAL_POSITION
        return None

    def distance_to_own_goal(self, position):
        goal_position = self.own_goal_position()
        if position is None or goal_position is None:
            return None

        return math.hypot(
            float(position[0]) - goal_position[0],
            float(position[1]) - goal_position[1],
        )

    def own_goal_localised_dist(self):
        """Distance (cm) from the localised bot position to the own goal."""
        dist_mm = self.distance_to_own_goal(self.bot_position)
        if dist_mm is None:
            return None
        return dist_mm / 10.0

    def ball_field_position(self):
        """Estimated field position (mm) of the ball, combining the localised
        bot position with the camera-relative ball bearing/distance."""
        if self.bot_position is None or self.ball_dir is None or self.ball_dist is None:
            return None

        bx, by = self.bot_position
        if bx is None or by is None:
            return None

        absolute_dir = self.to_absolute_dir(self.ball_dir)
        if absolute_dir is None:
            return None

        rad = math.radians(absolute_dir)
        ball_dist_mm = self.ball_dist * 10.0  # camera distances are in cm
        return float(bx) + ball_dist_mm * math.sin(rad), float(by) + ball_dist_mm * math.cos(rad)

    def goalie_target_position(self):
        """Desired standing spot for the goalie: a fixed distance
        (DEFENCE_GOAL_DIST, cm) in front of the own goal, shifted side-to-side
        to track the ball but clamped to within GOALIE_MAX_ANGLE_FROM_GOAL
        degrees of the goal's centre line (both measured via localisation)."""
        own_position = self.own_goal_position()
        if own_position is None:
            return None
        own_x, own_y = own_position

        into_field = 1.0 if own_x <= FIELD_X / 2.0 else -1.0
        stand_dist_mm = self.DEFENCE_GOAL_DIST * 10.0
        stand_x = own_x + into_field * stand_dist_mm

        max_lateral_mm = stand_dist_mm * math.tan(math.radians(self.GOALIE_MAX_ANGLE_FROM_GOAL))

        ball_position = self.ball_field_position()
        desired_y = ball_position[1] if ball_position is not None else own_y
        desired_y = min(max(desired_y, own_y - max_lateral_mm), own_y + max_lateral_mm)

        return stand_x, desired_y

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
                self.state = RobotState.LINING_UP
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

        own_goal_dist_localised = self.own_goal_localised_dist()

        # If ball very close, turn to attack mode
        if self.have_ball or (
            self.see_ball 
            and self.ball_dist is not None
            and self.ball_dist < self.TURN_TO_OFFENCE_BALL_DIST
            and (
                abs(self.wrap_angle(self.to_absolute_dir(self.ball_dir))) < 120
                or (own_goal_dist_localised is not None and own_goal_dist_localised > 80)
            ) # Ball not too behind or not too close to goal (reduce risk of own goaling)
            ):

            previous_mode = self.mode
            self.request_mode_change(RobotMode.OFFENCE)
            if self.mode != previous_mode:
                return

        elif self.bot_position is None or self.own_goal_position() is None:
            # No localisation fix yet; fall back to camera-only behaviour.
            if self.see_ball:
                self.ball_capture()
            else:
                self.move_dir = 0
                self.move_spd = 0
        else:
            self.execute_goalie()

    def execute_goalie(self):
        """Hold a fixed distance from the own goal and stay within
        GOALIE_MAX_ANGLE_FROM_GOAL of its centre line, using localisation for
        both the standing distance and the lateral tracking angle instead of
        the noisy camera-derived goal bearing/distance."""
        target_position = self.goalie_target_position()
        if target_position is None or self.bot_position is None:
            self.move_dir = 0
            self.move_spd = 0
            return

        bx, by = self.bot_position
        tx, ty = target_position
        dx = tx - float(bx)
        dy = ty - float(by)
        dist_mm = math.hypot(dx, dy)
        target_bearing = math.degrees(math.atan2(dx, dy))

        # Swerve around the ball rather than driving straight through it if it
        # sits directly on the path to the target spot and we're still far off.
        if (
            dist_mm > 100
            and self.see_ball
            and self.ball_dir is not None
            and abs(self.wrap_angle(target_bearing - self.to_absolute_dir(self.ball_dir))) < 20
        ):
            self.ball_capture()
            return

        if dist_mm < 20:
            self.move_dir = 0
            self.move_spd = 0
            return

        self.move_dir = self.to_relative_dir(target_bearing)
        self.move_spd = min(1.0, max(self.HEAD_TO_OWN_GOAL_SPD, dist_mm / 300.0))


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
            if self.is_ready_to_shoot() or (self.is_ready_to_rebound_shoot() and self.ENABLE_REBOUND_SHOT):
                print("Ready to kick")
                self.stop_dribbler()
                self.kick()
                self.possession_state = PossessionState.NONE
                return
            if self.see_goal and self.goal_dir is not None:
                print("see goal; trying to move and yaw correct towards goal")
                # Head toward the goal; the possession orbit in Drive aims us by
                # orbiting the ball toward target_yaw, and is_ready_to_shoot kicks.
                self.move_dir = self.goal_dir
                self.move_spd = self.HEAD_TO_GOAL_SPD
                print(self.goal_dir, self.bot_dir)
                self.target_yaw = self.to_absolute_dir(self.goal_dir)
            elif self.see_own_goal and self.own_goal_dir is not None:
                print("don't see goal; trying to head towards goal based on own goal")
                away_from_own_goal_dir = self.wrap_angle(self.own_goal_dir + 180)
                self.move_dir = away_from_own_goal_dir
                self.move_spd = self.HEAD_TO_GOAL_SPD
                #self.target_yaw = self.to_absolute_dir(away_from_own_goal_dir)
            else:
                print("Can't see either goal; centring x and kicking when aligned")
                self.try_to_centre_x()

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

    def own_goal_wall_bearing(self):
        """Absolute field bearing (deg) from the bot toward the wall behind its own ("back") goal."""
        if self.target_goal == GoalColour.YELLOW:
            return -90.0  # Own (Blue) goal sits on the West wall
        if self.target_goal == GoalColour.BLUE:
            return 90.0  # Own (Yellow) goal sits on the East wall
        return None

    def own_goal_wall_dist(self):
        """Distance (cm) from the bot to the wall behind its own goal, via localisation."""
        wall_dists = self.axis_wall_dists()
        if self.target_goal == GoalColour.YELLOW:
            return wall_dists["W"]
        if self.target_goal == GoalColour.BLUE:
            return wall_dists["E"]
        return None

    def enforce_wall_boundaries(self, move_dir, move_spd, target_yaw):
        """Hard floor so the bot never drives within MIN_DIST_FROM_FRONT_WALL of
        whichever wall its front currently faces (both modes), or - in offence
        mode only - within MIN_DIST_FROM_BACK_GOAL of the wall behind its own
        goal. Overrides the intended movement for the cycle if violated.
        """
        if self.mode == RobotMode.PENALTY:
            return move_dir, move_spd, target_yaw

        front_dist = self.wall_dist(0)
        if front_dist is not None and front_dist < self.MIN_DIST_FROM_FRONT_WALL:
            return self.hold_wall_boundary(self.bot_dir, self.MIN_DIST_FROM_FRONT_WALL, front_dist)

        if self.mode == RobotMode.OFFENCE:
            back_bearing = self.own_goal_wall_bearing()
            back_dist = self.own_goal_wall_dist()
            if back_bearing is not None and back_dist is not None and back_dist < self.MIN_DIST_FROM_BACK_GOAL:
                return self.hold_wall_boundary(back_bearing, self.MIN_DIST_FROM_BACK_GOAL, back_dist)

        return move_dir, move_spd, target_yaw

    def hold_wall_boundary(self, wall_bearing, min_dist, wall_dist):
        """Shared response for being nearer than `min_dist` to a boundary wall.

        With the ball: stop advancing into the wall, aim at the target goal and
        shoot as soon as lined up. Without the ball: if the ball is roughly
        between the bot and the wall, slide sideways along the min_dist line to
        stay in front of it - the same way the goalie blocks a ball
        approaching from the left or right.
        """
        if self.have_ball:
            self.dribble()
            target_yaw = self.target_yaw
            if self.see_goal and self.goal_dir is not None:
                target_yaw = self.to_absolute_dir(self.goal_dir)
            if self.is_ready_to_shoot() or self.is_ready_to_rebound_shoot():
                self.stop_dribbler()
                self.kick()
                self.possession_state = PossessionState.NONE
            return 0, 0, target_yaw

        wall_error = max(0.0, min_dist - wall_dist)  # > 0 while still too close
        away_from_wall = self.wrap_angle(wall_bearing + 180)
        correction_strength = min(1.0, self.WALL_BOUNDARY_CORRECTION_KP * wall_error)

        ball_dir = self.ball_dir if self.see_ball else self.last_ball_dir
        if ball_dir is None:
            away_rel = math.radians(self.to_relative_dir(away_from_wall))
            move_dir = math.degrees(math.atan2(math.sin(away_rel), math.cos(away_rel)))
            return move_dir, correction_strength, self.target_yaw

        ball_abs_dir = self.to_absolute_dir(ball_dir)
        lateral_sign = np.sign(self.wrap_angle(ball_abs_dir - wall_bearing))
        if lateral_sign == 0:
            lateral_sign = 1
        tangent_dir = self.wrap_angle(wall_bearing + 90 * lateral_sign)

        tangent_rel = math.radians(self.to_relative_dir(tangent_dir))
        away_rel = math.radians(self.to_relative_dir(away_from_wall))

        move_x = self.WALL_BOUNDARY_BLOCK_SPD * math.sin(tangent_rel) + correction_strength * math.sin(away_rel)
        move_y = self.WALL_BOUNDARY_BLOCK_SPD * math.cos(tangent_rel) + correction_strength * math.cos(away_rel)

        move_dir = math.degrees(math.atan2(move_x, move_y))
        move_spd = min(math.hypot(move_x, move_y), 1.0)
        target_yaw = self.to_absolute_dir(ball_dir)
        return move_dir, move_spd, target_yaw

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
            #self.target_yaw = self.ball_dir
            pass

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
            self.move_spd = 0.2
    
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

    def try_to_centre_x(self):
        """When holding the ball but neither goal is visible, only correct the
        x position (ignore y) and kick once x is centred and yaw is near 0."""
        self.target_yaw = 0

        if self.bot_position is None:
            self.move_dir = 0
            self.move_spd = 0
            return

        x, _ = self.bot_position
        if x is None:
            self.move_dir = 0
            self.move_spd = 0
            return

        centre_x = FIELD_X / 2.0
        dx = centre_x - float(x)
        x_centred = abs(dx) < self.CENTRE_X_TOLERANCE
        yaw_centred = abs(self.wrap_angle(self.bot_dir)) < self.CENTRE_X_READY_TO_SHOOT_YAW

        if x_centred and yaw_centred:
            print("X centred and yaw aligned; kicking")
            self.stop_dribbler()
            self.kick()
            self.possession_state = PossessionState.NONE
            return

        if x_centred:
            self.move_dir = 0
            self.move_spd = 0
            return

        absolute_dir = 90 if dx > 0 else -90
        self.move_dir = self.to_relative_dir(absolute_dir)
        self.move_spd = min(self.HEAD_TO_GOAL_SPD, max(0.03, abs(dx) / 1000.0 * self.HEAD_TO_GOAL_SPD))

    # ------ Action Functions ------ #

    def move(self):
        move_dir, move_spd, target_yaw = self.enforce_wall_boundaries(self.move_dir, self.move_spd, 9)
        move_dir, move_spd = self.avoid_wall(move_dir, move_spd)
        # print(move_dir, move_spd, target_yaw, self.have_ball)
        self.drive.move(move_dir, move_spd, target_yaw, self.have_ball)
    
    def closest_wall(self):
        """Return the closest field-wall normal (absolute bearing) and distance
        in cm, from the localised bot position rather than raw ToF reads."""
        wall_dists = self.axis_wall_dists()
        wall_normals = {"N": 0, "E": 90, "S": 180, "W": -90}

        closest_normal = None
        closest_dist = None
        for side, wall_normal in wall_normals.items():
            distance_cm = wall_dists[side]
            if distance_cm is None:
                continue
            if closest_dist is None or distance_cm < closest_dist:
                closest_normal = wall_normal
                closest_dist = distance_cm

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