#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import ColorSensor, Motor, UltrasonicSensor, TouchSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Color, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

import math
from time import sleep
import threading


# EV3
ev3 = EV3Brick()

# Motors
left_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
right_motor = Motor(Port.C, positive_direction=Direction.CLOCKWISE)
loader_motor = Motor(Port.A)

# Sensors
color_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S4)
touch_sensor = TouchSensor(Port.S3)

# Drive Base
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=168)


class Helper():

    # @staticmethod
    # def hypotenuse(a: int | float, b: int | float) -> float:
    #     return math.sqrt(a ** 2 + b ** 2)

    # @staticmethod
    # def angle(a: int | float, b: int | float) -> float:
    #     return math.degrees(math.atan(a / b))

    def calculate_hypotenuse_length(self, side: int, angle: int) -> float:
        return side / math.cos(math.radians(angle))


class PidController:

    def __init__(self):
        self.last_error = 0
        self.integral = 0

        self.target_reflection = 40  # TODO najit nejlepsi hodnotu

        self.PROPORTIONAL = 1.15
        self.INTEGRAL = 0.005
        self.DERIVATIVE = 0

    # Follows a black line on given side
    def _follow_line(self, speed: int, side: str = "l") -> None:
        error = color_sensor.reflection() - self.target_reflection
        p_fix = error * self.PROPORTIONAL

        self.integral += error
        i_fix = self.integral * self.INTEGRAL

        derivative = self.last_error - error
        d_fix = derivative * self.DERIVATIVE

        self.last_error = error

        if side == "r":
            robot.drive(speed, p_fix + i_fix + d_fix)

        if side == "l":
            robot.drive(speed, -p_fix - i_fix - d_fix)

    # Follows a black line on given side with distance
    def run_distance(self, speed: int, distance: int, side: str = "l") -> None:
        robot.reset()

        while robot.distance() < distance:
            self._follow_line(speed, side)

        self.last_error = 0
        self.integral = 0

    def run_ultrasonic(self, speed: int, distance: int, side: str = "l") -> None:
        robot.reset()

        while ultrasonic_sensor.distance() > distance:
            self._follow_line(speed, side)

        self.last_error = 0
        self.integral = 0


class SimpleDrive:

    @staticmethod
    def pickup_setup(side: int, angle: int) -> None:
        distance = helper.calculate_hypotenuse_length(side, angle)

        robot.turn(angle)
        robot.straight(-distance)
        robot.turn(-angle)

    @staticmethod
    def pickup(speed: int) -> None:
        distance_to_ball = 100  # TODO zmerit!!!
        while ultrasonic_sensor.distance() > distance_to_ball:
            robot.drive(speed / 2, 0)


class Arm:

    @staticmethod
    def initialize() -> None:
        loader_motor.run_time(-200, 750, then=Stop.BRAKE, wait=True)
        wait(50)
        loader_motor.run_time(200, 500, then=Stop.HOLD, wait=True)
        wait(50)
        loader_motor.run_angle(200, -5, then=Stop.HOLD, wait=False)

    @staticmethod
    def reset() -> None:
        loader_motor.run_time(200, 700, then=Stop.HOLD, wait=True)
        wait(50)
        loader_motor.run_angle(200, -5, then=Stop.HOLD, wait=False)
        wait(50)

    @staticmethod
    def lift_and_keep() -> None:
        loader_motor.run_angle(100, -20.0, then=Stop.HOLD, wait=True)

    @staticmethod
    def shoot(help_distance: int = 0) -> None:
        simple_drive.straight(-help_distance)
        loader_motor.run_time(-3000, 666, then=Stop.HOLD, wait=True)
        simple_drive.straight(help_distance)


class Config:
    SPEED = 190
    TURN_RATE = 100
    WHEEL_DIAMETER = 44.6  # mm (TODO premerit)
    AXLE_TRACK = 120  # mm
    CENSOR_TO_CENTER = 0.0  # TODO zmerit!!!


# robot.settings(turn_rate=Config.TURN_RATE, straight_speed=Config.SPEED)

class Robot:
    def __init__(self):
        # Initialize the robot's position
        self.position = (0, 0)
        self.orientation = 0  # Orientation in degrees

        self.grid = [[0] * 5 for _ in range(10)]

    def update_position(self, new_position, new_orientation):
        self.position = new_position
        self.orientation = new_orientation

    def detect_balls(self):
        ball_positions = [(2, 1), (4, 3), (1, 4)]

        for ball_position in ball_positions:
            self.grid[ball_position[0]][ball_position[1]] = 1

    def navigate_to_ball(self, ball_position):
        print("Navigating to ball at position: %s".format(ball_position))

    def collect_ball(self, ball_position):
        print("Collecting ball at position: %s".format(ball_position))
        self.grid[ball_position[0]][ball_position[1]] = 0

    def explore_playing_field(self):
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                if self.grid[row][col] == 1:  # Ball detected
                    ball_position = (col, row)
                    self.navigate_to_ball(ball_position)
                    self.collect_ball(ball_position)

    def print_grid(self):
        for row in self.grid:
            print(row)

pid = PidController()

helper = Helper()
arm = Arm()
simple_drive = SimpleDrive()

map = [
    0, -1, 1
]

balls = 0

def main() -> None:
    arm.initialize()
    pid.run_ultrasonic(Config.SPEED, 100)
    simple_drive.pickup_setup(100, 45)
    simple_drive.pickup(Config.SPEED)
    arm.lift_and_keep()

def return_to_base(index: int) -> None:
    pass


while not touch_sensor.pressed():
    continue
else:
    pid.run_ultrasonic(Config.SPEED / 3, 100, side="l")

