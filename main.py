#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import ColorSensor, Motor, UltrasonicSensor, TouchSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Color, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

import math


# EV3
ev3 = EV3Brick()

# Motors
left_motor = Motor(Port.A,  positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
loader_motor = Motor(Port.C)

# Sensors
color_sensor = ColorSensor(Port.S4)
ultrasonic_sensor = UltrasonicSensor()
touch_sensor = TouchSensor()

# Drive Base
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=168)


class Helper():

    # @staticmethod
    # def hypotenuse(a: int | float, b: int | float) -> float:
    #     return math.sqrt(a ** 2 + b ** 2)

    # @staticmethod
    # def angle(a: int | float, b: int | float) -> float:
    #     return math.degrees(math.atan(a / b))

    def calculate_hypotenuse_length(self, side: int | float, angle: int) -> float:
        return side / math.cos(math.radians(angle))


class PidController:

    def __init__(self):
        self.last_error = 0
        self.integral = 0

        self.target_reflection = 18  # TODO najit nejlepsi hodnotu

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

        if side == "l":
            robot.drive(speed, p_fix + i_fix + d_fix)

        if side == "r":
            robot.drive(speed, -p_fix - i_fix - d_fix)

    # Follows a black line on given side with distance
    def distance(self, speed: int, distance: int | float, side: str = "l") -> None:
        robot.reset()

        while robot.distance() < distance:
            self._follow_line(speed, side)

        self.last_error = 0
        self.integral = 0


class SimpleDrive:

    @staticmethod
    def pickup_setup(side: int | float, angle: int) -> None:
        distance = helper.calculate_hypotenuse_length(side, angle)

        robot.turn(angle)
        robot.straight(-distance)
        robot.turn(-angle)

    @staticmethod
    def pickup() -> None:
        distance_to_ball = 100  # TODO zmerit!!!
        while ultrasonic_sensor.distance() > distance_to_ball:
            robot.drive(speed, 0)


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


robot.settings(turn_rate=Config.TURN_RATE, straight_speed=Config.SPEED)

pid = PidController()

helper = Helper()
print(helper.calculate_hypotenuse_length(100, 45))
