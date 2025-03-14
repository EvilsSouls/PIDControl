#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from ./pid import PIDControl

# Defines the starting tuning values
BLACK = 7
WHITE = 61

MOTOR_SPEED = 300
WHEEL_DIAMETER = 82
AXLE_TRACK = 120

PROPORTIONAL_GAIN = 1
INTEGRAL_GAIN = 0.5
DERIVATIVE_GAIN = 0

# Initializes the EV3Brick
ev3 = EV3Brick()
# Initializes and configures the motors
motorA = Motor(Port.A)
motorB = Motor(Port.D)
driveBase = DriveBase(motorA, motorB, WHEEL_DIAMETER, AXLE_TRACK)
# Initializes and configures the color / light intensity sensor
lightSensor = ColorSensor(Port.S4)

pid = PIDControl((BLACK + WHITE) / 2, PROPORTIONAL_GAIN, INTEGRAL_GAIN, DERIVATIVE_GAIN)

while True:
    turnRate = pid.calcPID(lightSensor.reflection())

    # Drives the Robot
    driveBase.drive(MOTOR_SPEED, turnRate)