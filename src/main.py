#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Defines the starting values
TARGET_VALUE = 50
MOTOR_SPEED = 20
WHEEL_DIAMETER = 20
AXLE_TRACK = 100
PROPORTIONAL_COEFFICIENT = 0.5
INTEGRAL_COEFFICIENT = 1
DERIVATIVE_COEFFICIENT = 4

prevError = 0
prevErrors = []
integralValue = 0

# Initialize the EV3Brick
ev3 = EV3Brick()
# Initialize and configure the motors
motorA = Motor("A")
motorB = Motor("B")

driveBase = DriveBase(motorA, motorB, )
# Initialize and configure the color / light intensity sensor
lightSensor = ColorSensor("S1")

def proportionalController(error: int, coefficient: float) -> float:
    return error * coefficient

def integralController(errorValues: int[], cachedSum: int, newError: int, coefficient: float, maxLength: int = 1000) -> dict:
    errorValues.append(newError)
    cachedSum += newError

    # When the maximum Length of the array is reached, removes the oldest value and subtracts it from the cached sum
    if len(errorValues) > maxLength:
        reversedErrorValues = errorValues.reverse()
        oldVal = reversedErrorValues.pop()
        errorValues = reversedErrorValues.reverse()
        cachedSum -= oldVal

    return {integralValue: cachedSum * coefficient, errorValues: errorValues}

def derivativeController(prevError: int, newError: int, coefficient: float) -> float:
    return (prevError - newError) * coefficient

lightIntensity = lightSensor.reflection()
error = TARGET_VALUE - lightIntensity

proportionalValue = proportionalController(error, PROPORTIONAL_COEFFICIENT)

integralDict = integralController(prevErrors, integralValue, error, INTEGRAL_COEFFICIENT)
integralValue = integralDict.integralValue
prevErrors = integralDict.errorValues

derivativeValue = derivativeController(prevError, error, DERIVATIVE_COEFFICIENT)

turnRate = proportionalValue + integralValue + derivativeValue

# Drives the Robot
driveBase.drive(MOTOR_SPEED, turnRate)